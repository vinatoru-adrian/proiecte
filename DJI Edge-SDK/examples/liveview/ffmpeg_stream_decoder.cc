/**
 ********************************************************************
 *
 * @copyright (c) 2023 DJI. All rights reserved.
 *
 *********************************************************************
 */
#include "ffmpeg_stream_decoder.h"

#include <mutex>
#include <unistd.h>

extern "C" {
#include <libavutil/error.h>
#include <libavutil/imgutils.h>
}

#include "logger.h"
#include "opencv2/opencv.hpp"

using namespace edge_sdk;

namespace edge_app {

FFmpegStreamDecoder::FFmpegStreamDecoder(const std::string &name)
    : StreamDecoder(name) {}

FFmpegStreamDecoder::~FFmpegStreamDecoder() {}

int32_t FFmpegStreamDecoder::Init() {
    // FFmpeg 5/6: nu mai există avcodec_register_all()

    const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) {
        ERROR("Failed to find H264 decoder codec");
        return -2;
    }

    pCodecParserCtx = av_parser_init(codec->id);
    if (!pCodecParserCtx) {
        ERROR("Failed to init codec parser");
        return -3;
    }

    pCodecCtx = avcodec_alloc_context3(codec);
    if (!pCodecCtx) {
        ERROR("Failed to alloc codec context");
        return -1;
    }

    // (opțional) threading
    pCodecCtx->thread_count = 4;

    int ret = avcodec_open2(pCodecCtx, codec, nullptr);
    if (ret < 0) {
        char buferr[128];
        ERROR("avcodec_open2 failed: %s",
              av_make_error_string(buferr, sizeof(buferr), ret));
        return -6;
    }

    // păstrăm membrul existent (header-ul are AVCodec*)
    pCodec = const_cast<AVCodec *>(codec);

    pFrameYUV = av_frame_alloc();
    if (!pFrameYUV) {
        ERROR("Failed to alloc YUV frame");
        return -4;
    }

    pFrameRGB = av_frame_alloc();
    if (!pFrameRGB) {
        ERROR("Failed to alloc RGB frame");
        return -5;
    }

    pSwsCtx = nullptr;
    rgbBuf = nullptr;
    bufSize = 0;
    decode_width = 0;
    decode_hight = 0;

    return 0;
}

int32_t FFmpegStreamDecoder::DeInit() {
    if (pSwsCtx) {
        sws_freeContext(pSwsCtx);
        pSwsCtx = nullptr;
    }

    if (pFrameYUV) {
        av_frame_free(&pFrameYUV);
        pFrameYUV = nullptr;
    }

    if (pFrameRGB) {
        av_frame_free(&pFrameRGB);
        pFrameRGB = nullptr;
    }

    if (pCodecParserCtx) {
        av_parser_close(pCodecParserCtx);
        pCodecParserCtx = nullptr;
    }

    if (pCodecCtx) {
        avcodec_free_context(&pCodecCtx);
        pCodecCtx = nullptr;
    }

    pCodec = nullptr;

    if (rgbBuf) {
        av_free(rgbBuf);
        rgbBuf = nullptr;
    }

    return 0;
}

int32_t FFmpegStreamDecoder::Decode(const uint8_t *data, size_t length,
                                    DecodeResultCallback result_callback) {
    const uint8_t *pData = data;
    int remainingLen = static_cast<int>(length);
    int processedLen = 0;

    std::lock_guard<std::mutex> l(decode_mutex);

    AVPacket pkt;
    av_init_packet(&pkt);
    pkt.data = nullptr;
    pkt.size = 0;

    while (remainingLen > 0) {
        if (!pCodecParserCtx || !pCodecCtx) break;

        processedLen = av_parser_parse2(
            pCodecParserCtx,
            pCodecCtx,
            &pkt.data,
            &pkt.size,
            pData,
            remainingLen,
            AV_NOPTS_VALUE,
            AV_NOPTS_VALUE,
            AV_NOPTS_VALUE);

        if (processedLen < 0) {
            ERROR("av_parser_parse2 failed: %d", processedLen);
            break;
        }

        remainingLen -= processedLen;
        pData += processedLen;

        if (pkt.size <= 0) continue;

        // API nou: send_packet / receive_frame
        int ret = avcodec_send_packet(pCodecCtx, &pkt);
        if (ret < 0) {
            char errbuf[128];
            ERROR("avcodec_send_packet error: %s",
                  av_make_error_string(errbuf, sizeof(errbuf), ret));
            av_packet_unref(&pkt);
            continue;
        }

        while (true) {
            ret = avcodec_receive_frame(pCodecCtx, pFrameYUV);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
                break;
            } else if (ret < 0) {
                char errbuf[128];
                ERROR("avcodec_receive_frame error: %s",
                      av_make_error_string(errbuf, sizeof(errbuf), ret));
                break;
            }

            int w = pFrameYUV->width;
            int h = pFrameYUV->height;
            if (w <= 0 || h <= 0) continue;

            if (w != decode_width || h != decode_hight) {
                decode_width = w;
                decode_hight = h;

                if (pSwsCtx) {
                    sws_freeContext(pSwsCtx);
                    pSwsCtx = nullptr;
                }
                if (rgbBuf) {
                    av_free(rgbBuf);
                    rgbBuf = nullptr;
                }
                bufSize = 0;

                INFO("New H264 Width: %d, Height: %d", w, h);
            }

            const AVPixelFormat src_fmt = static_cast<AVPixelFormat>(pFrameYUV->format);

            if (!pSwsCtx) {
                pSwsCtx = sws_getContext(
                    w, h, src_fmt,
                    w, h, AV_PIX_FMT_RGB24,
                    SWS_BILINEAR, nullptr, nullptr, nullptr);
                if (!pSwsCtx) {
                    ERROR("sws_getContext failed");
                    break;
                }
            }

            if (!rgbBuf) {
                bufSize = av_image_get_buffer_size(AV_PIX_FMT_RGB24, w, h, 1);
                rgbBuf = static_cast<uint8_t *>(av_malloc(bufSize));
                if (!rgbBuf) {
                    ERROR("Failed to allocate rgbBuf");
                    break;
                }

                int fill_ret = av_image_fill_arrays(
                    pFrameRGB->data,
                    pFrameRGB->linesize,
                    rgbBuf,
                    AV_PIX_FMT_RGB24,
                    w, h, 1);
                if (fill_ret < 0) {
                    char errbuf[128];
                    ERROR("av_image_fill_arrays failed: %s",
                          av_make_error_string(errbuf, sizeof(errbuf), fill_ret));
                    break;
                }
            }

            sws_scale(pSwsCtx,
                      (uint8_t const *const *)pFrameYUV->data,
                      pFrameYUV->linesize,
                      0,
                      h,
                      pFrameRGB->data,
                      pFrameRGB->linesize);

            if (pFrameRGB->data[0]) {
                cv::Mat tmp(h, w, CV_8UC3, pFrameRGB->data[0], pFrameRGB->linesize[0]);
                cv::Mat cvtmp;
                cv::cvtColor(tmp, cvtmp, cv::COLOR_RGB2BGR);
                auto mat_ptr = std::make_shared<cv::Mat>(cvtmp);
                result_callback(mat_ptr);
            }
        }

        av_packet_unref(&pkt);
    }

    return 0;
}

}  // namespace edge_app
