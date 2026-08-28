# DJI Dock Edge SDK — Live Video & Media Pipeline

![](https://img.shields.io/badge/version-V1.2.0-green.svg)
![](https://img.shields.io/badge/platform-linux-cyan.svg)
![](https://img.shields.io/badge/language-C%2B%2B14-blue.svg)
![](https://img.shields.io/badge/license-MIT-purple.svg)

Edge-computing app built on DJI's official Edge SDK, running on a companion computer next to a DJI Dock. It authenticates with DJI's cloud, pulls the aircraft's live H.264 feed and mission media files off the dock, and re-exposes them as a browser-viewable HLS stream and a resumable local file sync.

## Features

- **Secure device link** — RSA-2048 identity + app license handshake with DJI Cloud API.
- **Live video** — subscribes to the aircraft's H.264 stream (up to 1080p) and decodes it in real time.
- **Media sync** — lists and downloads mission photos/videos straight from the dock.
- **Cloud interconnect** — custom event messaging channel to DJI Cloud.

## What I Added

- Ported the FFmpeg decoder to the modern FFmpeg 5/6 API (`avcodec_send_packet`/`receive_frame`), replacing deprecated calls.
- Built a live-view pipeline: decoded stream → FIFO → `ffmpeg` → HLS → browser (`hls.js`), with non-blocking, backpressure-aware writes so a slow reader can't corrupt frames.
- Rewrote the media downloader to stream to disk (no full-file RAM buffering), dedupe by path, write to date-partitioned folders, and commit via atomic rename — including a fix for files >2GB where reported size saturates.

## Build

```bash
mkdir build && cd build
cmake .. && make -j$(nproc)
```

Requires OpenSSL, libssh2, OpenCV ≥ 3.4.16, and FFmpeg 4.x–6.x.

## Credentials

`examples/init/app_info.h` uses placeholder values — replace with your own app credentials from the [DJI Developer Console](https://developer.dji.com/). 

## Attribution

Core SDK (`libedgesdk`, headers) is provided by DJI under MIT license — see [LICENSE](LICENSE). This repo extends DJI's examples with the pipeline work described above.

## Docs

[DJI Developer Documentation](https://developer.dji.com/doc/edge-sdk-tutorial/en/) · [API Reference](https://developer.dji.com/doc/edge-sdk-api-reference/en/)
