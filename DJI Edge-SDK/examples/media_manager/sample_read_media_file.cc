// examples/media_manager/sample_read_media_file.cc
//
// Varianta care descarca fisierele media (JPG/MP4 etc.) prin MediaFilesReader
// si le salveaza local in /home/esdk/dji_dock_media_files (implicit).
//
// FIX IMPORTANT:
// - Face si "listare initiala" (MediaFilesReader::FileList) ca sa descarce
//   fisierele deja existente (din cea mai recenta misiune wayline), chiar daca
//   ai pornit sample-ul DUPA ce misiunea s-a terminat.
// - Dedupe pe file_path ca sa evite descarcari duble (FileList + observer).
// - Pentru fisiere >2GB: uneori file_size este saturat la INT32_MAX (2147483647).
//   In acel caz NU blocam rename() doar pe mismatch de size.

#include <unistd.h>

#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_set>

#include <climits>    // INT32_MAX
#include <cinttypes>  // PRIu64

#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>

#include "logger.h"
#include "media_manager.h"

using namespace edge_sdk;

ErrorCode ESDKInit();

namespace {

std::queue<MediaFile> g_queue;
std::mutex g_mtx;
std::condition_variable g_cv;

std::shared_ptr<MediaFilesReader> g_reader;

// dedupe: file_path e cheia cea mai buna (vine din callback si din FileList)
std::unordered_set<std::string> g_seen_paths;

// implicit: scrie aici; mai tarziu schimbi cu mountpoint-ul HDD-ului extern
std::string g_out_dir = "/home/esdk/dji_dock_media_files";

static std::string SanitizeFileName(const std::string& name)
{
    // elimina / sau \ ca sa nu creeze path-uri nedorite
    std::string out;
    out.reserve(name.size());
    for (char c : name) {
        if (c == '/' || c == '\\') out.push_back('_');
        else out.push_back(c);
    }
    if (out.empty()) out = "unnamed_file";
    return out;
}

static bool MkdirIfNotExists(const std::string& path, mode_t mode = 0775)
{
    if (path.empty()) return false;
    if (::mkdir(path.c_str(), mode) == 0) return true;
    if (errno == EEXIST) return true;
    return false;
}

static bool MkdirRecursive(const std::string& path)
{
    // creeaza /a/b/c incremental
    if (path.empty()) return false;

    std::string cur;
    cur.reserve(path.size());
    for (size_t i = 0; i < path.size(); i++) {
        cur.push_back(path[i]);
        if (path[i] == '/' && cur.size() > 1) {
            (void)MkdirIfNotExists(cur);
        }
    }
    return MkdirIfNotExists(path);
}

static bool StatFile(const std::string& p, struct stat* st)
{
    std::memset(st, 0, sizeof(*st));
    return (::stat(p.c_str(), st) == 0);
}

static bool FileExistsWithSize(const std::string& p, size_t expected_size)
{
    struct stat st;
    if (!StatFile(p, &st)) return false;
    return (size_t)st.st_size == expected_size;
}

static std::string DateFolderFromUnixTime(uint64_t unix_time_sec)
{
    // create_time din MediaFile e de obicei timestamp (sec). Il tratam ca secunde.
    time_t t = (time_t)unix_time_sec;
    struct tm tmv;
    localtime_r(&t, &tmv);

    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d", &tmv);
    return std::string(buf);
}

static std::string JoinPath(const std::string& a, const std::string& b)
{
    if (a.empty()) return b;
    if (b.empty()) return a;
    if (a.back() == '/') return a + b;
    return a + "/" + b;
}

static bool ShouldEnqueueLocked(const MediaFile& file)
{
    // IMPORTANT: se apeleaza doar sub g_mtx lock
    const std::string& key = file.file_path.empty() ? file.file_name : file.file_path;
    auto ins = g_seen_paths.insert(key);
    return ins.second;
}

static ErrorCode SaveMediaFileStreaming(const MediaFile& file)
{
    if (!g_reader) return kErrorSystemError;

    const std::string safe_name = SanitizeFileName(file.file_name);

    // Folder pe zi (poti simplifica la doar g_out_dir daca vrei)
    const std::string day_dir = JoinPath(g_out_dir, DateFolderFromUnixTime((uint64_t)file.create_time));
    if (!MkdirRecursive(day_dir)) {
        ERROR("Nu pot crea folderul: %s (errno=%d: %s)",
              day_dir.c_str(), errno, std::strerror(errno));
        return kErrorSystemError;
    }

    const std::string final_path = JoinPath(day_dir, safe_name);
    const std::string tmp_path = final_path + ".part";

    // Daca exista deja cu aceeasi dimensiune, nu mai descarca
    if (FileExistsWithSize(final_path, (size_t)file.file_size)) {
        INFO("SKIP (deja exista): %s (size=%zu)", final_path.c_str(), (size_t)file.file_size);
        return kOk;
    }

    auto fd = g_reader->Open(file.file_path);
    if (fd < 0) {
        ERROR("Open(remote) a esuat: %s", file.file_path.c_str());
        return kErrorSystemError;
    }

    FILE* out = std::fopen(tmp_path.c_str(), "wb");
    if (!out) {
        ERROR("Nu pot deschide pentru scriere: %s (errno=%d: %s)",
              tmp_path.c_str(), errno, std::strerror(errno));
        g_reader->Close(fd);
        return kErrorSystemError;
    }

    // buffer 1MB
    static constexpr size_t kBufSize = 1024 * 1024;
    char buf[kBufSize];

    size_t total = 0;
    while (true) {
        auto nread = g_reader->Read(fd, buf, sizeof(buf));
        if (nread <= 0) break;

        size_t nw = std::fwrite(buf, 1, (size_t)nread, out);
        if (nw != (size_t)nread) {
            ERROR("Eroare la fwrite: %s (scris=%zu, asteptat=%zu)",
                  tmp_path.c_str(), nw, (size_t)nread);
            std::fclose(out);
            g_reader->Close(fd);
            // lasam .part ca semn ca a picat
            return kErrorSystemError;
        }

        total += (size_t)nread;
    }

    std::fclose(out);
    g_reader->Close(fd);

    // Verificare de consistenta:
    // In unele versiuni/fluxuri, file_size poate fi saturat la INT32_MAX pentru fisiere > 2GB.
    // In cazul asta tratam marimea ca "necunoscuta" si NU blocam rename().
    const uint64_t expected = (uint64_t)file.file_size;
    const bool expected_known = (expected != 0 && expected != (uint64_t)INT32_MAX);

    if (expected_known) {
        if (total < expected) {
            // download incomplet -> pastram .part
            WARN("Dimensiune mai mica pentru %s: asteptat=%" PRIu64 ", descarcat=%zu (ramane .part)",
                 safe_name.c_str(), expected, total);
            return kErrorSystemError;
        }
        if (total != expected) {
            // total > expected: metadata poate fi gresita; continuam cu rename
            WARN("Dimensiune diferita pentru %s: asteptat=%" PRIu64 ", descarcat=%zu (continui rename)",
                 safe_name.c_str(), expected, total);
        }
    } else {
        WARN("file_size necunoscut/saturat pentru %s: raportat=%" PRIu64 ", descarcat=%zu (continui rename)",
             safe_name.c_str(), expected, total);
    }

    // rename atomic
    if (::rename(tmp_path.c_str(), final_path.c_str()) != 0) {
        ERROR("rename() a esuat: %s -> %s (errno=%d: %s)",
              tmp_path.c_str(), final_path.c_str(), errno, std::strerror(errno));
        return kErrorSystemError;
    }

    INFO("SALVAT: %s (size=%zu)", final_path.c_str(), total);
    return kOk;
}

static ErrorCode MediaFilesUpdateCallback(const MediaFile& file)
{
    INFO("File update: %s (size=%zu) path=%s",
         file.file_name.c_str(), (size_t)file.file_size, file.file_path.c_str());

    {
        std::lock_guard<std::mutex> lk(g_mtx);
        if (ShouldEnqueueLocked(file)) {
            g_queue.push(file);
            g_cv.notify_one();
        } else {
            INFO("DUP (ignore): %s path=%s", file.file_name.c_str(), file.file_path.c_str());
        }
    }

    return kOk;
}

static void WorkerThread()
{
    while (true) {
        MediaFile file;

        {
            std::unique_lock<std::mutex> lk(g_mtx);
            g_cv.wait(lk, [] { return !g_queue.empty(); });
            file = g_queue.front();
            g_queue.pop();
        }

        INFO("Process: %s", file.file_name.c_str());
        auto rc = SaveMediaFileStreaming(file);
        if (rc != kOk) {
            ERROR("Save failed: %s (rc=%d)", file.file_name.c_str(), (int)rc);
        }
    }
}

static ErrorCode EnqueueInitialFileList()
{
    if (!g_reader) return kErrorSystemError;

    // IMPORTANT: in Edge-SDK_official, tipul este:
    // MediaFilesReader::MediaFileList = std::list<std::shared_ptr<MediaFile>>
    MediaFilesReader::MediaFileList list;
    int32_t n = g_reader->FileList(list);
    if (n < 0) {
        WARN("MediaFilesReader::FileList() a esuat (n=%d).", (int)n);
        return kOk; // nu fatal; observer-ul va prinde viitoarele fisiere
    }

    INFO("Initial FileList: n=%d, list.size()=%zu", (int)n, list.size());

    {
        std::lock_guard<std::mutex> lk(g_mtx);
        for (const auto& pf : list) {
            if (!pf) continue;
            const MediaFile& f = *pf;

            if (ShouldEnqueueLocked(f)) {
                INFO("Initial enqueue: %s path=%s size=%zu",
                     f.file_name.c_str(), f.file_path.c_str(), (size_t)f.file_size);
                g_queue.push(f); // copiem obiectul in coada
            } else {
                INFO("Initial DUP (ignore): %s path=%s",
                     f.file_name.c_str(), f.file_path.c_str());
            }
        }
    }
    g_cv.notify_one();
    return kOk;
}

static ErrorCode StartSample()
{
    // asigura base folder
    if (!MkdirRecursive(g_out_dir)) {
        ERROR("Nu pot crea base folder: %s (errno=%d: %s)",
              g_out_dir.c_str(), errno, std::strerror(errno));
        return kErrorSystemError;
    }

    auto media_manager = MediaManager::Instance();

    g_reader = media_manager->CreateMediaFilesReader();
    auto rc = g_reader->Init();
    if (rc != kOk) {
        ERROR("Init MediaFilesReader a esuat (rc=%d)", (int)rc);
        return rc;
    }

    rc = media_manager->RegisterMediaFilesObserver(
        std::bind(MediaFilesUpdateCallback, std::placeholders::_1));
    if (rc != kOk) {
        ERROR("RegisterMediaFilesObserver a esuat (rc=%d)", (int)rc);
        return rc;
    }

    std::thread t(&WorkerThread);
    t.detach();

    INFO("Output dir: %s", g_out_dir.c_str());

    // FIX: listeaza media existenta (din cea mai recenta misiune) si pune in coada
    rc = EnqueueInitialFileList();
    if (rc != kOk) {
        WARN("EnqueueInitialFileList a intors rc=%d (continui cu observer).", (int)rc);
    }

    return kOk;
}

} // namespace

int main(int argc, char** argv)
{
    // optional: permite schimbarea path-ului fara recompilare:
    // ./sample_read_media_file /cale/catre/output
    if (argc >= 2 && argv[1] && std::strlen(argv[1]) > 0) {
        g_out_dir = argv[1];
    }

    auto rc = ESDKInit();
    if (rc != kOk) {
        ERROR("ESDKInit failed (rc=%d)", (int)rc);
        return -1;
    }

    // Ca sa nu se stearga fisierele de pe Dock inainte sa apuci sa le descarci
    rc = MediaManager::Instance()->SetDroneNestAutoDelete(false);
    if (rc != kOk) {
        ERROR("SetDroneNestAutoDelete(false) failed (rc=%d)", (int)rc);
        return -1;
    }

    rc = StartSample();
    if (rc != kOk) {
        ERROR("StartSample failed (rc=%d)", (int)rc);
        return -1;
    }

    while (true) {
        sleep(3);
    }
    return 0;
}
