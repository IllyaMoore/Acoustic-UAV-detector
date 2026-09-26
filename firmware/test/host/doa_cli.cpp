// doa_cli - run the FIRMWARE pipeline over a WAV file on the host.
//
//   doa_cli recording.wav [--m3-lag N] [--band LO HI] > blocks.csv
//
// Input: the 3-channel (or wider; extra channels ignored) WAV the firmware
// writes to microSD, or anything from analysis/scripts/simulate_wav.py.
// PCM 16/24/32-bit and float32 are accepted. Output: one CSV row per block.
//
// Because this is the exact code that runs on the ESP32, a recording pulled
// off the card replays bit-for-bit (up to float rounding) what the device
// would have decided - which is how thresholds get tuned without reflashing.
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "dsp/aligner.hpp"
#include "dsp/pipeline.hpp"

namespace {

struct Wav {
    int channels = 0, bits = 0, format = 0;
    float fs = 0;
    std::vector<float> samples;  // interleaved, scaled to [-1, 1)
};

bool read_wav(const char *path, Wav &w) {
    FILE *f = std::fopen(path, "rb");
    if (!f) return false;
    char riff[12];
    if (std::fread(riff, 1, 12, f) != 12 || std::memcmp(riff, "RIFF", 4) || std::memcmp(riff + 8, "WAVE", 4)) {
        std::fclose(f);
        return false;
    }
    bool have_fmt = false;
    for (;;) {
        char id[4];
        uint32_t size;
        if (std::fread(id, 1, 4, f) != 4 || std::fread(&size, 4, 1, f) != 1) break;
        if (!std::memcmp(id, "fmt ", 4)) {
            std::vector<uint8_t> b(size);
            if (std::fread(b.data(), 1, size, f) != size) break;
            uint16_t fmt, ch, bits;
            uint32_t rate;
            std::memcpy(&fmt, &b[0], 2);
            std::memcpy(&ch, &b[2], 2);
            std::memcpy(&rate, &b[4], 4);
            std::memcpy(&bits, &b[14], 2);
            if (fmt == 0xFFFE && size >= 26) std::memcpy(&fmt, &b[24], 2);  // WAVE_FORMAT_EXTENSIBLE
            w.format = fmt;
            w.channels = ch;
            w.fs = static_cast<float>(rate);
            w.bits = bits;
            have_fmt = true;
        } else if (!std::memcmp(id, "data", 4) && have_fmt) {
            // A WAV whose header was never finalised (power cut mid-file)
            // carries size 0 or 0xFFFFFFFF - read to end of file instead.
            const int bps = w.bits / 8;
            std::vector<uint8_t> raw;
            if (size == 0 || size == 0xFFFFFFFFu) {
                uint8_t buf[65536];
                std::size_t got;
                while ((got = std::fread(buf, 1, sizeof buf, f)) > 0) raw.insert(raw.end(), buf, buf + got);
            } else {
                raw.resize(size);
                raw.resize(std::fread(raw.data(), 1, size, f));
            }
            const std::size_t n = raw.size() / bps;
            w.samples.resize(n);
            for (std::size_t i = 0; i < n; ++i) {
                const uint8_t *p = &raw[i * bps];
                float v = 0;
                if (w.format == 3 && bps == 4) {
                    std::memcpy(&v, p, 4);
                } else if (bps == 2) {
                    int16_t s;
                    std::memcpy(&s, p, 2);
                    v = s / 32768.0f;
                } else if (bps == 3) {
                    int32_t s = (p[0] << 8) | (p[1] << 16) | (p[2] << 24);
                    v = static_cast<float>(s) / 2147483648.0f;
                } else if (bps == 4) {
                    int32_t s;
                    std::memcpy(&s, p, 4);
                    v = static_cast<float>(s) / 2147483648.0f;
                }
                w.samples[i] = v;
            }
            std::fclose(f);
            return true;
        } else {
            std::fseek(f, size + (size & 1), SEEK_CUR);
        }
    }
    std::fclose(f);
    return false;
}

}  // namespace

int main(int argc, char **argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s file.wav [--m3-lag N] [--band LO HI]\n", argv[0]);
        return 2;
    }
    uav::PipelineConfig cfg;
    int m3_lag = 0;
    for (int i = 2; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--m3-lag") && i + 1 < argc) {
            m3_lag = std::atoi(argv[++i]);
        } else if (!std::strcmp(argv[i], "--band") && i + 2 < argc) {
            cfg.f_lo = std::strtof(argv[++i], nullptr);
            cfg.f_hi = std::strtof(argv[++i], nullptr);
        } else {
            std::fprintf(stderr, "unknown argument: %s\n", argv[i]);
            return 2;
        }
    }
    Wav w;
    if (!read_wav(argv[1], w) || w.channels < uav::kNumMics) {
        std::fprintf(stderr, "cannot read a >=3-channel WAV from %s\n", argv[1]);
        return 1;
    }
    cfg.fs = w.fs;

    uav::Pipeline pl(cfg);
    uav::ChannelAligner aligner(m3_lag);
    const std::size_t frames = w.samples.size() / w.channels;
    std::vector<float> ch[uav::kNumMics];
    for (auto &c : ch) c.resize(cfg.frame_len);

    std::printf("block,t_s,az_deg,el_deg,lag01,lag02,lag12,peak01,peak02,peak12,"
                "coherence,closure_samples,band_db,snr_db,tonality,confidence,detected,tracking,track_az_deg\n");
    uav::BlockResult r;
    const float block_s = cfg.frame_len * cfg.avg_frames / cfg.fs;
    for (std::size_t off = 0; off + cfg.frame_len <= frames; off += cfg.frame_len) {
        for (std::size_t k = 0; k < cfg.frame_len; ++k)
            for (int m = 0; m < uav::kNumMics; ++m) ch[m][k] = w.samples[(off + k) * w.channels + m];
        float *x[uav::kNumMics] = {ch[0].data(), ch[1].data(), ch[2].data()};
        aligner.process(x, cfg.frame_len);
        const float *cx[uav::kNumMics] = {x[0], x[1], x[2]};
        if (!pl.push_frame(cx, r)) continue;
        std::printf("%u,%.4f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.4f,%.4f,%d,%d,%.3f\n",
                    r.index, (r.index + 1) * block_s, r.bearing.az_deg, r.bearing.el_deg,
                    r.pairs[0].lag, r.pairs[1].lag, r.pairs[2].lag,
                    r.pairs[0].peak, r.pairs[1].peak, r.pairs[2].peak, r.coherence,
                    r.bearing.closure_s * cfg.fs, r.band_db, r.snr_db, r.tonality, r.confidence,
                    r.detected ? 1 : 0, r.tracking ? 1 : 0, r.track_az_deg);
    }
    return 0;
}
