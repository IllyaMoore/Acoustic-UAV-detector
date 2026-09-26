// Host unit tests for components/dsp. Run: make -C firmware/test/host test
#include <cmath>
#include <complex>
#include <random>
#include <string>
#include <vector>

#include "check.hpp"
#include "dsp/aligner.hpp"
#include "dsp/doa.hpp"
#include "dsp/fft.hpp"
#include "dsp/pipeline.hpp"
#include "report.hpp"
#include "synth.hpp"

using namespace uav;

static void test_fft_matches_dft() {
    const std::size_t n = 64;
    std::mt19937 rng(1);
    std::normal_distribution<float> g;
    std::vector<cf32> x(n), X(n);
    for (auto &v : x) v = cf32(g(rng), g(rng));
    X = x;
    Fft(n).forward(X.data());
    for (std::size_t k = 0; k < n; ++k) {
        std::complex<double> ref = 0;
        for (std::size_t t = 0; t < n; ++t)
            ref += std::complex<double>(x[t]) * std::polar(1.0, -2 * M_PI * k * t / n);
        CHECK_NEAR(X[k].real(), ref.real(), 1e-3);
        CHECK_NEAR(X[k].imag(), ref.imag(), 1e-3);
    }
}

static void test_doa_inverts_ideal_delays() {
    DoaSolver doa;
    for (double az = 0; az < 360; az += 7.5) {
        for (double el : {0.0, 30.0, 60.0}) {
            std::array<float, kNumPairs> tau{};
            for (int p = 0; p < kNumPairs; ++p)
                tau[p] = static_cast<float>(arrival_s(kPairs[p].i, az, el) - arrival_s(kPairs[p].j, az, el));
            const Bearing b = doa.solve(tau);
            CHECK_NEAR(angle_diff_deg(b.az_deg, static_cast<float>(az)), 0.0, 0.01);
            CHECK_NEAR(b.el_deg, el, 0.2);
            CHECK_NEAR(b.closure_s, 0.0, 1e-9);
        }
    }
}

static void test_angle_helpers() {
    CHECK_NEAR(wrap_deg(-10), 350, 1e-4);
    CHECK_NEAR(wrap_deg(725), 5, 1e-4);
    CHECK_NEAR(angle_diff_deg(2, 358), 4, 1e-4);
    CHECK_NEAR(angle_diff_deg(358, 2), -4, 1e-4);
}

// Runs the pipeline over `blocks` blocks of a synthetic scene, returns the last result.
static BlockResult run_scene(Pipeline &pl, const std::vector<Tone> &tones, double az, double el,
                             double snr_db, int blocks, unsigned seed) {
    const auto &cfg = pl.config();
    const std::size_t n = cfg.frame_len * cfg.avg_frames * blocks;
    std::vector<float> ch[kNumMics];
    for (int m = 0; m < kNumMics; ++m) ch[m] = render(tones, m, az, el, n, cfg.fs, snr_db, seed + m);
    BlockResult r, last;
    for (std::size_t off = 0; off + cfg.frame_len <= n; off += cfg.frame_len) {
        const float *f[kNumMics] = {ch[0].data() + off, ch[1].data() + off, ch[2].data() + off};
        if (pl.push_frame(f, r)) last = r;
    }
    return last;
}

static void test_pipeline_recovers_azimuth() {
    const auto tones = drone_tones(7);
    for (double az : {0.0, 33.0, 95.0, 180.0, 241.0, 300.0, 359.0}) {
        Pipeline pl;
        const BlockResult r = run_scene(pl, tones, az, 25.0, 10.0, 1, 100);
        CHECK_NEAR(angle_diff_deg(r.bearing.az_deg, static_cast<float>(az)), 0.0, 1.5);
        CHECK(r.coherence > 0.3f);
        CHECK(std::fabs(r.bearing.closure_s * 48000) < 0.5f);
    }
}

static void test_lags_are_subsample_accurate() {
    const auto tones = drone_tones(3);
    Pipeline pl;
    const double az = 71.3, el = 10;
    const BlockResult r = run_scene(pl, tones, az, el, 30.0, 1, 5);
    for (int p = 0; p < kNumPairs; ++p) {
        const double truth = (arrival_s(kPairs[p].i, az, el) - arrival_s(kPairs[p].j, az, el)) * 48000;
        CHECK_NEAR(r.pairs[p].lag, truth, 0.15);
    }
}

static void test_detector_quiet_then_drone() {
    Pipeline pl;
    const auto &cfg = pl.config();
    const std::size_t block = cfg.frame_len * cfg.avg_frames;
    std::mt19937 rng(42);
    std::normal_distribution<float> g(0, 0.01f);
    const auto tones = drone_tones(11);

    std::vector<float> ch[kNumMics];
    const int quiet_blocks = 8, drone_blocks = 8;
    for (int m = 0; m < kNumMics; ++m) {
        ch[m].resize(block * (quiet_blocks + drone_blocks));
        for (auto &v : ch[m]) v = g(rng);  // independent sensor noise on every mic
        const auto d = render(tones, m, 150.0, 20.0, block * drone_blocks, cfg.fs, 300, 0);
        for (std::size_t k = 0; k < d.size(); ++k) ch[m][block * quiet_blocks + k] += 0.05f * d[k];
    }
    int false_alarms = 0, hits = 0;
    BlockResult r;
    for (std::size_t off = 0; off + cfg.frame_len <= ch[0].size(); off += cfg.frame_len) {
        const float *f[kNumMics] = {ch[0].data() + off, ch[1].data() + off, ch[2].data() + off};
        if (!pl.push_frame(f, r)) continue;
        if (r.index < static_cast<uint32_t>(quiet_blocks)) {
            false_alarms += r.detected;
        } else if (r.detected) {
            ++hits;
            CHECK_NEAR(angle_diff_deg(r.bearing.az_deg, 150.0f), 0.0, 2.0);
        }
    }
    CHECK(false_alarms == 0);
    CHECK(hits >= drone_blocks - 1);
    CHECK(r.tracking);
    CHECK_NEAR(angle_diff_deg(r.track_az_deg, 150.0f), 0.0, 2.0);
}

static void test_aligner_removes_offset() {
    for (int lag : {3, -2}) {
        ChannelAligner al(lag);
        const std::size_t n = 32;
        std::vector<float> a(n), b(n), c(n);
        for (std::size_t k = 0; k < n; ++k) a[k] = b[k] = static_cast<float>(k);
        // M3 is `lag` samples late (or early): sample k of M3 is event k - lag.
        for (std::size_t k = 0; k < n; ++k) c[k] = static_cast<float>(static_cast<int>(k) - lag);
        float *x[kNumMics] = {a.data(), b.data(), c.data()};
        al.process(x, n);
        for (std::size_t k = 8; k < n; ++k) {
            CHECK_NEAR(a[k], c[k], 0);
            CHECK_NEAR(b[k], c[k], 0);
        }
    }
}

static void test_report_sentence() {
    BlockResult r;
    r.detected = true;
    r.bearing.az_deg = 350.0f;
    r.bearing.el_deg = NAN;
    r.confidence = 0.5f;
    char buf[160];
    const std::size_t n = report::format_doa(buf, sizeof buf, 7, 1234, r, 20.0f);
    const std::string s(buf, n);
    CHECK(s.rfind("$UAVDOA,7,1234,1,350.0,10.0,-1.0,", 0) == 0);  // heading wraps, NaN el -> -1
    const std::size_t star = s.find('*');
    CHECK(star != std::string::npos && s.size() == star + 5);      // *HH\r\n
    char hex[3];
    std::snprintf(hex, sizeof hex, "%02X", report::nmea_checksum(buf + 1, star - 1));
    CHECK(s.substr(star + 1, 2) == hex);
}

int main() {
    test_fft_matches_dft();
    test_doa_inverts_ideal_delays();
    test_angle_helpers();
    test_pipeline_recovers_azimuth();
    test_lags_are_subsample_accurate();
    test_detector_quiet_then_drone();
    test_aligner_removes_offset();
    test_report_sentence();
    return summarize("test_dsp");
}
