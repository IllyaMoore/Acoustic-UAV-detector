// The whole on-device signal chain, hardware-free so it runs unchanged in the
// host tests and in tools/doa_cli on recordings pulled off the SD card.
//
//   3 x 1024 samples ─► Hann ─► FFT (2048, zero-padded) ─► keep in-band bins
//          │                                                   │
//          │                accumulate over K frames:          ▼
//          │                S_ij += X_i conj(X_j),  P += |X_i|^2
//          ▼                                                   │
//   every K frames (K=12 -> one block per 256 ms at 48 kHz) ◄──┘
//          ├─ GCC-PHAT peak per pair (direct lag evaluation)
//          ├─ least-squares bearing + closure check
//          ├─ detector: band SNR vs adaptive floor, tonality, coherence
//          └─ tracker: smoothed bearing while detections continue
//
// Averaging the cross-spectra BEFORE the PHAT weighting (rather than
// averaging K separate correlation peaks) is what makes this robust: a
// propeller harmonic adds up coherently frame after frame while wind, which is
// uncorrelated between capsules, averages toward zero.
//
// Mirrored step-for-step by analysis/uavdoa/pipeline.py.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "dsp/doa.hpp"
#include "dsp/fft.hpp"
#include "dsp/gcc_phat.hpp"

namespace uav {

struct PipelineConfig {
    float fs = 48000.0f;
    std::size_t frame_len = 1024;  // samples per channel per frame
    std::size_t avg_frames = 12;   // frames accumulated per decision block
    float f_lo = 200.0f;           // analysis band, Hz. Small quads put their
    float f_hi = 4000.0f;          // blade-pass harmonics here; see docs/RESEARCH.md
    float c = kSpeedOfSound;

    // Detector thresholds. These are starting points tuned on synthetic data
    // only - they MUST be re-tuned on field recordings (analysis/scripts/).
    float min_snr_db = 6.0f;        // band power over the adaptive noise floor
    float min_coherence = 0.25f;    // mean GCC-PHAT peak over the three pairs
    float min_tonality = 0.30f;     // 1 - spectral flatness; propellers are harmonic
    float max_closure_samples = 1.5f;

    float floor_rise_db_per_s = 1.0f;  // how fast the noise floor may creep up
    float track_alpha = 0.35f;         // bearing smoothing, 0 = frozen, 1 = raw
    int track_hold_blocks = 4;         // blocks without detection before the track drops
};

struct BlockResult {
    uint32_t index = 0;  // block counter since start
    std::array<PairDelay, kNumPairs> pairs{};
    Bearing bearing;        // raw, this block only
    float band_db = 0;      // band power, dB (arbitrary reference)
    float snr_db = 0;       // band power over noise floor
    float tonality = 0;     // 0 (noise-like) .. 1 (pure tones)
    float coherence = 0;    // mean pair peak
    float confidence = 0;   // 0..1, heuristic combination of the above
    bool detected = false;
    bool tracking = false;  // a smoothed track exists (may coast through misses)
    float track_az_deg = 0; // smoothed azimuth, valid while tracking
};

class Pipeline {
public:
    explicit Pipeline(const PipelineConfig &cfg = PipelineConfig());

    const PipelineConfig &config() const { return cfg_; }
    int max_lag() const { return max_lag_; }
    std::size_t bin_lo() const { return bin_lo_; }
    std::size_t n_bins() const { return n_bins_; }

    // Feed one frame: ch[m] points to frame_len samples of mic m (any scale).
    // Returns true when a block completed; the result is then in `out`.
    bool push_frame(const float *const ch[kNumMics], BlockResult &out);

private:
    void finish_block(BlockResult &out);

    PipelineConfig cfg_;
    std::size_t nfft_;
    std::size_t bin_lo_ = 0, n_bins_ = 0;
    int max_lag_ = 0;
    Fft fft_;
    DoaSolver doa_;
    std::vector<float> window_;
    std::vector<cf32> scratch_;                          // nfft
    std::array<std::vector<cf32>, kNumMics> band_;       // this frame, in-band bins
    std::array<std::vector<cf32>, kNumPairs> cross_;     // accumulated
    std::vector<float> power_;                           // accumulated, summed over mics
    std::size_t frames_in_block_ = 0;
    uint32_t block_index_ = 0;

    // Detector / tracker state.
    float floor_db_ = 0;
    bool floor_init_ = false;
    bool tracking_ = false;
    int miss_count_ = 0;
    float track_ux_ = 0, track_uy_ = 0;
};

}  // namespace uav
