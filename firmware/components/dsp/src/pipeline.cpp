#include "dsp/pipeline.hpp"

#include <algorithm>
#include <cmath>

namespace uav {

namespace {
constexpr float kPi = 3.14159265358979f;
constexpr float kRadToDeg = 57.29577951308232f;

float clamp01(float v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
}  // namespace

Pipeline::Pipeline(const PipelineConfig &cfg)
    : cfg_(cfg), nfft_(2 * cfg.frame_len), fft_(nfft_), doa_(cfg.c),
      window_(cfg.frame_len), scratch_(nfft_) {
    // In-band FFT bins: f_lo <= k * fs / nfft <= f_hi, never DC or Nyquist.
    const float df = cfg_.fs / static_cast<float>(nfft_);
    std::size_t lo = static_cast<std::size_t>(std::ceil(cfg_.f_lo / df));
    std::size_t hi = static_cast<std::size_t>(std::floor(cfg_.f_hi / df));
    lo = std::max<std::size_t>(lo, 1);
    hi = std::min<std::size_t>(hi, nfft_ / 2 - 1);
    bin_lo_ = lo;
    n_bins_ = hi >= lo ? hi - lo + 1 : 0;

    // Largest physical lag plus one so the parabolic fit always has a
    // neighbour on both sides of a peak sitting right at end-fire.
    float max_base = 0;
    for (const Pair &p : kPairs) max_base = std::max(max_base, pair_baseline_m(p));
    max_lag_ = static_cast<int>(std::ceil(max_base / cfg_.c * cfg_.fs)) + 1;

    // Periodic Hann window.
    for (std::size_t n = 0; n < cfg_.frame_len; ++n)
        window_[n] = 0.5f - 0.5f * std::cos(2.0f * kPi * n / cfg_.frame_len);

    for (auto &b : band_) b.assign(n_bins_, cf32(0, 0));
    for (auto &c : cross_) c.assign(n_bins_, cf32(0, 0));
    power_.assign(n_bins_, 0.0f);
}

bool Pipeline::push_frame(const float *const ch[kNumMics], BlockResult &out) {
    for (int m = 0; m < kNumMics; ++m) {
        for (std::size_t n = 0; n < cfg_.frame_len; ++n) scratch_[n] = cf32(ch[m][n] * window_[n], 0);
        std::fill(scratch_.begin() + cfg_.frame_len, scratch_.end(), cf32(0, 0));
        fft_.forward(scratch_.data());
        for (std::size_t b = 0; b < n_bins_; ++b) {
            const cf32 v = scratch_[bin_lo_ + b];
            band_[m][b] = v;
            power_[b] += std::norm(v);
        }
    }
    for (int p = 0; p < kNumPairs; ++p) {
        const auto &xi = band_[kPairs[p].i];
        const auto &xj = band_[kPairs[p].j];
        for (std::size_t b = 0; b < n_bins_; ++b) cross_[p][b] += xi[b] * std::conj(xj[b]);
    }
    if (++frames_in_block_ < cfg_.avg_frames) return false;
    finish_block(out);
    return true;
}

void Pipeline::finish_block(BlockResult &out) {
    out = BlockResult();
    out.index = block_index_++;

    std::array<float, kNumPairs> tau{};
    float coh = 0;
    for (int p = 0; p < kNumPairs; ++p) {
        out.pairs[p] = gcc_phat_peak(cross_[p].data(), bin_lo_, n_bins_, nfft_, max_lag_, cfg_.fs);
        tau[p] = out.pairs[p].tau_s;
        coh += out.pairs[p].peak;
    }
    out.coherence = coh / kNumPairs;
    out.bearing = doa_.solve(tau);

    // Band power and spectral flatness (geometric mean / arithmetic mean).
    double sum = 0, log_sum = 0;
    for (std::size_t b = 0; b < n_bins_; ++b) {
        const double v = static_cast<double>(power_[b]) + 1e-30;
        sum += v;
        log_sum += std::log(v);
    }
    const double mean = sum / static_cast<double>(n_bins_ ? n_bins_ : 1);
    const double gmean = std::exp(log_sum / static_cast<double>(n_bins_ ? n_bins_ : 1));
    out.tonality = clamp01(1.0f - static_cast<float>(gmean / mean));
    const double norm = static_cast<double>(cfg_.avg_frames) * kNumMics;
    out.band_db = 10.0f * static_cast<float>(std::log10(mean / norm));

    // Adaptive noise floor: drops quickly, rises slowly - and ten times slower
    // still while something is being detected, so a drone hovering for a
    // minute is not absorbed into the "background".
    const float block_s = cfg_.avg_frames * cfg_.frame_len / cfg_.fs;
    if (!floor_init_) {
        floor_db_ = out.band_db;
        floor_init_ = true;
    }
    out.snr_db = out.band_db - floor_db_;

    const float closure_samples = std::fabs(out.bearing.closure_s) * cfg_.fs;
    out.detected = out.snr_db >= cfg_.min_snr_db && out.coherence >= cfg_.min_coherence &&
                   out.tonality >= cfg_.min_tonality &&
                   closure_samples <= cfg_.max_closure_samples;

    if (out.band_db < floor_db_) {
        floor_db_ += 0.5f * (out.band_db - floor_db_);
    } else {
        const float rise = cfg_.floor_rise_db_per_s * block_s * (out.detected ? 0.1f : 1.0f);
        floor_db_ = std::min(out.band_db, floor_db_ + rise);
    }

    // Heuristic confidence: geometric mean of three margins, each 0.5 at its
    // threshold and saturating at twice the threshold.
    const float c_snr = clamp01(out.snr_db / (2 * cfg_.min_snr_db));
    const float c_coh = clamp01(out.coherence / (2 * cfg_.min_coherence));
    const float c_ton = clamp01(out.tonality / (2 * cfg_.min_tonality));
    out.confidence = std::cbrt(c_snr * c_coh * c_ton);

    // Tracker: exponential smoothing of the bearing as a unit vector (so the
    // 359 -> 1 degree wrap is a non-event), coasting through short dropouts.
    if (out.detected) {
        const float az = out.bearing.az_deg / kRadToDeg;
        const float ux = std::sin(az), uy = std::cos(az);
        if (!tracking_) {
            track_ux_ = ux;
            track_uy_ = uy;
            tracking_ = true;
        } else {
            const float a = cfg_.track_alpha;
            track_ux_ = (1 - a) * track_ux_ + a * ux;
            track_uy_ = (1 - a) * track_uy_ + a * uy;
            const float r = std::sqrt(track_ux_ * track_ux_ + track_uy_ * track_uy_);
            if (r > 1e-6f) {
                track_ux_ /= r;
                track_uy_ /= r;
            }
        }
        miss_count_ = 0;
    } else if (tracking_ && ++miss_count_ > cfg_.track_hold_blocks) {
        tracking_ = false;
    }
    out.tracking = tracking_;
    if (tracking_) out.track_az_deg = wrap_deg(std::atan2(track_ux_, track_uy_) * kRadToDeg);

    for (auto &c : cross_) std::fill(c.begin(), c.end(), cf32(0, 0));
    std::fill(power_.begin(), power_.end(), 0.0f);
    frames_in_block_ = 0;
}

}  // namespace uav
