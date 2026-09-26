// Analytic test signals with EXACT fractional delays: every component is a
// sinusoid, so delaying by d seconds is just a phase shift - no interpolation
// error contaminates what the tests measure.
#pragma once

#include <cmath>
#include <random>
#include <vector>

#include "dsp/array_geometry.hpp"

struct Tone {
    double f, amp, phase;
};

// Drone-like: a blade-pass harmonic series plus a broadband bed of many
// random tones inside [200, 4000] Hz.
inline std::vector<Tone> drone_tones(unsigned seed, double bpf = 180.0, int harmonics = 12) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> ph(0, 2 * M_PI), fr(200, 4000);
    std::vector<Tone> t;
    for (int h = 1; h <= harmonics; ++h) t.push_back({bpf * h, 1.0 / std::pow(h, 0.7), ph(rng)});
    for (int k = 0; k < 150; ++k) t.push_back({fr(rng), 0.03, ph(rng)});
    return t;
}

// Arrival time at mic m relative to the centroid, seconds (plane wave).
inline double arrival_s(int m, double az_deg, double el_deg, double c = uav::kSpeedOfSound) {
    const double az = az_deg * M_PI / 180, el = el_deg * M_PI / 180;
    const double ux = std::cos(el) * std::sin(az), uy = std::cos(el) * std::cos(az);
    return -(uav::kMics[m].x * ux + uav::kMics[m].y * uy) / c;
}

// n samples of the mixture for mic m, source at (az, el), white noise at snr_db.
inline std::vector<float> render(const std::vector<Tone> &tones, int m, double az, double el,
                                 std::size_t n, double fs, double snr_db, unsigned noise_seed,
                                 std::size_t start = 0) {
    const double d = arrival_s(m, az, el);
    std::vector<float> x(n);
    double p = 0;
    for (std::size_t k = 0; k < n; ++k) {
        const double t = (start + k) / fs - d;
        double v = 0;
        for (const Tone &tn : tones) v += tn.amp * std::sin(2 * M_PI * tn.f * t + tn.phase);
        x[k] = static_cast<float>(v);
        p += v * v;
    }
    if (snr_db < 200) {
        std::mt19937 rng(noise_seed);
        std::normal_distribution<double> g(0, std::sqrt(p / n) * std::pow(10, -snr_db / 20));
        for (auto &v : x) v += static_cast<float>(g(rng));
    }
    return x;
}
