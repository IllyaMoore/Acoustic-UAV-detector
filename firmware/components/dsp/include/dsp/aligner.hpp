// Removes the constant FIFO offset between the two I2S data lines.
//
// Both controllers latch on the same WS edge (one clock domain), so the SD_A
// stream (M1, M2) and the SD_B stream (M3) cannot drift - but their DMA
// buffers start filling at slightly different moments, which shows up as a
// fixed whole-sample offset. The click test measures it once
// (analysis/scripts/click_test.py) and it goes into CONFIG_UAV_M3_LAG_SAMPLES.
//
// m3_lag > 0: M3 appears that many samples LATE, so M1 and M2 are delayed to
// match. m3_lag < 0: the other way round. Header-only, no allocation after
// construction.
#pragma once

#include <cstddef>
#include <cstdlib>
#include <vector>

#include "dsp/array_geometry.hpp"

namespace uav {

class ChannelAligner {
public:
    explicit ChannelAligner(int m3_lag) {
        const std::size_t d = static_cast<std::size_t>(std::abs(m3_lag));
        for (int m = 0; m < kNumMics; ++m) {
            const bool is_m3 = (m == 2);
            delay_[m] = (m3_lag != 0 && (m3_lag > 0) != is_m3) ? d : 0;  // delay the EARLY side
            line_[m].assign(delay_[m], 0.0f);
        }
    }

    // In-place: x[m] holds n samples of mic m.
    void process(float *const x[kNumMics], std::size_t n) {
        for (int m = 0; m < kNumMics; ++m) {
            const std::size_t d = delay_[m];
            if (d == 0) continue;
            std::vector<float> &buf = line_[m];
            for (std::size_t k = 0; k < n; ++k) {
                const float in = x[m][k];
                x[m][k] = buf[pos_[m]];
                buf[pos_[m]] = in;
                pos_[m] = (pos_[m] + 1) % d;
            }
        }
    }

    std::size_t delay(int m) const { return delay_[m]; }

private:
    std::size_t delay_[kNumMics]{};
    std::size_t pos_[kNumMics]{};
    std::vector<float> line_[kNumMics];
};

}  // namespace uav
