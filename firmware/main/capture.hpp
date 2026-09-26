// Synchronous 3-microphone capture on two I2S controllers, one clock domain.
//
//   I2S0  MASTER  drives SCK + WS to all three mics, reads SD_A (M1 L, M2 R)
//   I2S1  SLAVE   takes SCK + WS back in, reads SD_B (M3 L, R slot empty)
//
// Both controllers latch on the same WS edge, so the streams cannot drift.
// What can differ is when each DMA starts filling: a constant whole-sample
// offset, removed downstream by uav::ChannelAligner.
#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/i2s_std.h"
#include "esp_err.h"

class ArrayCapture {
public:
    static constexpr uint32_t kSampleRate = 48000;

    esp_err_t start(std::size_t frame_len);

    // Blocks until frame_len samples per mic are in. Output is the raw I2S
    // word: 24 significant bits, MSB-aligned in an int32.
    esp_err_t read_frame(int32_t *m1, int32_t *m2, int32_t *m3);

private:
    i2s_chan_handle_t rx_a_ = nullptr;  // I2S0 master
    i2s_chan_handle_t rx_b_ = nullptr;  // I2S1 slave
    std::size_t frame_len_ = 0;
    int32_t *buf_a_ = nullptr;  // interleaved L/R, frame_len * 2
    int32_t *buf_b_ = nullptr;
};
