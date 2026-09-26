// microSD logging: raw 3-channel WAV + detections.csv, in its own task.
//
// SD cards stall - a single write can take 100+ ms while the card erases a
// block. The capture task must never wait for that, so audio goes through a
// ring buffer: capture drops a chunk (and counts it) rather than block, and a
// low-priority task drains the buffer to the card at whatever pace the card
// manages.
#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"

class SdLogger {
public:
    // Mounts the card at /sd and starts the writer task. On failure the
    // logger stays disabled and every push is a cheap no-op.
    esp_err_t start(uint32_t sample_rate, int channels, bool record_wav, int file_seconds);

    bool enabled() const { return enabled_; }

    // Non-blocking. n = total int16 samples (frames * channels).
    void push_audio(const int16_t *interleaved, std::size_t n);

    // Non-blocking, one CSV line (without newline), truncated to 120 chars.
    void push_line(const char *line);

    uint32_t dropped_chunks() const { return dropped_; }

private:
    static void task_entry(void *arg);
    void run();

    bool enabled_ = false;
    bool record_wav_ = false;
    uint32_t sample_rate_ = 0;
    int channels_ = 0;
    int file_seconds_ = 60;
    void *ring_ = nullptr;   // RingbufHandle_t
    void *lines_ = nullptr;  // QueueHandle_t
    volatile uint32_t dropped_ = 0;
};
