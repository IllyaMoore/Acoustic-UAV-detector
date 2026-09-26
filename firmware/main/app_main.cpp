// Acoustic UAV detector - firmware entry point.
//
// Three tasks, pinned so the hard-real-time part never shares a core with the
// number crunching:
//
//   core 0  capture   (prio 10)  I2S0+I2S1 -> align -> float frame -> dsp queue
//                                                     -> int16 chunk -> SD ring
//   core 0  sd_logger (prio 3)   ring -> rec_NNNNN.wav, lines -> detections.csv
//   core 1  dsp       (prio 8)   frames -> uav::Pipeline -> $UAVDOA on UART
//
// Frames travel through a small fixed pool (no malloc in the steady state).
// If DSP ever falls behind, capture drops a whole frame and counts it; it
// never blocks the I2S DMA, which would lose samples silently instead.
#include <cmath>
#include <cstdio>
#include <cstring>

#include "capture.hpp"
#include "dsp/aligner.hpp"
#include "dsp/pipeline.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "report.hpp"
#include "sd_logger.hpp"
#include "sdkconfig.h"

namespace {
const char *TAG = "main";

constexpr std::size_t kFrameLen = 1024;  // 21.3 ms at 48 kHz
constexpr int kPoolSize = 4;

struct Frame {
    float ch[uav::kNumMics][kFrameLen];
};

Frame g_pool[kPoolSize];
QueueHandle_t g_free;    // Frame* ready to be filled
QueueHandle_t g_filled;  // Frame* ready for DSP
ArrayCapture g_capture;
SdLogger g_logger;
volatile uint32_t g_frames_dropped = 0;

#ifdef CONFIG_UAV_RECORD_WAV
constexpr bool kRecordWav = true;
constexpr int kWavSeconds = CONFIG_UAV_WAV_FILE_SECONDS;
constexpr int kWavGainBits = CONFIG_UAV_WAV_GAIN_BITS;
#else
constexpr bool kRecordWav = false;
constexpr int kWavSeconds = 60;
constexpr int kWavGainBits = 0;
#endif

#ifdef CONFIG_UAV_CALIBRATION_MODE
constexpr bool kCalibration = true;
#else
constexpr bool kCalibration = false;
#endif

uint32_t uptime_ms() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

int16_t to_int16(int32_t raw) {
    // Top 16 of the 24 significant bits, shifted up by the configured gain.
    int32_t v = raw >> (16 - kWavGainBits);
    return static_cast<int16_t>(v > 32767 ? 32767 : (v < -32768 ? -32768 : v));
}

void capture_task(void *) {
    static int32_t raw[uav::kNumMics][kFrameLen];
    static int16_t pcm[kFrameLen * uav::kNumMics];
    // The raw WAV keeps the FIFO offset (so it can be re-measured offline);
    // only the DSP path is aligned. In calibration mode nothing is aligned.
    uav::ChannelAligner aligner(kCalibration ? 0 : CONFIG_UAV_M3_LAG_SAMPLES);

    for (;;) {
        if (g_capture.read_frame(raw[0], raw[1], raw[2]) != ESP_OK) continue;

        if (kRecordWav) {
            for (std::size_t k = 0; k < kFrameLen; ++k)
                for (int m = 0; m < uav::kNumMics; ++m) pcm[k * uav::kNumMics + m] = to_int16(raw[m][k]);
            g_logger.push_audio(pcm, kFrameLen * uav::kNumMics);
        }

        Frame *f = nullptr;
        if (xQueueReceive(g_free, &f, 0) != pdTRUE) {
            g_frames_dropped = g_frames_dropped + 1;
            continue;
        }
        constexpr float kScale = 1.0f / 2147483648.0f;
        for (int m = 0; m < uav::kNumMics; ++m)
            for (std::size_t k = 0; k < kFrameLen; ++k) f->ch[m][k] = static_cast<float>(raw[m][k]) * kScale;
        float *x[uav::kNumMics] = {f->ch[0], f->ch[1], f->ch[2]};
        aligner.process(x, kFrameLen);
        xQueueSend(g_filled, &f, portMAX_DELAY);
    }
}

void set_led(bool on) {
    if (CONFIG_UAV_STATUS_LED_GPIO >= 0) gpio_set_level(static_cast<gpio_num_t>(CONFIG_UAV_STATUS_LED_GPIO), on);
}

void dsp_task(void *) {
    uav::PipelineConfig cfg;
    cfg.fs = ArrayCapture::kSampleRate;
    cfg.frame_len = kFrameLen;
    if (kCalibration) {
        cfg.f_lo = 500.0f;  // clicks are broadband; wider band, sharper peak
        cfg.f_hi = 8000.0f;
    }
    uav::Pipeline pipeline(cfg);
    ESP_LOGI(TAG, "pipeline: band %.0f-%.0f Hz (%u bins), max lag %d, block %.0f ms", cfg.f_lo, cfg.f_hi,
             static_cast<unsigned>(pipeline.n_bins()), pipeline.max_lag(),
             1000.0f * cfg.frame_len * cfg.avg_frames / cfg.fs);

    uav::BlockResult r;
    char line[160];
    int64_t busy_us = 0, window_start = esp_timer_get_time();
    for (;;) {
        Frame *f = nullptr;
        xQueueReceive(g_filled, &f, portMAX_DELAY);
        const int64_t t0 = esp_timer_get_time();
        const float *x[uav::kNumMics] = {f->ch[0], f->ch[1], f->ch[2]};
        const bool block_done = pipeline.push_frame(x, r);
        xQueueSend(g_free, &f, portMAX_DELAY);
        if (!block_done) {
            busy_us += esp_timer_get_time() - t0;
            continue;
        }

        const uint32_t now = uptime_ms();
        const std::size_t n = kCalibration
                                  ? report::format_cal(line, sizeof line, CONFIG_UAV_NODE_ID, now, r)
                                  : report::format_doa(line, sizeof line, CONFIG_UAV_NODE_ID, now, r,
                                                       CONFIG_UAV_ARRAY_HEADING_DEG);
        std::fwrite(line, 1, n, stdout);
        set_led(r.detected);

        if (r.detected) {
            std::snprintf(line, sizeof line, "%lu,%d,1,%.1f,%.1f,%.1f,%.2f,%.1f,%.2f",
                          static_cast<unsigned long>(now), CONFIG_UAV_NODE_ID, r.bearing.az_deg,
                          uav::wrap_deg(r.bearing.az_deg + CONFIG_UAV_ARRAY_HEADING_DEG),
                          std::isnan(r.bearing.el_deg) ? -1.0f : r.bearing.el_deg, r.confidence, r.snr_db,
                          r.coherence);
            g_logger.push_line(line);
        }

        busy_us += esp_timer_get_time() - t0;
        const int64_t t = esp_timer_get_time();
        if (t - window_start >= 10'000'000) {
            // Every 10 s: how much of core 1 the DSP is using, and any losses.
            ESP_LOGI(TAG, "dsp load %.1f %%, frames dropped %u, sd chunks dropped %u, free heap %u",
                     100.0 * busy_us / (t - window_start), static_cast<unsigned>(g_frames_dropped),
                     static_cast<unsigned>(g_logger.dropped_chunks()),
                     static_cast<unsigned>(esp_get_free_heap_size()));
            busy_us = 0;
            window_start = t;
        }
    }
}
}  // namespace

extern "C" void app_main(void) {
    ESP_LOGI(TAG, "acoustic UAV detector, node %d, M1 heading %d deg, M3 lag %d samples%s", CONFIG_UAV_NODE_ID,
             CONFIG_UAV_ARRAY_HEADING_DEG, CONFIG_UAV_M3_LAG_SAMPLES, kCalibration ? ", CALIBRATION MODE" : "");

    if (CONFIG_UAV_STATUS_LED_GPIO >= 0) {
        gpio_reset_pin(static_cast<gpio_num_t>(CONFIG_UAV_STATUS_LED_GPIO));
        gpio_set_direction(static_cast<gpio_num_t>(CONFIG_UAV_STATUS_LED_GPIO), GPIO_MODE_OUTPUT);
    }

    g_logger.start(ArrayCapture::kSampleRate, uav::kNumMics, kRecordWav, kWavSeconds);

    g_free = xQueueCreate(kPoolSize, sizeof(Frame *));
    g_filled = xQueueCreate(kPoolSize, sizeof(Frame *));
    for (auto &f : g_pool) {
        Frame *p = &f;
        xQueueSend(g_free, &p, 0);
    }

    ESP_ERROR_CHECK(g_capture.start(kFrameLen));

    xTaskCreatePinnedToCore(dsp_task, "dsp", 8192, nullptr, 8, nullptr, 1);
    xTaskCreatePinnedToCore(capture_task, "capture", 4096, nullptr, 10, nullptr, 0);
}
