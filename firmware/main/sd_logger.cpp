#include "sd_logger.hpp"

#include <sys/stat.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>

#include "board_pins.hpp"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "sdmmc_cmd.h"

namespace {
const char *TAG = "sd";
constexpr char kMount[] = "/sd";
constexpr std::size_t kRingBytes = 64 * 1024;  // ~220 ms of 3ch/16-bit/48 kHz
constexpr std::size_t kLineLen = 128;
constexpr int64_t kSyncEveryUs = 2'000'000;  // header rewrite + fsync period

struct Line {
    char text[kLineLen];
};

void put_u32(uint8_t *p, uint32_t v) { std::memcpy(p, &v, 4); }
void put_u16(uint8_t *p, uint16_t v) { std::memcpy(p, &v, 2); }

// Canonical 44-byte PCM header. Rewritten every couple of seconds with the
// current length, so a file cut off by a power loss is still a valid WAV
// holding everything up to the last sync.
void write_wav_header(FILE *f, uint32_t fs, int ch, uint32_t data_bytes) {
    uint8_t h[44];
    std::memcpy(h, "RIFF", 4);
    put_u32(h + 4, 36 + data_bytes);
    std::memcpy(h + 8, "WAVEfmt ", 8);
    put_u32(h + 16, 16);
    put_u16(h + 20, 1);  // PCM
    put_u16(h + 22, static_cast<uint16_t>(ch));
    put_u32(h + 24, fs);
    put_u32(h + 28, fs * ch * 2);
    put_u16(h + 32, static_cast<uint16_t>(ch * 2));
    put_u16(h + 34, 16);
    std::memcpy(h + 36, "data", 4);
    put_u32(h + 40, data_bytes);
    const long pos = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    std::fwrite(h, 1, sizeof h, f);
    if (pos > static_cast<long>(sizeof h)) std::fseek(f, pos, SEEK_SET);
}

FILE *open_next_wav(uint32_t fs, int ch) {
    static int next = 0;
    char path[48];
    struct stat st;
    for (; next < 100000; ++next) {
        std::snprintf(path, sizeof path, "%s/rec_%05d.wav", kMount, next);
        if (stat(path, &st) != 0) break;
    }
    FILE *f = std::fopen(path, "wb");
    if (f) {
        write_wav_header(f, fs, ch, 0);
        ESP_LOGI(TAG, "recording %s", path);
        ++next;
    }
    return f;
}
}  // namespace

esp_err_t SdLogger::start(uint32_t sample_rate, int channels, bool record_wav, int file_seconds) {
    sample_rate_ = sample_rate;
    channels_ = channels;
    record_wav_ = record_wav;
    file_seconds_ = file_seconds;

    spi_bus_config_t bus = {};
    bus.mosi_io_num = pins::kSdMosi;
    bus.miso_io_num = pins::kSdMiso;
    bus.sclk_io_num = pins::kSdSck;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 16 * 1024;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    // 20 MHz is the SPI-mode ceiling and fine on a PCB; on jumper wires drop
    // this if mounting fails intermittently.
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    esp_err_t err = spi_bus_initialize(static_cast<spi_host_device_t>(host.slot), &bus, SDSPI_DEFAULT_DMA);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "SPI bus init failed (%s) - running without the card", esp_err_to_name(err));
        return err;
    }
    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.gpio_cs = static_cast<gpio_num_t>(pins::kSdCs);
    slot.host_id = static_cast<spi_host_device_t>(host.slot);

    esp_vfs_fat_sdmmc_mount_config_t mount = {};
    mount.format_if_mount_failed = false;  // never wipe a card that holds recordings
    mount.max_files = 4;
    mount.allocation_unit_size = 32 * 1024;
    sdmmc_card_t *card = nullptr;
    err = esp_vfs_fat_sdspi_mount(kMount, &host, &slot, &mount, &card);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "no card (%s) - running without recording", esp_err_to_name(err));
        return err;
    }
    sdmmc_card_print_info(stdout, card);

    ring_ = xRingbufferCreate(kRingBytes, RINGBUF_TYPE_NOSPLIT);
    lines_ = xQueueCreate(16, sizeof(Line));
    if (!ring_ || !lines_) return ESP_ERR_NO_MEM;
    enabled_ = true;
    // Core 0, below capture: the writer only ever gets the leftovers.
    xTaskCreatePinnedToCore(task_entry, "sd_logger", 4096, this, 3, nullptr, 0);
    return ESP_OK;
}

void SdLogger::push_audio(const int16_t *interleaved, std::size_t n) {
    if (!enabled_ || !record_wav_) return;
    if (xRingbufferSend(static_cast<RingbufHandle_t>(ring_), interleaved, n * sizeof(int16_t), 0) != pdTRUE)
        dropped_ = dropped_ + 1;
}

void SdLogger::push_line(const char *text) {
    if (!enabled_) return;
    Line l;
    std::strncpy(l.text, text, kLineLen - 1);
    l.text[kLineLen - 1] = '\0';
    xQueueSend(static_cast<QueueHandle_t>(lines_), &l, 0);
}

void SdLogger::task_entry(void *arg) { static_cast<SdLogger *>(arg)->run(); }

void SdLogger::run() {
    auto *ring = static_cast<RingbufHandle_t>(ring_);
    auto *lines = static_cast<QueueHandle_t>(lines_);

    char csv_path[32];
    std::snprintf(csv_path, sizeof csv_path, "%s/detections.csv", kMount);
    struct stat st;
    const bool fresh = stat(csv_path, &st) != 0;
    FILE *csv = std::fopen(csv_path, "a");
    if (csv && fresh)
        std::fputs("uptime_ms,node,detected,az_rel_deg,az_true_deg,el_deg,confidence,snr_db,coherence\n", csv);

    FILE *wav = nullptr;
    uint32_t data_bytes = 0;
    const uint32_t bytes_per_file = sample_rate_ * channels_ * 2 * file_seconds_;
    int64_t last_sync = esp_timer_get_time();

    for (;;) {
        std::size_t len = 0;
        void *item = xRingbufferReceive(ring, &len, pdMS_TO_TICKS(100));
        if (item) {
            if (!wav) {
                wav = open_next_wav(sample_rate_, channels_);
                data_bytes = 0;
            }
            if (wav) {
                data_bytes += std::fwrite(item, 1, len, wav);
                if (data_bytes >= bytes_per_file) {
                    write_wav_header(wav, sample_rate_, channels_, data_bytes);
                    std::fclose(wav);
                    wav = nullptr;
                }
            }
            vRingbufferReturnItem(ring, item);
        }

        Line l;
        while (xQueueReceive(lines, &l, 0) == pdTRUE)
            if (csv) std::fprintf(csv, "%s\n", l.text);

        const int64_t now = esp_timer_get_time();
        if (now - last_sync > kSyncEveryUs) {
            last_sync = now;
            if (wav) {
                write_wav_header(wav, sample_rate_, channels_, data_bytes);
                std::fflush(wav);
                fsync(fileno(wav));
            }
            if (csv) {
                std::fflush(csv);
                fsync(fileno(csv));
            }
            if (dropped_) ESP_LOGW(TAG, "%u audio chunks dropped so far (card too slow?)",
                                   static_cast<unsigned>(dropped_));
        }
    }
}
