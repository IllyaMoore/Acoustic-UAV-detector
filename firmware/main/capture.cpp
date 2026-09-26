#include "capture.hpp"

#include <cstdlib>

#include "board_pins.hpp"
#include "driver/gpio.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "sdkconfig.h"
#include "soc/soc_caps.h"

namespace {
const char *TAG = "capture";

// 32-bit slots, stereo: 64 SCK per WS period, 3.072 MHz at 48 kHz - what the
// INMP441 wants (it needs 64 x fs). Philips framing: data starts one SCK
// after the WS edge, which is the INMP441's native format.
i2s_std_config_t std_config(int bclk, int ws, int din) {
    i2s_std_config_t cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(ArrayCapture::kSampleRate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = static_cast<gpio_num_t>(bclk),
            .ws = static_cast<gpio_num_t>(ws),
            .dout = I2S_GPIO_UNUSED,
            .din = static_cast<gpio_num_t>(din),
            .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false},
        },
    };
#if SOC_I2S_SUPPORTS_APLL
    // The audio PLL divides down to exactly 48 kHz; the default 160 MHz
    // source needs a fractional divider, i.e. SCK jitter. Only the master's
    // clock matters, the slave just follows the pins.
    cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
#endif
    return cfg;
}
}  // namespace

esp_err_t ArrayCapture::start(std::size_t frame_len) {
    frame_len_ = frame_len;
    buf_a_ = static_cast<int32_t *>(std::malloc(frame_len * 2 * sizeof(int32_t)));
    buf_b_ = static_cast<int32_t *>(std::malloc(frame_len * 2 * sizeof(int32_t)));
    ESP_RETURN_ON_FALSE(buf_a_ && buf_b_, ESP_ERR_NO_MEM, TAG, "frame buffers");

    // DMA: 8 x 256 frames = 43 ms of slack per controller before an overrun.
    auto chan_cfg = [](int port, i2s_role_t role) {
        i2s_chan_config_t c = I2S_CHANNEL_DEFAULT_CONFIG(port, role);
        c.dma_desc_num = 8;
        c.dma_frame_num = 256;
        return c;
    };

    // 1) SLAVE FIRST. In internal-loopback mode it attaches as an input to
    // the same pins the master will drive; the master's later output setup
    // leaves that input routing alone, whereas the reverse order would not.
    i2s_chan_config_t cb = chan_cfg(I2S_NUM_1, I2S_ROLE_SLAVE);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&cb, nullptr, &rx_b_), TAG, "I2S1 channel");
#if CONFIG_UAV_CLOCK_LOOPBACK_INTERNAL
    i2s_std_config_t sb = std_config(pins::kSck, pins::kWs, pins::kSdB);
#else
    i2s_std_config_t sb = std_config(pins::kSckIn, pins::kWsIn, pins::kSdB);
#endif
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx_b_, &sb), TAG, "I2S1 std mode");

    // 2) Master.
    i2s_chan_config_t ca = chan_cfg(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_RETURN_ON_ERROR(i2s_new_channel(&ca, nullptr, &rx_a_), TAG, "I2S0 channel");
    i2s_std_config_t sa = std_config(pins::kSck, pins::kWs, pins::kSdA);
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(rx_a_, &sa), TAG, "I2S0 std mode");

    // SD_B's right slot is driven by nobody. The external ~100k pull-down in
    // the schematic makes it read as 0; the internal one is a backup.
    gpio_pulldown_en(static_cast<gpio_num_t>(pins::kSdB));

    // 3) Enable the slave before the clock exists, then start the clock. The
    // slave then sees the very first WS edge, which keeps the FIFO offset
    // small - but it is still measured, never assumed.
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx_b_), TAG, "enable I2S1");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(rx_a_), TAG, "enable I2S0");

    ESP_LOGI(TAG, "I2S0 master SCK=%d WS=%d SD_A=%d | I2S1 slave SD_B=%d", pins::kSck, pins::kWs,
             pins::kSdA, pins::kSdB);
#if CONFIG_UAV_CLOCK_LOOPBACK_INTERNAL
    ESP_LOGI(TAG, "clock loopback: internal (GPIO matrix)");
#else
    ESP_LOGI(TAG, "clock loopback: external - jumper GPIO%d->GPIO%d and GPIO%d->GPIO%d", pins::kSck,
             pins::kSckIn, pins::kWs, pins::kWsIn);
#endif
    return ESP_OK;
}

esp_err_t ArrayCapture::read_frame(int32_t *m1, int32_t *m2, int32_t *m3) {
    const std::size_t bytes = frame_len_ * 2 * sizeof(int32_t);
    std::size_t got = 0;
    // Same clock, same rate: after the first frame both reads complete within
    // microseconds of each other, so reading them back to back never starves
    // either DMA.
    ESP_RETURN_ON_ERROR(i2s_channel_read(rx_a_, buf_a_, bytes, &got, portMAX_DELAY), TAG, "read SD_A");
    ESP_RETURN_ON_ERROR(i2s_channel_read(rx_b_, buf_b_, bytes, &got, portMAX_DELAY), TAG, "read SD_B");
#if CONFIG_UAV_SWAP_SD_A_SLOTS
    constexpr int kM1 = 1, kM2 = 0;
#else
    constexpr int kM1 = 0, kM2 = 1;
#endif
    for (std::size_t k = 0; k < frame_len_; ++k) {
        m1[k] = buf_a_[2 * k + kM1];
        m2[k] = buf_a_[2 * k + kM2];
        m3[k] = buf_b_[2 * k];  // left slot; right is the pulled-down empty slot
    }
    return ESP_OK;
}
