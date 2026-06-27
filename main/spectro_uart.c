#include "spectro_uart.h"
#include "sdkconfig.h"

#if CONFIG_SPECTRUM_ENABLE_SPECTRO

#include "state.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"

static const char *TAG = "spectro";

#define PORT       UART_NUM_2
#define RX_BUF     8192
#define MAX_FRAME  5000   // no-TM30 frame ~1706B; headroom for larger N

// Pre-computed command packets (checksums per the H1 protocol).
static const uint8_t CMD_GET_WL[]   = {0xCC, 0x01, 0x09, 0x00, 0x00, 0x0F, 0xE5, 0x0D, 0x0A};
static const uint8_t CMD_SINGLE[]   = {0xCC, 0x01, 0x09, 0x00, 0x00, 0x32, 0x08, 0x0D, 0x0A};
static const uint8_t CMD_STREAM[]   = {0xCC, 0x01, 0x0A, 0x00, 0x00, 0x41, 0x00, 0x18, 0x0D, 0x0A};
static const uint8_t CMD_AUTO_EXP[] = {0xCC, 0x01, 0x0A, 0x00, 0x00, 0x0A, 0x01, 0xE2, 0x0D, 0x0A};

static uint32_t u32le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int read_exact(uint8_t *buf, int n, int to_ms)
{
    int got = 0;
    while (got < n) {
        int r = uart_read_bytes(PORT, buf + got, n - got, pdMS_TO_TICKS(to_ms));
        if (r <= 0) {
            return got;
        }
        got += r;
    }
    return got;
}

// Sync to CC 81, read LEN, read the rest, verify checksum + terminator.
// Returns total frame length, or -1 on failure. TYPE is pkt[5].
static int read_frame(uint8_t *pkt, int sync_to_ms)
{
    uint8_t b;
    bool synced = false;
    for (int i = 0; i < RX_BUF && !synced; i++) {
        if (read_exact(&b, 1, sync_to_ms) != 1) {
            return -1;
        }
        if (b != 0xCC) {
            continue;
        }
        if (read_exact(&b, 1, 300) != 1) {
            return -1;
        }
        if (b == 0x81) {
            synced = true;
        }
    }
    if (!synced) {
        return -1;
    }
    pkt[0] = 0xCC;
    pkt[1] = 0x81;
    if (read_exact(pkt + 2, 3, 300) != 3) {
        return -1;
    }
    int len = pkt[2] | (pkt[3] << 8) | (pkt[4] << 16);
    if (len < 9 || len > MAX_FRAME) {
        ESP_LOGW(TAG, "bad LEN %d", len);
        return -1;
    }
    if (read_exact(pkt + 5, len - 5, 500) != len - 5) {
        return -1;
    }
    uint32_t sum = 0;
    for (int i = 0; i < len - 3; i++) {
        sum += pkt[i];
    }
    if ((sum & 0xFF) != pkt[len - 3]) {
        ESP_LOGW(TAG, "checksum fail");
        return -1;
    }
    if (pkt[len - 2] != 0x0D || pkt[len - 1] != 0x0A) {
        ESP_LOGW(TAG, "bad terminator");
        return -1;
    }
    return len;
}

// Read frames until one of the wanted TYPE arrives (a few attempts).
static int read_frame_type(uint8_t *pkt, uint8_t want, int sync_to_ms)
{
    for (int attempt = 0; attempt < 12; attempt++) {
        int len = read_frame(pkt, sync_to_ms);
        if (len < 0) {
            continue; // Keep trying next frames even if this one had a checksum or timeout error
        }
        if (pkt[5] == want) {
            if (want == 0x32 && len == 10 && pkt[6] == 0xFF) {
                ESP_LOGI(TAG, "received 0x32 ACK (0xFF), waiting for data...");
                continue;
            }
            return len;
        } else {
            ESP_LOGI(TAG, "ignored unexpected frame type 0x%02X (waiting for 0x%02X)", pkt[5], want);
        }
    }
    return -1;
}

// Parse a 0x32/0x33 (no-TM30) frame into spec_state.
static void parse_spectrum(const uint8_t *pkt, int len,
                           uint16_t wl_start, uint16_t wl_end)
{
    const uint8_t *d = pkt + 6;     // skip CC 81 LEN(3) TYPE
    int dlen = len - 9;             // TYPE..DATA..CHK..0D0A removed
    int p = 0;

    spec_state_t s;
    memset(&s, 0, sizeof(s));

    if (dlen < 1 + 4 + 47 * 4 + 4 + 3 * 4 + 16 * 4 + 2) {
        ESP_LOGW(TAG, "frame too short (%d), payload data: 0x%02X", dlen, d[0]);
        return;
    }

    s.exposure_status = d[p]; p += 1;
    s.exposure_us = u32le(d + p); p += 4;
    memcpy(s.photometric, d + p, 47 * 4); p += 47 * 4;
    s.eb = 0; memcpy(&s.eb, d + p, 4); p += 4;          // blue-light hazard Eb
    memcpy(s.nir, d + p, 3 * 4); p += 3 * 4;
    memcpy(s.plant, d + p, 16 * 4); p += 16 * 4;
    p += 2;                                              // int16 coefficient N (unused for raw display)

    int spec_bytes = dlen - p;
    int n = spec_bytes / 2;
    if (n > SPEC_MAX_POINTS) {
        n = SPEC_MAX_POINTS;
    }
    for (int i = 0; i < n; i++) {
        s.spectrum[i] = (uint16_t)(d[p + 2 * i] | (d[p + 2 * i + 1] << 8));
    }
    s.n_points = (uint16_t)n;
    s.wl_start = wl_start;
    s.wl_end = wl_end;

    // Convenience scalars from the 47-item photometric array.
    s.x       = s.photometric[3];
    s.y       = s.photometric[4];
    s.cct     = s.photometric[9];
    s.duv     = s.photometric[14];
    s.ra      = s.photometric[15];
    s.peak_nm = s.photometric[31];   // Lp
    s.lux     = s.photometric[38];

    s.spec_valid = true;
    s.spec_ts = esp_timer_get_time();
    state_set_spec(&s);

    // Print key photometric metrics to serial terminal for easy reading
    ESP_LOGI(TAG, "Data: CCT=%.0fK Lux=%.1f Ra=%.1f Peak=%.0fnm x,y=%.4f,%.4f pts=%u exp_stat=%u(%.1fms)",
             s.cct, s.lux, s.ra, s.peak_nm, s.x, s.y, s.n_points,
             s.exposure_status, (float)s.exposure_us / 1000.0f);
}

static void spectro_task(void *arg)
{
    vTaskDelay(pdMS_TO_TICKS(5200));

    uint8_t *pkt = heap_caps_malloc(MAX_FRAME, MALLOC_CAP_DEFAULT);
    if (!pkt) {
        ESP_LOGE(TAG, "no memory for frame buffer");
        vTaskDelete(NULL);
        return;
    }

    // 1. Ensure stream mode and consume its handshake response (0x41)
    uart_write_bytes(PORT, CMD_STREAM, sizeof(CMD_STREAM));
    int rc = read_frame_type(pkt, 0x41, 300);
    ESP_LOGI(TAG, "set stream mode response: %d", rc);

#if CONFIG_SPECTRUM_SPEC_AUTO_EXPOSURE
    // 2. Ensure auto exposure and consume its handshake response (0x0A)
    uart_write_bytes(PORT, CMD_AUTO_EXP, sizeof(CMD_AUTO_EXP));
    rc = read_frame_type(pkt, 0x0A, 300);
    ESP_LOGI(TAG, "set auto exposure response: %d", rc);
#endif

    uart_flush_input(PORT);

    // 3. Fetch wavelength range once.
    uint16_t wl_start = 340, wl_end = 1050;
    uart_write_bytes(PORT, CMD_GET_WL, sizeof(CMD_GET_WL));
    int len = read_frame_type(pkt, 0x0F, 300);
    if (len > 0) {
        wl_start = pkt[6] | (pkt[7] << 8);
        wl_end   = pkt[8] | (pkt[9] << 8);
        ESP_LOGI(TAG, "wavelength range %u..%u nm", wl_start, wl_end);
    } else {
        ESP_LOGW(TAG, "0x0F failed, assuming %u..%u nm", wl_start, wl_end);
    }

    while (1) {
        uart_flush_input(PORT);
        uart_write_bytes(PORT, CMD_SINGLE, sizeof(CMD_SINGLE));
        // Single frame capture may take time due to auto-exposure, use 1500ms timeout
        len = read_frame_type(pkt, 0x32, 1500);
        if (len > 0) {
            parse_spectrum(pkt, len, wl_start, wl_end);
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIG_SPECTRUM_SPEC_PERIOD_MS));
    }
}

void spectro_start(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(PORT, RX_BUF, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(PORT,
                                 CONFIG_SPECTRUM_SPECTRO_TX_GPIO,
                                 CONFIG_SPECTRUM_SPECTRO_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "H1 spectrometer on UART2 (RX=%d TX=%d)",
             CONFIG_SPECTRUM_SPECTRO_RX_GPIO, CONFIG_SPECTRUM_SPECTRO_TX_GPIO);
    xTaskCreate(spectro_task, "spectro_task", 6144, NULL, 5, NULL);
}

#else  /* !CONFIG_SPECTRUM_ENABLE_SPECTRO */

void spectro_start(void) { }

#endif
