#include "tinyf_uart.h"
#include "state.h"
#include "sdkconfig.h"
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "tinyf";

#define PORT    UART_NUM_1
#define RX_BUF  1024

// One frame is " <distance>, <confidence>\n", e.g. " 327, 61".
static void parse_line(char *line)
{
    char *comma = strchr(line, ',');
    if (!comma) {
        return;
    }
    *comma = '\0';
    int dist = atoi(line);
    int conf = atoi(comma + 1);
    if (dist <= 0) {
        return;
    }
    // Spec suggests discarding confidence < 5 samples; keep them but the UI
    // colours low confidence — store the raw value either way.
    state_update_tof((uint32_t)dist, (uint32_t)conf, esp_timer_get_time());

    static int dbg = 0;
    if (++dbg >= 10) {   // throttled health log
        dbg = 0;
        ESP_LOGI(TAG, "dist=%dmm conf=%d", dist, conf);
    }
}

static void tinyf_task(void *arg)
{
    uint8_t data[256];
    char line[128];
    int li = 0;
    int64_t last_diag = 0;
    uint32_t total = 0;

    while (1) {
        int n = uart_read_bytes(PORT, data, sizeof(data), pdMS_TO_TICKS(100));
        if (n > 0) {
            total += n;
            int64_t now = esp_timer_get_time();
            if (now - last_diag > 1000000) {   // ~1 Hz raw diagnostic
                last_diag = now;
                char hx[100];
                int hp = 0;
                for (int i = 0; i < n && i < 32; i++) {
                    hp += snprintf(hx + hp, sizeof(hx) - hp, "%02X ", (unsigned)data[i]);
                }
                ESP_LOGI(TAG, "rx %d bytes (total=%u): %s", n, (unsigned)total, hx);
            }
        }
        for (int i = 0; i < n; i++) {
            char c = (char)data[i];
            if (c == '\n' || c == '\r') {
                if (li > 0) {
                    line[li] = '\0';
                    parse_line(line);
                    li = 0;
                }
            } else if (li < (int)sizeof(line) - 1) {
                line[li++] = c;
            } else {
                li = 0;  // overrun, drop the line
            }
        }
    }
}

// Diagnostic: sample the RX line before claiming it for UART. A powered TinyF
// idles its TX high; data shows as a mix of high/low; an unpowered/disconnected
// line reads non-high or floating.
// Check one pin: count UART edges (data) and whether it survives an internal
// pull-down (actively driven). Logs only "interesting" pins (data or driven),
// plus 17/18. Returns true if interesting.
static bool probe_pin_report(int pin)
{
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io);
    int edges = 0, prev = -1;
    for (int i = 0; i < 2000; i++) {   // 100ms @ 50us
        int v = gpio_get_level(pin);
        if (prev >= 0 && v != prev) edges++;
        prev = v;
        esp_rom_delay_us(50);
    }
    io.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_config(&io);
    esp_rom_delay_us(2000);
    int pd = 0;
    for (int i = 0; i < 50; i++) {
        pd += gpio_get_level(pin);
        esp_rom_delay_us(50);
    }
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io);

    bool interesting = (edges > 0) || (pd > 25);
    if (interesting || pin == 17 || pin == 18) {
        const char *tag = edges > 0 ? "  <-- DATA (TXD here!)"
                        : (pd > 25 ? "  <-- driven high" : "");
        ESP_LOGI(TAG, "  GPIO%-2d: edges=%d pulldown_high=%d/50%s", pin, edges, pd, tag);
    }
    return interesting;
}

static void probe_rx_line(void)
{
    // Broad set of safe-to-read GPIOs (excludes I2C 8/9, USB 19/20,
    // flash/PSRAM 26-37). Includes the silk "TX/RX" pins 43/44.
    static const int pins[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15, 16, 17, 18, 21,
        38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48,
    };
    ESP_LOGI(TAG, "scanning GPIOs for the TinyF TXD line (edges=live data, "
                  "pulldown_high=actively driven):");
    int found = 0;
    for (int i = 0; i < (int)(sizeof(pins) / sizeof(pins[0])); i++) {
        if (probe_pin_report(pins[i])) {
            found++;
        }
    }
    ESP_LOGI(TAG, "scan done: %d interesting pin(s) (0 = TXD not reaching any scanned GPIO)", found);
}

void tinyf_start(void)
{
    probe_rx_line();

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
                                 CONFIG_SPECTRUM_TINYF_TX_GPIO,
                                 CONFIG_SPECTRUM_TINYF_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "TinyF on UART1 (RX=%d TX=%d)",
             CONFIG_SPECTRUM_TINYF_RX_GPIO, CONFIG_SPECTRUM_TINYF_TX_GPIO);
    xTaskCreate(tinyf_task, "tinyf_task", 3072, NULL, 5, NULL);
}
