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
    vTaskDelay(pdMS_TO_TICKS(5200));
    uint8_t data[256];
    char line[128];
    int li = 0;
    int64_t last_diag = 0;
    int64_t last_heartbeat = 0;
    uint32_t total = 0;

    ESP_LOGI(TAG, "tinyf_task started, listening on UART%d GPIO%d",
             PORT, CONFIG_SPECTRUM_TINYF_RX_GPIO);

    while (1) {
        int n = uart_read_bytes(PORT, data, sizeof(data), pdMS_TO_TICKS(100));
        int64_t now = esp_timer_get_time();

        if (n > 0) {
            total += n;
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

        // 心跳：每 5 秒打印一次，方便判断任务是否在跑以及有无字节到来
        if (now - last_heartbeat > 5000000) {
            last_heartbeat = now;
            if (total == 0) {
                ESP_LOGW(TAG, "heartbeat: NO bytes received — check TinyF power & wiring on GPIO%d",
                         CONFIG_SPECTRUM_TINYF_RX_GPIO);
            } else {
                ESP_LOGI(TAG, "heartbeat: total=%u bytes received so far", (unsigned)total);
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

void tinyf_start(void)
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
                                 CONFIG_SPECTRUM_TINYF_TX_GPIO,
                                 CONFIG_SPECTRUM_TINYF_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(TAG, "TinyF on UART1 (RX=%d TX=%d) @ 115200",
             CONFIG_SPECTRUM_TINYF_RX_GPIO, CONFIG_SPECTRUM_TINYF_TX_GPIO);
    xTaskCreate(tinyf_task, "tinyf_task", 3072, NULL, 5, NULL);
}

