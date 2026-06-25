#include "imu_i2c.h"
#include "state.h"
#include "sdkconfig.h"
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "imu";

#define IMU_ADDR    CONFIG_SPECTRUM_IMU_ADDR
#define FUNC_EULER  CONFIG_SPECTRUM_IMU_FUNC_EULER
#define FUNC_ACCEL  CONFIG_SPECTRUM_IMU_FUNC_ACCEL
#define FUNC_GYRO   CONFIG_SPECTRUM_IMU_FUNC_GYRO

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;

// Write the function-code register, then read n bytes back.
static esp_err_t read_func(uint8_t func, uint8_t *buf, size_t n)
{
    return i2c_master_transmit_receive(s_dev, &func, 1, buf, n, 100);
}

static void bus_scan(void)
{
    ESP_LOGI(TAG, "I2C bus scan (SDA=%d SCL=%d):",
             CONFIG_SPECTRUM_I2C_SDA_GPIO, CONFIG_SPECTRUM_I2C_SCL_GPIO);
    int found = 0;
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2c_master_probe(s_bus, a, 50) == ESP_OK) {
            ESP_LOGI(TAG, "  device responds @ 0x%02X", a);
            found++;
        }
    }
    if (!found) {
        ESP_LOGW(TAG, "  no I2C devices found — check wiring/pull-ups/power");
    }
}

#if CONFIG_SPECTRUM_IMU_DISCOVER
// Bring-up helper: read every function-code register and dump raw + decoded
// values so the Euler/accel/gyro registers can be identified by eye.
static void imu_discover(void)
{
    ESP_LOGI(TAG, "=== IMU register discovery on 0x%02X ===", IMU_ADDR);
    uint8_t buf[20];
    char hex[3 * 20 + 1];
    for (int func = 0; func < 0x80; func++) {
        memset(buf, 0, sizeof(buf));
        if (read_func((uint8_t)func, buf, sizeof(buf)) != ESP_OK) {
            continue;
        }
        bool all0 = true, allf = true;
        for (int i = 0; i < (int)sizeof(buf); i++) {
            if (buf[i] != 0x00) all0 = false;
            if (buf[i] != 0xFF) allf = false;
        }
        if (all0 || allf) {
            continue;
        }
        int hp = 0;
        for (int i = 0; i < (int)sizeof(buf); i++) {
            hp += snprintf(hex + hp, sizeof(hex) - hp, "%02X ", (unsigned)buf[i]);
        }
        float f3[3];   memcpy(f3, buf, 12);
        float q4[4];   memcpy(q4, buf, 16);
        int16_t s8[8]; memcpy(s8, buf, 16);
        float qn = q4[0]*q4[0] + q4[1]*q4[1] + q4[2]*q4[2] + q4[3]*q4[3];
        ESP_LOGI(TAG, "F=0x%02X | %s| 3f=[% .4f % .4f % .4f] |q4|^2=%.3f i16=[%d %d %d %d %d %d %d %d]",
                 (unsigned)func, hex, f3[0], f3[1], f3[2], qn,
                 s8[0], s8[1], s8[2], s8[3], s8[4], s8[5], s8[6], s8[7]);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    ESP_LOGI(TAG, "=== discovery done ===");
}
#endif

static void imu_task(void *arg)
{
    // Module runs ~5 s of self-test after power-on; skip it.
    vTaskDelay(pdMS_TO_TICKS(5200));
    bus_scan();
#if CONFIG_SPECTRUM_IMU_DISCOVER
    imu_discover();
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#endif
    ESP_LOGI(TAG, "polling IMU @ 0x%02X (euler=0x%02X accel=0x%02X gyro=0x%02X)",
             IMU_ADDR, FUNC_EULER, FUNC_ACCEL, FUNC_GYRO);

    const float r2d = 57.29578f;
    const float gyro_scale = (2000.0f / 32767.0f) * ((float)M_PI / 180.0f);
    const float accel_scale = 16.0f / 32767.0f;
    const TickType_t period = pdMS_TO_TICKS(1000 / CONFIG_SPECTRUM_IMU_RATE_HZ);
    uint8_t buf[16];
    int dbg = 0;

    while (1) {
        float roll = 0, pitch = 0, yaw = 0;
        float ax = 0, ay = 0, az = 0;
        float gx = 0, gy = 0, gz = 0;
        bool ok = true;

        if (read_func(FUNC_EULER, buf, 12) == ESP_OK) {
            float e[3];
            memcpy(e, buf, 12);
#if CONFIG_SPECTRUM_IMU_EULER_IN_RADIANS
            roll = e[0] * r2d; pitch = e[1] * r2d; yaw = e[2] * r2d;
#else
            roll = e[0]; pitch = e[1]; yaw = e[2];
#endif
        } else {
            ok = false;
        }

        if (read_func(FUNC_ACCEL, buf, 6) == ESP_OK) {
            int16_t a[3];
            memcpy(a, buf, 6);
            ax = a[0] * accel_scale; ay = a[1] * accel_scale; az = a[2] * accel_scale;
        } else {
            ok = false;
        }

        if (read_func(FUNC_GYRO, buf, 6) == ESP_OK) {
            int16_t g[3];
            memcpy(g, buf, 6);
            gx = g[0] * gyro_scale; gy = g[1] * gyro_scale; gz = g[2] * gyro_scale;
        } else {
            ok = false;
        }

        if (ok) {
            state_update_imu(roll, pitch, yaw, ax, ay, az, gx, gy, gz,
                             esp_timer_get_time());
        }
        if (++dbg >= CONFIG_SPECTRUM_IMU_RATE_HZ) {   // ~1 Hz health log
            dbg = 0;
            ESP_LOGI(TAG, "rpy=[%.1f %.1f %.1f]deg acc=[%.2f %.2f %.2f]g gyr=[%.2f %.2f %.2f] ok=%d",
                     roll, pitch, yaw, ax, ay, az, gx, gy, gz, ok);
        }
        vTaskDelay(period);
    }
}

void imu_start(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = CONFIG_SPECTRUM_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_SPECTRUM_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = IMU_ADDR,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev));

    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}
