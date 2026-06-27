#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "camera_usb.h"
#include "imu_i2c.h"
#include "net.h"
#include "spectro_uart.h"
#include "state.h"
#include "tinyf_uart.h"
#include "webserver.h"

static const char *TAG = "spectrum";

void app_main(void) {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
      err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  state_init();

  // Sensors (IMU + spectrometer tasks self-delay ~5.2s past their self-test).
  imu_start();
  tinyf_start();
  spectro_start(); // no-op unless CONFIG_SPECTRUM_ENABLE_SPECTRO
  camera_start();  // no-op unless CONFIG_SPECTRUM_ENABLE_CAMERA

  // Network + web.
  net_start();
  webserver_start();

  ESP_LOGI(TAG, "spectrum firmware up");
}
