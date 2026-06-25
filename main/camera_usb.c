#include "camera_usb.h"
#include "sdkconfig.h"

#if CONFIG_SPECTRUM_ENABLE_CAMERA

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "usb_stream.h"

static const char *TAG = "camera";

#define FRAME_W      CONFIG_SPECTRUM_CAM_FRAME_W
#define FRAME_H      CONFIG_SPECTRUM_CAM_FRAME_H
#define XFER_SZE     (32 * 1024)
#define FRAME_SZE    (FRAME_W * FRAME_H * 2)   // generous MJPEG ceiling

// Double buffer in PSRAM; the UVC callback fills the back buffer, then flips.
static uint8_t *s_buf[2];
static size_t   s_len[2];
static volatile int s_front = -1;   // index of the latest complete frame
static SemaphoreHandle_t s_mtx;

static void frame_cb(uvc_frame_t *frame, void *ptr)
{
    if (!frame || frame->data_bytes == 0) {
        return;
    }
    int back = (s_front == 0) ? 1 : 0;
    size_t n = frame->data_bytes;
    if (n > FRAME_SZE) {
        n = FRAME_SZE;
    }
    memcpy(s_buf[back], frame->data, n);
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    s_len[back] = n;
    s_front = back;
    xSemaphoreGive(s_mtx);
}

bool camera_get_jpeg(const uint8_t **data, size_t *len)
{
    if (s_front < 0) {
        return false;
    }
    xSemaphoreTake(s_mtx, portMAX_DELAY);
    int f = s_front;
    *data = s_buf[f];
    *len = s_len[f];
    xSemaphoreGive(s_mtx);
    return (*len > 0);
}

void camera_start(void)
{
    s_mtx = xSemaphoreCreateMutex();
    s_buf[0] = heap_caps_malloc(FRAME_SZE, MALLOC_CAP_SPIRAM);
    s_buf[1] = heap_caps_malloc(FRAME_SZE, MALLOC_CAP_SPIRAM);
    uint8_t *xfer_a = heap_caps_malloc(XFER_SZE, MALLOC_CAP_SPIRAM);
    uint8_t *xfer_b = heap_caps_malloc(XFER_SZE, MALLOC_CAP_SPIRAM);
    uint8_t *frame_buf = heap_caps_malloc(FRAME_SZE, MALLOC_CAP_SPIRAM);
    if (!s_buf[0] || !s_buf[1] || !xfer_a || !xfer_b || !frame_buf) {
        ESP_LOGE(TAG, "no PSRAM for camera buffers — is SPIRAM enabled?");
        return;
    }

    uvc_config_t uvc_config = {
        .frame_width = FRAME_W,
        .frame_height = FRAME_H,
        .frame_interval = FPS2INTERVAL(15),
        .xfer_buffer_size = XFER_SZE,
        .xfer_buffer_a = xfer_a,
        .xfer_buffer_b = xfer_b,
        .frame_buffer_size = FRAME_SZE,
        .frame_buffer = frame_buf,
        .frame_cb = &frame_cb,
        .frame_cb_arg = NULL,
    };

    esp_err_t err = uvc_streaming_config(&uvc_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uvc_streaming_config failed: %s", esp_err_to_name(err));
        return;
    }
    err = usb_streaming_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_streaming_start failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "USB UVC streaming %dx%d started", FRAME_W, FRAME_H);
}

#else  /* !CONFIG_SPECTRUM_ENABLE_CAMERA */

void camera_start(void) { }

bool camera_get_jpeg(const uint8_t **data, size_t *len)
{
    (void)data;
    (void)len;
    return false;
}

#endif
