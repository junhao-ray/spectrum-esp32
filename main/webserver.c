#include "webserver.h"
#include "state.h"
#include "camera_usb.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "web";

#define MOTION_STALE_US   1000000   // 1 s
#define SPEC_STALE_US     3000000   // 3 s
#define SPEC_JSON_CAP     20000

extern const uint8_t index_html_start[] asm("_binary_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_index_html_end");

static httpd_handle_t s_server;

// ---------- JSON builders ----------

static int build_motion_json(char *buf, size_t cap)
{
    fast_state_t f;
    state_get_fast(&f);
    int64_t now = esp_timer_get_time();
    bool iv = f.imu_valid && (now - f.imu_ts < MOTION_STALE_US);
    bool tv = f.tof_valid && (now - f.tof_ts < MOTION_STALE_US);
    return snprintf(buf, cap,
        "{\"type\":\"motion\",\"t\":%lld,"
        "\"imu\":{\"valid\":%s,\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
        "\"ax\":%.3f,\"ay\":%.3f,\"az\":%.3f,\"gx\":%.3f,\"gy\":%.3f,\"gz\":%.3f},"
        "\"tof\":{\"valid\":%s,\"distance_mm\":%u,\"confidence\":%u}}",
        (long long)(now / 1000), iv ? "true" : "false",
        f.roll, f.pitch, f.yaw, f.ax, f.ay, f.az, f.gx, f.gy, f.gz,
        tv ? "true" : "false", (unsigned)f.distance_mm, (unsigned)f.confidence);
}

static int append_float_array(char *buf, int n, int cap,
                              const char *key, const float *a, int count)
{
    n += snprintf(buf + n, cap - n, ",\"%s\":[", key);
    for (int i = 0; i < count && n < cap - 16; i++) {
        n += snprintf(buf + n, cap - n, "%s%.4g", i ? "," : "", a[i]);
    }
    n += snprintf(buf + n, cap - n, "]");
    return n;
}

// Caller frees. `full` adds the photometric/nir/plant arrays for the REST API.
static char *build_spectrum_json(bool full)
{
    spec_state_t *s = malloc(sizeof(spec_state_t));
    if (!s) {
        return NULL;
    }
    state_get_spec(s);
    char *buf = malloc(SPEC_JSON_CAP);
    if (!buf) {
        free(s);
        return NULL;
    }
    int64_t now = esp_timer_get_time();
    bool v = s->spec_valid && (now - s->spec_ts < SPEC_STALE_US);
    int n = 0;
    n += snprintf(buf + n, SPEC_JSON_CAP - n,
        "{\"type\":\"spectrum\",\"t\":%lld,\"valid\":%s,"
        "\"exposure_status\":%u,\"exposure_us\":%u,"
        "\"cct\":%.1f,\"lux\":%.1f,\"ra\":%.1f,\"x\":%.4f,\"y\":%.4f,"
        "\"peak_nm\":%.0f,\"duv\":%.4f,"
        "\"wl_start\":%u,\"wl_end\":%u,\"n_points\":%u,\"spectrum\":[",
        (long long)(now / 1000), v ? "true" : "false",
        (unsigned)s->exposure_status, (unsigned)s->exposure_us,
        s->cct, s->lux, s->ra, s->x, s->y, s->peak_nm, s->duv,
        (unsigned)s->wl_start, (unsigned)s->wl_end, (unsigned)s->n_points);
    for (int i = 0; i < s->n_points && n < SPEC_JSON_CAP - 12; i++) {
        n += snprintf(buf + n, SPEC_JSON_CAP - n, "%s%u", i ? "," : "", (unsigned)s->spectrum[i]);
    }
    n += snprintf(buf + n, SPEC_JSON_CAP - n, "]");
    if (full) {
        n = append_float_array(buf, n, SPEC_JSON_CAP, "photometric", s->photometric, 47);
        n = append_float_array(buf, n, SPEC_JSON_CAP, "nir", s->nir, 3);
        n = append_float_array(buf, n, SPEC_JSON_CAP, "plant", s->plant, 16);
        n += snprintf(buf + n, SPEC_JSON_CAP - n, ",\"eb\":%.4g", s->eb);
    }
    snprintf(buf + n, SPEC_JSON_CAP - n, "}");
    free(s);
    return buf;
}

// ---------- HTTP handlers ----------

static esp_err_t root_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html_start,
                           index_html_end - index_html_start);
}

static esp_err_t api_data_get(httpd_req_t *req)
{
    char buf[512];
    int n = build_motion_json(buf, sizeof(buf));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, n);
}

static esp_err_t api_spectrum_get(httpd_req_t *req)
{
    char *json = build_spectrum_json(true);
    if (!json) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "application/json");
    esp_err_t r = httpd_resp_send(req, json, strlen(json));
    free(json);
    return r;
}

static esp_err_t stream_get(httpd_req_t *req)
{
    const uint8_t *jpg;
    size_t len;
    if (!camera_get_jpeg(&jpg, &len)) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "camera disabled");
        return ESP_OK;
    }
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=frame");
    char hdr[96];
    while (1) {
        if (!camera_get_jpeg(&jpg, &len)) {
            vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }
        int h = snprintf(hdr, sizeof(hdr),
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
            (unsigned)len);
        if (httpd_resp_send_chunk(req, hdr, h) != ESP_OK) break;
        if (httpd_resp_send_chunk(req, (const char *)jpg, len) != ESP_OK) break;
        if (httpd_resp_send_chunk(req, "\r\n", 2) != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    return ESP_OK;
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        // Handshake completed by the server; nothing else to do.
        return ESP_OK;
    }
    // Drain any inbound frame (we only push, so just discard it).
    httpd_ws_frame_t frame = { 0 };
    frame.type = HTTPD_WS_TYPE_TEXT;
    if (httpd_ws_recv_frame(req, &frame, 0) != ESP_OK) {
        return ESP_OK;
    }
    if (frame.len) {
        uint8_t *b = malloc(frame.len + 1);
        if (b) {
            frame.payload = b;
            httpd_ws_recv_frame(req, &frame, frame.len);
            free(b);
        }
    }
    return ESP_OK;
}

// ---------- WebSocket broadcast ----------

static void ws_broadcast(const char *data, size_t len)
{
    if (!s_server) {
        return;
    }
    size_t num = 0;
    int fds[CONFIG_LWIP_MAX_SOCKETS];
    num = sizeof(fds) / sizeof(fds[0]);
    if (httpd_get_client_list(s_server, &num, fds) != ESP_OK) {
        return;
    }
    httpd_ws_frame_t frame = { 0 };
    frame.type = HTTPD_WS_TYPE_TEXT;
    frame.payload = (uint8_t *)data;
    frame.len = len;
    for (size_t i = 0; i < num; i++) {
        if (httpd_ws_get_fd_info(s_server, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
            httpd_ws_send_frame_async(s_server, fds[i], &frame);
        }
    }
}

static void ws_push_task(void *arg)
{
    char mbuf[512];
    int tick = 0;
    while (1) {
        int ml = build_motion_json(mbuf, sizeof(mbuf));
        ws_broadcast(mbuf, ml);

        if ((tick % 20) == 0) {     // 1 Hz spectrum
            char *sjson = build_spectrum_json(false);
            if (sjson) {
                ws_broadcast(sjson, strlen(sjson));
                free(sjson);
            }
        }
        tick++;
        vTaskDelay(pdMS_TO_TICKS(50));   // ~20 Hz motion
    }
}

// ---------- start ----------

void webserver_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_open_sockets = 7;
    cfg.lru_purge_enable = true;
    cfg.stack_size = 8192;

    ESP_ERROR_CHECK(httpd_start(&s_server, &cfg));

    httpd_uri_t uris[] = {
        { .uri = "/",             .method = HTTP_GET, .handler = root_get },
        { .uri = "/api/data",     .method = HTTP_GET, .handler = api_data_get },
        { .uri = "/api/spectrum", .method = HTTP_GET, .handler = api_spectrum_get },
        { .uri = "/stream",       .method = HTTP_GET, .handler = stream_get },
        { .uri = "/ws",           .method = HTTP_GET, .handler = ws_handler,
          .is_websocket = true },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        ESP_ERROR_CHECK(httpd_register_uri_handler(s_server, &uris[i]));
    }

    xTaskCreate(ws_push_task, "ws_push", 6144, NULL, 4, NULL);
    ESP_LOGI(TAG, "HTTP server started on :80");
}
