#include "state.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static fast_state_t s_fast;
static spec_state_t s_spec;
static SemaphoreHandle_t s_fast_mtx;
static SemaphoreHandle_t s_spec_mtx;

void state_init(void)
{
    s_fast_mtx = xSemaphoreCreateMutex();
    s_spec_mtx = xSemaphoreCreateMutex();
    memset(&s_fast, 0, sizeof(s_fast));
    memset(&s_spec, 0, sizeof(s_spec));
}

void state_update_imu(float roll, float pitch, float yaw,
                      float ax, float ay, float az,
                      float gx, float gy, float gz, int64_t ts)
{
    xSemaphoreTake(s_fast_mtx, portMAX_DELAY);
    s_fast.roll = roll; s_fast.pitch = pitch; s_fast.yaw = yaw;
    s_fast.ax = ax; s_fast.ay = ay; s_fast.az = az;
    s_fast.gx = gx; s_fast.gy = gy; s_fast.gz = gz;
    s_fast.imu_valid = true;
    s_fast.imu_ts = ts;
    xSemaphoreGive(s_fast_mtx);
}

void state_update_tof(uint32_t distance_mm, uint32_t confidence, int64_t ts)
{
    xSemaphoreTake(s_fast_mtx, portMAX_DELAY);
    s_fast.distance_mm = distance_mm;
    s_fast.confidence = confidence;
    s_fast.tof_valid = true;
    s_fast.tof_ts = ts;
    xSemaphoreGive(s_fast_mtx);
}

void state_get_fast(fast_state_t *out)
{
    xSemaphoreTake(s_fast_mtx, portMAX_DELAY);
    *out = s_fast;
    xSemaphoreGive(s_fast_mtx);
}

void state_set_spec(const spec_state_t *in)
{
    xSemaphoreTake(s_spec_mtx, portMAX_DELAY);
    s_spec = *in;
    xSemaphoreGive(s_spec_mtx);
}

void state_get_spec(spec_state_t *out)
{
    xSemaphoreTake(s_spec_mtx, portMAX_DELAY);
    *out = s_spec;
    xSemaphoreGive(s_spec_mtx);
}
