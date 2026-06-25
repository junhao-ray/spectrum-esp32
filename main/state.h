#pragma once
#include <stdint.h>
#include <stdbool.h>

// High-frequency block (IMU + TinyF ToF).
typedef struct {
    float roll, pitch, yaw;     // degrees
    float ax, ay, az;           // g
    float gx, gy, gz;           // rad/s
    bool  imu_valid;
    int64_t imu_ts;             // esp_timer us
    uint32_t distance_mm;
    uint32_t confidence;        // 0..62
    bool  tof_valid;
    int64_t tof_ts;             // esp_timer us
} fast_state_t;

#define SPEC_MAX_POINTS 1024

// Low-frequency block (H1 spectrometer, up to ~711 points).
typedef struct {
    uint8_t  exposure_status;   // 0 ok / 1 over / 2 under
    uint32_t exposure_us;
    float    cct, lux, ra, x, y, peak_nm, duv;  // convenience scalars
    float    photometric[47];
    float    nir[3];
    float    plant[16];
    float    eb;
    uint16_t wl_start, wl_end, n_points;
    uint16_t spectrum[SPEC_MAX_POINTS];         // raw counts
    bool     spec_valid;
    int64_t  spec_ts;           // esp_timer us
} spec_state_t;

void state_init(void);

void state_update_imu(float roll, float pitch, float yaw,
                      float ax, float ay, float az,
                      float gx, float gy, float gz, int64_t ts);
void state_update_tof(uint32_t distance_mm, uint32_t confidence, int64_t ts);
void state_get_fast(fast_state_t *out);

void state_set_spec(const spec_state_t *in);
void state_get_spec(spec_state_t *out);
