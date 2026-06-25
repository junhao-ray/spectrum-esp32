#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Start the USB UVC host. No-op unless CONFIG_SPECTRUM_ENABLE_CAMERA is set.
void camera_start(void);

// Get a pointer to the most recent MJPEG frame. Returns false if no frame is
// available (or the camera is disabled). The pointer is valid until the next
// call; the MJPEG stream handler sends it immediately.
bool camera_get_jpeg(const uint8_t **data, size_t *len);
