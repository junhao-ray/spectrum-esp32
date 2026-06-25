set(srcs
    "app_main.c"
    "state.c"
    "imu_i2c.c"
    "tinyf_uart.c"
    "spectro_uart.c"
    "camera_usb.c"
    "net.c"
    "webserver.c")

set(reqs
    driver
    esp_driver_i2c
    esp_driver_uart
    esp_driver_gpio
    esp_wifi
    esp_netif
    esp_event
    nvs_flash
    esp_http_server
    esp_timer
    espressif__mdns)

# The UVC camera (usb_stream) is an optional managed dependency. Only require it
# when the camera is enabled — otherwise the component need not be fetched.
if(CONFIG_SPECTRUM_ENABLE_CAMERA)
    list(APPEND reqs espressif__usb_stream)
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    EMBED_FILES "web/index.html"
    REQUIRES ${reqs})
