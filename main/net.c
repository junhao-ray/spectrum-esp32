#include "net.h"
#include "sdkconfig.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "mdns.h"

static const char *TAG = "net";

static void on_wifi_event(void *arg, esp_event_base_t base,
                          int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *e = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "client " MACSTR " joined (aid=%d)", MAC2STR(e->mac), e->aid);
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t *e = (wifi_event_ap_stadisconnected_t *)data;
        ESP_LOGI(TAG, "client " MACSTR " left (aid=%d)", MAC2STR(e->mac), e->aid);
    }
}

static void start_mdns(void)
{
    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(CONFIG_SPECTRUM_MDNS_HOST));
    ESP_ERROR_CHECK(mdns_instance_name_set("SPECTRUM"));
    ESP_ERROR_CHECK(mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0));
    ESP_LOGI(TAG, "mDNS: %s.local", CONFIG_SPECTRUM_MDNS_HOST);
}

void net_start(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&ic));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &on_wifi_event, NULL, NULL));

    wifi_config_t wc = { 0 };
    strncpy((char *)wc.ap.ssid, CONFIG_SPECTRUM_WIFI_SSID, sizeof(wc.ap.ssid) - 1);
    wc.ap.ssid_len = strlen(CONFIG_SPECTRUM_WIFI_SSID);
    wc.ap.channel = 1;
    wc.ap.max_connection = 4;
    wc.ap.beacon_interval = 100;
    if (strlen(CONFIG_SPECTRUM_WIFI_PASSWORD) == 0) {
        wc.ap.authmode = WIFI_AUTH_OPEN;            // open network
    } else {
        strncpy((char *)wc.ap.password, CONFIG_SPECTRUM_WIFI_PASSWORD,
                sizeof(wc.ap.password) - 1);
        wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    start_mdns();
    ESP_LOGI(TAG, "SoftAP \"%s\" up — join it, then open http://192.168.4.1/ (or http://%s.local/)",
             CONFIG_SPECTRUM_WIFI_SSID, CONFIG_SPECTRUM_MDNS_HOST);
}
