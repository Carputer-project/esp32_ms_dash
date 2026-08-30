#include "espnow_link.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "can_rx.h"

static const char *TAG = "ESPNOW";

#define LINK_CHANNEL       1
#define LINK_AP_SSID       "ms-dash"
#define LINK_TX_MS         100   /* 10Hz outpc broadcast */
#define LINK_TX_STACK      4096
#define LINK_TX_PRIO       6
#define LINK_TX_CORE       0

#define FRAME_ECU          0xA0   /* dash -> iobox3: [A0][maskLo][maskHi][72B outpc] */
#define FRAME_STATUS       0xB0   /* iobox3 -> dash: [B0][anLatch][0][seq][warn][gas%][a1..a4 mV][iac%][fanMode][iacMode][buzzerOn][bootTestOn] */
#define FRAME_CMD          0xC0   /* dash -> iobox3: [C0][len][cmd...] */

static const uint8_t s_broadcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint32_t s_rxB0Count = 0;

static void link_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (!data || len < 1) return;
    if (data[0] == FRAME_STATUS && len >= 8) {
        s_rxB0Count++;
        can_rx_set_dash_status(data[1], data[4]);
        if (len >= 14) {
            can_rx_set_gas(data[5]);                              /* v3: gas% */
            can_rx_set_a4mv(data[12] | (data[13] << 8));          /* v2: A4 source-mV */
        }
        if (len >= 19) {                                          /* v4: mode state */
            can_rx_set_fan_mode(data[15]);
            can_rx_set_iac_mode(data[16]);
            can_rx_set_buzzer_on(data[17]);
            can_rx_set_boot_test(data[18]);
        }
    }
}

static void link_tx_task(void *arg)
{
    uint8_t frame[3 + 72];
    uint32_t last_report = 0;
    for (;;) {
        frame[0] = FRAME_ECU;
        uint16_t mask = 0;
        if (can_rx_get_link_payload(&frame[3], &mask) == ESP_OK) {
            frame[1] = (uint8_t)(mask & 0xFF);
            frame[2] = (uint8_t)(mask >> 8);
            esp_now_send(s_broadcast, frame, sizeof(frame));
        }
        vTaskDelay(pdMS_TO_TICKS(LINK_TX_MS));
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
        if (now - last_report >= 5000) {
            last_report = now;
            ESP_LOGI(TAG, "link: tx_0xA0 @10Hz, rx_0xB0=%lu", (unsigned long)s_rxB0Count);
        }
    }
}

esp_err_t espnow_link_send_cmd(char key, const char *val)
{
    char payload[64];
    int n = snprintf(payload, sizeof(payload), "%c%s", key, val ? val : "");
    if (n < 1 || n > 62) return ESP_ERR_INVALID_ARG;

    uint8_t frame[64];
    frame[0] = FRAME_CMD;
    frame[1] = (uint8_t)n;
    memcpy(&frame[2], payload, n);
    return esp_now_send(s_broadcast, frame, 2 + n);
}

uint32_t espnow_link_rx_status_count(void)
{
    return s_rxB0Count;
}

esp_err_t espnow_link_init(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t ap = {
        .ap = {
            .ssid      = LINK_AP_SSID,
            .ssid_len  = strlen(LINK_AP_SSID),
            .channel   = LINK_CHANNEL,
            .max_connection = 1,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(link_recv_cb));

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, s_broadcast, 6);
    peer.channel = LINK_CHANNEL;
    peer.ifidx = WIFI_IF_AP;
    peer.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    BaseType_t xret = xTaskCreatePinnedToCore(
        link_tx_task, "espnow_tx", LINK_TX_STACK, NULL,
        LINK_TX_PRIO, NULL, LINK_TX_CORE);
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create link TX task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ESP-NOW link up (AP %s ch%d, 0xA0 @10Hz)", LINK_AP_SSID, LINK_CHANNEL);
    return ESP_OK;
}