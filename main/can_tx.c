#include "can_tx.h"
#include "esp_log.h"
#include "espnow_link.h"

static const char *TAG = "CAN_TX";

void can_tx_init(void) {
    /* ESP-NOW link init (replaces CAN TX to iobox3) */
    ESP_LOGI(TAG, "commands now routed via ESP-NOW link");
}

esp_err_t can_tx_send_cmd(char key, const char *val) {
    esp_err_t ret = espnow_link_send_cmd(key, val);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "TX failed for %c%s: %s", key, val ? val : "", esp_err_to_name(ret));
    } else {
        ESP_LOGD(TAG, "TX: %c%s", key, val ? val : "");
    }
    return ret;
}

esp_err_t can_tx_fan_auto(bool auto_mode) {
    return can_tx_send_cmd('F', auto_mode ? "A" : "0");
}

esp_err_t can_tx_fan_manual(bool on) {
    return can_tx_send_cmd('F', on ? "1" : "0");
}

esp_err_t can_tx_iac_auto(bool auto_mode) {
    return can_tx_send_cmd('I', auto_mode ? "A" : "0");
}

esp_err_t can_tx_iac_follow(bool follow) {
    return can_tx_send_cmd('I', follow ? "F" : "0");
}

esp_err_t can_tx_iac_manual(uint8_t duty_pct) {
    char val[8];
    snprintf(val, sizeof(val), "%d", duty_pct);
    return can_tx_send_cmd('I', val);
}

esp_err_t can_tx_iac_target_rpm(int16_t rpm) {
    if (rpm < 500) return ESP_ERR_INVALID_ARG;
    char val[8];
    snprintf(val, sizeof(val), "%d", rpm);
    return can_tx_send_cmd('T', val);
}

esp_err_t can_tx_fan_output(uint8_t channel) {
    char val[8];
    snprintf(val, sizeof(val), "%d", channel);
    return can_tx_send_cmd('Y', val);
}

esp_err_t can_tx_shift_rpm(int16_t rpm) {
    if (rpm <= 0) return ESP_ERR_INVALID_ARG;
    char val[8];
    snprintf(val, sizeof(val), "%d", rpm);
    return can_tx_send_cmd('S', val);
}

esp_err_t can_tx_buzzer(bool on) {
    return can_tx_send_cmd('B', on ? "1" : "0");
}

esp_err_t can_tx_buzzer_test(void) {
    return can_tx_send_cmd('B', "T");
}

esp_err_t can_tx_boottest(bool on) {
    return can_tx_send_cmd('Z', on ? "1" : "0");
}

esp_err_t can_tx_gas_record(const char *slot) {
    return can_tx_send_cmd('Q', slot);
}

esp_err_t can_tx_led(uint8_t r, uint8_t g, uint8_t b) {
    char val[16];
    snprintf(val, sizeof(val), "%u %u %u", r, g, b);
    return can_tx_send_cmd('L', val);
}

esp_err_t can_tx_led_off(void) {
    return can_tx_send_cmd('L', "0");
}