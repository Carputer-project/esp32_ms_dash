#include "can_tx.h"
#include "esp_log.h"
#include "espnow_link.h"
#include <stdio.h>

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

/* NOTE: no bool parameter, deliberately. These used to take one and mapped
 * false -> "I 0", which looks like "turn auto off" but is NOT: the box parses
 * `I 0` as MANUAL 0% DUTY, i.e. park the idle-air valve at its closed stop.
 * The box's `I` command has exactly three forms - `I F` follow the ECU,
 * `I A` follow the coolant curve, `I <0-100>` manual duty - and there is no
 * "neither" state to select.
 *
 * So a false here was never a valid request, it was a way to shut the valve
 * shut by accident. Both call sites passed a literal true, so nothing was
 * broken; this removes the trap rather than documenting it. The UI has three
 * separate buttons (auto / follow / manual) and none of them is a toggle, so
 * nothing needs the "off" case.
 *
 * Contrast can_tx_fan_manual(), which DOES take a bool: `F 0` is a genuine
 * fan-off button and the fan has a real off state. */
esp_err_t can_tx_iac_auto(void) {
    return can_tx_send_cmd('I', "A");
}

esp_err_t can_tx_iac_follow(void) {
    return can_tx_send_cmd('I', "F");
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

esp_err_t can_tx_gas_damp(uint8_t damp) {
    char val[16];
    snprintf(val, sizeof(val), "D %d", damp);
    return can_tx_send_cmd('Q', val);
}

esp_err_t can_tx_low_fuel_pct(uint8_t pct) {
    char val[16];
    snprintf(val, sizeof(val), "W %d", pct);
    return can_tx_send_cmd('Q', val);
}

esp_err_t can_tx_buzzer(bool on) {
    return can_tx_send_cmd('B', on ? "1" : "0");
}

esp_err_t can_tx_buzzer_test(void) {
    return can_tx_send_cmd('B', "T");
}

esp_err_t can_tx_led(uint8_t r, uint8_t g, uint8_t b) {
    char val[16];
    snprintf(val, sizeof(val), "%u %u %u", r, g, b);
    return can_tx_send_cmd('L', val);
}

esp_err_t can_tx_led_off(void) {
    return can_tx_send_cmd('L', "0");
}