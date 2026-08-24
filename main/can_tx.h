#ifndef CAN_TX_H
#define CAN_TX_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

void can_tx_init(void);

esp_err_t can_tx_send_cmd(char key, const char *val);

esp_err_t can_tx_fan_auto(bool auto_mode);
esp_err_t can_tx_fan_manual(bool on);
esp_err_t can_tx_fan_temp(int16_t on_temp_f_x10);

esp_err_t can_tx_iac_auto(bool auto_mode);
esp_err_t can_tx_iac_follow(bool follow);
esp_err_t can_tx_iac_manual(uint8_t duty_pct);
esp_err_t can_tx_iac_target_rpm(int16_t rpm);

esp_err_t can_tx_fan_output(uint8_t channel);
esp_err_t can_tx_shift_rpm(int16_t rpm);

esp_err_t can_tx_buzzer(bool on);
esp_err_t can_tx_buzzer_test(void);
esp_err_t can_tx_boottest(bool on);
esp_err_t can_tx_gas_record(const char *slot);  /* 'Q' <slot> — slot: F,3,2,1,E */

esp_err_t can_tx_led(uint8_t r, uint8_t g, uint8_t b);
esp_err_t can_tx_led_off(void);

#endif