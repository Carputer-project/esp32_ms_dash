#ifndef CAN_TX_H
#define CAN_TX_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

void can_tx_init(void);

esp_err_t can_tx_send_cmd(char key, const char *val);

esp_err_t can_tx_fan_auto(bool auto_mode);
esp_err_t can_tx_fan_manual(bool on);

esp_err_t can_tx_iac_auto(void);
esp_err_t can_tx_iac_follow(void);
esp_err_t can_tx_iac_manual(uint8_t duty_pct);
esp_err_t can_tx_iac_target_rpm(int16_t rpm);

/* NOTE: can_tx_shift_rpm ('S') and can_tx_fan_output ('Y') were deleted — 'S'
 * re-arms box O1 into shift-light OM_RPM mode (sabotages the table-switch pin)
 * and neither had callers. If shift-light control ever returns it must go
 * through a dedicated output, not the MS table-switch pin. */

/* Launch arm / table switch are CAN-only since 2026-09-25 — the dash CAN
 * responder drives ECU Remote Port3 bits; the iobox3 O-outputs (O2/O3) are
 * freed and no longer commanded (see can_rx_set_launch_btn/set_table_btn). */

esp_err_t can_tx_buzzer(bool on);
esp_err_t can_tx_buzzer_test(void);

esp_err_t can_tx_gas_damp(uint8_t damp);
esp_err_t can_tx_low_fuel_pct(uint8_t pct);

esp_err_t can_tx_led(uint8_t r, uint8_t g, uint8_t b);
esp_err_t can_tx_led_off(void);

#endif