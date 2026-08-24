#ifndef ESPNOW_LINK_H
#define ESPNOW_LINK_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

esp_err_t espnow_link_init(void);

/* Send a command string to the iobox3 via ESP-NOW 0xC0 frame. */
esp_err_t espnow_link_send_cmd(char key, const char *val);

/* Number of 0xB0 status frames received from the iobox3. */
uint32_t espnow_link_rx_status_count(void);

#endif