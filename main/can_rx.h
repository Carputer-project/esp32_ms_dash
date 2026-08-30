#ifndef CAN_RX_H
#define CAN_RX_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define CAN_BASE_ID   0x5F0u
#define CAN_SDB_ID    0x5E8u
#define CAN_DASH_ID   0x710u
#define CAN_GROUPS    9u
#define CAN_SDB_MSGS  5u

typedef struct {
    bool     canOk;
    bool     sdbOk;
    bool     ioboxOk;   /* ESP-NOW link to iobox3 alive (0xB0 within 1.5s) */
    uint16_t rpm;
    int16_t  map;
    int16_t  clt;
    int16_t  mat;
    int16_t  tps;
    int16_t  batt;
    int16_t  afr;
    int16_t  iacstep;
    int16_t  baro;
    uint8_t  warnFlags;
    bool     indL;
    bool     indR;
    bool     highBeam;
    uint8_t  fanMode;      // 0=off, 1=auto, 2=manual_on (from iobox3 0xB0)
    uint8_t  iacMode;      // 0=man, 1=auto, 2=follow (from iobox3 0xB0)
    bool     buzzerOn;     // from iobox3 0xB0
    bool     bootTestOn;   // from iobox3 0xB0
    uint32_t lastRxMs;
    uint32_t lastSdbMs;
} dash_data_t;

typedef struct {
    uint32_t total_frames;
    uint32_t error_frames;
    uint32_t bus_off_count;
    uint32_t rx_queue_full;
    uint32_t frames_per_sec;
    uint32_t last_sec_frame_count;
    uint32_t last_sec_time_ms;
} can_stats_t;

esp_err_t can_rx_init(void);
const can_stats_t *can_rx_get_stats(void);
const dash_data_t *can_rx_get_data(void);
void can_rx_get_data_copy(dash_data_t *out);   /* mutex-protected snapshot */
void can_rx_selftest_tx(void);                 /* bench-only TX probe (CANTX cmd) */

/* ESP-NOW link support (dash -> iobox3). */
esp_err_t can_rx_get_link_payload(uint8_t outpc[72], uint16_t *mask);
void can_rx_set_dash_status(uint8_t b0, uint8_t b3);
void can_rx_set_gas(uint8_t pct);          /* B0 v3 telemetry byte [5] */
uint8_t can_rx_get_gas(void);              /* last reported gas %, 255 = none yet */
void can_rx_set_a4mv(uint16_t mv);         /* B0 v2 telemetry: A4 source-mV */
uint16_t can_rx_get_a4mv(void);            /* last reported A4 mV, 0xFFFF = none yet */
void can_rx_set_fan_mode(uint8_t mode);    /* B0 v4 byte [15]: 0=off, 1=auto, 2=man_on */
void can_rx_set_iac_mode(uint8_t mode);    /* B0 v4 byte [16]: 0=man, 1=auto, 2=follow */
void can_rx_set_buzzer_on(uint8_t on);     /* B0 v4 byte [17]: 0/1 */
void can_rx_set_boot_test(uint8_t on);     /* B0 v4 byte [18]: 0/1 */
uint8_t can_rx_get_fan_mode(void);         /* last box-echoed fan mode (0/1/2) */
uint8_t can_rx_get_iac_mode(void);         /* last box-echoed iac mode (0/1/2) */
uint8_t can_rx_get_buzzer_on(void);        /* last box-echoed buzzer state (0/1) */
uint8_t can_rx_get_boot_test(void);        /* last box-echoed boot-test state (0/1) */

#endif
