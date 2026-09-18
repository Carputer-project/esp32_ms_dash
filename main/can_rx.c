#include "can_rx.h"

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/twai.h"

static const char *TAG = "CAN";

#define CAN_TX_GPIO             20
#define CAN_RX_GPIO             19

#define CAN_RX_TASK_STACK       4096
#define CAN_RX_TASK_PRIO        20
#define CAN_RX_TASK_CORE        0

#define MA_WINDOW 10

static dash_data_t s_data;
static can_stats_t s_stats;
static SemaphoreHandle_t s_data_mutex;
static SemaphoreHandle_t s_stats_mutex;

static uint8_t  s_outpc[CAN_GROUPS * 8];
static bool     s_seen[CAN_GROUPS];
static uint8_t  s_sdb[CAN_SDB_MSGS * 8];
static bool     s_sdbSeen[CAN_SDB_MSGS];
static uint32_t s_lastRxMs = 0;
static uint32_t s_lastSdbMs = 0;
static uint32_t s_lastDashMs = 0;
static uint8_t  s_dashB0 = 0;
static uint8_t  s_dashB3 = 0;
static bool     s_dashSeen = false;

uint8_t can_rx_get_fan_mode(void);
uint8_t can_rx_get_iac_mode(void);
uint8_t can_rx_get_buzzer_on(void);
uint8_t can_rx_get_boot_test(void);

static int32_t s_maRpm[MA_WINDOW];
static int32_t s_maMap[MA_WINDOW];
static int32_t s_maClt[MA_WINDOW];
static int32_t s_maMat[MA_WINDOW];
static int32_t s_maTps[MA_WINDOW];
static int32_t s_maBatt[MA_WINDOW];
static int32_t s_maAfr[MA_WINDOW];
static uint8_t s_maIdx = 0;
static uint8_t s_maCount = 0;

static inline uint16_t rdU16(uint8_t off) {
    return (uint16_t)((s_outpc[off] << 8) | s_outpc[off + 1]);
}

static inline int16_t rdS16(uint8_t off) {
    return (int16_t)rdU16(off);
}

static inline uint16_t rdU16be(const uint8_t *d, uint8_t off) {
    return (uint16_t)((d[off] << 8) | d[off + 1]);
}

static inline int16_t rdS16be(const uint8_t *d, uint8_t off) {
    return (int16_t)rdU16be(d, off);
}

static int32_t maPush(int32_t *buf, int32_t val) {
    if (s_maCount >= MA_WINDOW) {
        buf[s_maIdx] = val;
    } else {
        buf[s_maIdx] = val;
        s_maCount++;
    }
    int64_t sum = 0;
    for (int i = 0; i < s_maCount; i++) sum += buf[i];
    s_maIdx = (s_maIdx + 1) % MA_WINDOW;
    return sum / s_maCount;
}

static void can_rx_task(void *arg)
{
    twai_message_t msg;
    printf("[CAN] RX task started on core %d\n", CAN_RX_TASK_CORE);

    // Print initial TWAI status
    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        printf("[CAN] Initial Status: state=%d msgs_to_tx=%" PRIu32 " msgs_to_rx=%" PRIu32 " tx_err=%" PRIu32 " rx_err=%" PRIu32 " tx_failed=%" PRIu32 "\n",
               status.state, status.msgs_to_tx, status.msgs_to_rx, status.tx_error_counter, status.rx_error_counter, status.tx_failed_count);
    }

    uint32_t status_print_counter = 0;

    for (;;) {
        if (twai_receive(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
            s_stats.total_frames++;

            if (msg.identifier == CAN_DASH_ID) {
                uint8_t dlc = msg.data_length_code > 8 ? 8 : msg.data_length_code;
                if (dlc >= 1) s_dashB0 = msg.data[0];
                if (dlc >= 4) s_dashB3 = msg.data[3];
                s_dashSeen = true;
                s_lastDashMs = esp_timer_get_time() / 1000;
                continue;
            }

            if (msg.identifier >= CAN_SDB_ID && msg.identifier < CAN_SDB_ID + CAN_SDB_MSGS) {
                uint8_t grp = (uint8_t)(msg.identifier - CAN_SDB_ID);
                uint8_t dlc = msg.data_length_code > 8 ? 8 : msg.data_length_code;
                if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    memcpy(&s_sdb[grp * 8], msg.data, dlc);
                    if (dlc < 8) memset(&s_sdb[grp * 8 + dlc], 0, 8 - dlc);
                    s_sdbSeen[grp] = true;
                    xSemaphoreGive(s_data_mutex);
                }
                s_lastSdbMs = esp_timer_get_time() / 1000;
                continue;
            }

            if (msg.identifier >= CAN_BASE_ID && msg.identifier < CAN_BASE_ID + CAN_GROUPS) {
                uint8_t grp = (uint8_t)(msg.identifier - CAN_BASE_ID);
                uint8_t dlc = msg.data_length_code > 8 ? 8 : msg.data_length_code;
                if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    memcpy(&s_outpc[grp * 8], msg.data, dlc);
                    if (dlc < 8) memset(&s_outpc[grp * 8 + dlc], 0, 8 - dlc);
                    s_seen[grp] = true;
                    xSemaphoreGive(s_data_mutex);
                }
                s_lastRxMs = esp_timer_get_time() / 1000;

            }
        }

        int64_t now_ms = esp_timer_get_time() / 1000;

        if (now_ms - s_stats.last_sec_time_ms >= 2000) {
            uint32_t fps = s_stats.total_frames - s_stats.last_sec_frame_count;
            s_stats.frames_per_sec = fps;
            s_stats.last_sec_frame_count = s_stats.total_frames;
            s_stats.last_sec_time_ms = now_ms;
            printf("[CAN] Stats: FPS=%" PRIu32 " Total=%" PRIu32 " Err=%" PRIu32 " BusOff=%" PRIu32 "\n",
                     fps, s_stats.total_frames,
                     s_stats.error_frames, s_stats.bus_off_count);
        }

        // Print TWAI status every 10 seconds
        status_print_counter++;
        if (status_print_counter >= 5000) { // 5000 * 10ms = 50 seconds
            status_print_counter = 0;
            twai_status_info_t status;
            if (twai_get_status_info(&status) == ESP_OK) {
                printf("[CAN] Status: state=%d msgs_to_tx=%" PRIu32 " msgs_to_rx=%" PRIu32 " tx_err=%" PRIu32 " rx_err=%" PRIu32 " tx_failed=%" PRIu32 "\n",
                       status.state, status.msgs_to_tx, status.msgs_to_rx, status.tx_error_counter, status.rx_error_counter, status.tx_failed_count);
            }
        }

        bool fresh = (now_ms - s_lastRxMs) < 1000;
        bool sdbFresh = (now_ms - s_lastSdbMs) < 1000;
        bool dashFresh = s_dashSeen && (now_ms - s_lastDashMs) < 1500;

        /* ECU data stale -> drop the group-seen mask so the ESP-NOW link
         * forwards an empty mask (heartbeat) instead of frozen outpc bytes.
         * iobox3 treats mask==0 as "no data" and trips its failsafe. */
        if (!fresh) {
            if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                memset(s_seen, 0, sizeof(s_seen));
                xSemaphoreGive(s_data_mutex);
            }
        }

        if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            int32_t rawRpm, rawMap, rawClt, rawMat, rawTps, rawBatt, rawAfr, rawBaro;
            int16_t rawIac;
            rawRpm  = fresh ? rdU16(6)  : (sdbFresh ? rdU16be(&s_sdb[2], 2) : 0);
            rawMap  = fresh ? rdS16(18) : (sdbFresh ? rdS16be(&s_sdb[0], 0) : 0);
            rawClt  = fresh ? rdS16(22) : (sdbFresh ? rdS16be(&s_sdb[4], 4) : 0);
            rawMat  = fresh ? rdS16(20) : (sdbFresh ? rdS16be(&s_sdb[8], 4) : 0);
            rawTps  = fresh ? rdS16(24) : (sdbFresh ? rdS16be(&s_sdb[6], 6) : 0);
            rawBatt = fresh ? rdS16(26) : (sdbFresh ? rdS16be(&s_sdb[22], 6) : 0);
            rawAfr  = fresh ? rdS16(28) : 0;
            rawIac  = fresh ? rdS16(54) : 0;
            rawBaro = fresh ? rdS16(16) : 1000;

            s_data.canOk = fresh || sdbFresh;
            s_data.sdbOk = sdbFresh;
            s_data.rpm    = (uint16_t)maPush(s_maRpm, rawRpm);
            s_data.map    = (int16_t)maPush(s_maMap, rawMap);
            s_data.clt    = (int16_t)maPush(s_maClt, rawClt);
            s_data.mat    = (int16_t)maPush(s_maMat, rawMat);
            s_data.tps    = (int16_t)maPush(s_maTps, rawTps);
            s_data.batt   = (int16_t)maPush(s_maBatt, rawBatt);
            s_data.afr    = (int16_t)maPush(s_maAfr, rawAfr);
            s_data.iacstep = rawIac;
            s_data.baro   = rawBaro;
            s_data.warnFlags = dashFresh ? s_dashB3 : 0;
            s_data.indL   = dashFresh && (s_dashB0 & 0x01);
            s_data.indR   = dashFresh && (s_dashB0 & 0x02);
            s_data.highBeam = dashFresh && (s_dashB0 & 0x04);
            s_data.ioboxOk = dashFresh;
            s_data.fanMode   = can_rx_get_fan_mode();
            s_data.iacMode   = can_rx_get_iac_mode();
            s_data.buzzerOn  = can_rx_get_buzzer_on();
            s_data.bootTestOn = can_rx_get_boot_test();
            s_data.speedMph  = can_rx_get_speed();
            s_data.lastRxMs = s_lastRxMs;
            s_data.lastSdbMs = s_lastSdbMs;

            xSemaphoreGive(s_data_mutex);
        }
    }
}

esp_err_t can_rx_init(void)
{
    s_data_mutex = xSemaphoreCreateMutex();
    if (!s_data_mutex) {
        ESP_LOGE(TAG, "Failed to create data mutex");
        return ESP_FAIL;
    }
    s_stats_mutex = xSemaphoreCreateMutex();
    if (!s_stats_mutex) {
        ESP_LOGE(TAG, "Failed to create stats mutex");
        return ESP_FAIL;
    }
    memset(&s_data, 0, sizeof(s_data));
    memset(&s_stats, 0, sizeof(s_stats));
    memset(s_outpc, 0, sizeof(s_outpc));
    memset(s_seen, 0, sizeof(s_seen));
    memset(s_sdb, 0, sizeof(s_sdb));
    memset(s_sdbSeen, 0, sizeof(s_sdbSeen));
    memset(s_maRpm, 0, sizeof(s_maRpm));
    memset(s_maMap, 0, sizeof(s_maMap));
    memset(s_maClt, 0, sizeof(s_maClt));
    memset(s_maMat, 0, sizeof(s_maMat));
    memset(s_maTps, 0, sizeof(s_maTps));
    memset(s_maBatt, 0, sizeof(s_maBatt));
    memset(s_maAfr, 0, sizeof(s_maAfr));
    s_maIdx = 0;
    s_maCount = 0;

    const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    const twai_filter_config_t f_config = {
        .acceptance_code = 0,
        .acceptance_mask = 0xFFFFFFFF,
        .single_filter = false
    };

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(TAG, "TWAI driver installed");

    ESP_ERROR_CHECK(twai_start());
    printf("[CAN] TWAI 500k NORMAL on GPIO%d/TX GPIO%d/RX\n", CAN_TX_GPIO, CAN_RX_GPIO);

    twai_status_info_t status;
    twai_get_status_info(&status);
    printf("[CAN] Status: state=%d msgs_to_tx=%" PRIu32 " msgs_to_rx=%" PRIu32 " tx_err=%" PRIu32 " rx_err=%" PRIu32 " tx_failed=%" PRIu32 "\n",
           status.state, status.msgs_to_tx, status.msgs_to_rx, status.tx_error_counter, status.rx_error_counter, status.tx_failed_count);

    /* TEMP RESTORE (A/B test): boot-time TX probe reinstated to check whether
     * its presence affects RX. Original gating rationale unchanged otherwise. */
    twai_message_t test = {0};
    test.identifier = 0x7FF;
    test.data_length_code = 2;
    test.data[0] = 0xAA;
    test.data[1] = 0x55;
    if (twai_transmit(&test, pdMS_TO_TICKS(100)) == ESP_OK) {
        printf("[CAN] Self-test TX OK (check bus wiring if no RX)\n");
    } else {
        printf("[CAN] Self-test TX FAILED (bus off? no termination?)\n");
    }

    BaseType_t xret = xTaskCreatePinnedToCore(
        can_rx_task, "can_rx", CAN_RX_TASK_STACK, NULL,
        CAN_RX_TASK_PRIO, NULL, CAN_RX_TASK_CORE);
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CAN RX task");
        return ESP_FAIL;
    }

    printf("[CAN] RX task created on core %d\n", CAN_RX_TASK_CORE);

    return ESP_OK;
}

const can_stats_t *can_rx_get_stats(void)
{
    return &s_stats;
}

const dash_data_t *can_rx_get_data(void)
{
    return &s_data;
}

/* Mutex-protected snapshot for cross-task consumers (UI loop).
 * can_rx_task mutates s_data continuously; reading the live struct
 * without the lock risks torn/inconsistent field values. */
void can_rx_get_data_copy(dash_data_t *out)
{
    if (!out) return;
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        *out = s_data;
        xSemaphoreGive(s_data_mutex);
    }
}

/* Copy current outpc buffer + group-seen mask for the ESP-NOW link. */
esp_err_t can_rx_get_link_payload(uint8_t outpc[72], uint16_t *mask)
{
    if (!outpc || !mask) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    memcpy(outpc, s_outpc, sizeof(s_outpc));
    uint16_t m = 0;
    for (uint8_t i = 0; i < CAN_GROUPS; i++) {
        if (s_seen[i]) m |= (uint16_t)(1u << i);
    }
    *mask = m;
    xSemaphoreGive(s_data_mutex);
    return ESP_OK;
}

/* On-demand TX probe (serial command CANTX): verifies transceiver + bus wiring
 * by transmitting one frame. BENCH TOOL ONLY — never fires on its own, so the
 * car bus sees no dash transmissions at boot or any other time. */
void can_rx_selftest_tx(void)
{
    twai_message_t test = {0};
    test.identifier = 0x7FF;
    test.data_length_code = 2;
    test.data[0] = 0xAA;
    test.data[1] = 0x55;
    if (twai_transmit(&test, pdMS_TO_TICKS(100)) == ESP_OK) {
        printf("[CAN] Self-test TX OK (check bus wiring if no RX)\n");
    } else {
        printf("[CAN] Self-test TX FAILED (bus off? no termination?)\n");
    }
}

/* Set dash indicator/warn state from iobox3 (0xB0 frame). */
void can_rx_set_dash_status(uint8_t b0, uint8_t b3)
{
    s_dashB0 = b0;
    s_dashB3 = b3;
    s_dashSeen = true;
    s_lastDashMs = esp_timer_get_time() / 1000;
}

/* Gas % from B0 v3 telemetry byte [5] — single-byte write, same benign
 * atomicity class as s_dashB0. 255 = no telemetry seen yet. */
static volatile uint8_t s_gasPct = 255;
void can_rx_set_gas(uint8_t pct) { s_gasPct = pct; }
uint8_t can_rx_get_gas(void)     { return s_gasPct; }

/* Raw A4 source-mV from B0 v2 telemetry bytes [12..13]. 0xFFFF = none yet. */
static volatile uint16_t s_a4mv = 0xFFFF;
void can_rx_set_a4mv(uint16_t mv) { s_a4mv = mv; }
uint16_t can_rx_get_a4mv(void)    { return s_a4mv; }

/* Mode state from B0 v4 telemetry bytes [15..18] */
static volatile uint8_t s_fanMode = 0;
static volatile uint8_t s_iacMode = 0;
static volatile uint8_t s_buzzerOn = 0;
static volatile uint8_t s_bootTestOn = 0;

void can_rx_set_fan_mode(uint8_t mode)    { s_fanMode = mode; }
uint8_t can_rx_get_fan_mode(void)        { return s_fanMode; }
void can_rx_set_iac_mode(uint8_t mode)    { s_iacMode = mode; }
uint8_t can_rx_get_iac_mode(void)        { return s_iacMode; }
void can_rx_set_buzzer_on(uint8_t on)     { s_buzzerOn = on; }
uint8_t can_rx_get_buzzer_on(void)       { return s_buzzerOn; }
void can_rx_set_boot_test(uint8_t on)     { s_bootTestOn = on; }
uint8_t can_rx_get_boot_test(void)       { return s_bootTestOn; }

/* Speed mph from B0 v5 telemetry byte [2]. 255 = no telemetry seen yet. */
static volatile uint8_t s_speedMph = 255;
void can_rx_set_speed(uint8_t mph) { s_speedMph = mph; }
uint8_t can_rx_get_speed(void)     { return s_speedMph; }