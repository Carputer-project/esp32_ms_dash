#include "can_rx.h"

#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"
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

static uint8_t  s_outpc[CAN_RX_GROUPS * 8];
static bool     s_seen[CAN_RX_GROUPS];
static uint8_t  s_sdb[CAN_SDB_MSGS * 8];
static bool     s_sdbSeen[CAN_SDB_MSGS];
static uint32_t s_lastRxMs = 0;
static uint32_t s_lastSdbMs = 0;

/* Dash status block - TWO concurrent writers, NEITHER holding s_data_mutex:
 *
 *   1. can_rx_task(), from the CAN RX path (msg.identifier == CAN_DASH_ID)
 *   2. can_rx_set_dash_status(), called from link_recv_cb() in the WiFi task
 *      on the 0xB0 ESP-NOW frame from iobox3
 *
 * The reader (can_rx_get_data_copy) takes s_data_mutex, which gives NO ordering
 * guarantee against either of them. So the mutex is not what makes these reads
 * safe, and anyone reasoning "it is under the lock" is wrong.
 *
 * What actually makes them safe is width: on Xtensa a uint8_t store is a single
 * atomic byte and s_lastDashMs is a naturally-aligned 32-bit store, so no value
 * can be torn or read half-updated. The residual, accepted, and harmless here,
 * is that B0 and B3 can come from different writers and be a mismatched pair -
 * both paths carry the same box's state, so the values agree.
 *
 * volatile, not a critical section, deliberately: taking s_data_mutex from the
 * WiFi callback would risk stalling the WiFi task on a 5 ms timeout, to fix a
 * hazard that width already removes. This is the same reasoning the iobox3 side
 * applies to s_dashMac. Documented rather than papered over, because the claim
 * "it is under the lock" is exactly the kind of thing that stops being true
 * during a later refactor. */
static volatile uint32_t s_lastDashMs = 0;
static volatile uint8_t  s_dashB0 = 0;
static volatile uint8_t  s_dashB3 = 0;
static volatile bool     s_dashSeen = false;

/* ---- CAN Remote Port responder (2026-09-24) ----
 * Answers the ECU's MS2/Extra CAN-poll ports request (MSG_REQ) with a MSG_RSP
 * pushing 2 bytes into outpc gpioport[1..2] (Remote Port 3 = gpioport[2]).
 * Dash buttons drive the responder byte: bit1 CLEARED = VE3 table switch,
 * bit0 CLEARED = launch active (active-low, mirrors pulled-up physical input).
 * ECU polls every 10ms; launch indicator requires fresh polls (<100ms). */
#define PROBE_MSG_RSP 2u
/* The ECU's CAN-poll ports request: 29-bit EXTENDED id, DLC3, payload
 * 07 14 E2 (-> var_off 167 = gpioport[2] = Remote Port 3). Not to be confused
 * with CAN_BASE_ID 0x5F0, which is the ECU's outpc broadcast that this same
 * firmware consumes. */
static const uint32_t kPortsPollId = 0x9990570u;
static volatile uint8_t s_remotePorts = 0x03u;   /* default: VE1 + launch released */
static uint32_t s_extFrames      = 0;
static uint32_t s_portsPollSeen  = 0;
static uint32_t s_portsRespSent  = 0;
static uint32_t s_txFailSeen     = 0;   /* twai_transmit() refused the reply */
static uint32_t s_portsPollId    = 0;      /* wire ID of the ECU's ports poll */
static uint32_t s_rspId          = 0;      /* wire ID we transmit back */
static uint32_t s_lastPortsPollMs = 0;     /* when ECU last polled Remote Port 3 */
static uint32_t s_extSeen[8];
static uint32_t s_extCnt[8];
static uint8_t  s_extSeenN = 0;

void can_rx_set_table_btn(bool on) {
    s_remotePorts = (uint8_t)((s_remotePorts & ~0x02u) | (on ? 0x00u : 0x02u));
}
void can_rx_set_launch_btn(bool armed) {
    s_remotePorts = (uint8_t)((s_remotePorts & ~0x01u) | (armed ? 0x00u : 0x01u));
}

uint8_t can_rx_probe_ports(void) {
    return s_remotePorts;
}

uint8_t can_rx_get_fan_mode(void);
uint8_t can_rx_get_iac_mode(void);
uint8_t can_rx_get_buzzer_on(void);

/* Moving-average window, one index/count pair PER channel. Shared state across
 * channels breaks slot coverage if the channel count and MA_WINDOW ever share
 * a divisor >1 (e.g. 8 channels vs. 10: gcd=2 -> half of every buffer never
 * gets written, diluting every average ~50%). Keep idx/count bundled with the
 * buffer so channel count and MA_WINDOW are independent. */
typedef struct {
    int32_t buf[MA_WINDOW];
    uint8_t idx;
    uint8_t count;
} ma_t;

static ma_t s_maRpm, s_maMap, s_maClt, s_maMat, s_maTps, s_maBatt, s_maAfr;

/* AFR/IAC have NO SDB source. When the primary outpc stream drops but the SDB
 * broadcast is still alive, holding the last known values (rather than 0) keeps
 * the AFR average from being dragged through the RICH threshold and stops
 * "IAC 0%" showing during what is a data-loss event, not a fueling event. */
static int32_t s_lastAfr = 0;
static int16_t s_lastIac = 0;

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

static int32_t maPush(ma_t *ma, int32_t val) {
    ma->buf[ma->idx] = val;
    if (ma->count < MA_WINDOW) ma->count++;
    int64_t sum = 0;
    for (int i = 0; i < ma->count; i++) sum += ma->buf[i];
    ma->idx = (uint8_t)((ma->idx + 1) % MA_WINDOW);
    return (int32_t)(sum / ma->count);
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

            if (!msg.extd && msg.identifier == CAN_DASH_ID) {
                uint8_t dlc = msg.data_length_code > 8 ? 8 : msg.data_length_code;
                if (dlc >= 1) s_dashB0 = msg.data[0];
                if (dlc >= 4) s_dashB3 = msg.data[3];
                s_dashSeen = true;
                s_lastDashMs = esp_timer_get_time() / 1000;
            }
            else if (!msg.extd && msg.identifier >= CAN_SDB_ID && msg.identifier < CAN_SDB_ID + CAN_SDB_MSGS) {
                uint8_t grp = (uint8_t)(msg.identifier - CAN_SDB_ID);
                uint8_t dlc = msg.data_length_code > 8 ? 8 : msg.data_length_code;
                if (xSemaphoreTake(s_data_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                    memcpy(&s_sdb[grp * 8], msg.data, dlc);
                    if (dlc < 8) memset(&s_sdb[grp * 8 + dlc], 0, 8 - dlc);
                    s_sdbSeen[grp] = true;
                    xSemaphoreGive(s_data_mutex);
                }
                s_lastSdbMs = esp_timer_get_time() / 1000;
            }
            else if (!msg.extd && msg.identifier >= CAN_BASE_ID && msg.identifier < CAN_BASE_ID + CAN_RX_GROUPS) {
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

            /* ---- PROBE: extended frames (ECU CAN polls + anything else) ---- */
            if (msg.extd) {
                s_extFrames++;
                bool known = false;
                for (uint8_t i = 0; i < s_extSeenN; i++) {
                    if (s_extSeen[i] == msg.identifier) { s_extCnt[i]++; known = true; break; }
                }
                if (!known && s_extSeenN < 8) {
                    s_extSeen[s_extSeenN] = msg.identifier;
                    s_extCnt[s_extSeenN] = 1;
                    printf("[CAN] EXT id=0x%X dlc=%u data=", (unsigned)msg.identifier,
                           msg.data_length_code);
                    for (uint8_t i = 0; i < msg.data_length_code; i++) printf("%02X ", msg.data[i]);
                    printf("\n");
                    s_extSeenN++;
                }

                /* MS2/Extra ports poll request: MSG_REQ, reply-to table 7, 2 bytes
                 * requested (ADC polls request 8 bytes -> excluded by mask).
                 * Only answer the Remote Port 3 poll (var_off == 167 = gpioport[2]).
                 *
                 * SENDER GUARD: extended frame 0x9990570 only.
                 *
                 * This used to read `msg.identifier == 0x5F0`, which was
                 * UNSATISFIABLE and silently killed the whole responder: the
                 * enclosing block only runs `if (msg.extd)`, and 0x5F0 is a
                 * STANDARD id, so no frame can ever match both. That made
                 * launch-arm and the VE3 sports-table switch dead on the car
                 * while the code still compiled and the counters still printed.
                 * 0x5F0 is the ECU's *outpc broadcast* id, not its poll id.
                 * The poll is the 29-bit EXTENDED 0x9990570 (DLC3 07 14 E2),
                 * verified against ms2extra-3.4.3 CanRxIsr. The bus is
                 * unauthenticated, so matching the exact poll id is also the
                 * anti-spoof check — the other polls (0x590570 ADC03, 0x690570
                 * ADC47, 0x1E10570 PWM) all share the low 16 bits 0x0570 and are
                 * separated only by the high bits, so the full id is required.
                 *
                 * If that id is ever wrong, the mismatch line below names the
                 * real one instead of leaving the responder quietly dead. */
                if (msg.data_length_code >= 3 && msg.data[0] == 7 && (msg.data[2] & 0x1F) == 2) {
                    uint32_t var_off = ((uint32_t)msg.data[1] << 3) | (msg.data[2] >> 5);
                    if (var_off == 167 && msg.identifier != kPortsPollId) {
                        /* Data says Remote Port 3 poll, id says otherwise. Once,
                         * then stay quiet — this is the only diagnostic that can
                         * tell us the id assumption is stale. */
                        static bool s_pollIdWarned = false;
                        if (!s_pollIdWarned) {
                            s_pollIdWarned = true;
                            printf("[CAN] ports poll id UNEXPECTED: got 0x%X expected 0x%X "
                                   "- responder will NOT answer until this is corrected\n",
                                   (unsigned)msg.identifier, (unsigned)kPortsPollId);
                        }
                    }
                    if (var_off == 167 && msg.identifier == kPortsPollId) {  /* Remote Port 3 = gpioport[2] */
                        s_portsPollSeen++;
                        s_lastPortsPollMs = esp_timer_get_time() / 1000;
                        if (s_portsPollId == 0) s_portsPollId = msg.identifier;
                        /* Response wire ID — the CAR-VERIFIED value, restored.
                         * 2026-09-24 tested on the engine: ECU poll 0x9990570,
                         * dash reply 0x29D8070, TABLE press -> ftblsw=1 flip
                         * confirmed, launch bit0 verified, Polls=539 Resp=539
                         * (09-25), user "good it works now" (session_log.md).
                         *   (var_off<<18) | (3<<15) | (0<<11) | 0x70  = 0x029D8070
                         * An Oct-3 source re-analysis claimed this decodes in
                         * CanRxIsr as OUTMSG_REQ and is ignored, and swapped it
                         * for 0x14FA0070 -- that replacement was NEVER car-tested
                         * and contradicts the verified record. The car test is
                         * the ground truth; keep the proven reply. */
                        uint32_t rsp_id = (var_off & 0x7FFu) << 18;
                        rsp_id |= 3u << 15;   /* msg_type = 3 (MSG_RSP) */
                        rsp_id |= 0u << 11;   /* From = 0 */
                        rsp_id |= 0x70u;      /* var_blk = 0x70 */
                        twai_message_t rsp = {0};
                        rsp.identifier = rsp_id;
                        rsp.extd = true;
                        rsp.data_length_code = 2;
                        rsp.data[0] = 0x00;                   /* gpioport[1] unused */
                        rsp.data[1] = (uint8_t)s_remotePorts; /* gpioport[2] = Remote Port3 */
                        if (twai_transmit(&rsp, pdMS_TO_TICKS(10)) == ESP_OK) {
                            s_portsRespSent++;
                            s_rspId = rsp_id;
                        } else {
                            /* Silent failure is the other half of the problem:
                             * the responder could be firing every 10 ms and
                             * transmitting nothing, and Resp= would just stay
                             * at 0 looking identical to "never polled". */
                            s_txFailSeen++;
                        }
                    }
                }
            }
        }

        int64_t now_ms = esp_timer_get_time() / 1000;

        if (now_ms - s_stats.last_sec_time_ms >= 2000) {
            uint32_t fps = s_stats.total_frames - s_stats.last_sec_frame_count;
            s_stats.frames_per_sec = fps;
            s_stats.last_sec_frame_count = s_stats.total_frames;
            s_stats.last_sec_time_ms = now_ms;
            printf("[CAN] Stats: FPS=%" PRIu32 " Total=%" PRIu32 " Err=%" PRIu32 " BusOff=%" PRIu32
                   " Ext=%" PRIu32 " Polls=%" PRIu32 " Resp=%" PRIu32 " txfail=%" PRIu32 " Spd=%" PRIu32 "\n",
                     fps, s_stats.total_frames,
                     s_stats.error_frames, s_stats.bus_off_count,
                     s_extFrames, s_portsPollSeen, s_portsRespSent, s_txFailSeen,
                      (uint32_t)can_rx_get_speed());
            /* Diagnostics only - the wire format above is unchanged.
             *
             * The counters alone cannot separate the three states this can be
             * in, and that is why it stayed broken across three rounds:
             *   Ext>0, Polls=0  -> the ECU is on the bus but is NOT asking for
             *                       the remote ports. Not a dash problem.
             *   Polls>0, Resp=0 -> twai_transmit() is failing. Transceiver or
             *                       bus. txfail says so explicitly.
             *   Polls>0, Resp>0 -> the dash is answering correctly and the bits
             *                       are not landing. Then it is the ECU side.
             * Report the first two once; the third is already obvious. */
            static bool s_noPollWarned = false, s_txFailWarned = false, s_pollOkWarned = false;
            if (s_extFrames > 20 && s_portsPollSeen == 0 && !s_noPollWarned) {
                s_noPollWarned = true;
                printf("[CAN] DIAG: %" PRIu32 " extended frames, ZERO ports polls. The ECU is "
                       "transmitting but never requests the remote ports, so the dash "
                       "cannot answer. Not a dash-side fault.\n", s_extFrames);
            }
            if (s_txFailSeen && !s_txFailWarned) {
                s_txFailWarned = true;
                printf("[CAN] DIAG: %" PRIu32 " ports replies FAILED to transmit (no ACK / "
                       "bus error). The responder is firing but nothing is reaching the "
                       "wire - transceiver, its enable, or termination.\n", s_txFailSeen);
            }
            if (s_portsPollSeen > 0 && !s_pollOkWarned) {
                s_pollOkWarned = true;
                printf("[CAN] DIAG: ports poll live, replying id=0x%X var_off=167 DLC=2 "
                       "(steady 00 03).\n", (unsigned)s_rspId);
            }
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
            /* sdbFresh is a TIME gate only: it says an SDB frame arrived within
             * 1 s, not WHICH groups arrived. The fallback below reads fixed
             * offsets in s_sdb[], so a partial SDB stream was treated as a
             * complete one - an ECU that broadcasts only group 0 would have had
             * MAT (group 1) and BATT (group 3) read out of groups that never
             * existed, i.e. zeroed memory presented as a live reading. Group
             * offsets in s_sdb[]: 0 -> rpm/map/clt/tps, 1 -> mat, 3 -> batt.
             *
             * s_sdbSeen[] has been written on every SDB frame since it was
             * added and read by nothing, which is exactly the question this
             * needs answered. A missing group now yields 0, which the UI already
             * renders as "no data" - the same value the !sdbFresh branch gives.
             *
             * AFR and IAC step have no SDB equivalent and are already held in
             * s_lastAfr/s_lastIac, updated only while the outpc path is fresh. */
            const bool sdb0 = sdbFresh && s_sdbSeen[0];
            const bool sdb1 = sdbFresh && s_sdbSeen[1];
            const bool sdb3 = sdbFresh && s_sdbSeen[3];
            rawRpm  = fresh ? rdU16(6)  : (sdb0 ? rdU16be(&s_sdb[2], 0)  : 0);
            rawMap  = fresh ? rdS16(18) : (sdb0 ? rdS16be(&s_sdb[0], 0)  : 0);
            rawClt  = fresh ? rdS16(22) : (sdb0 ? rdS16be(&s_sdb[4], 0)  : 0);
            rawMat  = fresh ? rdS16(20) : (sdb1 ? rdS16be(&s_sdb[12], 0) : 0);
            rawTps  = fresh ? rdS16(24) : (sdb0 ? rdS16be(&s_sdb[6], 0)  : 0);
            rawBatt = fresh ? rdS16(26) : (sdb3 ? rdS16be(&s_sdb[24], 0) : 0);
            if (fresh) {
                s_lastAfr = rdS16(28);
                s_lastIac = rdS16(54);
            }
            rawAfr  = s_lastAfr;
            rawIac  = s_lastIac;
            rawBaro = fresh ? rdS16(16) : 1000;

            /* Spark table + rev-lim from gp10 status bytes, live timing from
             * gp01 (adv_deg @8). status1 bit6 stblsw = 3rd spark table active
             * (T3); status3 bit5 REVLIMSFT = soft/hard rev limiter engaged. */
            /* Table select + rev limiter. Offsets verified against
             * megasquirt2.ini [OutputChannels] lines 5383-5387:
             *   status1  = U08, 78   status3 = U08, 80   looptime = U16, 82
             * and [Indicator] lines 5120/5121/5135:
             *   { status1 & 32 } = "Fuel Tbl sw"  (ftblsw, VE select)
             *   { status1 & 64 } = "Spk Tbl sw"   (stblsw)
             *   { status3 & 32 } = "No soft limit"/"Soft limiter"
             * Broadcast group N covers bytes N*8..N*8+7, so status1@78 is in
             * group 9 and status3@80 is in group 10 -- they are NOT the same
             * group and must be gated separately. Previously all three reads
             * used byte 80/82 gated on group 10, which meant the VE indicator
             * was driven by the rev limiter and revLimOn by the loop timer. */
            bool ftSeen = s_seen[9];    /* status1 @78 -> group 9  (bytes 72-79) */
            bool stSeen = s_seen[10];   /* status3 @80 -> group 10 (bytes 80-87) */
            s_data.fuelTable = (fresh && ftSeen) ? ((s_outpc[78] & 0x20) ? 3 : 1) : 0;
            s_data.revLimOn  = (fresh && stSeen) && (s_outpc[80] & 0x20);
            /* Launch active only if: dash armed (bit0 cleared) AND ECU is polling us
             * (poll seen within 100ms). If polls stop, ECU holds last value but we
             * don't know it -> show inactive for safety. */
            bool portsFresh = (now_ms - s_lastPortsPollMs) < 100;
            s_data.launchActive = portsFresh && ((s_remotePorts & 0x01) == 0);
            s_data.probePolls = s_portsPollSeen;
            s_data.probeResp  = s_portsRespSent;
            s_data.probeReqId = s_portsPollId;
            s_data.probeRspId = s_rspId;

            /* Print status1 bit transitions: bit5 ftblsw (fuel table switch, the
             * TRUTH for VE) and bit6 stblsw (spark). status1 is byte 78. */
            static uint8_t s_lastSt1 = 0xFF;
            if (fresh && ftSeen && s_outpc[78] != s_lastSt1) {
                s_lastSt1 = s_outpc[78];
                printf("[CAN] status1=0x%02X ftblsw=%d stblsw=%d\n", s_outpc[78],
                       (s_outpc[78] & 0x20) ? 1 : 0, (s_outpc[78] & 0x40) ? 1 : 0);
            }

            bool advSeen = s_seen[1];
            s_data.sparkAdv10 = (fresh && advSeen) ? rdS16(8) : INT32_MIN;   /* INT32_MIN = stale */

            /* gp17 [0-1] = outpc.boost_targ, S16 kPa x0.1 (absolute). Firmware only
             * writes it inside the closed-loop branch -> reads 0 while tune is
             * open-loop, so the marker stays off until the user flips to CL. */
            bool tgSeen = s_seen[17];
            s_data.bstTargKpa = (fresh && tgSeen) ? (rdS16(136) / 10) : 0;
            /* SYNC LOSS reason: NOT decoded. gp43 (bytes 344-347) is outside
             * CAN_RX_GROUPS=18 (max byte 143). Decoding needs CAN_RX_GROUPS >= 44
             * (~208 more bytes of RX buffer). Field kept for struct compatibility;
             * UI label stays hidden. */
            s_data.syncLossReason = 0;

            s_data.canOk = fresh || sdbFresh;
            s_data.sdbOk = sdbFresh;
            s_data.rpm    = (uint16_t)maPush(&s_maRpm, rawRpm);
            s_data.map    = (int16_t)maPush(&s_maMap, rawMap);
            s_data.clt    = (int16_t)maPush(&s_maClt, rawClt);
            s_data.mat    = (int16_t)maPush(&s_maMat, rawMat);
            s_data.tps    = (int16_t)maPush(&s_maTps, rawTps);
            s_data.batt   = (int16_t)maPush(&s_maBatt, rawBatt);
            s_data.afr    = (int16_t)maPush(&s_maAfr, rawAfr);
            s_data.iacstep = rawIac;
            s_data.baro   = rawBaro;
            s_data.warnFlags = dashFresh ? s_dashB3 : 0;
            s_data.indL   = dashFresh && (s_dashB0 & 0x01);
            s_data.indR   = dashFresh && (s_dashB0 & 0x02);
            s_data.highBeam = dashFresh && (s_dashB0 & 0x04);
            s_data.ioboxOk = dashFresh;
            /* Box mode echoes: gate on link health so a dead link shows
             * "unknown" (0xFF / false) instead of the last frozen mode. */
            s_data.fanMode   = dashFresh ? can_rx_get_fan_mode()   : 0xFF;
            s_data.iacMode   = dashFresh ? can_rx_get_iac_mode()   : 0xFF;
            s_data.buzzerOn  = dashFresh && can_rx_get_buzzer_on();
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
    memset(&s_data, 0, sizeof(s_data));
    memset(&s_stats, 0, sizeof(s_stats));
    memset(s_outpc, 0, sizeof(s_outpc));
    memset(s_seen, 0, sizeof(s_seen));
    memset(s_sdb, 0, sizeof(s_sdb));
    memset(s_sdbSeen, 0, sizeof(s_sdbSeen));
    memset(&s_maRpm, 0, sizeof(s_maRpm));
    memset(&s_maMap, 0, sizeof(s_maMap));
    memset(&s_maClt, 0, sizeof(s_maClt));
    memset(&s_maMat, 0, sizeof(s_maMat));
    memset(&s_maTps, 0, sizeof(s_maTps));
    memset(&s_maBatt, 0, sizeof(s_maBatt));
    memset(&s_maAfr, 0, sizeof(s_maAfr));

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 32;
    g_config.rx_queue_len = 32;
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

    /* Boot-time CAN TX probe removed (2026-10-01).
     * It sent an unsolicited 0x7FF frame on the vehicle bus at every power-up.
     * 0x7FF is the OBD-II "functional tester present" ID, so this was an
     * unrequested broadcast from production firmware, gated by nothing, on a
     * customer's car. It existed only as a bench "is the bus wired?" check;
     * the serial-command path can_rx_selftest_tx() covers that case properly. */

    BaseType_t xret = xTaskCreatePinnedToCoreWithCaps(
        can_rx_task, "can_rx", CAN_RX_TASK_STACK, NULL,
        CAN_RX_TASK_PRIO, NULL, CAN_RX_TASK_CORE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
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
    memcpy(outpc, s_outpc, CAN_GROUPS * 8);   /* link stays 72B — iobox3 protocol unchanged */
    uint16_t m = 0;
    for (uint8_t i = 0; i < CAN_GROUPS; i++) {
        if (s_seen[i]) m |= (uint16_t)(1u << i);
    }
    *mask = m;
    xSemaphoreGive(s_data_mutex);
    return ESP_OK;
}

/* On-demand TX probe (serial command CANTX): verifies transceiver + bus wiring
 * by transmitting one frame. BENCH TOOL ONLY.
 * NOTE: the dash also transmits CAN-poll port responses (MSG_RSP) on the bus
 * when the ECU polls Remote Port 3 (var_off=167). */
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

/* Mode state from B0 v4 telemetry bytes [15..18].
 * Power-on defaults must match the box's own factory defaults (iobox3 Cfg:
 * fanAuto=true, iacFollow=true) so a v4-less/short 0xB0 frame leaves the dash
 * showing the mode the box is actually in. 0 = manual, and a stale 0 here
 * read as "IDLE MAN" until the first full frame arrived. */
static volatile uint8_t s_fanMode = 1;   /* 0=man 1=auto 2=follow */
static volatile uint8_t s_iacMode = 2;   /* 0=man 1=auto 2=follow */
static volatile uint8_t s_buzzerOn = 0;

void can_rx_set_fan_mode(uint8_t mode)    { s_fanMode = mode; }
uint8_t can_rx_get_fan_mode(void)        { return s_fanMode; }
void can_rx_set_iac_mode(uint8_t mode)    { s_iacMode = mode; }
uint8_t can_rx_get_iac_mode(void)        { return s_iacMode; }
void can_rx_set_buzzer_on(uint8_t on)     { s_buzzerOn = on; }
uint8_t can_rx_get_buzzer_on(void)       { return s_buzzerOn; }

/* IAC duty from B0 v6 telemetry byte [14]. 0xFF = no telemetry seen yet. */
static volatile uint8_t s_iacDuty = 0xFF;
void can_rx_set_iac_duty(uint8_t duty) { s_iacDuty = duty; }
uint8_t can_rx_get_iac_duty(void)       { return s_iacDuty; }

/* Speed mph from B0 v5 telemetry byte [2]. 255 = no telemetry seen yet. */
static volatile uint8_t s_speedMph = 255;
void can_rx_set_speed(uint8_t mph) { s_speedMph = mph; }
uint8_t can_rx_get_speed(void)     { return s_speedMph; }

/* Launch status from ECU CAN-poll ports response. */
bool can_rx_get_launch_active(void)
{
    return (s_remotePorts & 0x01) == 0;  /* active-low: bit0 cleared = launch active */
}