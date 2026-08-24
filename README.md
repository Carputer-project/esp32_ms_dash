# esp32_ms_dash — MicroSquirt 7" Gauge Cluster

Touch dashboard for MS2/Extra (MicroSquirt V3) built on the **Waveshare
ESP32-S3-Touch-LCD-7** (800×480 RGB, ESP32-S3 N16R8, GT911 touch, TJA1051 CAN).
Native **ESP-IDF v5.5.2 + LVGL 9** — no Arduino layer.

Receives the MS2/Extra `outpc` realtime broadcast over CAN and renders a full
gauge cluster; talks to the [esp32_ms_iobox3](https://github.com/Carputer-project/esp32_ms_iobox3)
I/O box over an ESP-NOW link (actuator commands out, indicator/warn/fuel data in).

```
MicroSquirt ──CAN 500k──► TWAI RX ──► parser ──► LVGL UI (gauges)
                                 │
                                 └─► ESP-NOW 0xA0 @10Hz ──► iobox3 (fan/IAC/outputs)
       iobox3 ──ESP-NOW 0xB0 @10Hz──► indicators / warn bits / fuel %
       dash    ──ESP-NOW 0xC0──────► box commands (fan, IAC, cal, buzzer…)
```

## Features

- **Tachometer** with RPM load-up ring: three stacked zone arcs (green → amber → red)
  that fill toward the shift point as RPM climbs; star cursor with zone-tinted glow.
  All zone boundaries derive from the live NVS shift setting — changing SHIFT LIGHT
  RPM in settings reshapes ring, strip, glow and banner at once, engine off or on.
- **5-segment shift light strip** in gauge center: progressive from
  `shift − 1000 rpm`, all segments flash blue at ≥ shift point (~3 Hz).
- **Warning banner** above the strip (priority first-hit-wins): OVERREV → OVERHEAT →
  OVERBOOST → HOT AIR → LOW BATTERY → HIGH BATT → LEAN. Blinking for OVERREV /
  OVERBOOST / LEAN. Box-sourced warns arrive pre-latched with hold time;
  OVERREV + LEAN are computed dash-side (see "Warning model").
- **Digital readouts**: MAP (kPa), CLT (°F bar), MAT (°F), battery (V), AFR gauge,
  IAC %.
- **Indicators**: L/R arrows blink ~1.4 Hz, high-beam lamp steady — driven by the
  box's analog latch byte, freshness-gated at the link layer.
- **Settings page** (touch): fan mode/on-temp, IAC mode (auto/follow/manual),
  idle target RPM, shift-light RPM (NVS-persisted 4000–9000), manual IAC duty,
  warning buzzer + test beep, box boot self-test toggle, fuel-gauge calibration
  (SET E / ¼ / ½ / ¾ / F buttons + damping + low-fuel %), clock backlight LED color.

## Data sources

### CAN in (TWAI)

`can_rx.c` — 500 kbps NORMAL mode, accept-all filter, TX=GPIO20 RX=GPIO19.

| Range | Purpose |
|-------|---------|
| 0x5F0–0x5F8 | MS2/Extra `outpc`, 9 groups × 8 B = 72 B buffer |
| 0x5E8–0x5EC | SDB small-data-block (5 msgs) |
| 0x710 | legacy dash frame (listened, feeds warnFlags/latch path) |

Fields decoded (all big-endian ×10): rpm@6, baro@16, map@18, mat@20, clt@22,
tps@24, batt@26, afr@28, iacstep@54. A 20-sample moving average smooths each
channel. If ECU groups go stale (>1 s) the decoder falls back to SDB-sourced
rpm/map/clt/mat/tps/batt so gauges stay alive with ignition-off dash power.

Freshness: `canOk = fresh || sdbFresh`. When ECU data goes stale the group mask
is cleared so the ESP-NOW heartbeat carries mask=0 — iobox3 treats that as
"no engine data" and trips its failsafe instead of acting on frozen values.

### ESP-NOW link to iobox3

Fixed channel 1, no association needed. Dash hosts softAP **`ms-dash`**; box
hosts **`iobox3-ms`**. Broadcast frames, unencrypted:

| Frame | Direction | Layout |
|-------|-----------|--------|
| `0xA0` | dash → box @10 Hz | `[A0][maskLo][maskHi][72B outpc]` |
| `0xB0` | box → dash @10 Hz | v3: `[B0][anLatch][0][seq][warnLo][gasPct][a1..a4 mV LE][iacDuty%]` (15 B; receivers guard len≥8) |
| `0xC0` | dash → box | `[C0][len][cmd...]` — ASCII command, same grammar as the box serial console |

`anLatch` bits: 0=indicator L, 1=indicator R, 2=high beam, 3=gas-channel latch.
`warnLo` is the low byte of the box's latched engine-warning flags (see below).

## Warning model

| Warn | Source | Trigger | Notes |
|------|--------|---------|-------|
| OVERREV | dash-side | rpm ≥ shift setting | release −150 rpm hysteresis; shares one threshold with ring/strip/glow |
| OVERHEAT | box bit 3 | clt > profile max | latched+held box-side (default 3 s) |
| OVERBOOST | box bit 7 | map > profile max | |
| HOT AIR | box bit 4 | mat > profile max | |
| LOW BATTERY | box bit 5 | batt < profile min | |
| HIGH BATT | box bit 6 | batt > profile max | |
| LEAN | dash-side | afr > 16.0 && rpm > 2500 | release ≤ 15.4; dash-side because the B0 frame carries only the low warn byte (bits 8–9 = LEAN/RICH are truncated in transit) |

Idle warnings (bits 0/1) exist in the protocol but are deliberately not shown —
cold-start warmup would spam IDLE-HIGH every start.

## Build & flash

```bash
source ~/esp/v5.5.2/esp-idf/export.sh    # or your IDF path
idf.py build
idf.py -p /dev/ttyACM0 -b 115200 flash   # 115200 is the reliable baud on this board
idf.py monitor                            # or any serial terminal
```

Gotchas learned on this hardware:

- The S3 native USB enumerates and can drop within ~250 ms of attach — if the
  port disappears mid-flash, re-seat the cable and retry at `-b 115200`.
- Copying this project to another path requires `idf.py fullclean` first (CMake
  caches absolute paths).
- App size ~1.29 MB against a 1.5 MB factory partition (~17% free).
- **Do NOT enable a USB-Serial-JTAG console driver here.** GPIO19/20 are both
  the CAN transceiver pins *and* the S3 USB D−/D+ pads: installing the USJ
  driver reconfigures the pad mux after TWAI init and silently severs CAN RX
  (zero frames, zero errors — very confusing). The serial console task exists
  in `main.c` but is disabled for exactly this reason; boot logs flow through
  the secondary-USJ VFS path instead, which does not touch the pads.

## Serial output

No interactive console; the firmware prints status only:

```
[CAN] Stats: FPS=1037 Total=2258 Err=0 BusOff=0        # every 2 s
I (15595) ESPNOW: link: tx_0xA0 @10Hz, rx_0xB0=388     # every 5 s
[CAN] Status: state=1 msgs_to_tx=0 ...                 # every ~50 s
```

Healthy bench/in-car numbers: FPS in the hundreds-to-thousand range with the
ECU broadcasting; `rx_0xB0` climbing when the box is powered. `FPS=0 Total=0
Err=0` means electrically silent bus — check wiring/ECU power, not software.

## Bench testing

There is no synthetic simulator in this tree by design — it is the production
firmware. Options:

1. Real MicroSquirt on the bench (KOEO broadcasts outpc).
2. In-car install.
3. The demo/sandbox tree (`DASH_DEMO` synth) exists separately for UI work —
   never flash a demo build onto a dash wired to an installed box (it
   broadcasts synthetic engine data and will drive the box's actuators).

## Repository layout

```
main/
  main.c         app_main: NVS → display/touch init → ui_init → CAN → ESP-NOW
  can_rx.c/h     TWAI receive + decode + moving averages + snapshot API
  can_tx.c/h     command surface (fan/IAC/targets/cal) routed over ESP-NOW
  espnow_link.c/h  0xA0 TX task, 0xB0/C0 handling, softAP setup
  ui.c/h         all LVGL: boot screen, main cluster, settings, NVS settings
components/
  waveshare_rgb_lcd_port.c  board bring-up incl. CAN_SEL (CH422G EXIO5) routing
```

Dash-side persisted settings live in NVS namespace `dashui` (currently
`shift_rpm`). All other setpoints persist in the iobox3 itself.

## Related repos

- [esp32_ms_iobox3](https://github.com/Carputer-project/esp32_ms_iobox3) — I/O box this dash pairs with
- [esp32_diag_module](https://github.com/Carputer-project/esp32_diag_module) — wireless passive monitor/console for the pair
