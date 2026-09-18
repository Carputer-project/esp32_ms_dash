#include "ui.h"
#include "can_rx.h"
#include "can_tx.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_lv_adapter.h"
#include "nvs.h"
#include <string.h>
#include <math.h>
#include "lvgl.h"

static const char *TAG = "UI";

#define COL_BG    0x0A0A10
#define COL_TICK  0x3A3A46
#define COL_TEXT  0xE8E8F0
#define COL_DIM   0x80889A
#define COL_GOOD  0x33CC66
#define COL_BAD   0xE84040
#define COL_AFR   0x33AADD
#define COL_BOOST 0xD2A24A
#define COL_CLT   0xE07B4A
#define COL_TPS   0x9A66CC
#define COL_WARN  0xFF8C00
#define COL_MAG   0xCC33AA
#define COL_HB    0x3A6FE8

#define BOOT_MS   2600

#define LCD_W     800
#define LCD_H     480
#define RPM_MIN   0
#define RPM_MAX   9000
#define RPM_REDLINE 7000

#define GAUGE_CX  400
#define GAUGE_CY  220
#define GAUGE_R   185
#define GAUGE_A0  135.0f
#define GAUGE_A1  405.0f
#define GAUGE_SWEEP (GAUGE_A1 - GAUGE_A0)

static lv_obj_t *s_scr_main;
static lv_obj_t *s_scr_set;
static lv_obj_t *s_mark_glow;
static lv_obj_t *s_mark;

/* gauge face: three fully-painted static dials (classic / OEM / needle), each
 * on its own carrier object. The moving parts (ring, star/needle, labels, bars)
 * are siblings that sit above all of them, so only the visible dial swaps. Tap
 * the dial to cycle through the ENABLED faces ("which" from settings). Each
 * face has a colour state: AUTO follows the global accent, or a fixed theme
 * colour — "colour" from settings.
 *  - FACE_CLASSIC: neon look, star cursor + load-up ring
 *  - FACE_OEM:     neutral factory look, star cursor + load-up ring
 *  - FACE_NEEDLE:  classic analog tach with a real needle (no star/ring) */
#define FACE_CLASSIC 0
#define FACE_OEM     1
#define FACE_NEEDLE  2
#define FACE_COUNT   3
static lv_obj_t *s_face_oem, *s_face_classic, *s_face_needle;
static lv_obj_t *s_face_oem_canvas, *s_face_classic_canvas, *s_face_needle_canvas;
static int s_face_active = FACE_CLASSIC;
static bool s_face_dirty = false;   /* face changed; ui_face_persist_once() writes to NVS */
/* Per-face colour state: -1 = OFF (excluded from tap cycle, kept dormant),
 * 0 = AUTO (follow global accent), 1..THEME_COUNT = fixed theme colour. */
static int8_t s_face_state[FACE_COUNT] = { 0, 0, 0 };
static const char *s_face_names[FACE_COUNT] = { "CLS", "OEM", "NDL" };

/* needle-pointer geometry (FACE_NEEDLE only): a line from ~0.1R behind the hub
 * out to ~0.62R, recoloured by the face colour. A hub dot covers the base. */
static lv_obj_t *s_needle, *s_needle_hub;
/* lv_line_set_points() stores a POINTER, not a copy — the array must live as
 * long as the widget does. Keep one static buffer and rewrite it each tick. */
static lv_point_precise_t s_needle_pts[2];

/* star cursor (3-point, pre-rendered canvas) + zone-tinted glow halo */
#define STARSZ 30
static int s_mark_sz = STARSZ;
static uint32_t s_glow_col_prev = 0xFFFFFFFF;

/* load-up ring: three stacked zone arcs whose end angles advance with rpm.
 * Zone boundaries derive from s_shift_rpm_cfg — both at build time and live
 * in ui_update, so settings changes reshape the ring without a rebuild. */
static lv_obj_t *s_ring_g, *s_ring_a, *s_ring_r;

/* ---- warning banner ----
 * The dash hears the raw outpc over CAN, so it evaluates the threshold
 * warnings itself (rpm/clt/mat/batt/map/afr) — that keeps warnings working
 * even if the ESP-NOW link to the iobox3 drops, and lets RICH surface (the
 * box's single warn byte truncates bits 8-9). The box warn byte is kept as a
 * low-priority fallback: its 3s latch catches brief transients the dash's
 * instantaneous read would miss. Bit numbers mirror iobox3's WarnBit enum. */
#define WB_IDLE_LO   (1 << 0)
#define WB_IDLE_HI   (1 << 1)
#define WB_OVERREV   (1 << 2)
#define WB_OVERHEAT  (1 << 3)
#define WB_HOTAIR    (1 << 4)
#define WB_LOWBATT   (1 << 5)
#define WB_HIBATT    (1 << 6)
#define WB_OVERBOOST (1 << 7)
#define WB_LEAN      (1 << 8)
#define WB_RICH      (1 << 9)

static lv_obj_t *s_warn_box, *s_warn_lbl;
static const char *s_warn_cur = "";
static bool s_ovr_latch;
static lv_obj_t *s_val_lbl;
static lv_obj_t *s_can_lbl;
static lv_obj_t *s_mat_lbl;
static lv_obj_t *s_iobox_lbl;
static lv_obj_t *s_speed_val, *s_speed_unit;
static bool s_speed_unit_seen;
static lv_obj_t *s_iac_lbl;
static lv_obj_t *s_idle_lbl;
static lv_obj_t *s_fan_lbl;

static lv_obj_t *s_map_track, *s_map_fill, *s_map_lbl, *s_map_val;
static lv_obj_t *s_bst_track, *s_bst_fill, *s_bst_lbl, *s_bst_val;
static lv_obj_t *s_afr_track, *s_afr_fill, *s_afr_lbl, *s_afr_val;
static lv_obj_t *s_gas_track, *s_gas_fill, *s_gas_lbl, *s_gas_val;
static lv_obj_t *s_clt_track, *s_clt_fill, *s_clt_lbl, *s_clt_val;
static lv_obj_t *s_tps_track, *s_tps_fill, *s_tps_lbl, *s_tps_val;
static lv_obj_t *s_batt_track, *s_batt_fill, *s_batt_lbl, *s_batt_val;

/* indicator + high beam lamps (iobox3 anLatch bits) */
static lv_obj_t *s_ind_l, *s_ind_r, *s_hb;
static bool s_indL_on, s_indR_on, s_hb_on;
static bool s_l_vis, s_r_vis, s_hb_vis;

/* Main screen mode buttons (fan/IAC) */
static lv_obj_t *s_main_fauto, *s_main_fon, *s_main_foff;
static lv_obj_t *s_main_iauto, *s_main_ifollow, *s_main_iman;

/* Settings page mode buttons (fan/IAC/buzzer/boot) */
static lv_obj_t *s_btn_fauto, *s_btn_fon, *s_btn_foff;
static lv_obj_t *s_btn_iauto, *s_btn_ifollow, *s_btn_iman;
static lv_obj_t *s_buzz_btn, *s_boot_btn;

/* shift light strip: progressive from SHIFT_START, all flash blue at redline.
 * Thresholds are runtime values persisted in NVS (settings page adjusts them);
 * these defines are only the factory defaults. */
#define SHIFT_RPM_DEFAULT 7000
#define SHIFT_START_GAP   1000   /* start = shift point - gap */
#define SHIFT_SEGS  5
static int32_t s_shift_rpm_cfg = SHIFT_RPM_DEFAULT;
static lv_obj_t *s_shift_seg[SHIFT_SEGS];
static int32_t s_shift_last_rpm = -9999;
static bool s_shift_flash = false;
static const uint32_t kShiftColors[SHIFT_SEGS] = { COL_GOOD, COL_GOOD, COL_WARN, COL_WARN, COL_BAD };
#define SHIFT_OFF_COLOR 0x1A1A24

/* Accent theme selection + night mode (ported from the old 5-face build).
 * The accent drives the bezel/major-ticks on both gauge faces plus the cursor
 * glow/needle; night mode halves it for reduced glare. Persisted in NVS.
 * s_face_dirty-style flag lets the persist task write to NVS on a real stack. */
#define THEME_COUNT 4
static const uint32_t s_theme_colors[THEME_COUNT] = { COL_GOOD, COL_AFR, COL_MAG, COL_BOOST };
static uint32_t s_accent = COL_GOOD;   /* current accent colour */
static bool     s_night  = false;      /* reduced-glare night mode */
static bool     s_theme_dirty = false;      /* theme or night changed; persist pending */
bool     s_theme_apply_pending = false; /* theme changed; ui_theme_apply_once() needed */
static lv_obj_t *s_btn_theme[THEME_COUNT];
static lv_obj_t *s_night_btn;
static lv_obj_t *s_night_lbl;
static lv_obj_t *s_btn_face[FACE_COUNT];
static lv_obj_t *s_face_lbl[FACE_COUNT];

/* Dash-side engine-warning thresholds. The dash hears the raw outpc over CAN
 * directly, so it evaluates the threshold warnings itself (works even if the
 * ESP-NOW link to the iobox3 drops) and can surface RICH, which the box's
 * single warn byte truncates away. Defaults mirror the iobox3 engine profile;
 * tunable via NVS ("dashui" / "w_*") — no settings UI yet. Values are x10
 * (clt/mat/batt/map) like the rest of the dash data. */
static int16_t s_warn_clt_max   = 2300;   /* coolant 230 C */
static int16_t s_warn_mat_max   = 1600;   /* intake  160 C */
static int16_t s_warn_batt_min  = 110;    /* 11.0 V */
static int16_t s_warn_batt_max  = 160;    /* 16.0 V */
static int16_t s_warn_map_max   = 2800;   /* 280 kPa */
static int16_t s_warn_afr_low   = 100;    /* 10.0 rich */
static int16_t s_warn_afr_high  = 165;    /* 16.5 lean */

/* gating windows / release margins (avoid banner flapping at the edge) */
#define WARN_CLT_MIN   100       /* 10 C: ignore pre-warmup garbage */
#define WARN_MAT_MAX   3000      /* sanity cap on MAT reading */
#define WARN_BATT_MIN_F 0
#define WARN_REV_MARGIN 150

static dash_data_t s_prev;
static int32_t s_last_rpm = -9999;

static const char *warn_evaluate(const dash_data_t *d, uint32_t *color_out, bool *blink_out)
{
    if (!d->canOk) {
        s_ovr_latch = false;
        return NULL;
    }

    /* OVERREV from the live shift config so banner/strip/ring/glow share one
     * threshold; release with margin. */
    if (d->rpm >= s_shift_rpm_cfg) s_ovr_latch = true;
    else if (d->rpm < s_shift_rpm_cfg - WARN_REV_MARGIN) s_ovr_latch = false;

    if (s_ovr_latch) { *color_out = COL_BAD;  *blink_out = true;  return "OVERREV - SHIFT NOW"; }

    /* dash-side thresholds straight from the raw outpc. Gate each read window
     * so power-up / unplug garbage can't trip a false alert. */
    bool cltOk  = d->clt > WARN_CLT_MIN;
    bool matOk  = d->mat > 0 && d->mat < WARN_MAT_MAX;
    bool onThr  = d->tps >= 50;
    bool battOk = d->batt > 0;
    bool afrOk  = onThr && d->afr >= 90 && d->afr <= 260;
    bool mapOk  = d->map > 0 && d->map < 4000;   /* x10 units, sanity window */

    uint16_t ds = 0;
    if (cltOk  && d->clt  > s_warn_clt_max)  ds |= WB_OVERHEAT;
    if (matOk  && d->mat  > s_warn_mat_max)  ds |= WB_HOTAIR;
    if (battOk && d->batt < s_warn_batt_min) ds |= WB_LOWBATT;
    if (battOk && d->batt > s_warn_batt_max) ds |= WB_HIBATT;
    if (mapOk  && d->map > s_warn_map_max)   ds |= WB_OVERBOOST;
    if (afrOk  && d->afr  > s_warn_afr_high) ds |= WB_LEAN;
    if (afrOk  && d->afr  < s_warn_afr_low)  ds |= WB_RICH;

    /* priority order, first-hit-wins */
    static const struct { uint16_t bit; const char *txt; uint32_t col; bool blink; } kMap[] = {
        { WB_OVERHEAT,  "OVERHEAT",    COL_WARN, false },
        { WB_OVERBOOST, "OVERBOOST",   COL_BAD,  true  },
        { WB_HOTAIR,    "HOT AIR",     COL_WARN, false },
        { WB_LOWBATT,   "LOW BATTERY", COL_WARN, false },
        { WB_HIBATT,    "HIGH BATT",   COL_WARN, false },
        { WB_LEAN,      "LEAN",        COL_WARN, true  },
        { WB_RICH,      "RICH",        COL_WARN, false },
    };
    for (size_t i = 0; i < sizeof(kMap) / sizeof(kMap[0]); i++) {
        if (ds & kMap[i].bit) {
            *color_out = kMap[i].col;
            *blink_out = kMap[i].blink;
            return kMap[i].txt;
        }
    }

    /* box warn byte as a fallback: iobox3 latches with a 3s hold, catching
     * brief transients the dash's instant read misses. Only low 8 bits arrive
     * (LEAN/RICH never in the byte); IDLE bits skipped — warmup would spam
     * IDLE HIGH every cold start. */
    uint8_t box = d->warnFlags & ~(WB_IDLE_LO | WB_IDLE_HI | WB_OVERREV);
    for (size_t i = 0; i < sizeof(kMap) / sizeof(kMap[0]); i++) {
        if ((uint8_t)kMap[i].bit && (box & (uint8_t)kMap[i].bit)) {
            *color_out = kMap[i].col;
            *blink_out = kMap[i].blink;
            return kMap[i].txt;
        }
    }

    return NULL;
}

static void warn_banner_update(const dash_data_t *d)
{
    uint32_t col = COL_BAD;
    bool blink = false;
    const char *hit = warn_evaluate(d, &col, &blink);

    if (hit && blink) {
        if ((lv_tick_get() / 300) & 1) hit = NULL;   /* blink phase: hide */
    }
    if (hit != s_warn_cur) {
        s_warn_cur = hit;
        if (hit) {
            lv_label_set_text(s_warn_lbl, hit);
            lv_obj_set_style_bg_color(s_warn_box, lv_color_hex(col), 0);
            lv_obj_clear_flag(s_warn_box, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_add_flag(s_warn_box, LV_OBJ_FLAG_HIDDEN);
        }
    }
}

static lv_obj_t *s_scr_main_boot;
static lv_obj_t *s_boot_bar;
static uint32_t s_boot_start_ms;
static bool s_boot_done;

static void ui_init_main_build(void);
static void ui_init_main_show(void);
static void ui_show_settings(void);
static void lamp_draw_cb(lv_event_t *e);
static void settings_nvs_load(void);

static void fan_auto_evt(lv_event_t *e);
static void fan_on_evt(lv_event_t *e);
static void fan_off_evt(lv_event_t *e);
static void iac_auto_evt(lv_event_t *e);
static void iac_follow_evt(lv_event_t *e);
static void iac_man_evt(lv_event_t *e);
static void settings_show_evt(lv_event_t *e);
static void settings_back_evt(lv_event_t *e);
static void settings_iac_target_minus_evt(lv_event_t *e);
static void settings_iac_target_plus_evt(lv_event_t *e);
static void settings_shift_minus_evt(lv_event_t *e);
static void update_fan_mode_buttons(uint8_t mode);
static void update_iac_mode_buttons(uint8_t mode);
static void update_buzzer_button(bool on);
static void update_boot_button(bool on);
static void settings_shift_plus_evt(lv_event_t *e);
static void settings_duty_minus_evt(lv_event_t *e);
static void settings_duty_plus_evt(lv_event_t *e);
static void settings_buzz_evt(lv_event_t *e);
static void settings_beep_evt(lv_event_t *e);
static void settings_boot_evt(lv_event_t *e);

static void face_toggle_evt(lv_event_t *e);
static void ui_face_apply_live(void);

#define DEG2RAD(d) ((d) * 3.14159265f / 180.0f)

static float gauge_angle(float pct) {
    return GAUGE_A0 + GAUGE_SWEEP * pct / 100.0f;
}

static float gauge_pct_rpm(int32_t v) {
    if (v <= RPM_MIN) return 0.0f;
    if (v >= RPM_MAX) return 100.0f;
    return (float)(v - RPM_MIN) * 100.0f / (float)(RPM_MAX - RPM_MIN);
}

/* draw a line on a canvas layer */
static void face_line(lv_layer_t *layer, int cx, int cy, float deg,
                      int rIn, int rOut, uint32_t color, int width) {
    float a = DEG2RAD(deg);
    lv_draw_line_dsc_t d;
    lv_draw_line_dsc_init(&d);
    d.base.layer = layer;
    d.p1.x = (lv_coord_t)(cx + rIn * cosf(a));
    d.p1.y = (lv_coord_t)(cy + rIn * sinf(a));
    d.p2.x = (lv_coord_t)(cx + rOut * cosf(a));
    d.p2.y = (lv_coord_t)(cy + rOut * sinf(a));
    d.color = lv_color_hex(color);
    d.width = width;
    d.opa = LV_OPA_COVER;
    lv_draw_line(layer, &d);
}

/* draw an arc on a canvas layer */
static void face_arc(lv_layer_t *layer, int cx, int cy, int r,
                     float a0, float a1, uint32_t color, int width, lv_opa_t opa) {
    lv_draw_arc_dsc_t d;
    lv_draw_arc_dsc_init(&d);
    d.base.layer = layer;
    d.color = lv_color_hex(color);
    d.width = width;
    d.opa = opa;
    d.rounded = 0;
    d.center.x = cx;
    d.center.y = cy;
    d.radius = r;
    d.start_angle = a0;
    d.end_angle = a1;
    lv_draw_arc(layer, &d);
}

/* face selector: 1 = OEM/factory look (ported from the old 5-face build),
 * 0 = classic neon. */
static uint32_t accent_live(void) {
    if (!s_night) return s_accent;
    return (s_accent >> 1) & 0x7F7F7F;
}

/* Effective colour for a face: fixed theme colour if the face has one pinned
 * (s_face_state >= 1), else the global accent (night-dimming applied). */
static uint32_t face_colour(int f) {
    if (f >= 0 && f < FACE_COUNT && s_face_state[f] >= 1 &&
        s_face_state[f] <= THEME_COUNT)
        return s_theme_colors[s_face_state[f] - 1];
    return accent_live();
}

/* Dial background: TACH_FILL_CUSTOM_02 from the NFSU2 gauge pack, pre-scaled to
 * 370x370 and embedded as a raw 8-bit luminance binary (main/gauge_bg.bin) so
 * the blit is a 1:1 memcpy-then-expand - no runtime scaling, no LVGL transform.
 * Stored as L8 rather than RGB565 because the art is perfectly neutral (R=G=B)
 * and the app partition only has ~252 KB free (RGB565 would be 267 KB and 16 KB
 * over). Written at a 10px inset (370 + 2*10 = the 390px face canvas) so it
 * fills the dial circle exactly like the approved mockup. Additive only: every-
 * thing painted after it (rails, redline, ticks, numbers, needle, star) is
 * untouched. */
extern const uint8_t gauge_bg_bin_start[] asm("_binary_gauge_bg_bin_start");
extern const uint8_t gauge_bg_bin_end[]   asm("_binary_gauge_bg_bin_end");

#define GAUGE_BG_SZ    370
#define GAUGE_BG_INSET 10

static void gauge_bg_blit(lv_obj_t *canvas) {
    lv_draw_buf_t *db = lv_canvas_get_draw_buf(canvas);
    if (!db || !db->data) return;
    const uint8_t *src = gauge_bg_bin_start;
    if ((size_t)(gauge_bg_bin_end - gauge_bg_bin_start) < (size_t)GAUGE_BG_SZ * GAUGE_BG_SZ) return;
    uint8_t *base = db->data;
    uint32_t stride = db->header.stride;
    int c = GAUGE_BG_SZ / 2;
    int rr = GAUGE_R;                     /* clip to the dial circle so the art's
                                             square corners (not pure black) don't
                                             paint a visible square silhouette */
    int rr2 = rr * rr;
    for (int y = 0; y < GAUGE_BG_SZ; y++) {
        int dy = y - c;
        for (int x = 0; x < GAUGE_BG_SZ; x++) {
            int dx = x - c;
            if (dx * dx + dy * dy > rr2) continue;
            uint8_t v = src[(size_t)y * GAUGE_BG_SZ + (size_t)x];
            uint16_t px = (uint16_t)(((v >> 3) << 11) | ((v >> 2) << 5) | (v >> 3));
            uint8_t *dst = base + (size_t)(y + GAUGE_BG_INSET) * stride
                               + (size_t)(x + GAUGE_BG_INSET) * 2;
            dst[0] = (uint8_t)(px & 0xFF);
            dst[1] = (uint8_t)(px >> 8);
        }
    }
}

static void gauge_paint_classic(lv_obj_t *parent) {
    int cx = GAUGE_CX, cy = GAUGE_CY, r = GAUGE_R;
    int sz = r * 2 + 20;

    /* create canvas */
    size_t fbytes = (size_t)sz * sz * 2 + 64;
    void *fbuf = heap_caps_malloc(fbytes, MALLOC_CAP_SPIRAM);
    if (!fbuf) fbuf = malloc(fbytes);
    lv_obj_t *canvas = lv_canvas_create(parent);
    lv_obj_remove_style_all(canvas);
    lv_canvas_set_buffer(canvas, fbuf, sz, sz, LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(canvas, lv_color_hex(COL_BG), LV_OPA_COVER);
    gauge_bg_blit(canvas);
    lv_obj_set_pos(canvas, cx - sz / 2, cy - sz / 2);
    s_face_classic_canvas = canvas;

    lv_layer_t layer;
    lv_canvas_init_layer(canvas, &layer);

    int ox = sz / 2, oy = sz / 2;

    /* dial arc rail */
    face_arc(&layer, ox, oy, (int)(r * 0.72f), GAUGE_A0, GAUGE_A1, COL_TICK, 3, LV_OPA_80);

    /* redline band */
    if (RPM_REDLINE > RPM_MIN) {
        float rpct = gauge_pct_rpm(RPM_REDLINE);
        float a0 = gauge_angle(rpct);
        face_arc(&layer, ox, oy, (int)(r * 0.83f), a0, GAUGE_A1, COL_BAD, (int)(r * 0.14f), LV_OPA_60);
    }

    /* amber warn zone */
    if (RPM_REDLINE > RPM_MIN) {
        float rpct = gauge_pct_rpm(RPM_REDLINE);
        float w0 = gauge_pct_rpm(RPM_REDLINE - 1000);
        if (w0 < 0) w0 = 0;
        face_arc(&layer, ox, oy, (int)(r * 0.76f), gauge_angle(w0), gauge_angle(rpct),
                 COL_WARN, (int)(r * 0.09f), LV_OPA_50);
    }

    /* theme-colored bezel ring */
    face_arc(&layer, ox, oy, r, 0, 360, face_colour(FACE_CLASSIC), 2, LV_OPA_50);

    /* minor ticks (every 6 degrees) */
    for (int a = (int)GAUGE_A0; a <= (int)GAUGE_A1; a += 6)
        face_line(&layer, ox, oy, (float)a, (int)(r * 0.76f), (int)(r * 0.90f), COL_TICK, 1);

    /* major ticks (every 25%) */
    for (int p = 0; p <= 100; p += 25)
        face_line(&layer, ox, oy, gauge_angle((float)p), (int)(r * 0.70f), (int)(r * 0.92f), face_colour(FACE_CLASSIC), 2);

    /* scale numbers */
    static const char *scale_txt[] = {"0", "1k", "2k", "3k", "4k", "5k", "6k", "7k", "8k"};
    static const int scale_pct[] = {0, 11, 22, 33, 44, 56, 67, 78, 89};
    int lr = (int)(r * 0.58f);
    for (int i = 0; i < 9; i++) {
        float a = gauge_angle((float)scale_pct[i]);
        float rad = DEG2RAD(a);
        int lx = (int)(ox + lr * cosf(rad));
        int ly = (int)(oy + lr * sinf(rad));
        lv_obj_t *l = lv_label_create(parent);
        lv_label_set_text(l, scale_txt[i]);
        lv_obj_set_style_text_color(l, lv_color_hex(COL_DIM), 0);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_12, 0);
        lv_obj_set_width(l, 28);
        lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(l, (cx - sz / 2) + lx - 14, (cy - sz / 2) + ly - 8);
    }

    lv_canvas_finish_layer(canvas, &layer);
}

/* OEM / factory look: neutral white ticks, hairline bezel, thin redline arc.
 * Ported from the old 5-face reference build; stays neutral (no theme).
 * Redline/warn arcs derive from the live NVS shift setting like the ring. */
static void gauge_paint_oem(lv_obj_t *parent) {
    int cx = GAUGE_CX, cy = GAUGE_CY, r = GAUGE_R;
    int sz = r * 2 + 20;

    size_t fbytes = (size_t)sz * sz * 2 + 64;
    void *fbuf = heap_caps_malloc(fbytes, MALLOC_CAP_SPIRAM);
    if (!fbuf) fbuf = malloc(fbytes);
    lv_obj_t *canvas = lv_canvas_create(parent);
    lv_obj_remove_style_all(canvas);
    lv_canvas_set_buffer(canvas, fbuf, sz, sz, LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(canvas, lv_color_hex(COL_BG), LV_OPA_COVER);
    gauge_bg_blit(canvas);
    lv_obj_set_pos(canvas, cx - sz / 2, cy - sz / 2);
    s_face_oem_canvas = canvas;

    lv_layer_t layer;
    lv_canvas_init_layer(canvas, &layer);

    int ox = sz / 2, oy = sz / 2;

    /* hairline arc rail the needle rides on */
    face_arc(&layer, ox, oy, (int)(r * 0.78f), GAUGE_A0, GAUGE_A1, 0xD8D8E0, 1, LV_OPA_80);

    /* thin factory redline arc: band + crisp edge (not a fat block) */
    int32_t red = s_shift_rpm_cfg;
    int32_t warn = red - SHIFT_START_GAP;
    if (warn < RPM_MIN) warn = RPM_MIN;
    float rpct = gauge_pct_rpm(red);
    float wpct = gauge_pct_rpm(warn);
    int ra0 = (int)gauge_angle(rpct);
    face_arc(&layer, ox, oy, (int)(r * 0.78f), ra0, GAUGE_A1, COL_BAD, (int)(r * 0.08f), LV_OPA_50);
    face_arc(&layer, ox, oy, (int)(r * 0.78f), ra0, GAUGE_A1, COL_BAD, 2, LV_OPA_90);

    /* thin amber warnline arc, just inside */
    face_arc(&layer, ox, oy, (int)(r * 0.72f), gauge_angle(wpct), gauge_angle(rpct),
             COL_WARN, 2, LV_OPA_80);

    /* white minor + major ticks */
    for (int a = (int)GAUGE_A0; a <= (int)GAUGE_A1; a += 5)
        face_line(&layer, ox, oy, (float)a, (int)(r * 0.80f), (int)(r * 0.88f), 0xD8D8E0, 1);
    for (int p = 0; p <= 100; p += 25)
        face_line(&layer, ox, oy, gauge_angle((float)p), (int)(r * 0.80f), (int)(r * 0.92f), face_colour(FACE_OEM), 2);

    /* hairline bezel */
    face_arc(&layer, ox, oy, r, 0, 360, face_colour(FACE_OEM), 1, LV_OPA_90);

    /* scale numbers, OEM-neutral */
    static const char *scale_txt[] = {"0", "1k", "2k", "3k", "4k", "5k", "6k", "7k", "8k"};
    static const int scale_pct[] = {0, 11, 22, 33, 44, 56, 67, 78, 89};
    int lr = (int)(r * 0.58f);
    for (int i = 0; i < 9; i++) {
        float a = gauge_angle((float)scale_pct[i]);
        float rad = DEG2RAD(a);
        int lx = (int)(ox + lr * cosf(rad));
        int ly = (int)(oy + lr * sinf(rad));
        lv_obj_t *l = lv_label_create(parent);
        lv_label_set_text(l, scale_txt[i]);
        lv_obj_set_style_text_color(l, lv_color_hex(0xD8D8E0), 0);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_12, 0);
        lv_obj_set_width(l, 28);
        lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(l, (cx - sz / 2) + lx - 14, (cy - sz / 2) + ly - 8);
    }

    lv_canvas_finish_layer(canvas, &layer);
}

/* Needle gauge (FACE_NEEDLE): classic analog tach — white tick marks, clean
 * sweep, thin redline band, accent-coloured bezel + major ticks. The pointer is
 * a real needle (s_needle) instead of the star cursor; it sweeps 135..405 deg.
 * Redline/warn arcs derive from the live NVS shift setting like the other faces. */
static void gauge_paint_needle(lv_obj_t *parent) {
    int cx = GAUGE_CX, cy = GAUGE_CY, r = GAUGE_R;
    int sz = r * 2 + 20;

    size_t fbytes = (size_t)sz * sz * 2 + 64;
    void *fbuf = heap_caps_malloc(fbytes, MALLOC_CAP_SPIRAM);
    if (!fbuf) fbuf = malloc(fbytes);
    lv_obj_t *canvas = lv_canvas_create(parent);
    lv_obj_remove_style_all(canvas);
    lv_canvas_set_buffer(canvas, fbuf, sz, sz, LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(canvas, lv_color_hex(COL_BG), LV_OPA_COVER);
    gauge_bg_blit(canvas);
    lv_obj_set_pos(canvas, cx - sz / 2, cy - sz / 2);
    s_face_needle_canvas = canvas;

    lv_layer_t layer;
    lv_canvas_init_layer(canvas, &layer);

    int ox = sz / 2, oy = sz / 2;

    /* arc rail the needle rides on (accent) */
    face_arc(&layer, ox, oy, (int)(r * 0.60f), GAUGE_A0, GAUGE_A1, face_colour(FACE_NEEDLE), 2, LV_OPA_80);

    /* thin redline band at the tail of the sweep */
    if (RPM_REDLINE > RPM_MIN) {
        float rpct = gauge_pct_rpm(RPM_REDLINE);
        face_arc(&layer, ox, oy, (int)(r * 0.60f), gauge_angle(rpct), GAUGE_A1,
                 COL_BAD, (int)(r * 0.10f), LV_OPA_60);
    }

    /* amber warn zone just below redline */
    if (RPM_REDLINE > RPM_MIN) {
        float rpct = gauge_pct_rpm(RPM_REDLINE);
        float w0 = gauge_pct_rpm(RPM_REDLINE - 1000);
        if (w0 < 0) w0 = 0;
        face_arc(&layer, ox, oy, (int)(r * 0.60f), gauge_angle(w0), gauge_angle(rpct),
                 COL_WARN, (int)(r * 0.12f), LV_OPA_40);
    }

    /* accent-coloured bezel ring */
    face_arc(&layer, ox, oy, r, 0, 360, face_colour(FACE_NEEDLE), 2, LV_OPA_80);

    /* minor ticks (every 5 degrees) */
    for (int a = (int)GAUGE_A0; a <= (int)GAUGE_A1; a += 5)
        face_line(&layer, ox, oy, (float)a, (int)(r * 0.66f), (int)(r * 0.76f), 0xD8D8E0, 1);

    /* major ticks (every 25%) */
    for (int p = 0; p <= 100; p += 25)
        face_line(&layer, ox, oy, gauge_angle((float)p), (int)(r * 0.63f), (int)(r * 0.80f), face_colour(FACE_NEEDLE), 2);

    /* scale numbers */
    static const char *scale_txt[] = {"0", "1k", "2k", "3k", "4k", "5k", "6k", "7k", "8k"};
    static const int scale_pct[] = {0, 11, 22, 33, 44, 56, 67, 78, 89};
    int lr = (int)(r * 0.46f);
    for (int i = 0; i < 9; i++) {
        float a = gauge_angle((float)scale_pct[i]);
        float rad = DEG2RAD(a);
        int lx = (int)(ox + lr * cosf(rad));
        int ly = (int)(oy + lr * sinf(rad));
        lv_obj_t *l = lv_label_create(parent);
        lv_label_set_text(l, scale_txt[i]);
        lv_obj_set_style_text_color(l, lv_color_hex(COL_DIM), 0);
        lv_obj_set_style_text_font(l, &lv_font_montserrat_12, 0);
        lv_obj_set_width(l, 28);
        lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(l, (cx - sz / 2) + lx - 14, (cy - sz / 2) + ly - 8);
    }

    lv_canvas_finish_layer(canvas, &layer);
}

/* Re-paint a face canvas. LVGL's canvas destructor only drops the image cache
 * (it never frees the caller's buffer), so we free the old SPIRAM buffer here.
 * s_face_baked[] records the colour each canvas was last painted with so the
 * face toggle only repaints a canvas when its colour is actually stale. */
static uint32_t s_face_baked[FACE_COUNT] = { 0 };

static void face_canvas_del(lv_obj_t **slot) {
    if (!*slot) return;
    lv_draw_buf_t *db = lv_canvas_get_draw_buf(*slot);
    void *old = db ? db->data : NULL;
    lv_obj_del(*slot);
    *slot = NULL;
    if (old) free(old);
}

static void face_paint(int f, bool force) {
    lv_obj_t **slot = NULL, **carrier = NULL;
    if (f == FACE_CLASSIC)  { slot = &s_face_classic_canvas; carrier = &s_face_classic; }
    else if (f == FACE_OEM) { slot = &s_face_oem_canvas;     carrier = &s_face_oem; }
    else                    { slot = &s_face_needle_canvas;  carrier = &s_face_needle; }
    if (!*carrier) return;
    uint32_t want = face_colour(f);
    if (!force && s_face_baked[f] == want) return;   /* already correct */
    face_canvas_del(slot);
    if (f == FACE_CLASSIC) gauge_paint_classic(*carrier);
    else if (f == FACE_OEM) gauge_paint_oem(*carrier);
    else gauge_paint_needle(*carrier);
    s_face_baked[f] = want;
    s_face_dirty = true;
}

/* Re-paint the active gauge face after an accent/fixed-colour/night change.
 * We only repaint the VISIBLE face immediately; hidden faces are marked stale
 * and repainted on next face toggle (so they're ready when shown). This keeps
 * the per-frame LVGL work under the watchdog budget. The needle's colour is an
 * object style, so we just restyle it (baked canvas colour is already tracked).
 * Statically-coloured bar fills (MAP/BOOST/CLT/TPS/BATT) follow the accent;
 * AFR and GAS keep their value-driven warning colours. */
static void ui_theme_apply(void) {
    face_paint(s_face_active, true);
    uint32_t bar_col = accent_live();
    lv_obj_t *static_bars[][2] = {
        { s_map_fill,  s_map_val  },
        { s_bst_fill,  s_bst_val  },
        { s_clt_fill,  s_clt_val  },
        { s_tps_fill,  s_tps_val  },
        { s_batt_fill, s_batt_val },
    };
    for (size_t i = 0; i < sizeof(static_bars) / sizeof(static_bars[0]); i++) {
        if (static_bars[i][0]) lv_obj_set_style_bg_color(static_bars[i][0], lv_color_hex(bar_col), 0);
        if (static_bars[i][1]) lv_obj_set_style_text_color(static_bars[i][1], lv_color_hex(bar_col), 0);
    }
    if (s_face_active == FACE_NEEDLE) {
        if (s_needle) lv_obj_set_style_line_color(s_needle, lv_color_hex(face_colour(FACE_NEEDLE)), 0);
        if (s_needle_hub) lv_obj_set_style_bg_color(s_needle_hub, lv_color_hex(face_colour(FACE_NEEDLE)), 0);
        if (s_needle_hub) lv_obj_set_style_border_color(s_needle_hub, lv_color_hex(face_colour(FACE_NEEDLE)), 0);
        for (int i = 0; i < FACE_COUNT; i++) if (i != FACE_NEEDLE) s_face_baked[i] = 0xFFFFFFFF;
    } else {
        for (int i = 0; i < FACE_COUNT; i++) if (i != s_face_active) s_face_baked[i] = 0xFFFFFFFF;
    }
}

static void create_bar(lv_obj_t **track, lv_obj_t **fill, lv_obj_t **name_lbl,
                       lv_obj_t **val_lbl, lv_obj_t *parent,
                       int x, int y, int w, int h,
                       const char *name, uint32_t color) {
    *name_lbl = lv_label_create(parent);
    lv_label_set_text(*name_lbl, name);
    lv_obj_set_style_text_color(*name_lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_style_text_font(*name_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_width(*name_lbl, w);
    lv_obj_set_style_text_align(*name_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(*name_lbl, x, y);

    *track = lv_obj_create(parent);
    lv_obj_remove_style_all(*track);
    lv_obj_set_size(*track, w, h);
    lv_obj_set_pos(*track, x, y + 18);
    lv_obj_set_style_bg_color(*track, lv_color_hex(0x16161F), 0);
    lv_obj_set_style_bg_opa(*track, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(*track, 6, 0);
    lv_obj_set_style_pad_all(*track, 2, 0);

    *fill = lv_obj_create(*track);
    lv_obj_remove_style_all(*fill);
    lv_obj_set_style_bg_color(*fill, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(*fill, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(*fill, 4, 0);
    lv_obj_set_size(*fill, w - 4, 0);
    lv_obj_align(*fill, LV_ALIGN_BOTTOM_MID, 0, 0);

    *val_lbl = lv_label_create(parent);
    lv_label_set_text(*val_lbl, "--");
    lv_obj_set_style_text_color(*val_lbl, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_style_text_font(*val_lbl, &lv_font_montserrat_20, 0);
    lv_obj_set_width(*val_lbl, w);
    lv_obj_set_style_text_align(*val_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(*val_lbl, x, y + 18 + h + 4);
}

static void bar_set_val(lv_obj_t *fill, lv_obj_t *val_lbl,
                         int h, bool ok, int32_t minv, int32_t maxv,
                         int32_t raw, const char *txt) {
    if (ok) {
        int pct = 0;
        if (raw > minv && raw < maxv)
            pct = (int)((float)(raw - minv) * 100.0f / (float)(maxv - minv));
        else if (raw >= maxv)
            pct = 100;
        lv_obj_set_height(fill, (h - 4) * pct / 100);
    } else {
        lv_obj_set_height(fill, 0);
    }
    if (strcmp(lv_label_get_text(val_lbl), txt) != 0)
        lv_label_set_text(val_lbl, txt);
}

static uint32_t afr_color(float afr) {
    if (afr < 10.5f)  return COL_BAD;
    if (afr < 11.5f)  return COL_WARN;
    if (afr <= 16.5f) return accent_live();
    if (afr <= 18.0f) return COL_WARN;
    return COL_BAD;
}

/* neon glow label: dim halo offset + bright main copy */
static lv_obj_t *neon_label(lv_obj_t *parent, const char *txt, const lv_font_t *font,
                            uint32_t color, int xoff, int yoff) {
    lv_obj_t *halo = lv_label_create(parent);
    lv_label_set_text(halo, txt);
    lv_obj_set_style_text_font(halo, font, 0);
    lv_obj_set_style_text_color(halo, lv_color_hex(color), 0);
    lv_obj_set_style_text_opa(halo, LV_OPA_50, 0);
    lv_obj_align(halo, LV_ALIGN_CENTER, xoff + 3, yoff + 3);

    lv_obj_t *l = lv_label_create(parent);
    lv_label_set_text(l, txt);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, lv_color_hex(color), 0);
    lv_obj_align(l, LV_ALIGN_CENTER, xoff, yoff);
    return l;
}

static void build_boot(void) {
    s_scr_main_boot = lv_obj_create(NULL);
    lv_obj_remove_style_all(s_scr_main_boot);
    lv_obj_set_style_bg_color(s_scr_main_boot, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_bg_opa(s_scr_main_boot, LV_OPA_COVER, 0);

    /* Title "7A-GTE" - neon cyan */
    neon_label(s_scr_main_boot, "7A-GTE", &lv_font_montserrat_48, COL_AFR, 0, -90);

    /* Subtitle "MS DASH" - neon magenta */
    neon_label(s_scr_main_boot, "MS DASH", &lv_font_montserrat_30, COL_MAG, 0, -15);

    /* Progress bar */
    s_boot_bar = lv_bar_create(s_scr_main_boot);
    lv_obj_remove_style_all(s_boot_bar);
    lv_obj_set_size(s_boot_bar, 420, 10);
    lv_obj_align(s_boot_bar, LV_ALIGN_CENTER, 0, 85);
    lv_bar_set_range(s_boot_bar, 0, 100);
    lv_bar_set_value(s_boot_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(s_boot_bar, lv_color_hex(0x16161F), 0);
    lv_obj_set_style_bg_opa(s_boot_bar, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_boot_bar, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_boot_bar, lv_color_hex(COL_AFR), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(s_boot_bar, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_boot_bar, LV_RADIUS_CIRCLE, LV_PART_INDICATOR);

    /* "BOOTING" label */
    lv_obj_t *booting = lv_label_create(s_scr_main_boot);
    lv_label_set_text(booting, "BOOTING");
    lv_obj_set_style_text_color(booting, lv_color_hex(COL_DIM), 0);
    lv_obj_set_style_text_font(booting, &lv_font_montserrat_14, 0);
    lv_obj_align(booting, LV_ALIGN_CENTER, 0, 113);

    /* Footer */
    lv_obj_t *foot = lv_label_create(s_scr_main_boot);
    lv_label_set_text(foot, "MS2/EXTRA 3.4.4   CAN DASH");
    lv_obj_set_style_text_color(foot, lv_color_hex(COL_DIM), 0);
    lv_obj_set_style_text_font(foot, &lv_font_montserrat_12, 0);
    lv_obj_align(foot, LV_ALIGN_CENTER, 0, 215);

    lv_scr_load(s_scr_main_boot);
}

static void boot_timer_cb(lv_timer_t *timer) {
    (void)timer;
    if (s_boot_done) {
        lv_timer_del(timer);
        return;
    }

    uint32_t el = lv_tick_get() - s_boot_start_ms;
    if (el < BOOT_MS) {
        lv_bar_set_value(s_boot_bar, el * 100 / BOOT_MS, LV_ANIM_OFF);
    } else {
        s_boot_done = true;
        lv_timer_del(timer);
        /* Show main UI */
        ui_init_main_show();
    }
}

void ui_init(void) {
    ESP_LOGI(TAG, "UI init - boot screen");

    settings_nvs_load();
    build_boot();
    s_boot_start_ms = lv_tick_get();
    s_boot_done = false;
    lv_timer_create(boot_timer_cb, 16, NULL);  /* ~60fps */

    /* Build main UI in background but don't show yet */
    ui_init_main_build();

    /* Retint everything to the loaded accent (bars, faces, needle, buttons).
     * Without this the widgets keep their creation-time colours until the user
     * taps an accent swatch — the first can_update_task tick applies it. */
    s_theme_apply_pending = true;
}

/* mode: 0 = left arrow, 1 = right arrow, 2 = high beam lamp */
static void lamp_draw_cb(lv_event_t *e) {
    lv_obj_t *obj = lv_event_get_target(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    int mode = (int)(intptr_t)lv_event_get_user_data(e);
    lv_area_t co;
    lv_obj_get_coords(obj, &co);
    lv_coord_t x = co.x1, y = co.y1;
    lv_coord_t w = lv_obj_get_width(obj), h = lv_obj_get_height(obj);

    if (mode == 2) {
        /* high beam: blue lamp + 3 beams */
        lv_draw_rect_dsc_t rd;
        lv_draw_rect_dsc_init(&rd);
        rd.base.layer = layer;
        rd.bg_color = lv_color_hex(COL_HB);
        rd.bg_opa = LV_OPA_COVER;
        rd.radius = LV_RADIUS_CIRCLE;
        lv_area_t ra = { (lv_coord_t)(x + w * 55 / 100), y,
                         (lv_coord_t)(x + w - 1), (lv_coord_t)(y + h - 1) };
        lv_draw_rect(layer, &rd, &ra);

        lv_draw_line_dsc_t ld;
        lv_draw_line_dsc_init(&ld);
        ld.base.layer = layer;
        ld.color = lv_color_hex(COL_TEXT);
        ld.width = 3;
        ld.opa = LV_OPA_COVER;
        for (int i = 0; i < 3; i++) {
            ld.p1.x = (lv_coord_t)(x + 2);
            ld.p2.x = (lv_coord_t)(x + w * 42 / 100);
            ld.p1.y = ld.p2.y = (lv_coord_t)(y + h * (20 + i * 30) / 100);
            lv_draw_line(layer, &ld);
        }
    } else {
        lv_draw_triangle_dsc_t td;
        lv_draw_triangle_dsc_init(&td);
        td.base.layer = layer;
        td.color = lv_color_hex(COL_GOOD);
        td.opa = LV_OPA_COVER;
        if (mode == 0) {
            td.p[0].x = x;           td.p[0].y = (lv_coord_t)(y + h / 2);
            td.p[1].x = (lv_coord_t)(x + w - 1); td.p[1].y = y;
            td.p[2].x = (lv_coord_t)(x + w - 1); td.p[2].y = (lv_coord_t)(y + h - 1);
        } else {
            td.p[0].x = (lv_coord_t)(x + w - 1); td.p[0].y = (lv_coord_t)(y + h / 2);
            td.p[1].x = x;           td.p[1].y = y;
            td.p[2].x = x;           td.p[2].y = (lv_coord_t)(y + h - 1);
        }
        lv_draw_triangle(layer, &td);
    }
}

static void ui_init_main_build(void) {
    ESP_LOGI(TAG, "Building main UI (background)");

    s_scr_main = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(s_scr_main, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_bg_opa(s_scr_main, LV_OPA_COVER, 0);

    /* three gauge faces: each a fully-painted static dial on its own carrier.
     * The moving parts below (ring, star/needle, labels, bars) are siblings
     * layered above all of them, so tap-to-toggle only swaps the dial. */
    s_face_oem = lv_obj_create(s_scr_main);
    lv_obj_remove_style_all(s_face_oem);
    lv_obj_set_size(s_face_oem, 800, 480);
    lv_obj_set_pos(s_face_oem, 0, 0);
    gauge_paint_oem(s_face_oem);
    s_face_baked[FACE_OEM] = face_colour(FACE_OEM);

    s_face_classic = lv_obj_create(s_scr_main);
    lv_obj_remove_style_all(s_face_classic);
    lv_obj_set_size(s_face_classic, 800, 480);
    lv_obj_set_pos(s_face_classic, 0, 0);
    gauge_paint_classic(s_face_classic);
    s_face_baked[FACE_CLASSIC] = face_colour(FACE_CLASSIC);

    s_face_needle = lv_obj_create(s_scr_main);
    lv_obj_remove_style_all(s_face_needle);
    lv_obj_set_size(s_face_needle, 800, 480);
    lv_obj_set_pos(s_face_needle, 0, 0);
    gauge_paint_needle(s_face_needle);
    s_face_baked[FACE_NEEDLE] = face_colour(FACE_NEEDLE);

    /* FM face active: show the needle, hide star cursor + load-up ring. The
     * dial swap + moving-part visibility are driven by ui_face_apply_live(). */
    lv_obj_add_flag(s_face_oem, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_face_classic, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_face_needle, LV_OBJ_FLAG_HIDDEN);
    if (s_face_active == FACE_OEM)
        lv_obj_clear_flag(s_face_oem, LV_OBJ_FLAG_HIDDEN);
    else if (s_face_active == FACE_NEEDLE)
        lv_obj_clear_flag(s_face_needle, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_clear_flag(s_face_classic, LV_OBJ_FLAG_HIDDEN);
    ui_face_apply_live();

    /* load-up ring: 3 stacked zone arcs (green/amber/red), end angle advances
     * with rpm like a shift indicator filling up. Zero-span until first update. */
    {
        int sz = GAUGE_R * 2;
        int32_t start_cfg = s_shift_rpm_cfg - SHIFT_START_GAP;
        if (start_cfg < RPM_MIN) start_cfg = RPM_MIN;
        float pct_warn = 100.0f * (start_cfg - RPM_MIN) / (RPM_MAX - RPM_MIN);
        float pct_red  = 100.0f * (s_shift_rpm_cfg - RPM_MIN) / (RPM_MAX - RPM_MIN);
        float starts[3] = { GAUGE_A0, gauge_angle(pct_warn), gauge_angle(pct_red) };
        uint32_t cols[3] = { COL_GOOD, COL_WARN, COL_BAD };
        lv_obj_t **slot[3] = { &s_ring_g, &s_ring_a, &s_ring_r };
        for (int i = 0; i < 3; i++) {
            lv_obj_t *a = lv_arc_create(s_scr_main);
            lv_obj_remove_style_all(a);
            lv_obj_set_size(a, sz, sz);
            lv_obj_set_pos(a, GAUGE_CX - GAUGE_R, GAUGE_CY - GAUGE_R);
            int sd = (int)starts[i];
            lv_arc_set_rotation(a, 0);
            lv_arc_set_bg_angles(a, sd, sd);
            lv_arc_set_angles(a, sd, sd);          /* zero span until loaded */
            lv_obj_set_style_arc_width(a, 9, LV_PART_INDICATOR);
            lv_obj_set_style_arc_color(a, lv_color_hex(cols[i]), LV_PART_INDICATOR);
            lv_obj_set_style_arc_opa(a, LV_OPA_80, LV_PART_INDICATOR);
            *slot[i] = a;
        }
    }

    /* glowing halo behind the cursor */
    int msize = 22;
    s_mark_glow = lv_obj_create(s_scr_main);
    lv_obj_remove_style_all(s_mark_glow);
    lv_obj_set_size(s_mark_glow, msize, msize);
    lv_obj_set_style_bg_color(s_mark_glow, lv_color_hex(COL_GOOD), 0);
    lv_obj_set_style_bg_opa(s_mark_glow, LV_OPA_40, 0);
    lv_obj_set_style_radius(s_mark_glow, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(s_mark_glow, 0, 0);

    /* star cursor: pre-rendered 3-point star on transparent canvas.
     * Falls back to the old dot marker if allocation fails (never deref NULL). */
    void *starbuf = heap_caps_malloc(STARSZ * STARSZ * 4, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!starbuf) starbuf = malloc(STARSZ * STARSZ * 4);
    s_mark_sz = STARSZ;
    if (starbuf) {
        memset(starbuf, 0, STARSZ * STARSZ * 4);
        s_mark = lv_canvas_create(s_scr_main);
        lv_canvas_set_buffer(s_mark, starbuf, STARSZ, STARSZ, LV_COLOR_FORMAT_ARGB8888);
        lv_layer_t sl;
        lv_canvas_init_layer(s_mark, &sl);
        /* 3-point star: outer tips at top/120/240deg, valleys between */
        lv_draw_line_dsc_t d;
        lv_draw_line_dsc_init(&d);
        d.base.layer = &sl;
        d.width = 2;
        d.opa = LV_OPA_COVER;
        d.color = lv_color_hex(0xFFFFFF);
        lv_point_precise_t pts[7];
        for (int i = 0; i <= 6; i++) {
            float ang = DEG2RAD(i * 60 - 90);
            float rr = (i % 2 == 0) ? 13.0f : 4.5f;
            pts[i].x = STARSZ / 2 + rr * cosf(ang);
            pts[i].y = STARSZ / 2 + rr * sinf(ang);
        }
        for (int i = 0; i < 6; i++) {
            d.p1 = pts[i];
            d.p2 = pts[i + 1];
            lv_draw_line(&sl, &d);
        }
        lv_canvas_finish_layer(s_mark, &sl);
    } else {
        ESP_LOGE(TAG, "star alloc failed — dot fallback");
        s_mark_sz = 10;
        s_mark = lv_obj_create(s_scr_main);
        lv_obj_remove_style_all(s_mark);
        lv_obj_set_size(s_mark, s_mark_sz, s_mark_sz);
        lv_obj_set_style_bg_color(s_mark, lv_color_hex(COL_GOOD), 0);
        lv_obj_set_style_bg_opa(s_mark, LV_OPA_COVER, 0);
        lv_obj_set_style_radius(s_mark, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_border_width(s_mark, 0, 0);
    }

    /* Initial marker position at RPM=0 (135°) */
    float rad0 = DEG2RAD(GAUGE_A0);
    lv_coord_t dx0 = (lv_coord_t)(GAUGE_CX + GAUGE_R * 0.85f * cosf(rad0));
    lv_coord_t dy0 = (lv_coord_t)(GAUGE_CY + GAUGE_R * 0.85f * sinf(rad0));
    lv_obj_set_pos(s_mark_glow, dx0 - msize / 2, dy0 - msize / 2);
    lv_obj_set_pos(s_mark, dx0 - s_mark_sz / 2, dy0 - s_mark_sz / 2);

    /* FACE_NEEDLE pointer: a line sweeping from behind the hub out to the rail,
     * plus a small hub dot. Recoloured by ui_theme_apply / face tap. */
    s_needle = lv_line_create(s_scr_main);
    lv_obj_remove_style_all(s_needle);
    s_needle_pts[0] = (lv_point_precise_t){ GAUGE_CX, GAUGE_CY };
    s_needle_pts[1] = (lv_point_precise_t){ GAUGE_CX, GAUGE_CY };
    lv_line_set_points(s_needle, s_needle_pts, 2);
    lv_obj_set_style_line_width(s_needle, 5, 0);
    lv_obj_set_style_line_color(s_needle, lv_color_hex(face_colour(FACE_NEEDLE)), 0);
    lv_obj_set_style_line_rounded(s_needle, true, 0);
    lv_obj_set_style_line_opa(s_needle, LV_OPA_COVER, 0);
    lv_obj_add_flag(s_needle, LV_OBJ_FLAG_HIDDEN);   /* ui_face_apply_live() reveals */

    int hsz = 16;
    s_needle_hub = lv_obj_create(s_scr_main);
    lv_obj_remove_style_all(s_needle_hub);
    lv_obj_set_size(s_needle_hub, hsz, hsz);
    lv_obj_set_pos(s_needle_hub, GAUGE_CX - hsz / 2, GAUGE_CY - hsz / 2);
    lv_obj_set_style_bg_color(s_needle_hub, lv_color_hex(COL_BG), 0);
    lv_obj_set_style_bg_opa(s_needle_hub, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_needle_hub, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_border_width(s_needle_hub, 3, 0);
    lv_obj_set_style_border_color(s_needle_hub, lv_color_hex(face_colour(FACE_NEEDLE)), 0);
    lv_obj_add_flag(s_needle_hub, LV_OBJ_FLAG_HIDDEN);

    /* "RPM" title label */
    lv_obj_t *title = lv_label_create(s_scr_main);
    lv_label_set_text(title, "RPM");
    lv_obj_set_style_text_color(title, lv_color_hex(COL_GOOD), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_set_width(title, 44);
    lv_obj_set_pos(title, GAUGE_CX - 22, GAUGE_CY + (int)(GAUGE_R * 0.36f) - 8);

    /* big RPM value label */
    s_val_lbl = lv_label_create(s_scr_main);
    lv_label_set_text(s_val_lbl, "--");
    lv_obj_set_style_text_color(s_val_lbl, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_style_text_font(s_val_lbl, &lv_font_montserrat_30, 0);
    lv_obj_set_width(s_val_lbl, 200);
    lv_obj_set_style_text_align(s_val_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(s_val_lbl, GAUGE_CX - 100, GAUGE_CY + (int)(GAUGE_R * 0.50f));

    /* top labels */
    s_can_lbl = lv_label_create(s_scr_main);
    lv_label_set_text(s_can_lbl, "CAN LOST");
    lv_obj_set_style_text_color(s_can_lbl, lv_color_hex(COL_BAD), 0);
    lv_obj_set_style_text_font(s_can_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(s_can_lbl, 350, 8);

    s_mat_lbl = lv_label_create(s_scr_main);
    lv_label_set_text(s_mat_lbl, "MAT --F");
    lv_obj_set_style_text_color(s_mat_lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_style_text_font(s_mat_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(s_mat_lbl, 700, 8);

    /* iobox3 link-health chip (top-left): BOX OK when the ESP-NOW link is
     * alive, BOX LOST when the box goes silent. CAN can be fine while the box
     * is dead, so this is the only direct proof the box is on the link. */
    s_iobox_lbl = lv_label_create(s_scr_main);
    lv_label_set_text(s_iobox_lbl, "BOX LOST");
    lv_obj_set_style_text_color(s_iobox_lbl, lv_color_hex(0x0A0A10), 0);
    lv_obj_set_style_bg_color(s_iobox_lbl, lv_color_hex(COL_BAD), 0);
    lv_obj_set_style_bg_opa(s_iobox_lbl, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(s_iobox_lbl, 3, 0);
    lv_obj_set_style_pad_ver(s_iobox_lbl, 2, 0);
    lv_obj_set_style_pad_hor(s_iobox_lbl, 6, 0);
    lv_obj_set_style_text_font(s_iobox_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(s_iobox_lbl, 10, 8);

    /* bar gauges */
    create_bar(&s_map_track, &s_map_fill, &s_map_lbl, &s_map_val,
               s_scr_main, 10, 44, 48, 120, "MAP", COL_BOOST);
    create_bar(&s_bst_track, &s_bst_fill, &s_bst_lbl, &s_bst_val,
               s_scr_main, 62, 44, 48, 120, "BOOST", COL_BOOST);
    create_bar(&s_afr_track, &s_afr_fill, &s_afr_lbl, &s_afr_val,
               s_scr_main, 10, 210, 48, 120, "AFR", COL_AFR);
    create_bar(&s_gas_track, &s_gas_fill, &s_gas_lbl, &s_gas_val,
               s_scr_main, 62, 210, 48, 120, "GAS", COL_GOOD);
    create_bar(&s_clt_track, &s_clt_fill, &s_clt_lbl, &s_clt_val,
               s_scr_main, 600, 170, 63, 100, "CLT", COL_CLT);
    create_bar(&s_tps_track, &s_tps_fill, &s_tps_lbl, &s_tps_val,
               s_scr_main, 667, 170, 63, 100, "TPS", COL_TPS);
    create_bar(&s_batt_track, &s_batt_fill, &s_batt_lbl, &s_batt_val,
               s_scr_main, 734, 170, 63, 100, "BATT", COL_GOOD);

    /* indicator + high beam lamps (hidden until iobox3 latches them on) */
    s_ind_l = lv_obj_create(s_scr_main);
    lv_obj_remove_style_all(s_ind_l);
    lv_obj_set_size(s_ind_l, 56, 44);
    lv_obj_set_pos(s_ind_l, 84, 48);
    lv_obj_add_flag(s_ind_l, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_ind_l, lamp_draw_cb, LV_EVENT_DRAW_MAIN, (void *)(intptr_t)0);

    s_ind_r = lv_obj_create(s_scr_main);
    lv_obj_remove_style_all(s_ind_r);
    lv_obj_set_size(s_ind_r, 56, 44);
    lv_obj_set_pos(s_ind_r, 660, 48);
    lv_obj_add_flag(s_ind_r, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_ind_r, lamp_draw_cb, LV_EVENT_DRAW_MAIN, (void *)(intptr_t)1);

    s_hb = lv_obj_create(s_scr_main);
    lv_obj_remove_style_all(s_hb);
    lv_obj_set_size(s_hb, 64, 26);
    lv_obj_set_pos(s_hb, GAUGE_CX + 160, 8);   /* top-right: bottom-centre is the mph readout */
    lv_obj_add_flag(s_hb, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(s_hb, lamp_draw_cb, LV_EVENT_DRAW_MAIN, (void *)(intptr_t)2);

    /* shift light strip (gauge center, above RPM title) */
    for (int i = 0; i < SHIFT_SEGS; i++) {
        s_shift_seg[i] = lv_obj_create(s_scr_main);
        lv_obj_remove_style_all(s_shift_seg[i]);
        lv_obj_set_size(s_shift_seg[i], 24, 10);
        lv_obj_set_pos(s_shift_seg[i], GAUGE_CX - (SHIFT_SEGS * 28 - 4) / 2 + i * 28, 240);
        lv_obj_set_style_bg_color(s_shift_seg[i], lv_color_hex(SHIFT_OFF_COLOR), 0);
        lv_obj_set_style_bg_opa(s_shift_seg[i], LV_OPA_COVER, 0);
        lv_obj_set_style_radius(s_shift_seg[i], 2, 0);
    }

    /* warning banner (above shift strip, hidden until a warn fires) */
    s_warn_box = lv_obj_create(s_scr_main);
    lv_obj_remove_style_all(s_warn_box);
    lv_obj_set_size(s_warn_box, 150, 20);
    lv_obj_set_pos(s_warn_box, GAUGE_CX - 75, 214);
    lv_obj_set_style_radius(s_warn_box, 8, 0);
    lv_obj_set_style_bg_opa(s_warn_box, LV_OPA_80, 0);
    lv_obj_set_style_border_width(s_warn_box, 0, 0);
    lv_obj_add_flag(s_warn_box, LV_OBJ_FLAG_HIDDEN);

    s_warn_lbl = lv_label_create(s_warn_box);
    lv_obj_center(s_warn_lbl);
    lv_obj_set_style_text_font(s_warn_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(s_warn_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(s_warn_lbl, "");

    /* big mph readout: iobox3 ABS speed (0xB0 byte 2). Stacked directly below
     * the RPM number, in the dial's bottom gap (classic tach odometer spot).
     * Draws on top of the needle by construction (created after it). */
    s_speed_val = lv_label_create(s_scr_main);
    lv_label_set_text(s_speed_val, "--");
    lv_obj_set_style_text_color(s_speed_val, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_style_text_font(s_speed_val, &lv_font_montserrat_30, 0);
    lv_obj_set_width(s_speed_val, 150);
    lv_obj_set_style_text_align(s_speed_val, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(s_speed_val, GAUGE_CX - 75, 350);

    s_speed_unit = lv_label_create(s_scr_main);
    lv_label_set_text(s_speed_unit, "MPH");
    lv_obj_set_style_text_color(s_speed_unit, lv_color_hex(COL_DIM), 0);
    lv_obj_set_style_text_font(s_speed_unit, &lv_font_montserrat_14, 0);
    lv_obj_set_width(s_speed_unit, 60);
    lv_obj_set_style_text_align(s_speed_unit, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_pos(s_speed_unit, GAUGE_CX - 30, 383);
    lv_obj_add_flag(s_speed_unit, LV_OBJ_FLAG_HIDDEN);

    /* status labels */
    s_idle_lbl = lv_label_create(s_scr_main);
    lv_label_set_text(s_idle_lbl, "IDLE AUTO");
    lv_obj_set_style_text_color(s_idle_lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_style_text_font(s_idle_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(s_idle_lbl, 16, 414);

    s_iac_lbl = lv_label_create(s_scr_main);
    lv_label_set_text(s_iac_lbl, "IAC --");
    lv_obj_set_style_text_color(s_iac_lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_style_text_font(s_iac_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(s_iac_lbl, 220, 414);

    s_fan_lbl = lv_label_create(s_scr_main);
    lv_label_set_text(s_fan_lbl, "FAN AUTO");
    lv_obj_set_style_text_color(s_fan_lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_style_text_font(s_fan_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(s_fan_lbl, 420, 414);

    /* iobox control buttons */
    static lv_style_t btn_style;
    lv_style_init(&btn_style);
    lv_style_set_bg_color(&btn_style, lv_color_hex(0x2A2A3A));
    lv_style_set_bg_opa(&btn_style, LV_OPA_COVER);
    lv_style_set_radius(&btn_style, 6);
    lv_style_set_pad_ver(&btn_style, 6);
    lv_style_set_pad_hor(&btn_style, 10);
    lv_style_set_text_font(&btn_style, &lv_font_montserrat_14);
    lv_style_set_text_color(&btn_style, lv_color_hex(COL_TEXT));

    static lv_style_t btn_style_on;
    lv_style_init(&btn_style_on);
    lv_style_set_bg_color(&btn_style_on, lv_color_hex(COL_GOOD));
    lv_style_set_bg_opa(&btn_style_on, LV_OPA_COVER);
    lv_style_set_radius(&btn_style_on, 6);
    lv_style_set_pad_ver(&btn_style_on, 6);
    lv_style_set_pad_hor(&btn_style_on, 10);
    lv_style_set_text_font(&btn_style_on, &lv_font_montserrat_14);
    lv_style_set_text_color(&btn_style_on, lv_color_hex(0x0A0A10));

    /* Fan controls row */
    s_main_fauto = lv_btn_create(s_scr_main);
    lv_obj_add_style(s_main_fauto, &btn_style, 0);
    lv_obj_set_size(s_main_fauto, 100, 40);
    lv_obj_set_pos(s_main_fauto, 10, 438);
    lv_obj_t *fan_auto_lbl = lv_label_create(s_main_fauto);
    lv_label_set_text(fan_auto_lbl, "FAN AUTO");
    lv_obj_center(fan_auto_lbl);
    lv_obj_add_event_cb(s_main_fauto, fan_auto_evt, LV_EVENT_CLICKED, NULL);

    s_main_fon = lv_btn_create(s_scr_main);
    lv_obj_add_style(s_main_fon, &btn_style, 0);
    lv_obj_set_size(s_main_fon, 80, 40);
    lv_obj_set_pos(s_main_fon, 120, 438);
    lv_obj_t *fan_on_lbl = lv_label_create(s_main_fon);
    lv_label_set_text(fan_on_lbl, "FAN ON");
    lv_obj_center(fan_on_lbl);
    lv_obj_add_event_cb(s_main_fon, fan_on_evt, LV_EVENT_CLICKED, NULL);

    s_main_foff = lv_btn_create(s_scr_main);
    lv_obj_add_style(s_main_foff, &btn_style, 0);
    lv_obj_set_size(s_main_foff, 80, 40);
    lv_obj_set_pos(s_main_foff, 210, 438);
    lv_obj_t *fan_off_lbl = lv_label_create(s_main_foff);
    lv_label_set_text(fan_off_lbl, "FAN OFF");
    lv_obj_center(fan_off_lbl);
    lv_obj_add_event_cb(s_main_foff, fan_off_evt, LV_EVENT_CLICKED, NULL);

    /* IAC controls row */
    s_main_iauto = lv_btn_create(s_scr_main);
    lv_obj_add_style(s_main_iauto, &btn_style, 0);
    lv_obj_set_size(s_main_iauto, 100, 40);
    lv_obj_set_pos(s_main_iauto, 310, 438);
    lv_obj_t *iac_auto_lbl = lv_label_create(s_main_iauto);
    lv_label_set_text(iac_auto_lbl, "IAC AUTO");
    lv_obj_center(iac_auto_lbl);
    lv_obj_add_event_cb(s_main_iauto, iac_auto_evt, LV_EVENT_CLICKED, NULL);

    s_main_ifollow = lv_btn_create(s_scr_main);
    lv_obj_add_style(s_main_ifollow, &btn_style, 0);
    lv_obj_set_size(s_main_ifollow, 100, 40);
    lv_obj_set_pos(s_main_ifollow, 420, 438);
    lv_obj_t *iac_follow_lbl = lv_label_create(s_main_ifollow);
    lv_label_set_text(iac_follow_lbl, "IAC FOLLOW");
    lv_obj_center(iac_follow_lbl);
    lv_obj_add_event_cb(s_main_ifollow, iac_follow_evt, LV_EVENT_CLICKED, NULL);

    s_main_iman = lv_btn_create(s_scr_main);
    lv_obj_add_style(s_main_iman, &btn_style, 0);
    lv_obj_set_size(s_main_iman, 80, 40);
    lv_obj_set_pos(s_main_iman, 530, 438);
    lv_obj_t *iac_man_lbl = lv_label_create(s_main_iman);
    lv_label_set_text(iac_man_lbl, "IAC MAN");
    lv_obj_center(iac_man_lbl);
    lv_obj_add_event_cb(s_main_iman, iac_man_evt, LV_EVENT_CLICKED, NULL);

    /* SETTINGS button (navigate to iobox3 settings page) */
    lv_obj_t *btn_set = lv_btn_create(s_scr_main);
    lv_obj_add_style(btn_set, &btn_style, 0);
    lv_obj_set_size(btn_set, 100, 40);
    lv_obj_set_pos(btn_set, 650, 438);
    lv_obj_t *l = lv_label_create(btn_set);
    lv_label_set_text(l, "SETTINGS");
    lv_obj_center(l);
    lv_obj_add_event_cb(btn_set, settings_show_evt, LV_EVENT_CLICKED, NULL);

    /* transparent tap-zone over the dial: tap to swap between the OEM and
     * classic faces. Sits on top (transparent, no paint) but only covers the
     * gauge, never the bottom button row. */
    lv_obj_t *face_zone = lv_obj_create(s_scr_main);
    lv_obj_remove_style_all(face_zone);
    lv_obj_set_size(face_zone, 420, 420);
    lv_obj_set_pos(face_zone, GAUGE_CX - 210, GAUGE_CY - 210);
    lv_obj_add_flag(face_zone, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(face_zone, face_toggle_evt, LV_EVENT_CLICKED, NULL);

    /* moving parts (ring/star/needle) now exist — apply live visibility final */
    ui_face_apply_live();

    ESP_LOGI(TAG, "Main UI built (not shown)");
    memset(&s_prev, 0, sizeof(s_prev));
}

/* Show/hide the moving parts matching s_face_active:
 *  - CLASSIC / OEM use the star cursor + load-up ring (+ glow halo)
 *  - NEEDLE uses the needle line + hub (no star/ring) */
static void ui_face_apply_live(void) {
    bool needleFace = (s_face_active == FACE_NEEDLE);
    if (s_needle) {
        if (needleFace) lv_obj_clear_flag(s_needle, LV_OBJ_FLAG_HIDDEN);
        else            lv_obj_add_flag(s_needle, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_needle_hub) {
        if (needleFace) lv_obj_clear_flag(s_needle_hub, LV_OBJ_FLAG_HIDDEN);
        else            lv_obj_add_flag(s_needle_hub, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_mark) {
        if (needleFace) lv_obj_add_flag(s_mark, LV_OBJ_FLAG_HIDDEN);
        else            lv_obj_clear_flag(s_mark, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_mark_glow) {
        if (needleFace) lv_obj_add_flag(s_mark_glow, LV_OBJ_FLAG_HIDDEN);
        else            lv_obj_clear_flag(s_mark_glow, LV_OBJ_FLAG_HIDDEN);
    }
    lv_obj_t *rings[3] = { s_ring_g, s_ring_a, s_ring_r };
    for (int i = 0; i < 3; i++) {
        if (!rings[i]) continue;
        if (needleFace) lv_obj_add_flag(rings[i], LV_OBJ_FLAG_HIDDEN);
        else            lv_obj_clear_flag(rings[i], LV_OBJ_FLAG_HIDDEN);
    }
}

/* Tap the dial: advance to the NEXT face whose s_face_state != -1 (wrap).
 * If only the current face is enabled (or none), it stays put. */
static void face_toggle_evt(lv_event_t *e) {
    (void)e;
    if (s_face_active >= 0 && s_face_active < FACE_COUNT) {
        int nxt = s_face_active;
        for (int i = 1; i <= FACE_COUNT; i++) {
            int f = (s_face_active + i) % FACE_COUNT;
            if (s_face_state[f] != -1) { nxt = f; break; }
        }
        if (nxt != s_face_active) {
            s_face_active = nxt;
            lv_obj_t *faces[FACE_COUNT] = { s_face_classic, s_face_oem, s_face_needle };
            for (int i = 0; i < FACE_COUNT; i++)
                if (faces[i]) lv_obj_add_flag(faces[i], LV_OBJ_FLAG_HIDDEN);
            if (faces[s_face_active])
                lv_obj_clear_flag(faces[s_face_active], LV_OBJ_FLAG_HIDDEN);
            /* Repaint the now-shown face if its colour is stale (fixed colour
             * changed while hidden), plus restyle the needle for the new face. */
            face_paint(s_face_active, false);
            if (s_face_active == FACE_NEEDLE && s_needle) {
                lv_obj_set_style_line_color(s_needle, lv_color_hex(face_colour(FACE_NEEDLE)), 0);
                lv_obj_set_style_bg_color(s_needle_hub, lv_color_hex(face_colour(FACE_NEEDLE)), 0);
                lv_obj_set_style_border_color(s_needle_hub, lv_color_hex(face_colour(FACE_NEEDLE)), 0);
            }
            ui_face_apply_live();
        }
    }
    /* Do NOT flash-write from this LVGL event handler — the NVS commit's
     * cache-freeze path overflows the LVGL task stack (assert). Just flag it;
     * a dedicated task (ui_face_persist_once) does the actual NVS write. */
    s_face_dirty = true;
}

/* Called from a dedicated (non-LVGL) task so the NVS flash commit runs on a
 * real stack. Flash write outside the touch/render context is safe. */
void ui_face_persist_once(void)
{
    if (!s_face_dirty && !s_theme_dirty) return;
    nvs_handle_t h;
    if (nvs_open("dashui", NVS_READWRITE, &h) == ESP_OK) {
        if (s_face_dirty) {
            nvs_set_i32(h, "face", s_face_active);
            for (int i = 0; i < FACE_COUNT; i++) {
                char key[8];
                snprintf(key, sizeof(key), "fst%d", i);
                nvs_set_i32(h, key, (int32_t)s_face_state[i]);
            }
            s_face_dirty = false;
        }
        if (s_theme_dirty) {
            nvs_set_i32(h, "accent", (int32_t)s_accent);
            nvs_set_i32(h, "night",  s_night ? 1 : 0);
            s_theme_dirty = false;
        }
        nvs_commit(h);
        nvs_close(h);
    }
}

void ui_theme_apply_once(void)
{
    if (s_theme_apply_pending) {
        s_theme_apply_pending = false;
        ui_theme_apply();
    }
}


/* Mode buttons send the command and let the iobox3 echo its new state back
 * over the link; the highlight is then driven by that echo in ui_update().
 * We deliberately DO NOT highlight locally on press — if the command is lost
 * or the link drops, a local highlight would stick on the pressed (wrong)
 * mode forever because the diff-guard in ui_update() can't see a change to
 * correct it. Box-echoed state is the single source of truth. */
static void fan_auto_evt(lv_event_t *e) {
    (void)e;
    can_tx_fan_auto(true);
}

static void fan_on_evt(lv_event_t *e) {
    (void)e;
    can_tx_fan_manual(true);
}

static void fan_off_evt(lv_event_t *e) {
    (void)e;
    can_tx_fan_manual(false);
}

static void iac_auto_evt(lv_event_t *e) {
    (void)e;
    can_tx_iac_auto(true);
}

static void iac_follow_evt(lv_event_t *e) {
    (void)e;
    can_tx_iac_follow(true);
}

static void iac_man_evt(lv_event_t *e) {
    (void)e;
    can_tx_iac_manual(50);  /* default 50% */
}

static void ui_init_main_show(void) {
    ESP_LOGI(TAG, "Showing main UI");
    /* auto_del frees the boot screen after the transition (was leaking) */
    lv_scr_load_anim(s_scr_main, LV_SCR_LOAD_ANIM_FADE_IN, 300, 0, true);
}

/* ============================================================================
 * Settings page (iobox3 control)
 * ============================================================================ */

static int16_t  s_shift_rpm_val  = SHIFT_RPM_DEFAULT;
static int8_t   s_iac_duty_val   = 50;
static bool     s_buzz_on        = false;
static bool     s_boottest_on    = false;
static int16_t  s_iac_target_val = 900;    /* rpm */
static lv_obj_t *s_shift_val_lbl, *s_duty_val_lbl, *s_buzz_lbl, *s_boot_lbl;
static lv_obj_t *s_buzz_btn, *s_boot_btn;
static lv_obj_t *s_gas_lbl, *s_gdamp_val_lbl, *s_glow_val_lbl;
static lv_timer_t *s_gas_timer;
static int16_t s_gdamp_val = 0;   /* mirror of box gasDamp */
static int16_t s_glow_val  = 20;  /* mirror of box lowFuelPct */
static lv_obj_t *s_tgt_val_lbl;

/* NVS persistence for dash-side settings ("dashui" namespace).
 * app_main calls nvs_flash_init() before ui_init(), so this is safe. */
static void settings_nvs_load(void) {
    nvs_handle_t h;
    if (nvs_open("dashui", NVS_READONLY, &h) == ESP_OK) {
        int32_t v = 0;
        if (nvs_get_i32(h, "shift_rpm", &v) == ESP_OK && v >= 4000 && v <= 9000)
            s_shift_rpm_cfg = v;
        if (nvs_get_i32(h, "w_clt", &v) == ESP_OK && v >= 1000 && v <= 2600)
            s_warn_clt_max = v;
        if (nvs_get_i32(h, "w_mat", &v) == ESP_OK && v >= 500 && v <= 3000)
            s_warn_mat_max = v;
        if (nvs_get_i32(h, "w_battlo", &v) == ESP_OK && v >= 80 && v <= 150)
            s_warn_batt_min = v;
        if (nvs_get_i32(h, "w_batthi", &v) == ESP_OK && v >= 120 && v <= 200)
            s_warn_batt_max = v;
        if (nvs_get_i32(h, "w_map", &v) == ESP_OK && v >= 1500 && v <= 4000)
            s_warn_map_max = v;
        if (nvs_get_i32(h, "w_afrlo", &v) == ESP_OK && v >= 80 && v <= 140)
            s_warn_afr_low = v;
        if (nvs_get_i32(h, "w_afrhi", &v) == ESP_OK && v >= 120 && v <= 220)
            s_warn_afr_high = v;
        if (nvs_get_i32(h, "face", &v) == ESP_OK && v >= 0 && v < FACE_COUNT)
            s_face_active = v;
        for (int i = 0; i < FACE_COUNT; i++) {
            char key[8];
            snprintf(key, sizeof(key), "fst%d", i);
            if (nvs_get_i32(h, key, &v) == ESP_OK && v >= -1 && v <= THEME_COUNT)
                s_face_state[i] = (int8_t)v;
        }
        /* Never boot onto an OFF face — move to the first enabled one. */
        if (s_face_state[s_face_active] == -1) {
            for (int i = 0; i < FACE_COUNT; i++)
                if (s_face_state[i] != -1) { s_face_active = i; break; }
            s_face_dirty = true;
        }
        if (nvs_get_i32(h, "accent", &v) == ESP_OK)
            for (int i = 0; i < THEME_COUNT; i++)
                if ((uint32_t)v == s_theme_colors[i]) { s_accent = (uint32_t)v; break; }
        if (nvs_get_i32(h, "night", &v) == ESP_OK && v >= 0 && v <= 1)
            s_night = (v == 1);
        nvs_close(h);
    }
}

static void settings_nvs_save_shift(void) {
    nvs_handle_t h;
    if (nvs_open("dashui", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i32(h, "shift_rpm", s_shift_rpm_cfg);
        nvs_commit(h);
        nvs_close(h);
    }
}

static void settings_back_evt(lv_event_t *e) {
    (void)e;
    lv_scr_load_anim(s_scr_main, LV_SCR_LOAD_ANIM_FADE_IN, 300, 0, true);
}

/* auto_del on the back animation frees the settings screen (and all its child
 * widgets). Clear EVERY pointer into that object exactly when the deletion
 * happens, or the next ui_update() dereferences freed memory: the fan/IAC
 * mode highlights call update_*_mode_buttons() every time the box reports a
 * mode change, and those touch the settings child buttons directly. Clearing
 * only s_scr_set left these dangling -> use-after-free on the frame after the
 * 300ms fade completes. Rebuilt lazily next ui_show_settings(). */
static void scr_set_delete_evt(lv_event_t *e) {
    (void)e;
    s_scr_set = NULL;
    if (s_gas_timer) { lv_timer_del(s_gas_timer); s_gas_timer = NULL; }

    s_btn_fauto = NULL; s_btn_fon = NULL; s_btn_foff = NULL;
    s_btn_iauto = NULL; s_btn_ifollow = NULL; s_btn_iman = NULL;
    s_buzz_btn = NULL; s_boot_btn = NULL;
    s_buzz_lbl = NULL; s_boot_lbl = NULL;
    s_shift_val_lbl = NULL; s_duty_val_lbl = NULL;
    s_gdamp_val_lbl = NULL; s_glow_val_lbl = NULL;
    s_gas_lbl = NULL;
    s_tgt_val_lbl = NULL;
    for (int i = 0; i < THEME_COUNT; i++) s_btn_theme[i] = NULL;
    for (int i = 0; i < FACE_COUNT; i++) { s_btn_face[i] = NULL; s_face_lbl[i] = NULL; }
    s_night_btn = NULL; s_night_lbl = NULL;
}

/* Fuel calibration — sends Q <slot> to iobox3 over the link; box records its
 * current A4 node reading into the 5-point table (E,1/4,HALF,3/4,F) and
 * persists to NVS. Damp/low-fuel steppers mirror Q D / Q W. */
static const char *const kGasSlots[5] = { "E", "1", "2", "3", "F" };

static void settings_gas_set_evt(lv_event_t *e) {
    uintptr_t idx = (uintptr_t)lv_event_get_user_data(e);
    if (idx < 5) can_tx_gas_record(kGasSlots[idx]);
}

/* Accent theme picker + night toggle (dash-side, persisted to NVS). */
static void theme_highlight(void);

static void theme_evt(lv_event_t *e) {
    uint32_t c = (uint32_t)(uintptr_t)lv_event_get_user_data(e);
    if (c == s_accent) return;
    s_accent = c;
    s_theme_dirty = true;
    s_theme_apply_pending = true;
    theme_highlight();
}

static void night_evt(lv_event_t *e) {
    (void)e;
    s_night = !s_night;
    s_theme_dirty = true;
    s_theme_apply_pending = true;
    theme_highlight();
}

/* Repaint the 4 theme swatches + night button to reflect s_accent/s_night.
 * Called from the evt handlers and on settings page show. */
static void theme_highlight(void) {
    for (int i = 0; i < THEME_COUNT; i++) {
        if (!s_btn_theme[i]) continue;
        bool act = (s_accent == s_theme_colors[i]);
        uint32_t c = s_theme_colors[i];
        lv_obj_set_style_bg_color(s_btn_theme[i],
            lv_color_hex(act ? c : ((c >> 2) & 0x3F3F3F)), 0);
        lv_obj_set_style_border_width(s_btn_theme[i], act ? 2 : 0, 0);
        lv_obj_set_style_border_color(s_btn_theme[i], lv_color_hex(COL_TEXT), 0);
    }
    if (s_night_btn && s_night_lbl) {
        lv_obj_set_style_bg_color(s_night_btn,
            lv_color_hex(s_night ? COL_GOOD : 0x2A2A3A), 0);
        lv_label_set_text(s_night_lbl, s_night ? "NIGHT ON" : "NIGHT OFF");
    }
}

static const char *kThemeNames[THEME_COUNT] = { "GRN", "BLU", "PUR", "AMB" };

static void face_highlight(void);

/* Per-face state button: cycles OFF -> AUTO -> GRN/BLU/PUR/AMB -> OFF.
 * Taps only change the NEXT face (so the active face never disappears); if the
 * face being toggled IS the active one, only its colour changes, never to OFF. */
static void face_cycle_evt(lv_event_t *e) {
    int f = (int)(intptr_t)lv_event_get_user_data(e);
    if (f < 0 || f >= FACE_COUNT) return;
    int8_t st = s_face_state[f];
    if (f == s_face_active) {
        /* active face: never go OFF, just advance colour (AUTO -> 1 -> ... -> n -> AUTO) */
        st++;
        if (st > THEME_COUNT) st = 0;
    } else {
        /* inactive face: OFF -> AUTO -> colours -> OFF */
        st++;
        if (st > THEME_COUNT) st = -1;
    }
    s_face_state[f] = st;
    s_face_dirty = true;
    if (f == s_face_active) {
        face_paint(f, true);   /* active: re-bake now (also frees old buffer) */
        if (f == FACE_NEEDLE && s_needle) {
            lv_obj_set_style_line_color(s_needle, lv_color_hex(face_colour(FACE_NEEDLE)), 0);
            lv_obj_set_style_bg_color(s_needle_hub, lv_color_hex(face_colour(FACE_NEEDLE)), 0);
            lv_obj_set_style_border_color(s_needle_hub, lv_color_hex(face_colour(FACE_NEEDLE)), 0);
        }
    } else {
        s_face_baked[f] = 0xFFFFFFFF;   /* stale: repainted on next toggle to it */
    }
    face_highlight();
}

/* Reflect s_face_state[] onto the 3 face buttons. */
static void face_highlight(void) {
    for (int i = 0; i < FACE_COUNT; i++) {
        lv_obj_t *b = s_btn_face[i];
        if (!b) continue;
        int8_t st = s_face_state[i];
        uint32_t col;
        const char *txt;
        if (st <= 0) {
            col = 0x2A2A3A;
            txt = (st == 0) ? "AUTO" : "OFF";
        } else {
            col = s_theme_colors[st - 1];
            txt = kThemeNames[st - 1];
        }
        lv_obj_set_style_bg_color(b, lv_color_hex(col), 0);
        lv_label_set_text(s_face_lbl[i], txt);
        lv_obj_set_style_border_width(b, i == s_face_active ? 2 : 0, 0);
        lv_obj_set_style_border_color(b, lv_color_hex(COL_TEXT), 0);
    }
}


static void settings_gdamp_evt(lv_event_t *e) {
    int d = (int)(intptr_t)lv_event_get_user_data(e);
    int v = s_gdamp_val + d;
    if (v < 0) v = 0;
    if (v > 15) v = 15;
    s_gdamp_val = (int16_t)v;
    char buf[12];
    lv_snprintf(buf, sizeof(buf), "%d", s_gdamp_val);
    lv_label_set_text(s_gdamp_val_lbl, buf);
    char cmd[10];
    lv_snprintf(cmd, sizeof(cmd), "D %d", s_gdamp_val);
    can_tx_send_cmd('Q', cmd);
}

static void settings_glow_evt(lv_event_t *e) {
    int d = (int)(intptr_t)lv_event_get_user_data(e);
    int v = s_glow_val + d;
    if (v < 5) v = 5;
    if (v > 90) v = 90;
    s_glow_val = (int16_t)v;
    char buf[12];
    lv_snprintf(buf, sizeof(buf), "%d%%", s_glow_val);
    lv_label_set_text(s_glow_val_lbl, buf);
    char cmd[10];
    lv_snprintf(cmd, sizeof(cmd), "W %d", s_glow_val);
    can_tx_send_cmd('Q', cmd);
}

static void gas_timer_cb(lv_timer_t *t) {
    (void)t;
    if (!s_gas_lbl) return;
    uint8_t g = can_rx_get_gas();
    uint16_t mv = can_rx_get_a4mv();
    char buf[28];
    if (g <= 100 && mv != 0xFFFF)
        lv_snprintf(buf, sizeof(buf), "GAS %u%%  %umV", g, mv);
    else if (g <= 100)
        lv_snprintf(buf, sizeof(buf), "GAS %u%%", g);
    else
        lv_snprintf(buf, sizeof(buf), "GAS --%");
    lv_label_set_text(s_gas_lbl, buf);
}

static void settings_iac_target_step(int16_t d) {
    int v = s_iac_target_val + d;
    if (v < 500) v = 500;
    if (v > 1500) v = 1500;
    s_iac_target_val = (int16_t)v;
    can_tx_iac_target_rpm(s_iac_target_val);
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d", s_iac_target_val);
    lv_label_set_text(s_tgt_val_lbl, buf);
}
static void settings_iac_target_minus_evt(lv_event_t *e) { (void)e; settings_iac_target_step(-100); }
static void settings_iac_target_plus_evt(lv_event_t *e)  { (void)e; settings_iac_target_step(100); }

static void settings_shift_step(int16_t d) {
    s_shift_rpm_val += d;
    if (s_shift_rpm_val < 4000) s_shift_rpm_val = 4000;
    if (s_shift_rpm_val > 9000) s_shift_rpm_val = 9000;
    s_shift_rpm_cfg = s_shift_rpm_val;      /* drive the on-screen strip too */
    settings_nvs_save_shift();
    can_tx_shift_rpm(s_shift_rpm_val);      /* keep iobox O1 threshold in step */
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d", s_shift_rpm_val);
    lv_label_set_text(s_shift_val_lbl, buf);
}
static void settings_shift_minus_evt(lv_event_t *e) { (void)e; settings_shift_step(-250); }
static void settings_shift_plus_evt(lv_event_t *e)  { (void)e; settings_shift_step(250); }

static void settings_duty_step(int8_t d) {
    int v = s_iac_duty_val + d;
    if (v < 0) v = 0;
    if (v > 100) v = 100;
    s_iac_duty_val = (int8_t)v;
    can_tx_iac_manual((uint8_t)s_iac_duty_val);
    char buf[16];
    lv_snprintf(buf, sizeof(buf), "%d%%", s_iac_duty_val);
    lv_label_set_text(s_duty_val_lbl, buf);
}
static void settings_duty_minus_evt(lv_event_t *e) { (void)e; settings_duty_step(-5); }
static void settings_duty_plus_evt(lv_event_t *e)  { (void)e; settings_duty_step(5); }

static void settings_buzz_evt(lv_event_t *e) {
    (void)e;
    /* Send the opposite of the iobox3's ACTUAL state (box echo is truth), not
     * a stale local flag — so the toggle always flips the real mode. No local
     * highlight here; ui_update() re-highlights from the box echo. */
    bool want = !can_rx_get_buzzer_on();
    s_buzz_on = want;
    can_tx_buzzer(want);
}

static void settings_beep_evt(lv_event_t *e) { (void)e; can_tx_buzzer_test(); }

static void settings_boot_evt(lv_event_t *e) {
    (void)e;
    bool want = !can_rx_get_boot_test();
    s_boottest_on = want;
    can_tx_boottest(want);
}

static void settings_show_evt(lv_event_t *e) {
    (void)e;
    ui_show_settings();
}

static void build_settings(void) {
    s_shift_rpm_val = (int16_t)s_shift_rpm_cfg;   /* reflect persisted value on rebuild */
    /* Sync buzzer/boot-test to the iobox3's actual state (box echo is truth);
     * the settings page must not pretend they're ON just because the local
     * default is ON — the box defaults OFF and stays quiet. */
    s_buzz_on      = can_rx_get_buzzer_on()  ? true : false;
    s_boottest_on  = can_rx_get_boot_test()  ? true : false;
    s_scr_set = lv_obj_create(NULL);
    lv_obj_remove_style_all(s_scr_set);
    lv_obj_set_style_bg_color(s_scr_set, lv_color_hex(COL_BG), 0);
    lv_obj_add_event_cb(s_scr_set, scr_set_delete_evt, LV_EVENT_DELETE, NULL);
    lv_obj_set_style_bg_opa(s_scr_set, LV_OPA_COVER, 0);

    lv_obj_t *title = lv_label_create(s_scr_set);
    lv_label_set_text(title, "SETTINGS  ->  I/O BOX");
    lv_obj_set_style_text_color(title, lv_color_hex(COL_MAG), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_set_pos(title, 10, 8);

    int y = 50;
    char buf[16];
    static lv_style_t btn_style;
    lv_style_init(&btn_style);
    lv_style_set_bg_color(&btn_style, lv_color_hex(0x2A2A3A));
    lv_style_set_bg_opa(&btn_style, LV_OPA_COVER);
    lv_style_set_radius(&btn_style, 6);
    lv_style_set_pad_ver(&btn_style, 6);
    lv_style_set_pad_hor(&btn_style, 12);
    lv_style_set_text_font(&btn_style, &lv_font_montserrat_14);
    lv_style_set_text_color(&btn_style, lv_color_hex(COL_TEXT));

    /* Fan Mode */
    lv_obj_t *lbl = lv_label_create(s_scr_set);
    lv_label_set_text(lbl, "FAN MODE");
    lv_obj_set_style_text_color(lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_pos(lbl, 10, y);

    s_btn_fauto = lv_btn_create(s_scr_set);
    lv_obj_add_style(s_btn_fauto, &btn_style, 0);
    lv_obj_set_size(s_btn_fauto, 100, 40);
    lv_obj_set_pos(s_btn_fauto, 10, y + 24);
    lv_obj_t *l = lv_label_create(s_btn_fauto);
    lv_label_set_text(l, "FAN AUTO");
    lv_obj_center(l);
    lv_obj_add_event_cb(s_btn_fauto, fan_auto_evt, LV_EVENT_CLICKED, NULL);

    s_btn_fon = lv_btn_create(s_scr_set);
    lv_obj_add_style(s_btn_fon, &btn_style, 0);
    lv_obj_set_size(s_btn_fon, 80, 40);
    lv_obj_set_pos(s_btn_fon, 120, y + 24);
    l = lv_label_create(s_btn_fon);
    lv_label_set_text(l, "FAN ON");
    lv_obj_center(l);
    lv_obj_add_event_cb(s_btn_fon, fan_on_evt, LV_EVENT_CLICKED, NULL);

    s_btn_foff = lv_btn_create(s_scr_set);
    lv_obj_add_style(s_btn_foff, &btn_style, 0);
    lv_obj_set_size(s_btn_foff, 80, 40);
    lv_obj_set_pos(s_btn_foff, 210, y + 24);
    l = lv_label_create(s_btn_foff);
    lv_label_set_text(l, "FAN OFF");
    lv_obj_center(l);
    lv_obj_add_event_cb(s_btn_foff, fan_off_evt, LV_EVENT_CLICKED, NULL);

    y += 80;

    /* IAC Mode */
    lbl = lv_label_create(s_scr_set);
    lv_label_set_text(lbl, "IAC MODE");
    lv_obj_set_style_text_color(lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_pos(lbl, 10, y);

    s_btn_iauto = lv_btn_create(s_scr_set);
    lv_obj_add_style(s_btn_iauto, &btn_style, 0);
    lv_obj_set_size(s_btn_iauto, 100, 40);
    lv_obj_set_pos(s_btn_iauto, 10, y + 24);
    l = lv_label_create(s_btn_iauto);
    lv_label_set_text(l, "IAC AUTO");
    lv_obj_center(l);
    lv_obj_add_event_cb(s_btn_iauto, iac_auto_evt, LV_EVENT_CLICKED, NULL);

    s_btn_ifollow = lv_btn_create(s_scr_set);
    lv_obj_add_style(s_btn_ifollow, &btn_style, 0);
    lv_obj_set_size(s_btn_ifollow, 100, 40);
    lv_obj_set_pos(s_btn_ifollow, 120, y + 24);
    l = lv_label_create(s_btn_ifollow);
    lv_label_set_text(l, "IAC FOLLOW");
    lv_obj_center(l);
    lv_obj_add_event_cb(s_btn_ifollow, iac_follow_evt, LV_EVENT_CLICKED, NULL);

    s_btn_iman = lv_btn_create(s_scr_set);
    lv_obj_add_style(s_btn_iman, &btn_style, 0);
    lv_obj_set_size(s_btn_iman, 80, 40);
    lv_obj_set_pos(s_btn_iman, 230, y + 24);
    l = lv_label_create(s_btn_iman);
    lv_label_set_text(l, "IAC MAN");
    lv_obj_center(l);
    lv_obj_add_event_cb(s_btn_iman, iac_man_evt, LV_EVENT_CLICKED, NULL);

    y += 80;

    /* Idle / Fan Settings */
    lbl = lv_label_create(s_scr_set);
    lv_label_set_text(lbl, "IDLE TARGET RPM");
    lv_obj_set_style_text_color(lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_pos(lbl, 10, y);

    /* IAC target RPM step control */
    lv_obj_t *btn_minus = lv_btn_create(s_scr_set);
    lv_obj_add_style(btn_minus, &btn_style, 0);
    lv_obj_set_size(btn_minus, 50, 40);
    lv_obj_set_pos(btn_minus, 10, y + 24);
    l = lv_label_create(btn_minus);
    lv_label_set_text(l, "-100");
    lv_obj_center(l);
    lv_obj_add_event_cb(btn_minus, settings_iac_target_minus_evt, LV_EVENT_CLICKED, NULL);

    lv_obj_t *btn_plus = lv_btn_create(s_scr_set);
    lv_obj_add_style(btn_plus, &btn_style, 0);
    lv_obj_set_size(btn_plus, 50, 40);
    lv_obj_set_pos(btn_plus, 70, y + 24);
    l = lv_label_create(btn_plus);
    lv_label_set_text(l, "+100");
    lv_obj_center(l);
    lv_obj_add_event_cb(btn_plus, settings_iac_target_plus_evt, LV_EVENT_CLICKED, NULL);

    s_tgt_val_lbl = lv_label_create(s_scr_set);
    lv_snprintf(buf, sizeof(buf), "%d", s_iac_target_val);
    lv_label_set_text(s_tgt_val_lbl, buf);
    lv_obj_set_style_text_color(s_tgt_val_lbl, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_pos(s_tgt_val_lbl, 135, y + 34);

    /* Gauge-face config: one button per face. Each cycles its own state:
     * OFF / AUTO (follow accent) / fixed accent colour. The ACTIVE face can
     * never be turned OFF from here (tap the dial to cycle faces). */
    lbl = lv_label_create(s_scr_set);
    lv_label_set_text(lbl, "GAUGE FACE (tap dial to switch)");
    lv_obj_set_style_text_color(lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_pos(lbl, 10, 290);
    for (int i = 0; i < FACE_COUNT; i++) {
        s_btn_face[i] = lv_btn_create(s_scr_set);
        lv_obj_add_style(s_btn_face[i], &btn_style, 0);
        lv_obj_set_size(s_btn_face[i], 64, 36);
        lv_obj_set_pos(s_btn_face[i], 10 + i * 70, 314);
        lv_obj_add_event_cb(s_btn_face[i], face_cycle_evt, LV_EVENT_CLICKED,
                            (void *)(intptr_t)i);
        s_face_lbl[i] = lv_label_create(s_btn_face[i]);
        lv_label_set_text(s_face_lbl[i], s_face_names[i]);
        lv_obj_center(s_face_lbl[i]);
    }
    face_highlight();

    /* Accent theme picker + night toggle (left column bottom, below IAC) */
    lbl = lv_label_create(s_scr_set);
    lv_label_set_text(lbl, "ACCENT");
    lv_obj_set_style_text_color(lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_pos(lbl, 10, 426);
    for (int i = 0; i < THEME_COUNT; i++) {
        s_btn_theme[i] = lv_btn_create(s_scr_set);
        lv_obj_add_style(s_btn_theme[i], &btn_style, 0);
        lv_obj_set_size(s_btn_theme[i], 44, 28);
        lv_obj_set_pos(s_btn_theme[i], 10 + i * 48, 448);
        lv_obj_add_event_cb(s_btn_theme[i], theme_evt, LV_EVENT_CLICKED,
                            (void *)(uintptr_t)s_theme_colors[i]);
    }
    s_night_btn = lv_btn_create(s_scr_set);
    lv_obj_add_style(s_night_btn, &btn_style, 0);
    lv_obj_set_size(s_night_btn, 100, 28);
    lv_obj_set_pos(s_night_btn, 10 + THEME_COUNT * 48 + 16, 448);
    s_night_lbl = lv_label_create(s_night_btn);
    lv_label_set_text(s_night_lbl, s_night ? "NIGHT ON" : "NIGHT OFF");
    lv_obj_center(s_night_lbl);
    lv_obj_add_event_cb(s_night_btn, night_evt, LV_EVENT_CLICKED, NULL);
    theme_highlight();

    /* ---- right column: extra iobox controls ---- */
    const int rx = 430;

    /* Shift light RPM */
    lbl = lv_label_create(s_scr_set);
    lv_label_set_text(lbl, "SHIFT LIGHT RPM");
    lv_obj_set_style_text_color(lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_pos(lbl, rx, 50);

    lv_obj_t *sh_minus = lv_btn_create(s_scr_set);
    lv_obj_add_style(sh_minus, &btn_style, 0);
    lv_obj_set_size(sh_minus, 60, 40);
    lv_obj_set_pos(sh_minus, rx, 74);
    l = lv_label_create(sh_minus);
    lv_label_set_text(l, "-250");
    lv_obj_center(l);
    lv_obj_add_event_cb(sh_minus, settings_shift_minus_evt, LV_EVENT_CLICKED, NULL);

    lv_obj_t *sh_plus = lv_btn_create(s_scr_set);
    lv_obj_add_style(sh_plus, &btn_style, 0);
    lv_obj_set_size(sh_plus, 60, 40);
    lv_obj_set_pos(sh_plus, rx + 66, 74);
    l = lv_label_create(sh_plus);
    lv_label_set_text(l, "+250");
    lv_obj_center(l);
    lv_obj_add_event_cb(sh_plus, settings_shift_plus_evt, LV_EVENT_CLICKED, NULL);

    s_shift_val_lbl = lv_label_create(s_scr_set);
    lv_snprintf(buf, sizeof(buf), "%d", s_shift_rpm_val);
    lv_label_set_text(s_shift_val_lbl, buf);
    lv_obj_set_style_text_color(s_shift_val_lbl, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_pos(s_shift_val_lbl, rx + 140, 84);

    /* IAC manual duty */
    lbl = lv_label_create(s_scr_set);
    lv_label_set_text(lbl, "IAC DUTY (MAN)");
    lv_obj_set_style_text_color(lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_pos(lbl, rx, 130);

    lv_obj_t *du_minus = lv_btn_create(s_scr_set);
    lv_obj_add_style(du_minus, &btn_style, 0);
    lv_obj_set_size(du_minus, 60, 40);
    lv_obj_set_pos(du_minus, rx, 154);
    l = lv_label_create(du_minus);
    lv_label_set_text(l, "-5");
    lv_obj_center(l);
    lv_obj_add_event_cb(du_minus, settings_duty_minus_evt, LV_EVENT_CLICKED, NULL);

    lv_obj_t *du_plus = lv_btn_create(s_scr_set);
    lv_obj_add_style(du_plus, &btn_style, 0);
    lv_obj_set_size(du_plus, 60, 40);
    lv_obj_set_pos(du_plus, rx + 66, 154);
    l = lv_label_create(du_plus);
    lv_label_set_text(l, "+5");
    lv_obj_center(l);
    lv_obj_add_event_cb(du_plus, settings_duty_plus_evt, LV_EVENT_CLICKED, NULL);

    s_duty_val_lbl = lv_label_create(s_scr_set);
    lv_snprintf(buf, sizeof(buf), "%d%%", s_iac_duty_val);
    lv_label_set_text(s_duty_val_lbl, buf);
    lv_obj_set_style_text_color(s_duty_val_lbl, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_pos(s_duty_val_lbl, rx + 140, 164);

    /* Buzzer */
    lbl = lv_label_create(s_scr_set);
    lv_label_set_text(lbl, "WARNING BUZZER");
    lv_obj_set_style_text_color(lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_pos(lbl, rx, 210);

    s_buzz_btn = lv_btn_create(s_scr_set);
    lv_obj_add_style(s_buzz_btn, &btn_style, 0);
    lv_obj_set_size(s_buzz_btn, 120, 40);
    lv_obj_set_pos(s_buzz_btn, rx, 234);
    s_buzz_lbl = lv_label_create(s_buzz_btn);
    lv_label_set_text(s_buzz_lbl, s_buzz_on ? "BUZZER ON" : "BUZZER OFF");
    lv_obj_center(s_buzz_lbl);
    lv_obj_add_event_cb(s_buzz_btn, settings_buzz_evt, LV_EVENT_CLICKED, NULL);

    lv_obj_t *beep_btn = lv_btn_create(s_scr_set);
    lv_obj_add_style(beep_btn, &btn_style, 0);
    lv_obj_set_size(beep_btn, 70, 40);
    lv_obj_set_pos(beep_btn, rx + 126, 234);
    l = lv_label_create(beep_btn);
    lv_label_set_text(l, "BEEP");
    lv_obj_center(l);
    lv_obj_add_event_cb(beep_btn, settings_beep_evt, LV_EVENT_CLICKED, NULL);

    /* Boot self-test */
    lbl = lv_label_create(s_scr_set);
    lv_label_set_text(lbl, "BOOT SELF-TEST");
    lv_obj_set_style_text_color(lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_pos(lbl, rx, 290);

    s_boot_btn = lv_btn_create(s_scr_set);
    lv_obj_add_style(s_boot_btn, &btn_style, 0);
    lv_obj_set_size(s_boot_btn, 130, 40);
    lv_obj_set_pos(s_boot_btn, rx, 314);
    s_boot_lbl = lv_label_create(s_boot_btn);
    lv_label_set_text(s_boot_lbl, s_boottest_on ? "BOOT TEST ON" : "BOOT TEST OFF");
    lv_obj_center(s_boot_lbl);
    lv_obj_add_event_cb(s_boot_btn, settings_boot_evt, LV_EVENT_CLICKED, NULL);

    /* Fuel gauge calibration + tuning (right column, below boot self-test) */
    lbl = lv_label_create(s_scr_set);
    lv_label_set_text(lbl, "FUEL GAUGE");
    lv_obj_set_style_text_color(lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_pos(lbl, rx, 360);

    s_gas_lbl = lv_label_create(s_scr_set);
    lv_label_set_text(s_gas_lbl, "GAS --%");
    lv_obj_set_style_text_color(s_gas_lbl, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_pos(s_gas_lbl, rx + 130, 360);
    s_gas_timer = lv_timer_create(gas_timer_cb, 500, NULL);

    static const char *gas_lbls[5] = { "SET E", "1/4", "1/2", "3/4", "SET F" };
    for (int i = 0; i < 5; i++) {
        lv_obj_t *b = lv_btn_create(s_scr_set);
        lv_obj_add_style(b, &btn_style, 0);
        lv_obj_set_size(b, 66, 34);
        lv_obj_set_pos(b, rx + i * 70, 386);
        l = lv_label_create(b);
        lv_label_set_text(l, gas_lbls[i]);
        lv_obj_center(l);
        lv_obj_add_event_cb(b, settings_gas_set_evt, LV_EVENT_CLICKED, (void *)(uintptr_t)i);
    }

    /* Damping + low-fuel steppers, compact second row */
    lbl = lv_label_create(s_scr_set);
    lv_label_set_text(lbl, "DMP");
    lv_obj_set_style_text_color(lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_pos(lbl, rx, 430);

    lv_obj_t *dm = lv_btn_create(s_scr_set);
    lv_obj_add_style(dm, &btn_style, 0);
    lv_obj_set_size(dm, 30, 34);
    lv_obj_set_pos(dm, rx + 42, 426);
    l = lv_label_create(dm);
    lv_label_set_text(l, "-");
    lv_obj_center(l);
    lv_obj_add_event_cb(dm, settings_gdamp_evt, LV_EVENT_CLICKED, (void *)(uintptr_t)-1);

    s_gdamp_val_lbl = lv_label_create(s_scr_set);
    lv_snprintf(buf, sizeof(buf), "%d", s_gdamp_val);
    lv_label_set_text(s_gdamp_val_lbl, buf);
    lv_obj_set_style_text_color(s_gdamp_val_lbl, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_pos(s_gdamp_val_lbl, rx + 76, 436);

    lv_obj_t *dp = lv_btn_create(s_scr_set);
    lv_obj_add_style(dp, &btn_style, 0);
    lv_obj_set_size(dp, 30, 34);
    lv_obj_set_pos(dp, rx + 108, 426);
    l = lv_label_create(dp);
    lv_label_set_text(l, "+");
    lv_obj_center(l);
    lv_obj_add_event_cb(dp, settings_gdamp_evt, LV_EVENT_CLICKED, (void *)(uintptr_t)+1);

    lbl = lv_label_create(s_scr_set);
    lv_label_set_text(lbl, "LOW");
    lv_obj_set_style_text_color(lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_pos(lbl, rx + 160, 430);

    lv_obj_t *lm = lv_btn_create(s_scr_set);
    lv_obj_add_style(lm, &btn_style, 0);
    lv_obj_set_size(lm, 30, 34);
    lv_obj_set_pos(lm, rx + 202, 426);
    l = lv_label_create(lm);
    lv_label_set_text(l, "-");
    lv_obj_center(l);
    lv_obj_add_event_cb(lm, settings_glow_evt, LV_EVENT_CLICKED, (void *)(uintptr_t)-1);

    s_glow_val_lbl = lv_label_create(s_scr_set);
    lv_snprintf(buf, sizeof(buf), "%d%%", s_glow_val);
    lv_label_set_text(s_glow_val_lbl, buf);
    lv_obj_set_style_text_color(s_glow_val_lbl, lv_color_hex(COL_TEXT), 0);
    lv_obj_set_pos(s_glow_val_lbl, rx + 236, 436);

    lv_obj_t *lp = lv_btn_create(s_scr_set);
    lv_obj_add_style(lp, &btn_style, 0);
    lv_obj_set_size(lp, 30, 34);
    lv_obj_set_pos(lp, rx + 278, 426);
    l = lv_label_create(lp);
    lv_label_set_text(l, "+");
    lv_obj_center(l);
    lv_obj_add_event_cb(lp, settings_glow_evt, LV_EVENT_CLICKED, (void *)(uintptr_t)+1);

    /* caption for the row above: what DMP/LOW actually do */
    lbl = lv_label_create(s_scr_set);
    lv_label_set_text(lbl, "DMP = needle smoothing 0-15   LOW % = warn threshold");
    lv_obj_set_style_text_color(lbl, lv_color_hex(COL_DIM), 0);
    lv_obj_set_pos(lbl, rx, 462);

    /* Back button */
    lv_obj_t *btn_back = lv_btn_create(s_scr_set);
    lv_obj_add_style(btn_back, &btn_style, 0);
    lv_obj_set_size(btn_back, 100, 40);
    lv_obj_set_pos(btn_back, 650, 420);
    l = lv_label_create(btn_back);
    lv_label_set_text(l, "BACK");
    lv_obj_center(l);
    lv_obj_add_event_cb(btn_back, settings_back_evt, LV_EVENT_CLICKED, NULL);

    ESP_LOGI(TAG, "Settings page built");
}

void ui_show_settings(void) {
    if (!s_scr_set) build_settings();
    lv_scr_load(s_scr_set);
    /* Apply current mode highlights on page show */
    update_fan_mode_buttons(s_prev.fanMode);
    update_iac_mode_buttons(s_prev.iacMode);
    update_buzzer_button(s_prev.buzzerOn);
    update_boot_button(s_prev.bootTestOn);
    theme_highlight();
    face_highlight();
}

/* Settings button highlighting helpers — guard against NULL (page not built yet) */
static void update_fan_mode_buttons(uint8_t mode) {
    /* mode: 0=off, 1=auto, 2=manual_on — map to correct button index */
    lv_obj_t *main_btns[3] = { s_main_foff, s_main_fauto, s_main_fon };
    lv_obj_t *set_btns[3]  = { s_btn_foff,  s_btn_fauto,  s_btn_fon };
    for (int i = 0; i < 3; i++) {
        bool active = (mode == i);
        if (main_btns[i]) {
            lv_obj_set_style_bg_opa(main_btns[i], active ? LV_OPA_100 : LV_OPA_50, 0);
            lv_obj_set_style_bg_color(main_btns[i], active ? lv_color_hex(accent_live()) : lv_color_hex(0x2A2A3A), 0);
        }
        if (set_btns[i]) {
            lv_obj_set_style_bg_opa(set_btns[i], active ? LV_OPA_100 : LV_OPA_50, 0);
            lv_obj_set_style_bg_color(set_btns[i], active ? lv_color_hex(accent_live()) : lv_color_hex(0x2A2A3A), 0);
        }
    }
}

static void update_iac_mode_buttons(uint8_t mode) {
    /* mode: 0=man, 1=auto, 2=follow */
    lv_obj_t *main_btns[3] = { s_main_iman, s_main_iauto, s_main_ifollow };
    lv_obj_t *set_btns[3]  = { s_btn_iman,  s_btn_iauto,  s_btn_ifollow };
    for (int i = 0; i < 3; i++) {
        bool active = (mode == i);
        if (main_btns[i]) {
            lv_obj_set_style_bg_opa(main_btns[i], active ? LV_OPA_100 : LV_OPA_50, 0);
            lv_obj_set_style_bg_color(main_btns[i], active ? lv_color_hex(accent_live()) : lv_color_hex(0x2A2A3A), 0);
        }
        if (set_btns[i]) {
            lv_obj_set_style_bg_opa(set_btns[i], active ? LV_OPA_100 : LV_OPA_50, 0);
            lv_obj_set_style_bg_color(set_btns[i], active ? lv_color_hex(accent_live()) : lv_color_hex(0x2A2A3A), 0);
        }
    }
}

static void update_buzzer_button(bool on) {
    if (!s_buzz_btn || !s_buzz_lbl) return;
    lv_obj_set_style_bg_opa(s_buzz_btn, on ? LV_OPA_100 : LV_OPA_50, 0);
    lv_obj_set_style_bg_color(s_buzz_btn, on ? lv_color_hex(accent_live()) : lv_color_hex(0x2A2A3A), 0);
    lv_label_set_text(s_buzz_lbl, on ? "BUZZER ON" : "BUZZER OFF");
}

static void update_boot_button(bool on) {
    if (!s_boot_btn || !s_boot_lbl) return;
    lv_obj_set_style_bg_opa(s_boot_btn, on ? LV_OPA_100 : LV_OPA_50, 0);
    lv_obj_set_style_bg_color(s_boot_btn, on ? lv_color_hex(accent_live()) : lv_color_hex(0x2A2A3A), 0);
    lv_label_set_text(s_boot_lbl, on ? "BOOT TEST ON" : "BOOT TEST OFF");
}

void ui_update(const dash_data_t *d) {
    bool ok = d->canOk;
    char buf[32];

    lv_obj_set_style_text_color(s_can_lbl, lv_color_hex(ok ? COL_GOOD : COL_BAD), 0);
    if (ok != s_prev.canOk)
        lv_label_set_text(s_can_lbl, ok ? "CAN OK" : "CAN LOST");

    /* BOX chip: ESP-NOW link to iobox3 alive (redraw only on transition) */
    if (d->ioboxOk != s_prev.ioboxOk) {
        bool bo = d->ioboxOk;
        lv_label_set_text(s_iobox_lbl, bo ? "BOX OK" : "BOX LOST");
        lv_obj_set_style_bg_color(s_iobox_lbl, lv_color_hex(bo ? COL_GOOD : COL_BAD), 0);
    }

    /* RPM: slide star cursor over the load-up ring; on the NEEDLE face rotate
     * the pointer instead (star/ring invisible). */
    int32_t rpm = ok ? d->rpm : 0;
    if (rpm != s_last_rpm) {
        s_last_rpm = rpm;
        float pct = gauge_pct_rpm(rpm);
        float a = gauge_angle(pct);
        float rad = DEG2RAD(a);
        int msize = 22;
        lv_coord_t dx = (lv_coord_t)(GAUGE_CX + GAUGE_R * 0.85f * cosf(rad));
        lv_coord_t dy = (lv_coord_t)(GAUGE_CY + GAUGE_R * 0.85f * sinf(rad));
        lv_obj_set_pos(s_mark_glow, dx - msize / 2, dy - msize / 2);
        lv_obj_set_pos(s_mark, dx - s_mark_sz / 2, dy - s_mark_sz / 2);
        if (s_needle && s_face_active == FACE_NEEDLE) {
            float nlen = (float)GAUGE_R * 0.62f;   /* tip just past the rail */
            s_needle_pts[0] = (lv_point_precise_t){
                (int)(GAUGE_CX - nlen * 0.16f * cosf(rad)),
                (int)(GAUGE_CY - nlen * 0.16f * sinf(rad)) };
            s_needle_pts[1] = (lv_point_precise_t){
                (int)(GAUGE_CX + nlen * cosf(rad)),
                (int)(GAUGE_CY + nlen * sinf(rad)) };
            lv_line_set_points(s_needle, s_needle_pts, 2);
        }
    }

    /* zone-tinted glow + load-up ring advance. Outside the rpm gate on purpose:
     * both are cheap, and the ring must react to shift-RPM setting changes
     * even while rpm sits still (engine off in settings). */
    {
        int32_t start_cfg = s_shift_rpm_cfg - SHIFT_START_GAP;
        if (start_cfg < RPM_MIN) start_cfg = RPM_MIN;

        uint32_t nc = accent_live();
        if (rpm >= s_shift_rpm_cfg)  nc = COL_BAD;
        else if (rpm >= start_cfg)   nc = COL_WARN;
        if (nc != s_glow_col_prev) {
            s_glow_col_prev = nc;
            lv_obj_set_style_bg_color(s_mark_glow, lv_color_hex(nc), 0);
        }

        static int64_t s_ring_last_ms = 0;
        static int32_t s_prev_sg = -1, s_prev_sa = -1, s_prev_sr = -1;
        static int32_t s_ring_cfg = -1;
        int64_t nowms = lv_tick_get();
        if (s_ring_cfg != s_shift_rpm_cfg) {   /* setting changed -> force redraw */
            s_ring_cfg = s_shift_rpm_cfg;
            s_prev_sg = s_prev_sa = s_prev_sr = -1;
        }
        if (nowms - s_ring_last_ms >= 50) {
            s_ring_last_ms = nowms;
            float a_now  = gauge_angle(gauge_pct_rpm(rpm));
            float a_warn = gauge_angle(100.0f * (start_cfg - RPM_MIN) / (RPM_MAX - RPM_MIN));
            float a_red  = gauge_angle(100.0f * (s_shift_rpm_cfg - RPM_MIN) / (RPM_MAX - RPM_MIN));
            int g0 = (int)GAUGE_A0, w0 = (int)a_warn, r0 = (int)a_red;
            /* each zone's END clamps inside its own span: below zone start =
             * empty (never wrap backwards past 0 rpm), above zone end = full */
            int sg = g0 + (int)(a_now - GAUGE_A0); if (sg > w0) sg = w0;
            int sa = w0 + (int)(a_now - a_warn);   if (sa > r0) sa = r0;
            if (sa < w0) sa = w0;
            int sr = r0 + (int)(a_now - a_red);
            if (sr < r0) sr = r0;
            if (sg != s_prev_sg) { lv_arc_set_angles(s_ring_g, g0, sg); s_prev_sg = sg; }
            if (sa != s_prev_sa) { lv_arc_set_angles(s_ring_a, w0, sa); s_prev_sa = sa; }
            if (sr != s_prev_sr) { lv_arc_set_angles(s_ring_r, r0, sr); s_prev_sr = sr; }
        }
    }

    lv_snprintf(buf, sizeof(buf), ok ? "%d" : "--", (int)rpm);
    if (strcmp(lv_label_get_text(s_val_lbl), buf) != 0)
        lv_label_set_text(s_val_lbl, buf);

    /* big mph readout (0xB0 byte 2 from iobox3 ABS speed). Gated on the
     * ESP-NOW link so a dead box shows "--" instead of a frozen 0. */
    uint8_t spd = can_rx_get_speed();
    bool spdOk = d->ioboxOk && spd <= 250;
    lv_snprintf(buf, sizeof(buf), spdOk ? "%d" : "--", (int)spd);
    if (strcmp(lv_label_get_text(s_speed_val), buf) != 0) {
        lv_label_set_text(s_speed_val, buf);
        bool showUnit = spdOk;
        if (showUnit != s_speed_unit_seen) {
            s_speed_unit_seen = showUnit;
            if (showUnit) lv_obj_clear_flag(s_speed_unit, LV_OBJ_FLAG_HIDDEN);
            else          lv_obj_add_flag(s_speed_unit, LV_OBJ_FLAG_HIDDEN);
        }
    }

    /* bars */
    bool mapOk = ok && d->map > 0 && d->map < 4000;
    lv_snprintf(buf, sizeof(buf), mapOk ? "%d" : "--", mapOk ? d->map / 10 : 0);
    bar_set_val(s_map_fill, s_map_val, 120, mapOk, 0, 250, mapOk ? d->map / 10 : 0, buf);

    /* turbo boost = MAP above atmospheric (~100 kPa absolute). Reads 0 at
     * vacuum/atmosphere, positive under boost; same pane as the MAP bar. */
    int32_t boost = mapOk ? (d->map / 10) - 100 : 0;
    bool bstOk = mapOk && boost >= 0;
    lv_snprintf(buf, sizeof(buf), bstOk ? "+%d" : "--", boost);
    bar_set_val(s_bst_fill, s_bst_val, 120, bstOk, 0, 150, bstOk ? boost : 0, buf);

    /* d->afr is tenths (147 = 14.7); display as "14.7" directly. */
    bool afrOk = ok && d->afr > 100 && d->afr < 2550;
    if (afrOk) lv_snprintf(buf, sizeof(buf), "%d.%d", d->afr / 10, d->afr % 10);
    else       lv_snprintf(buf, sizeof(buf), "--");
    bar_set_val(s_afr_fill, s_afr_val, 120, afrOk, 8, 20, afrOk ? d->afr / 10 : 8, buf);
    if (afrOk) {
        uint32_t c = afr_color(d->afr / 10.0f);
        lv_obj_set_style_bg_color(s_afr_fill, lv_color_hex(c), 0);
        lv_obj_set_style_text_color(s_afr_val, lv_color_hex(c), 0);
    }

    /* mini gas bar (from iobox3 0xB0 gas% telemetry, not CAN) */
    uint8_t gpct = can_rx_get_gas();
    bool gasOk = gpct <= 100;
    if (gasOk) lv_snprintf(buf, sizeof(buf), "%d%%", (int)gpct);
    else       lv_snprintf(buf, sizeof(buf), "--");
    bar_set_val(s_gas_fill, s_gas_val, 120, gasOk, 0, 100, gasOk ? gpct : 0, buf);
    if (gasOk) {
        /* low-fuel warning overrides the accent colour; normal range follows accent */
        uint32_t c = (gpct <= (uint8_t)s_glow_val) ? COL_WARN : accent_live();
        lv_obj_set_style_bg_color(s_gas_fill, lv_color_hex(c), 0);
        lv_obj_set_style_text_color(s_gas_val, lv_color_hex(c), 0);
    }

    bool cltOk = ok && d->clt > 100 && d->clt < 3500;
    lv_snprintf(buf, sizeof(buf), cltOk ? "%d" : "--", cltOk ? d->clt / 10 : 0);
    /* scale to 250F: normal running is 160-230F; the old 140 cap pegged the
     * bar at 100% the entire time the engine was warm */
    bar_set_val(s_clt_fill, s_clt_val, 100, cltOk, 0, 250, cltOk ? d->clt / 10 : 0, buf);

    bool tpsOk = ok && d->tps >= 0 && d->tps <= 1000;
    lv_snprintf(buf, sizeof(buf), tpsOk ? "%d" : "--", tpsOk ? d->tps / 10 : 0);
    bar_set_val(s_tps_fill, s_tps_val, 100, tpsOk, 0, 100, tpsOk ? d->tps / 10 : 0, buf);

    bool battOk = ok && d->batt > 80 && d->batt < 200;
    if (battOk) {
        int b10 = d->batt;  // 0.1V units, e.g. 126 = 12.6V
        lv_snprintf(buf, sizeof(buf), "%d.%d", b10 / 10, b10 % 10);
        bar_set_val(s_batt_fill, s_batt_val, 100, true, 80, 160, b10, buf);
    } else {
        bar_set_val(s_batt_fill, s_batt_val, 100, false, 80, 160, 0, "--");
    }

    if (ok && d->mat > 100 && d->mat < 2500)
        lv_snprintf(buf, sizeof(buf), "MAT %dF", d->mat / 10);
    else
        lv_snprintf(buf, sizeof(buf), "MAT --F");
    if (strcmp(lv_label_get_text(s_mat_lbl), buf) != 0)
        lv_label_set_text(s_mat_lbl, buf);

    if (ok) {
        lv_snprintf(buf, sizeof(buf), "IAC %d%%", (int)(d->iacstep * 100 / 255));
    } else {
        lv_snprintf(buf, sizeof(buf), "IAC --");
    }
    if (strcmp(lv_label_get_text(s_iac_lbl), buf) != 0)
        lv_label_set_text(s_iac_lbl, buf);

    /* indicators blink ~1.4Hz while latched; high beam steady.
     * NOTE: no canOk gate — these come from the ESP-NOW link and are
     * already freshness-gated (dashFresh) in can_rx.c */
    s_indL_on = d->indL;
    s_indR_on = d->indR;
    s_hb_on   = d->highBeam;
    bool phase = (lv_tick_get() / 350) & 1;
    bool lvis = s_indL_on && phase;
    bool rvis = s_indR_on && phase;
    if (lvis != s_l_vis) {
        s_l_vis = lvis;
        if (lvis) lv_obj_remove_flag(s_ind_l, LV_OBJ_FLAG_HIDDEN);
        else      lv_obj_add_flag(s_ind_l, LV_OBJ_FLAG_HIDDEN);
    }
    if (rvis != s_r_vis) {
        s_r_vis = rvis;
        if (rvis) lv_obj_remove_flag(s_ind_r, LV_OBJ_FLAG_HIDDEN);
        else      lv_obj_add_flag(s_ind_r, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_hb_on != s_hb_vis) {
        s_hb_vis = s_hb_on;
        if (s_hb_vis) lv_obj_remove_flag(s_hb, LV_OBJ_FLAG_HIDDEN);
        else          lv_obj_add_flag(s_hb, LV_OBJ_FLAG_HIDDEN);
    }

    /* shift light strip */
    int32_t srpm = ok ? d->rpm : 0;
    int32_t sh_start = s_shift_rpm_cfg - SHIFT_START_GAP;
    if (sh_start < 0) sh_start = 0;
    bool flash = srpm >= s_shift_rpm_cfg && ((lv_tick_get() / 150) & 1);
    if (srpm != s_shift_last_rpm || flash != s_shift_flash) {
        s_shift_last_rpm = srpm;
        s_shift_flash = flash;
        for (int i = 0; i < SHIFT_SEGS; i++) {
            uint32_t c;
            if (srpm >= s_shift_rpm_cfg)
                c = flash ? COL_HB : SHIFT_OFF_COLOR;
            else if (srpm >= sh_start + i * (s_shift_rpm_cfg - sh_start) / SHIFT_SEGS)
                c = kShiftColors[i];
            else
                c = SHIFT_OFF_COLOR;
            lv_obj_set_style_bg_color(s_shift_seg[i], lv_color_hex(c), 0);
        }
    }

    /* warning banner */
    warn_banner_update(d);

    /* Settings button highlighting - update on mode transition */
    if (d->fanMode != s_prev.fanMode)
        update_fan_mode_buttons(d->fanMode);
    if (d->iacMode != s_prev.iacMode)
        update_iac_mode_buttons(d->iacMode);
    if (d->buzzerOn != s_prev.buzzerOn)
        update_buzzer_button(d->buzzerOn);
    if (d->bootTestOn != s_prev.bootTestOn)
        update_boot_button(d->bootTestOn);

    s_prev = *d;
}
