#include <string.h>
#include "esp_log.h"
#include "esp_lv_adapter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "waveshare_rgb_lcd_port.h"
#include "can_rx.h"
#include "can_tx.h"
#include "espnow_link.h"
#include "ui.h"
#include "driver/twai.h"
#include "driver/usb_serial_jtag.h"

static const char *TAG = "main";

/* Minimal serial console (USB_SERIAL_JTAG secondary = ttyACM0).
 * Commands: CANTX = run bench TX probe; ? = help. Kept tiny on purpose —
 * exists mainly so the CAN TX probe is available without ever firing
 * automatically on the car bus. */
static void console_task(void *arg)
{
    char line[24];
    int len = 0;
    /* The USJ driver must be installed before read_bytes — the raw API
     * derefs an internal handle that is NULL until install() runs.
     * Logs keep flowing via the console VFS path (unaffected by this). */
    usb_serial_jtag_driver_config_t usj_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    if (usb_serial_jtag_driver_install(&usj_cfg) != ESP_OK) {
        printf("[CONS] USJ driver install failed - console disabled\n");
        vTaskDelete(NULL);
    }
    printf("[CONS] ready (CANTX | ?)\n");
    for (;;) {
        uint8_t c;
        if (usb_serial_jtag_read_bytes(&c, 1, pdMS_TO_TICKS(200)) != 1) continue;
        if (c == '\r' || c == '\n') {
            if (len == 0) continue;
            line[len] = '\0';
            len = 0;
            if (strcmp(line, "CANTX") == 0) {
                can_rx_selftest_tx();
            } else if (line[0] == '?') {
                printf("commands: CANTX (bench CAN TX probe) | ?\n");
            } else {
                printf("[CONS] ? for commands\n");
            }
        } else if (len < (int)sizeof(line) - 1 && c >= 0x20 && c < 0x7F) {
            line[len++] = c;
        }
    }
}

static void can_update_task(void *arg)
{
    static dash_data_t snap;
    for (;;) {
        can_rx_get_data_copy(&snap);
        if (esp_lv_adapter_lock(-1) == ESP_OK) {
            ui_update(&snap);
            ui_theme_apply_once();
            esp_lv_adapter_unlock();
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

/* Persist the dash's own settings (e.g. gauge face) to NVS. Runs on a real
 * stack so the flash commit's cache-freeze path doesn't overflow the LVGL
 * task (esp_cache_freeze assert seen when writing from the LVGL event
 * handler). Must not touch LVGL or hold the adapter lock. */
static void ui_persist_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(500));
        ui_face_persist_once();
    }
}

void app_main(void)
{
    /* NVS first: ui_init loads persisted settings (dashui namespace) and
     * espnow_link_init's own nvs_flash_init becomes a harmless no-op. */
    esp_err_t nvs_ret = nvs_flash_init();
    if (nvs_ret == ESP_ERR_NVS_NO_FREE_PAGES || nvs_ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    const esp_lv_adapter_rotation_t rotation = ESP_LV_ADAPTER_ROTATE_180;
    const esp_lv_adapter_tear_avoid_mode_t tear_mode = ESP_LV_ADAPTER_TEAR_AVOID_MODE_DEFAULT_RGB;

    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_touch_handle_t touch_handle = NULL;

    ESP_ERROR_CHECK(waveshare_esp32_s3_rgb_lcd_init(
        tear_mode, rotation, &panel_handle, &touch_handle));
    ESP_ERROR_CHECK(waveshare_rgb_lcd_backlight_on());

    esp_lv_adapter_config_t adapter_config = ESP_LV_ADAPTER_DEFAULT_CONFIG();
    adapter_config.task_stack_size = 12 * 1024;
    adapter_config.stack_in_psram = true;
    ESP_ERROR_CHECK(esp_lv_adapter_init(&adapter_config));

    esp_lv_adapter_display_config_t disp_config = ESP_LV_ADAPTER_DISPLAY_RGB_DEFAULT_CONFIG(
        panel_handle, NULL,
        EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES, rotation);
    disp_config.profile.use_psram = true;

    lv_display_t *disp = esp_lv_adapter_register_display(&disp_config);
    assert(disp != NULL);

    if (touch_handle != NULL) {
        esp_lv_adapter_touch_config_t touch_config = ESP_LV_ADAPTER_TOUCH_DEFAULT_CONFIG(disp, touch_handle);
        lv_indev_t *touch = esp_lv_adapter_register_touch(&touch_config);
        assert(touch != NULL);
    }

    ESP_ERROR_CHECK(esp_lv_adapter_start());

    ESP_LOGI(TAG, "Enabling CAN_SEL on IO expander");
    ESP_ERROR_CHECK(waveshare_can_sel_enable());

    ESP_LOGI(TAG, "Initializing UI");
    if (esp_lv_adapter_lock(-1) == ESP_OK) {
        ui_init();
        esp_lv_adapter_unlock();
    }

    ESP_LOGI(TAG, "Initializing CAN RX");
    ESP_ERROR_CHECK(can_rx_init());

    ESP_LOGI(TAG, "Initializing CAN TX");
    can_tx_init();

    ESP_LOGI(TAG, "Initializing ESP-NOW link to iobox3");
    ESP_ERROR_CHECK(espnow_link_init());

    ESP_LOGI(TAG, "Starting CAN UI update task (20Hz)");
    xTaskCreate(can_update_task, "can_update", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Starting UI persistence task");
    xTaskCreate(ui_persist_task, "ui_persist", 4096, NULL, 3, NULL);

    /* console DISABLED: usb_serial_jtag_driver_install reconfigures the
     * GPIO19/20 pads (S3 USB D-/D+) and severs the TWAI RX input — CAN
     * reception died with this enabled. See howtolog 2026-08-24. */
    // xTaskCreate(console_task, "console", 3072, NULL, 3, NULL);
}
