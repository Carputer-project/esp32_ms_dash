#ifndef UI_H
#define UI_H

#include "can_rx.h"

void ui_init(void);
void ui_update(const dash_data_t *d);
void ui_face_persist_once(void);  /* NVS-save pending face change; call from non-LVGL task */
void ui_theme_apply_once(void);   /* call from LVGL-locked context (e.g. can_update_task) */

extern bool s_theme_apply_pending;

#endif
