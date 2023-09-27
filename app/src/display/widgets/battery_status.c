/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/services/bas.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/display.h>
#include <zmk/display/widgets/battery_status.h>
#include <zmk/usb.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>

static sys_slist_t widgets = SYS_SLIST_STATIC_INIT(&widgets);

struct battery_status_state {
    uint8_t level;
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    bool usb_present;
#endif
};

static void set_battery_symbol(lv_obj_t *label, struct battery_status_state state) {
    const char usb_present[4] = LV_SYMBOL_CHARGE;
    const char usb_not_present[1] = {0};
    const char *charging = usb_not_present;

    uint8_t level = state.level;

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    if (state.usb_present) {
        charging = usb_present;
    } else {
        charging = usb_not_present;
    }
#endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK) */

#if IS_ENABLED(CONFIG_ZMK_WIDGET_BATTERY_STATUS_SHOW_PERCENTAGE)
    char text[9] = {};
    char perc[5] = {};
    snprintf(perc, sizeof(perc), "%3u%%", level);
    strcat(text, perc);
    lv_label_set_text(label, text);
#else
    if (level > 95) {
        lv_label_set_text_fmt(label, "%s %d%%", LV_SYMBOL_BATTERY_FULL, level);
    } else if (level > 65) {
        lv_label_set_text_fmt(label, "%s %d%%", LV_SYMBOL_BATTERY_3, level);
    } else if (level > 35) {
        lv_label_set_text_fmt(label, "%s %d%%", LV_SYMBOL_BATTERY_2, level);
    } else if (level > 5) {
        lv_label_set_text_fmt(label, "%s %d%%", LV_SYMBOL_BATTERY_1, level);
    } else {
        lv_label_set_text_fmt(label, "%s %d%%", LV_SYMBOL_BATTERY_EMPTY, level);
    }
#endif
}

void battery_status_update_cb(struct battery_status_state state) {
    struct zmk_widget_battery_status *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) { set_battery_symbol(widget->obj, state); }
}

static struct battery_status_state battery_status_get_state(const zmk_event_t *eh) {
    return (struct battery_status_state) {
        .level = bt_bas_get_battery_level(),
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
        .usb_present = zmk_usb_is_powered(),
#endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK) */
    };
}

ZMK_DISPLAY_WIDGET_LISTENER(widget_battery_status, struct battery_status_state,
                            battery_status_update_cb, battery_status_get_state)

ZMK_SUBSCRIPTION(widget_battery_status, zmk_battery_state_changed);
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
ZMK_SUBSCRIPTION(widget_battery_status, zmk_usb_conn_state_changed);
#endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK) */

int zmk_widget_battery_status_init(struct zmk_widget_battery_status *widget, lv_obj_t *parent) {
    widget->obj = lv_label_create(parent);

    sys_slist_append(&widgets, &widget->node);

    widget_battery_status_init();
    return 0;
}

lv_obj_t *zmk_widget_battery_status_obj(struct zmk_widget_battery_status *widget) {
    return widget->obj;
}
