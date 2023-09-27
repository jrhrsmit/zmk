/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/display.h>
#include <zmk/display/widgets/rgb_status.h>
#include <zmk/event_manager.h>
#include <zmk/events/rgb_cmd_event.h>
#include <zmk/rgb_underglow.h>
#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zmk/keymap.h>
#include <dt-bindings/zmk/rgb.h>
#include <zephyr/kernel.h>

void rgb_timer_expired_cb(struct k_timer *dummy);

K_TIMER_DEFINE(rgb_timer, rgb_timer_expired_cb, NULL);

static sys_slist_t widgets = SYS_SLIST_STATIC_INIT(&widgets);

enum rgb_status_show_states { RGB_SHOW_HUE, RGB_SHOW_SAT, RGB_SHOW_BRT, RGB_SHOW_EFF, RGB_SHOW_ON };

struct rgb_status_state {
    bool expired;
    bool on;
    int show;
};

static struct rgb_status_state state = {.expired = false, .on = false, .show = RGB_SHOW_ON};

struct rgb_status_state rgb_status_get_state(const zmk_event_t *eh) {
    if (state.expired) {
        state.on = false;
        state.expired = false;
    } else if (as_zmk_rgb_cmd_event(eh)) {
        struct zmk_rgb_cmd_event *ev = as_zmk_rgb_cmd_event(eh);
        switch (ev->cmd) {
        case RGB_EFF_CMD:
        case RGB_EFR_CMD:
            LOG_DBG("Showing rgb effect change");
            state.on = true;
            k_timer_start(&rgb_timer, K_MSEC(2500), K_NO_WAIT);
            state.show = RGB_SHOW_EFF;
            break;
        case RGB_HUI_CMD:
        case RGB_HUD_CMD:
            LOG_DBG("Showing rgb hue change");
            state.on = true;
            k_timer_start(&rgb_timer, K_MSEC(2500), K_NO_WAIT);
            state.show = RGB_SHOW_HUE;
            break;
        case RGB_SAI_CMD:
        case RGB_SAD_CMD:
            LOG_DBG("Showing rgb sat change");
            state.on = true;
            k_timer_start(&rgb_timer, K_MSEC(2500), K_NO_WAIT);
            state.show = RGB_SHOW_SAT;
            break;
        case RGB_BRI_CMD:
        case RGB_BRD_CMD:
            LOG_DBG("Showing rgb brt change");
            state.on = true;
            k_timer_start(&rgb_timer, K_MSEC(2500), K_NO_WAIT);
            state.show = RGB_SHOW_BRT;
            break;
        case RGB_TOG_CMD:
        case RGB_ON_CMD:
        case RGB_OFF_CMD:
            LOG_DBG("Showing rgb state change");
            state.on = true;
            k_timer_start(&rgb_timer, K_MSEC(2500), K_NO_WAIT);
            state.show = RGB_SHOW_ON;
            break;
        default:
            break;
        }
    }
    return state;
};

void set_rgb_symbol(lv_obj_t *label, struct rgb_status_state state) {
    char text[10] = {0};
    struct zmk_led_hsb color;
    int effect;
    bool on;
    switch (state.show) {
    case RGB_SHOW_HUE:
        zmk_rgb_underglow_get_hsb(&color);
        snprintf(text, 10, "HUE %d", color.h);
        break;
    case RGB_SHOW_SAT:
        zmk_rgb_underglow_get_hsb(&color);
        snprintf(text, 10, "SAT %d", color.s);
        break;
    case RGB_SHOW_BRT:
        zmk_rgb_underglow_get_hsb(&color);
        snprintf(text, 10, "BRT %d", color.b);
        break;
    case RGB_SHOW_EFF:
        zmk_rgb_underglow_get_effect(&effect);
        snprintf(text, 10, "Effect %d", effect & 0x7);
        break;
    case RGB_SHOW_ON:
        zmk_rgb_underglow_get_state(&on);
        snprintf(text, 10, "RGB %s", on ? "on" : "off");
        break;
    default:
        break;
    }
    lv_label_set_text(label, text);

    if (state.on) {
        lv_obj_clear_flag(label, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(label, LV_OBJ_FLAG_HIDDEN);
    }
}

void rgb_status_update_cb(struct rgb_status_state state) {
    struct zmk_widget_rgb_status *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) { set_rgb_symbol(widget->obj, state); }
}

ZMK_DISPLAY_WIDGET_LISTENER(widget_rgb_status, struct rgb_status_state, rgb_status_update_cb,
                            rgb_status_get_state)
ZMK_SUBSCRIPTION(widget_rgb_status, zmk_rgb_cmd_event);

void rgb_timer_expired_cb(struct k_timer *dummy) {
    state.expired = true;
    state.on = false;
    widget_rgb_status_cb(NULL);
}

int zmk_widget_rgb_status_init(struct zmk_widget_rgb_status *widget, lv_obj_t *parent) {
    lv_style_t style;
    widget->obj = lv_label_create(parent);

    lv_obj_set_size(widget->obj, 64, 15);

    lv_style_init(&style);
    lv_style_set_bg_opa(&style, LV_OPA_COVER);
    lv_obj_add_style(widget->obj, &style, 0);

    lv_obj_add_flag(widget->obj, LV_OBJ_FLAG_HIDDEN);

    sys_slist_append(&widgets, &widget->node);

    widget_rgb_status_init();
    return 0;
}

lv_obj_t *zmk_widget_rgb_status_obj(struct zmk_widget_rgb_status *widget) { return widget->obj; }
