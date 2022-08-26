/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/display.h>
#include <zmk/display/widgets/boot_screen_status.h>
#include <zmk/event_manager.h>
#include <zmk/endpoints.h>
#include <zmk/events/activity_state_changed.h>
#include <zephyr.h>
#include <lvgl.h>

LV_IMG_DECLARE(suzuki87_boot_screen);

void boot_screen_timer_expired_cb(struct k_timer *dummy);

K_TIMER_DEFINE(boot_screen_timer, boot_screen_timer_expired_cb, NULL);

static sys_slist_t widgets = SYS_SLIST_STATIC_INIT(&widgets);

struct boot_screen_status_state {
    bool expired;
    bool on;
};

static struct boot_screen_status_state state = {.expired = false, .on = true};

struct boot_screen_status_state boot_screen_status_get_state(const zmk_event_t *eh) {
    if (state.expired) {
        state.on = false;
        state.expired = false;
    } else if (as_zmk_activity_state_changed(eh)) {
        struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
        if (ev->state == false) {
            state.on = true;
            k_timer_start(&boot_screen_timer, K_MSEC(1000), K_NO_WAIT);
        }
    }
    return state;
};

void set_boot_screen_symbol(lv_obj_t *img, struct boot_screen_status_state state) {
    if (state.on) {
        lv_obj_set_hidden(img, false);
    } else {
        lv_obj_set_hidden(img, true);
    }
}

void boot_screen_status_update_cb(struct boot_screen_status_state state) {
    struct zmk_widget_boot_screen_status *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) {
        set_boot_screen_symbol(widget->obj, state);
    }
}

ZMK_DISPLAY_WIDGET_LISTENER(widget_boot_screen_status, struct boot_screen_status_state,
                            boot_screen_status_update_cb, boot_screen_status_get_state)
ZMK_SUBSCRIPTION(widget_boot_screen_status, zmk_activity_state_changed);

void boot_screen_timer_expired_cb(struct k_timer *dummy) {
    state.expired = true;
    state.on = false;
    widget_boot_screen_status_cb(NULL);
}

int zmk_widget_boot_screen_status_init(struct zmk_widget_boot_screen_status *widget,
                                       lv_obj_t *parent) {
    lv_style_t style;
    widget->obj = lv_img_create(parent, NULL);

    lv_obj_set_size(widget->obj, 128, 32);

    lv_style_init(&style);
    lv_style_set_bg_opa(&style, LV_STATE_DEFAULT, LV_OPA_COVER);

    lv_img_set_src(widget->obj, &suzuki87_boot_screen);
    lv_obj_add_style(widget->obj, LV_OBJ_PART_MAIN, &style);

    sys_slist_append(&widgets, &widget->node);

    widget_boot_screen_status_init();
    k_timer_start(&boot_screen_timer, K_MSEC(1000), K_NO_WAIT);
    return 0;
}

lv_obj_t *zmk_widget_boot_screen_status_obj(struct zmk_widget_boot_screen_status *widget) {
    return widget->obj;
}
