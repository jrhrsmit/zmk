/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/display.h>
#include <zmk/display/widgets/custom_status.h>
#include <zmk/event_manager.h>
#include <zmk/endpoints.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/matrix_transform.h>
#include <lvgl.h>

LV_IMG_DECLARE(bongocat_oled_64x32_frame_0);
LV_IMG_DECLARE(bongocat_oled_64x32_frame_1);
LV_IMG_DECLARE(capslock);

static sys_slist_t widgets = SYS_SLIST_STATIC_INIT(&widgets);

struct custom_status_state {
    int bongo_cat_frame;
    bool caps;
};

struct custom_status_state custom_status_get_state(const zmk_event_t *eh) {
    static struct custom_status_state state = {.bongo_cat_frame = 0, .caps = false};
    if (as_zmk_position_state_changed(eh)) {
        struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
        if (ev->state == 0 && ev->position == zmk_matrix_transform_row_column_to_position(3, 0)) {
            state.caps = !state.caps;
        }
        if (ev->state) {
            state.bongo_cat_frame++;
            if (state.bongo_cat_frame > 1)
                state.bongo_cat_frame = 0;
        }
    }
    return state;
};

void set_custom_symbol(lv_obj_t *img, struct custom_status_state state) {
    char text[10] = {};
    static int cur_frame = -1;

    if (state.caps && cur_frame != -1) {
        lv_img_set_src(img, &capslock);
        cur_frame = -1;
        LOG_DBG("Drawing bongo cat capslock");
    } else if (state.bongo_cat_frame != cur_frame) {
        switch (state.bongo_cat_frame) {
        case 0:
            lv_img_set_src(img, &bongocat_oled_64x32_frame_0);
            LOG_DBG("Drawing bongo cat frame 0");
            break;
        case 1:
        default:
            lv_img_set_src(img, &bongocat_oled_64x32_frame_1);
            LOG_DBG("Drawing bongo cat frame 1");
            break;
        }
        cur_frame = state.bongo_cat_frame;
    }
}

void custom_status_update_cb(struct custom_status_state state) {
    struct zmk_widget_custom_status *widget;
    SYS_SLIST_FOR_EACH_CONTAINER(&widgets, widget, node) { set_custom_symbol(widget->obj, state); }
}

ZMK_DISPLAY_WIDGET_LISTENER(widget_custom_status, struct custom_status_state,
                            custom_status_update_cb, custom_status_get_state)
ZMK_SUBSCRIPTION(widget_custom_status, zmk_position_state_changed);

int zmk_widget_custom_status_init(struct zmk_widget_custom_status *widget, lv_obj_t *parent) {
    widget->obj = lv_img_create(parent, NULL);

    lv_obj_set_size(widget->obj, 64, 32);

    sys_slist_append(&widgets, &widget->node);

    widget_custom_status_init();
    return 0;
}

lv_obj_t *zmk_widget_custom_status_obj(struct zmk_widget_custom_status *widget) {
    return widget->obj;
}
