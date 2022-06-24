/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <devicetree.h>

#if DT_HAS_CHOSEN(zmk_ledstrip_map)

#define ZMK_LEDSTRIP_MAP_NODE DT_CHOSEN(zmk_ledstrip_map)
#define ZMK_LEDSTRIP_MAP_LEN DT_PROP_LEN(ZMK_LEDSTRIP_MAP_NODE, map)
#define ZMK_LEDSTRIP_MAP DT_PROP(ZMK_LEDSTRIP_MAP_NODE, map)
#define ZMK_LEDSTRIP_INDEX DT_PROP(ZMK_LEDSTRIP_MAP_NODE, strip_index)

#endif

struct zmk_led_hsb {
    uint16_t h;
    uint8_t s;
    uint8_t b;
};

int zmk_rgb_underglow_toggle();
int zmk_rgb_underglow_get_state(bool *state);
int zmk_rgb_underglow_on();
int zmk_rgb_underglow_off();
int zmk_rgb_underglow_cycle_effect(int direction);
int zmk_rgb_underglow_calc_effect(int direction);
int zmk_rgb_underglow_select_effect(int effect);
struct zmk_led_hsb zmk_rgb_underglow_calc_hue(int direction);
struct zmk_led_hsb zmk_rgb_underglow_calc_sat(int direction);
struct zmk_led_hsb zmk_rgb_underglow_calc_brt(int direction);
int zmk_rgb_underglow_change_hue(int direction);
int zmk_rgb_underglow_change_sat(int direction);
int zmk_rgb_underglow_change_brt(int direction);
int zmk_rgb_underglow_change_spd(int direction);
int zmk_rgb_underglow_set_hsb(struct zmk_led_hsb color);
