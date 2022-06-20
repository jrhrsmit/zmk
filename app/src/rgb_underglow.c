/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>

#include <math.h>
#include <stdlib.h>

#include <zephyr/logging/log.h>

#include <zephyr/drivers/led_strip.h>
#include <drivers/ext_power.h>

#include <zmk/rgb_underglow.h>

#include <zmk/activity.h>
#include <zmk/usb.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/workqueue.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>

#include <random/rand32.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if !DT_HAS_CHOSEN(zmk_underglow)

#error "A zmk,underglow chosen node must be declared"

#endif

#define STRIP_CHOSEN DT_CHOSEN(zmk_underglow)
#define STRIP_NUM_PIXELS DT_PROP(STRIP_CHOSEN, chain_length)

#define HUE_MAX 360
#define SAT_MAX 100
#define BRT_MAX 100

BUILD_ASSERT(CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN <= CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX,
             "ERROR: RGB underglow maximum brightness is less than minimum brightness");

enum rgb_underglow_effect {
    UNDERGLOW_EFFECT_SOLID,
    UNDERGLOW_EFFECT_BREATHE,
    UNDERGLOW_EFFECT_SPECTRUM,
    UNDERGLOW_EFFECT_SWIRL,
    UNDERGLOW_EFFECT_RIPPLE,
    UNDERGLOW_EFFECT_NUMBER // Used to track number of underglow effects
};

struct rgb_underglow_state {
    struct zmk_led_hsb color;
    uint8_t animation_speed;
    uint8_t current_effect;
    uint16_t animation_step;
    bool on;
};

static const struct device *led_strip;

static struct led_rgb pixels[STRIP_NUM_PIXELS];

static struct rgb_underglow_state state;

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
static const struct device *ext_power;
#endif

static struct zmk_led_hsb hsb_scale_min_max(struct zmk_led_hsb hsb) {
    hsb.b = CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN +
            (CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX - CONFIG_ZMK_RGB_UNDERGLOW_BRT_MIN) * hsb.b / BRT_MAX;
    return hsb;
}

static struct zmk_led_hsb hsb_scale_zero_max(struct zmk_led_hsb hsb) {
    hsb.b = hsb.b * CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX / BRT_MAX;
    return hsb;
}

static struct led_rgb hsb_to_rgb(struct zmk_led_hsb hsb) {
    float r, g, b;

    uint8_t i = hsb.h / 60;
    float v = hsb.b / ((float)BRT_MAX);
    float s = hsb.s / ((float)SAT_MAX);
    float f = hsb.h / ((float)HUE_MAX) * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6) {
    case 0:
        r = v;
        g = t;
        b = p;
        break;
    case 1:
        r = q;
        g = v;
        b = p;
        break;
    case 2:
        r = p;
        g = v;
        b = t;
        break;
    case 3:
        r = p;
        g = q;
        b = v;
        break;
    case 4:
        r = t;
        g = p;
        b = v;
        break;
    case 5:
        r = v;
        g = p;
        b = q;
        break;
    }

    struct led_rgb rgb = {r : r * 255, g : g * 255, b : b * 255};

    return rgb;
}

static void zmk_rgb_underglow_effect_solid() {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        pixels[i] = hsb_to_rgb(hsb_scale_min_max(state.color));
    }
}

static void zmk_rgb_underglow_effect_breathe() {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.b = abs(state.animation_step - 1200) / 12;

        pixels[i] = hsb_to_rgb(hsb_scale_zero_max(hsb));
    }

    state.animation_step += state.animation_speed * 10;

    if (state.animation_step > 2400) {
        state.animation_step = 0;
    }
}

static void zmk_rgb_underglow_effect_spectrum() {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = state.animation_step;

        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }

    state.animation_step += state.animation_speed;
    state.animation_step = state.animation_step % HUE_MAX;
}

static void zmk_rgb_underglow_effect_swirl() {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = (HUE_MAX / STRIP_NUM_PIXELS * i + state.animation_step) % HUE_MAX;

        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }

    state.animation_step += state.animation_speed * 2;
    state.animation_step = state.animation_step % HUE_MAX;
}

struct pixel_location {
    uint8_t x;
    uint8_t y;
    uint8_t pos;
    uint8_t state;
};

struct ripple_effect_event {
    uint8_t pos;
    uint8_t ticks_passed;
    struct zmk_led_hsb hsb;
};
#define RIPPLE_EFFECTS_BUF_SIZE 16
static struct ripple_effect_event ripple_effects[RIPPLE_EFFECTS_BUF_SIZE] = {
    {0, 0, {0, SAT_MAX, BRT_MAX}}, {0, 0, {0, SAT_MAX, BRT_MAX}}, {0, 0, {0, SAT_MAX, BRT_MAX}},
    {0, 0, {0, SAT_MAX, BRT_MAX}}, {0, 0, {0, SAT_MAX, BRT_MAX}}, {0, 0, {0, SAT_MAX, BRT_MAX}},
    {0, 0, {0, SAT_MAX, BRT_MAX}}, {0, 0, {0, SAT_MAX, BRT_MAX}}, {0, 0, {0, SAT_MAX, BRT_MAX}},
    {0, 0, {0, SAT_MAX, BRT_MAX}}, {0, 0, {0, SAT_MAX, BRT_MAX}}, {0, 0, {0, SAT_MAX, BRT_MAX}},
    {0, 0, {0, SAT_MAX, BRT_MAX}}, {0, 0, {0, SAT_MAX, BRT_MAX}}, {0, 0, {0, SAT_MAX, BRT_MAX}},
    {0, 0, {0, SAT_MAX, BRT_MAX}}};

static struct pixel_location pixel_locations[STRIP_NUM_PIXELS] = {
    {.x = 0, .y = 0, .pos = 18, .state = 0},  // Esc
    {.x = 16, .y = 0, .pos = 17, .state = 0}, // F1
    {.x = 24, .y = 0, .pos = 16, .state = 0}, // F2
    {.x = 32, .y = 0, .pos = 15, .state = 0}, // F3
    {.x = 40, .y = 0, .pos = 14, .state = 0}, // F4
    {.x = 52, .y = 0, .pos = 13, .state = 0}, // F5
    {.x = 60, .y = 0, .pos = 12, .state = 0}, // F6
    {.x = 68, .y = 0, .pos = 11, .state = 0}, // F7
    {.x = 76, .y = 0, .pos = 10, .state = 0}, // F8
    {.x = 88, .y = 0, .pos = 9, .state = 0},  // F9
    {.x = 96, .y = 0, .pos = 8, .state = 0},  // F10
    {.x = 104, .y = 0, .pos = 7, .state = 0}, // F11
    {.x = 112, .y = 0, .pos = 6, .state = 0}, // F12
    {.x = 122, .y = 0, .pos = 5, .state = 0}, // PrtSc
    {.x = 130, .y = 0, .pos = 4, .state = 0}, // Scroll Lock
    {.x = 138, .y = 0, .pos = 3, .state = 0}, // Pause Break

    {.x = 0, .y = 12, .pos = 19, .state = 0},   // ~ `
    {.x = 8, .y = 12, .pos = 20, .state = 0},   // ! 1
    {.x = 16, .y = 12, .pos = 21, .state = 0},  // @ 2
    {.x = 24, .y = 12, .pos = 22, .state = 0},  // # 3
    {.x = 32, .y = 12, .pos = 23, .state = 0},  // $ 4
    {.x = 40, .y = 12, .pos = 24, .state = 0},  // % 5
    {.x = 48, .y = 12, .pos = 25, .state = 0},  // ^ 6
    {.x = 56, .y = 12, .pos = 26, .state = 0},  // & 7
    {.x = 64, .y = 12, .pos = 27, .state = 0},  // * 8
    {.x = 72, .y = 12, .pos = 28, .state = 0},  // ( 9
    {.x = 80, .y = 12, .pos = 29, .state = 0},  // ) 0
    {.x = 88, .y = 12, .pos = 30, .state = 0},  // _ -
    {.x = 96, .y = 12, .pos = 31, .state = 0},  // + =
    {.x = 108, .y = 12, .pos = 32, .state = 0}, // Backspace
    {.x = 122, .y = 12, .pos = 33, .state = 0}, // Insert
    {.x = 130, .y = 12, .pos = 34, .state = 0}, // Home
    {.x = 138, .y = 12, .pos = 35, .state = 0}, // PgUp

    {.x = 2, .y = 20, .pos = 52, .state = 0},   // Tab
    {.x = 12, .y = 20, .pos = 51, .state = 0},  // Q
    {.x = 20, .y = 20, .pos = 50, .state = 0},  // W
    {.x = 28, .y = 20, .pos = 49, .state = 0},  // E
    {.x = 36, .y = 20, .pos = 48, .state = 0},  // R
    {.x = 44, .y = 20, .pos = 47, .state = 0},  // T
    {.x = 52, .y = 20, .pos = 46, .state = 0},  // Y
    {.x = 60, .y = 20, .pos = 45, .state = 0},  // U
    {.x = 68, .y = 20, .pos = 44, .state = 0},  // I
    {.x = 76, .y = 20, .pos = 43, .state = 0},  // O
    {.x = 84, .y = 20, .pos = 42, .state = 0},  // P
    {.x = 92, .y = 20, .pos = 41, .state = 0},  // { [
    {.x = 100, .y = 20, .pos = 40, .state = 0}, // } ]
    {.x = 110, .y = 20, .pos = 39, .state = 0}, // | Backslash
    {.x = 122, .y = 20, .pos = 38, .state = 0}, // Delete
    {.x = 130, .y = 20, .pos = 37, .state = 0}, // End
    {.x = 138, .y = 20, .pos = 36, .state = 0}, // PgDn

    {.x = 3, .y = 28, .pos = 53, .state = 0},   // Caps Lock
    {.x = 14, .y = 28, .pos = 54, .state = 0},  // A
    {.x = 22, .y = 28, .pos = 55, .state = 0},  // S
    {.x = 30, .y = 28, .pos = 56, .state = 0},  // D
    {.x = 38, .y = 28, .pos = 57, .state = 0},  // F
    {.x = 46, .y = 28, .pos = 58, .state = 0},  // G
    {.x = 54, .y = 28, .pos = 59, .state = 0},  // H
    {.x = 62, .y = 28, .pos = 60, .state = 0},  // J
    {.x = 70, .y = 28, .pos = 61, .state = 0},  // K
    {.x = 78, .y = 28, .pos = 62, .state = 0},  // L
    {.x = 86, .y = 28, .pos = 63, .state = 0},  // : ;
    {.x = 94, .y = 28, .pos = 64, .state = 0},  // " '
    {.x = 107, .y = 28, .pos = 65, .state = 0}, // Enter

    {.x = 5, .y = 36, .pos = 78, .state = 0},   // Shift
    {.x = 18, .y = 36, .pos = 77, .state = 0},  // Z
    {.x = 26, .y = 36, .pos = 76, .state = 0},  // X
    {.x = 34, .y = 36, .pos = 75, .state = 0},  // C
    {.x = 42, .y = 36, .pos = 74, .state = 0},  // V
    {.x = 50, .y = 36, .pos = 73, .state = 0},  // B
    {.x = 58, .y = 36, .pos = 72, .state = 0},  // N
    {.x = 66, .y = 36, .pos = 71, .state = 0},  // M
    {.x = 74, .y = 36, .pos = 70, .state = 0},  // < ,
    {.x = 82, .y = 36, .pos = 69, .state = 0},  // > .
    {.x = 90, .y = 36, .pos = 68, .state = 0},  // ? /
    {.x = 105, .y = 36, .pos = 67, .state = 0}, // Shift
    {.x = 130, .y = 36, .pos = 66, .state = 0}, // ↑

    {.x = 1, .y = 44, .pos = 79, .state = 0},   // Ctrl
    {.x = 11, .y = 44, .pos = 80, .state = 0},  // Win
    {.x = 21, .y = 44, .pos = 81, .state = 0},  // Alt
    {.x = 51, .y = 44, .pos = 83, .state = 0},  // Space
    {.x = 81, .y = 44, .pos = 85, .state = 0},  // Alt
    {.x = 91, .y = 44, .pos = 86, .state = 0},  // Win
    {.x = 101, .y = 44, .pos = 87, .state = 0}, // Menu
    {.x = 111, .y = 44, .pos = 88, .state = 0}, // Ctrl
    {.x = 122, .y = 44, .pos = 89, .state = 0}, // ←
    {.x = 130, .y = 44, .pos = 90, .state = 0}, // ↓
    {.x = 138, .y = 44, .pos = 91, .state = 0}, // →

    {.x = 138, .y = 6, .pos = 0, .state = 0},   // Notification LED
    {.x = 130, .y = 6, .pos = 1, .state = 0},   // Notification LED
    {.x = 122, .y = 6, .pos = 2, .state = 0},   // Notification LED
    {.x = 36, .y = 44, .pos = 82, .state = 0},  // Space led 1
    {.x = 66, .y = 44, .pos = 84, .state = 0}}; // Space led 2

static uint8_t pixel_distances[STRIP_NUM_PIXELS][STRIP_NUM_PIXELS];
static void init_ripple_pixel_distances(void) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        for (int j = 0; j < STRIP_NUM_PIXELS; j++) {
            int dx = pixel_locations[i].x - pixel_locations[j].x;
            int dy = pixel_locations[i].y - pixel_locations[j].y;
            pixel_distances[i][j] = (uint8_t)round(sqrtf((float)(dx * dx + dy * dy)));
        }
    }
}

#define RIPPLE_THICKNESS 16

static void zmk_rgb_underglow_effect_ripple() {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        bool first_ripple = 1;
        struct zmk_led_hsb hsb = state.color;
        hsb.b = 0;
        for (int j = 0; j < RIPPLE_EFFECTS_BUF_SIZE; j++) {
            if (ripple_effects[j].ticks_passed == 0)
                continue;
            float pixel_distance;
            pixel_distance = pixel_distances[i][ripple_effects[j].pos];

            if (pixel_distance > ripple_effects[j].ticks_passed - RIPPLE_THICKNESS &&
                pixel_distance < ripple_effects[j].ticks_passed + RIPPLE_THICKNESS) {
                int intensity =
                    BRT_MAX / RIPPLE_THICKNESS *
                    (RIPPLE_THICKNESS - abs(pixel_distance - ripple_effects[j].ticks_passed));
                if (first_ripple) {
                    hsb.h = ripple_effects[j].hsb.h;
                    first_ripple = 0;
                } else {
                    int diff_left_mixing = abs(hsb.h - ripple_effects[j].hsb.h);
                    int diff_right_mixing = MIN(hsb.h, ripple_effects[j].hsb.h) + 360 -
                                            MAX(hsb.h, ripple_effects[j].hsb.h);
                    if (diff_left_mixing < diff_right_mixing) {
                        hsb.h = MIN(hsb.h, ripple_effects[j].hsb.h) + diff_left_mixing / 2;
                    } else {
                        hsb.h = MAX(hsb.h, ripple_effects[j].hsb.h) + diff_right_mixing / 2;
                    }
                }
                if (intensity + hsb.b > BRT_MAX) {
                    hsb.b = BRT_MAX;
                    // when we reach maximum brightness for one pixel, we don't
                    // have to consider the brightness caused by a ripple from
                    // other keys.
                    if (hsb.s == 0)
                        break;
                } else {
                    hsb.b += intensity;
                }
            }
        }

        pixels[pixel_locations[i].pos] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }
    for (int j = 0; j < RIPPLE_EFFECTS_BUF_SIZE; j++) {
        if (ripple_effects[j].ticks_passed > 0) {
            ripple_effects[j].ticks_passed++;
            if (ripple_effects[j].ticks_passed > 150)
                ripple_effects[j].ticks_passed = 0;
        }
    }
}

static void zmk_rgb_underglow_effect_ripple_handler(int pos) {
    static int ripple_effects_i = 0;
    pixel_locations[pos].state = 1;
    ripple_effects[ripple_effects_i].pos = pos;
    ripple_effects[ripple_effects_i].ticks_passed = 1;
    ripple_effects[ripple_effects_i].hsb.h = sys_rand32_get() % HUE_MAX;
    LOG_INF("Ripple update: pos: %d, x: %d, y: %d, led: %d, hue: %d", pos, pixel_locations[pos].x,
            pixel_locations[pos].y, pixel_locations[pos].pos,
            ripple_effects[ripple_effects_i].hsb.h);
    ripple_effects_i++;
    if (ripple_effects_i >= RIPPLE_EFFECTS_BUF_SIZE)
        ripple_effects_i = 0;
}

int rgb_underglow_listener(const zmk_event_t *eh) {
    struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev->state) {
        switch (state.current_effect) {
        case UNDERGLOW_EFFECT_RIPPLE:
            zmk_rgb_underglow_effect_ripple_handler(ev->position);
            break;
        }
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(rgb_underglow, rgb_underglow_listener);
ZMK_SUBSCRIPTION(rgb_underglow, zmk_position_state_changed);

static void zmk_rgb_underglow_tick(struct k_work *work) {
    switch (state.current_effect) {
    case UNDERGLOW_EFFECT_SOLID:
        zmk_rgb_underglow_effect_solid();
        break;
    case UNDERGLOW_EFFECT_BREATHE:
        zmk_rgb_underglow_effect_breathe();
        break;
    case UNDERGLOW_EFFECT_SPECTRUM:
        zmk_rgb_underglow_effect_spectrum();
        break;
    case UNDERGLOW_EFFECT_SWIRL:
        zmk_rgb_underglow_effect_swirl();
        break;
    case UNDERGLOW_EFFECT_RIPPLE:
        zmk_rgb_underglow_effect_ripple();
        break;
    }

    int err = led_strip_update_rgb(led_strip, pixels, STRIP_NUM_PIXELS);
    if (err < 0) {
        LOG_ERR("Failed to update the RGB strip (%d)", err);
    }
}

K_WORK_DEFINE(underglow_tick_work, zmk_rgb_underglow_tick);

static void zmk_rgb_underglow_tick_handler(struct k_timer *timer) {
    if (!state.on) {
        return;
    }

    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &underglow_tick_work);
}

K_TIMER_DEFINE(underglow_tick, zmk_rgb_underglow_tick_handler, NULL);

#if IS_ENABLED(CONFIG_SETTINGS)
static int rgb_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    const char *next;
    int rc;

    if (settings_name_steq(name, "state", &next) && !next) {
        if (len != sizeof(state)) {
            return -EINVAL;
        }

        rc = read_cb(cb_arg, &state, sizeof(state));
        if (rc >= 0) {
            return 0;
        }

        return rc;
    }

    return -ENOENT;
}

struct settings_handler rgb_conf = {.name = "rgb/underglow", .h_set = rgb_settings_set};

static void zmk_rgb_underglow_save_state_work() {
    settings_save_one("rgb/underglow/state", &state, sizeof(state));
}

static struct k_work_delayable underglow_save_work;
#endif

static int zmk_rgb_underglow_init(const struct device *_arg) {
    led_strip = device_get_binding(STRIP_LABEL);
    if (led_strip) {
        LOG_INF("Found LED strip device %s", STRIP_LABEL);
    } else {
        LOG_ERR("LED strip device %s not found", STRIP_LABEL);
        return -EINVAL;
    }
    init_ripple_pixel_distances();

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    ext_power = device_get_binding("EXT_POWER");
    if (ext_power == NULL) {
        LOG_ERR("Unable to retrieve ext_power device: EXT_POWER");
    }
#endif

    state = (struct rgb_underglow_state){
        color : {
            h : CONFIG_ZMK_RGB_UNDERGLOW_HUE_START,
            s : CONFIG_ZMK_RGB_UNDERGLOW_SAT_START,
            b : CONFIG_ZMK_RGB_UNDERGLOW_BRT_START,
        },
        animation_speed : CONFIG_ZMK_RGB_UNDERGLOW_SPD_START,
        current_effect : CONFIG_ZMK_RGB_UNDERGLOW_EFF_START,
        animation_step : 0,
        on : IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_ON_START)
    };

#if IS_ENABLED(CONFIG_SETTINGS)
    settings_subsys_init();

    int err = settings_register(&rgb_conf);
    if (err) {
        LOG_ERR("Failed to register the ext_power settings handler (err %d)", err);
        return err;
    }

    k_work_init_delayable(&underglow_save_work, zmk_rgb_underglow_save_state_work);

    settings_load_subtree("rgb/underglow");
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
    state.on = zmk_usb_is_powered();
#endif

    if (state.on) {
        k_timer_start(&underglow_tick, K_NO_WAIT, K_MSEC(50));
    }

    return 0;
}

int zmk_rgb_underglow_save_state() {
#if IS_ENABLED(CONFIG_SETTINGS)
    int ret = k_work_reschedule(&underglow_save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
    return MIN(ret, 0);
#else
    return 0;
#endif
}

int zmk_rgb_underglow_get_state(bool *on_off) {
    if (!led_strip)
        return -ENODEV;

    *on_off = state.on;
    return 0;
}

int zmk_rgb_underglow_on() {
    if (!led_strip)
        return -ENODEV;

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    if (ext_power != NULL) {
        int rc = ext_power_enable(ext_power);
        if (rc != 0) {
            LOG_ERR("Unable to enable EXT_POWER: %d", rc);
        }
    }
#endif

    state.on = true;
    state.animation_step = 0;
    k_timer_start(&underglow_tick, K_NO_WAIT, K_MSEC(50));

    return zmk_rgb_underglow_save_state();
}

static void zmk_rgb_underglow_off_handler(struct k_work *work) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        pixels[i] = (struct led_rgb){r : 0, g : 0, b : 0};
    }

    led_strip_update_rgb(led_strip, pixels, STRIP_NUM_PIXELS);
}

K_WORK_DEFINE(underglow_off_work, zmk_rgb_underglow_off_handler);

int zmk_rgb_underglow_off() {
    if (!led_strip)
        return -ENODEV;

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    if (ext_power != NULL) {
        int rc = ext_power_disable(ext_power);
        if (rc != 0) {
            LOG_ERR("Unable to disable EXT_POWER: %d", rc);
        }
    }
#endif

    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &underglow_off_work);

    k_timer_stop(&underglow_tick);
    state.on = false;

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_calc_effect(int direction) {
    return (state.current_effect + UNDERGLOW_EFFECT_NUMBER + direction) % UNDERGLOW_EFFECT_NUMBER;
}

int zmk_rgb_underglow_select_effect(int effect) {
    if (!led_strip)
        return -ENODEV;

    if (effect < 0 || effect >= UNDERGLOW_EFFECT_NUMBER) {
        return -EINVAL;
    }

    state.current_effect = effect;
    state.animation_step = 0;

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_cycle_effect(int direction) {
    return zmk_rgb_underglow_select_effect(zmk_rgb_underglow_calc_effect(direction));
}

int zmk_rgb_underglow_toggle() {
    return state.on ? zmk_rgb_underglow_off() : zmk_rgb_underglow_on();
}

int zmk_rgb_underglow_set_hsb(struct zmk_led_hsb color) {
    if (color.h > HUE_MAX || color.s > SAT_MAX || color.b > BRT_MAX) {
        return -ENOTSUP;
    }

    state.color = color;

    return 0;
}

struct zmk_led_hsb zmk_rgb_underglow_calc_hue(int direction) {
    struct zmk_led_hsb color = state.color;

    color.h += HUE_MAX + (direction * CONFIG_ZMK_RGB_UNDERGLOW_HUE_STEP);
    color.h %= HUE_MAX;

    return color;
}

struct zmk_led_hsb zmk_rgb_underglow_calc_sat(int direction) {
    struct zmk_led_hsb color = state.color;

    int s = color.s + (direction * CONFIG_ZMK_RGB_UNDERGLOW_SAT_STEP);
    if (s < 0) {
        s = 0;
    } else if (s > SAT_MAX) {
        s = SAT_MAX;
    }
    color.s = s;

    return color;
}

struct zmk_led_hsb zmk_rgb_underglow_calc_brt(int direction) {
    struct zmk_led_hsb color = state.color;

    int b = color.b + (direction * CONFIG_ZMK_RGB_UNDERGLOW_BRT_STEP);
    color.b = CLAMP(b, 0, BRT_MAX);

    return color;
}

int zmk_rgb_underglow_change_hue(int direction) {
    if (!led_strip)
        return -ENODEV;

    state.color = zmk_rgb_underglow_calc_hue(direction);

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_change_sat(int direction) {
    if (!led_strip)
        return -ENODEV;

    state.color = zmk_rgb_underglow_calc_sat(direction);

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_change_brt(int direction) {
    if (!led_strip)
        return -ENODEV;

    state.color = zmk_rgb_underglow_calc_brt(direction);

    return zmk_rgb_underglow_save_state();
}

int zmk_rgb_underglow_change_spd(int direction) {
    if (!led_strip)
        return -ENODEV;

    if (state.animation_speed == 1 && direction < 0) {
        return 0;
    }

    state.animation_speed += direction;

    if (state.animation_speed > 5) {
        state.animation_speed = 5;
    }

    return zmk_rgb_underglow_save_state();
}

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE) ||                                          \
    IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
static int rgb_underglow_auto_state(bool *prev_state, bool new_state) {
    if (state.on == new_state) {
        return 0;
    }
    if (new_state) {
        state.on = *prev_state;
        *prev_state = false;
        return zmk_rgb_underglow_on();
    } else {
        state.on = false;
        *prev_state = true;
        return zmk_rgb_underglow_off();
    }
}

static int rgb_underglow_event_listener(const zmk_event_t *eh) {

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE)
    if (as_zmk_activity_state_changed(eh)) {
        static bool prev_state = false;
        return rgb_underglow_auto_state(&prev_state,
                                        zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE);
    }
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
    if (as_zmk_usb_conn_state_changed(eh)) {
        static bool prev_state = false;
        return rgb_underglow_auto_state(&prev_state, zmk_usb_is_powered());
    }
#endif

    return -ENOTSUP;
}

ZMK_LISTENER(rgb_underglow, rgb_underglow_event_listener);
#endif // IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE) ||
       // IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE)
ZMK_SUBSCRIPTION(rgb_underglow, zmk_activity_state_changed);
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
ZMK_SUBSCRIPTION(rgb_underglow, zmk_usb_conn_state_changed);
#endif

SYS_INIT(zmk_rgb_underglow_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
