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
#include <zmk/events/keycode_state_changed.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>

#include <zephyr/random/rand32.h>

#include <zmk/rgb_underglow.h>

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
    UNDERGLOW_EFFECT_MATRIX,
    UNDERGLOW_EFFECT_HEATMAP,
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

struct ripple_effect_event {
    uint8_t pos;
    uint8_t ticks_passed;
    struct led_rgb rgb;
};

#define RIPPLE_EFFECTS_BUF_SIZE 24
static struct ripple_effect_event ripple_effects[RIPPLE_EFFECTS_BUF_SIZE] = {0};

#define STRIP_X_IDX(i) (2 * i)
#define STRIP_Y_IDX(i) (2 * i + 1)

static const uint8_t pixel_locations[ZMK_LEDSTRIP_MAP_LEN] = ZMK_LEDSTRIP_MAP;
static const uint8_t pixel_index[ZMK_LEDSTRIP_MAP_LEN] = ZMK_LEDSTRIP_INDEX;

static uint8_t pixel_distances[STRIP_NUM_PIXELS][STRIP_NUM_PIXELS];
static void init_ripple_pixel_distances(void) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        for (int j = 0; j < STRIP_NUM_PIXELS; j++) {
            int dx = pixel_locations[STRIP_X_IDX(i)] - pixel_locations[STRIP_X_IDX(j)];
            int dy = pixel_locations[STRIP_Y_IDX(i)] - pixel_locations[STRIP_Y_IDX(j)];
            pixel_distances[i][j] = (uint8_t)round(sqrtf((float)(dx * dx + dy * dy)));
        }
    }
}

#define RIPPLE_THICKNESS 12

const uint8_t gamma_lut[256] = {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   1,   1,   1,   1,
    1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   4,
    4,   4,   4,   5,   5,   5,   5,   6,   6,   6,   6,   7,   7,   7,   8,   8,   8,   9,   9,
    9,   10,  10,  11,  11,  11,  12,  12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  17,  17,
    18,  18,  19,  19,  20,  20,  21,  22,  22,  23,  23,  24,  25,  25,  26,  26,  27,  28,  28,
    29,  30,  30,  31,  32,  33,  33,  34,  35,  35,  36,  37,  38,  39,  39,  40,  41,  42,  43,
    43,  44,  45,  46,  47,  48,  49,  49,  50,  51,  52,  53,  54,  55,  56,  57,  58,  59,  60,
    61,  62,  63,  64,  65,  66,  67,  68,  69,  70,  71,  73,  74,  75,  76,  77,  78,  79,  81,
    82,  83,  84,  85,  87,  88,  89,  90,  91,  93,  94,  95,  97,  98,  99,  100, 102, 103, 105,
    106, 107, 109, 110, 111, 113, 114, 116, 117, 119, 120, 121, 123, 124, 126, 127, 129, 130, 132,
    133, 135, 137, 138, 140, 141, 143, 145, 146, 148, 149, 151, 153, 154, 156, 158, 159, 161, 163,
    165, 166, 168, 170, 172, 173, 175, 177, 179, 181, 182, 184, 186, 188, 190, 192, 194, 196, 197,
    199, 201, 203, 205, 207, 209, 211, 213, 215, 217, 219, 221, 223, 225, 227, 229, 231, 234, 236,
    238, 240, 242, 244, 246, 248, 251, 253, 255,
};

static void zmk_rgb_underglow_effect_ripple() {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct led_rgb rgb = {0};
        for (int j = 0; j < RIPPLE_EFFECTS_BUF_SIZE; j++) {
            if (ripple_effects[j].ticks_passed == 0)
                continue;
            float pixel_distance;
            float speed = 2;
            pixel_distance = pixel_distances[i][ripple_effects[j].pos];
            float ripple_distance = ripple_effects[j].ticks_passed * speed;

            if (pixel_distance > ripple_distance - RIPPLE_THICKNESS &&
                pixel_distance < ripple_distance + RIPPLE_THICKNESS) {
                float intensity = 1.0 / (float)RIPPLE_THICKNESS *
                                  (float)(RIPPLE_THICKNESS - abs(pixel_distance - ripple_distance));
                rgb.r = CLAMP(rgb.r + (int)((float)ripple_effects[j].rgb.r * intensity), 0, 255);
                rgb.g = CLAMP(rgb.g + (int)((float)ripple_effects[j].rgb.g * intensity), 0, 255);
                rgb.b = CLAMP(rgb.b + (int)((float)ripple_effects[j].rgb.b * intensity), 0, 255);
            }
        }

        rgb.r = CLAMP(gamma_lut[rgb.r], 0, 255 * CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX / 100);
        rgb.g = CLAMP(gamma_lut[rgb.g], 0, 255 * CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX / 100);
        rgb.b = CLAMP(gamma_lut[rgb.b], 0, 255 * CONFIG_ZMK_RGB_UNDERGLOW_BRT_MAX / 100);
        pixels[pixel_index[i]] = rgb;
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
    struct zmk_led_hsb hsb = state.color;
    ripple_effects[ripple_effects_i].pos = pos;
    ripple_effects[ripple_effects_i].ticks_passed = 1;
    hsb.h = sys_rand32_get() % HUE_MAX;
    ripple_effects[ripple_effects_i].rgb = hsb_to_rgb(hsb);
    ripple_effects_i++;
    if (ripple_effects_i >= RIPPLE_EFFECTS_BUF_SIZE)
        ripple_effects_i = 0;
}

static uint32_t heatmap_hits[STRIP_NUM_PIXELS] = {0};
static uint32_t heatmap_hits_max = 1;

static void zmk_rgb_underglow_effect_heatmap(void) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = 240.0 - (240.0 / (float)heatmap_hits_max) * (float)heatmap_hits[i];
        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }
}

static void zmk_rgb_underglow_effect_heatmap_handler(int pos) {
    int i = pixel_index[pos];
    heatmap_hits[i]++;
    heatmap_hits_max = MAX(heatmap_hits_max, heatmap_hits[i]);
}

#define MAX_ROWS 10

struct pixel_by_rows {
    uint8_t index[MAX_ROWS][STRIP_NUM_PIXELS];
    uint8_t row_elements[MAX_ROWS];
    uint8_t num_rows;
};

static struct pixel_by_rows pixel_rows = {.index = {{0}}, .row_elements = {0}, .num_rows = 0};

struct matrix_effect_event {
    float speed;
    int ticks_passed;
    int length;
};

#define MATRIX_EFFECT_DEFAULT(i, _)                                                                \
    { 0, 0, 0 }
static struct matrix_effect_event matrix_effects[MAX_ROWS] = {
    LISTIFY(MAX_ROWS, MATRIX_EFFECT_DEFAULT, (, ))};

void init_matrix_effect(void) {
    // set up the first pixel of the first row
    pixel_rows.num_rows = 1;
    pixel_rows.index[pixel_rows.num_rows - 1][pixel_rows.row_elements[pixel_rows.num_rows - 1]] = 0;
    pixel_rows.row_elements[pixel_rows.num_rows - 1]++;
    // fill in the rest
    for (int i = 1; i < STRIP_NUM_PIXELS; i++) {
        int j;
        for (j = 0; j < pixel_rows.num_rows; j++) {
            if (pixel_locations[STRIP_Y_IDX(i)] ==
                pixel_locations[STRIP_Y_IDX(pixel_rows.index[j][0])]) {
                break;
            }
        }
        if (j == pixel_rows.num_rows) {
            if (pixel_rows.num_rows >= MAX_ROWS)
                break;
            pixel_rows.num_rows++;
        }
        pixel_rows.index[j][pixel_rows.row_elements[j]] = i;
        pixel_rows.row_elements[j]++;
    }
}

#define MATRIX_EFFECT_GLOW_LENGTH 32

static void zmk_rgb_underglow_effect_matrix() {
    // create new lines
    for (int i = 0; i < pixel_rows.num_rows; i++) {
        if (matrix_effects[i].ticks_passed > 0) {
            matrix_effects[i].ticks_passed++;
        } else if (sys_rand32_get() % 20 == 0) {
            matrix_effects[i].ticks_passed = 1;
            matrix_effects[i].speed = 1.0 + (sys_rand32_get() % 50) / 10.0;
            matrix_effects[i].length = sys_rand32_get() % 100 + 20;
        }
    }

    // process old lines
    struct zmk_led_hsb hsb = state.color;
    for (int i = 0; i < pixel_rows.num_rows; i++) {
        if (matrix_effects[i].ticks_passed > 0) {
            for (int j = 0; j < pixel_rows.row_elements[i]; j++) {
                int x = pixel_locations[STRIP_X_IDX(pixel_rows.index[i][j])];
                int head_x = (float)matrix_effects[i].ticks_passed * matrix_effects[i].speed;
                int tail_x = head_x - matrix_effects[i].length;
                if (x == head_x) {
                    hsb.s = 0;
                } else if (x >= (head_x - MATRIX_EFFECT_GLOW_LENGTH) && x < head_x) {
                    hsb.s = (head_x - x) / (float)MATRIX_EFFECT_GLOW_LENGTH * SAT_MAX;
                } else if (x < (head_x - MATRIX_EFFECT_GLOW_LENGTH) && x >= tail_x) {
                    hsb.s = SAT_MAX;
                } else {
                    hsb.b = 0;
                }
                pixels[pixel_index[pixel_rows.index[i][j]]] = hsb_to_rgb(hsb_scale_min_max(hsb));
                if (tail_x > 138) {
                    matrix_effects[i].ticks_passed = 0;
                }
            }
        }
    }
}

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
    case UNDERGLOW_EFFECT_MATRIX:
        zmk_rgb_underglow_effect_matrix();
        break;
    case UNDERGLOW_EFFECT_HEATMAP:
        zmk_rgb_underglow_effect_heatmap();
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
    led_strip = device_get_binding(DT_PROP(STRIP_CHOSEN, label));
    if (led_strip) {
        LOG_INF("Found LED strip device %s", DT_PROP(STRIP_CHOSEN, label));
    } else {
        LOG_ERR("LED strip device %s not found", DT_PROP(STRIP_CHOSEN, label));
        return -EINVAL;
    }
    init_ripple_pixel_distances();
    init_matrix_effect();

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

int zmk_rgb_underglow_get_effect(int *effect) {
    *effect = state.current_effect;

    return 0;
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

int zmk_rgb_underglow_get_hsb(struct zmk_led_hsb *color) {
    *color = state.color;

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

    if (as_zmk_position_state_changed(eh)) {
        struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
        if (ev->state) {
            switch (state.current_effect) {
            case UNDERGLOW_EFFECT_RIPPLE:
                zmk_rgb_underglow_effect_ripple_handler(ev->position);
                break;
            case UNDERGLOW_EFFECT_HEATMAP:
                zmk_rgb_underglow_effect_heatmap_handler(ev->position);
                break;
            }
        }
    }
    return ZMK_EV_EVENT_BUBBLE;

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

ZMK_SUBSCRIPTION(rgb_underglow, zmk_position_state_changed);

SYS_INIT(zmk_rgb_underglow_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
