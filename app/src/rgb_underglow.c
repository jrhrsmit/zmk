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
#include <string.h>

#include <zephyr/logging/log.h>

#include <zephyr/drivers/led_strip.h>
#include <drivers/ext_power.h>

#include <zmk/rgb_underglow.h>

#include <zmk/activity.h>
#include <zmk/usb.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/workqueue.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if !DT_HAS_CHOSEN(zmk_underglow)

#error "A zmk,underglow chosen node must be declared"

#endif

#define STRIP_CHOSEN DT_CHOSEN(zmk_underglow)
#define STRIP_NUM_PIXELS DT_PROP(STRIP_CHOSEN, chain_length)

#define HAS_UNDERGLOW_MAP DT_HAS_CHOSEN(zmk_underglow_map)

#if HAS_UNDERGLOW_MAP
#define UNDERGLOW_MAP DT_CHOSEN(zmk_underglow_map)

BUILD_ASSERT(DT_PROP_LEN(UNDERGLOW_MAP, positions) == 2 * STRIP_NUM_PIXELS,
             "zmk,underglow-map positions must have one <x y> pair per LED");
BUILD_ASSERT(DT_PROP_LEN(UNDERGLOW_MAP, strip_index) == STRIP_NUM_PIXELS,
             "zmk,underglow-map strip-index must have one entry per LED");
#endif

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
#if HAS_UNDERGLOW_MAP
    UNDERGLOW_EFFECT_RIPPLE,
    UNDERGLOW_EFFECT_MATRIX,
#endif
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
static const struct device *const ext_power = DEVICE_DT_GET(DT_INST(0, zmk_ext_power_generic));
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
    float r = 0, g = 0, b = 0;

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

static void zmk_rgb_underglow_effect_solid(void) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        pixels[i] = hsb_to_rgb(hsb_scale_min_max(state.color));
    }
}

static void zmk_rgb_underglow_effect_breathe(void) {
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

static void zmk_rgb_underglow_effect_spectrum(void) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = state.animation_step;

        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }

    state.animation_step += state.animation_speed;
    state.animation_step = state.animation_step % HUE_MAX;
}

static void zmk_rgb_underglow_effect_swirl(void) {
    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        struct zmk_led_hsb hsb = state.color;
        hsb.h = (HUE_MAX / STRIP_NUM_PIXELS * i + state.animation_step) % HUE_MAX;

        pixels[i] = hsb_to_rgb(hsb_scale_min_max(hsb));
    }

    state.animation_step += state.animation_speed * 2;
    state.animation_step = state.animation_step % HUE_MAX;
}

#if HAS_UNDERGLOW_MAP
/*
 * Position-aware effects. LEDs are addressed by their logical index in the
 * zmk,underglow-map node: key positions first (so a keymap position is also
 * a logical LED index), then LEDs that are not under a key.
 */
static const uint16_t led_positions[] = DT_PROP(UNDERGLOW_MAP, positions);
static const uint16_t led_strip_index[] = DT_PROP(UNDERGLOW_MAP, strip_index);

#define LED_X(i) led_positions[2 * (i)]
#define LED_Y(i) led_positions[2 * (i) + 1]

static uint16_t map_min_x, map_max_x;
static float map_diagonal;

static void set_led(int i, struct led_rgb rgb) {
    if (led_strip_index[i] < STRIP_NUM_PIXELS) {
        pixels[led_strip_index[i]] = rgb;
    }
}

/* Tiny xorshift PRNG; the effects only need something that looks random */
static uint32_t rand_state = 1;

static uint32_t effect_rand(void) {
    rand_state ^= rand_state << 13;
    rand_state ^= rand_state >> 17;
    rand_state ^= rand_state << 5;
    return rand_state;
}

/*
 * Ripple: every key press starts a ring of a random hue that expands from the
 * pressed key and fades out as it grows.
 */
#define RIPPLE_MAX 24
/* Half the width of a ring, in map units */
#define RIPPLE_WIDTH 12.0f

struct ripple {
    bool active;
    uint16_t x, y;
    float radius;
    struct led_rgb color;
};

static struct ripple ripples[RIPPLE_MAX];
static uint8_t ripple_next;
static struct k_spinlock ripple_lock;

static void zmk_rgb_underglow_ripple_start(uint32_t position) {
    if (position >= STRIP_NUM_PIXELS) {
        return;
    }

    /* Key press timing is a decent source of entropy for the hue */
    rand_state ^= k_cycle_get_32();
    if (rand_state == 0) {
        rand_state = 1;
    }

    struct zmk_led_hsb hsb = state.color;
    hsb.h = effect_rand() % HUE_MAX;

    K_SPINLOCK(&ripple_lock) {
        ripples[ripple_next] = (struct ripple){
            .active = true,
            .x = LED_X(position),
            .y = LED_Y(position),
            .radius = 0,
            .color = hsb_to_rgb(hsb_scale_min_max(hsb)),
        };
        ripple_next = (ripple_next + 1) % RIPPLE_MAX;
    }
}

static void zmk_rgb_underglow_effect_ripple(void) {
    struct ripple current[RIPPLE_MAX];
    float max_radius = map_diagonal + RIPPLE_WIDTH;

    K_SPINLOCK(&ripple_lock) {
        memcpy(current, ripples, sizeof(ripples));

        for (int j = 0; j < RIPPLE_MAX; j++) {
            ripples[j].radius += state.animation_speed + 1;
            if (ripples[j].radius > max_radius) {
                ripples[j].active = false;
            }
        }
    }

    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        float r = 0, g = 0, b = 0;

        for (int j = 0; j < RIPPLE_MAX; j++) {
            if (!current[j].active) {
                continue;
            }

            float dx = (float)LED_X(i) - current[j].x;
            float dy = (float)LED_Y(i) - current[j].y;
            float offset = fabsf(sqrtf(dx * dx + dy * dy) - current[j].radius);
            if (offset >= RIPPLE_WIDTH) {
                continue;
            }

            /* Squared falloff across the ring looks smoother than linear */
            float ring = 1.0f - offset / RIPPLE_WIDTH;
            float intensity = ring * ring * (1.0f - current[j].radius / max_radius);

            r += current[j].color.r * intensity;
            g += current[j].color.g * intensity;
            b += current[j].color.b * intensity;
        }

        set_led(i, (struct led_rgb){
                       .r = MIN(r, 255),
                       .g = MIN(g, 255),
                       .b = MIN(b, 255),
                   });
    }
}

/*
 * Matrix: streaks run along each row of LEDs, with a white head fading into
 * a tail of the underglow color.
 */
#define MATRIX_MAX_ROWS 8
/* Length of the white-to-color transition behind a streak's head, in map units */
#define MATRIX_HEAD_LENGTH 8.0f

struct matrix_streak {
    bool active;
    float head;
    float speed;
    float length;
};

static uint8_t led_row[STRIP_NUM_PIXELS];
static struct matrix_streak streaks[MATRIX_MAX_ROWS];
static uint8_t matrix_num_rows;

static void zmk_rgb_underglow_effect_matrix(void) {
    for (int row = 0; row < matrix_num_rows; row++) {
        struct matrix_streak *streak = &streaks[row];

        if (!streak->active && effect_rand() % 20 == 0) {
            *streak = (struct matrix_streak){
                .active = true,
                .head = map_min_x,
                .speed = (1.0f + (effect_rand() % 30) / 10.0f) * (state.animation_speed + 1) / 2,
                .length = 16 + effect_rand() % 48,
            };
        }
    }

    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        const struct matrix_streak *streak = &streaks[led_row[i]];
        float behind = streak->head - LED_X(i);

        if (!streak->active || behind < 0 || behind > streak->length) {
            set_led(i, (struct led_rgb){0});
            continue;
        }

        struct zmk_led_hsb hsb = state.color;
        float fade = 1.0f - behind / streak->length;

        hsb.b = hsb.b * fade * fade;
        if (behind < MATRIX_HEAD_LENGTH) {
            hsb.s = hsb.s * behind / MATRIX_HEAD_LENGTH;
        }

        set_led(i, hsb_to_rgb(hsb_scale_zero_max(hsb)));
    }

    for (int row = 0; row < matrix_num_rows; row++) {
        struct matrix_streak *streak = &streaks[row];

        if (streak->active) {
            streak->head += streak->speed;
            if (streak->head - streak->length > map_max_x) {
                streak->active = false;
            }
        }
    }
}

static void zmk_rgb_underglow_map_init(void) {
    uint16_t row_y[MATRIX_MAX_ROWS];
    uint16_t min_y = UINT16_MAX, max_y = 0;

    map_min_x = UINT16_MAX;
    map_max_x = 0;
    matrix_num_rows = 0;

    for (int i = 0; i < STRIP_NUM_PIXELS; i++) {
        if (led_strip_index[i] >= STRIP_NUM_PIXELS) {
            LOG_ERR("zmk,underglow-map strip-index %d is out of range", led_strip_index[i]);
        }

        map_min_x = MIN(map_min_x, LED_X(i));
        map_max_x = MAX(map_max_x, LED_X(i));
        min_y = MIN(min_y, LED_Y(i));
        max_y = MAX(max_y, LED_Y(i));

        /* Group LEDs into rows by y; past the row limit, join the closest row */
        int row, closest = 0;
        for (row = 0; row < matrix_num_rows; row++) {
            if (row_y[row] == LED_Y(i)) {
                break;
            }
            if (abs(row_y[row] - LED_Y(i)) < abs(row_y[closest] - LED_Y(i))) {
                closest = row;
            }
        }
        if (row == matrix_num_rows) {
            if (matrix_num_rows < MATRIX_MAX_ROWS) {
                row_y[matrix_num_rows++] = LED_Y(i);
            } else {
                row = closest;
            }
        }
        led_row[i] = row;
    }

    float w = map_max_x - map_min_x;
    float h = max_y - min_y;
    map_diagonal = sqrtf(w * w + h * h);
    rand_state = k_cycle_get_32() | 1;
}
#endif // HAS_UNDERGLOW_MAP

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
#if HAS_UNDERGLOW_MAP
    case UNDERGLOW_EFFECT_RIPPLE:
        zmk_rgb_underglow_effect_ripple();
        break;
    case UNDERGLOW_EFFECT_MATRIX:
        zmk_rgb_underglow_effect_matrix();
        break;
#endif
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
            if (state.on) {
                k_timer_start(&underglow_tick, K_NO_WAIT, K_MSEC(50));
            }

            return 0;
        }

        return rc;
    }

    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(rgb_underglow, "rgb/underglow", NULL, rgb_settings_set, NULL, NULL);

static void zmk_rgb_underglow_save_state_work(struct k_work *_work) {
    settings_save_one("rgb/underglow/state", &state, sizeof(state));
}

static struct k_work_delayable underglow_save_work;
#endif

static int zmk_rgb_underglow_init(void) {
    led_strip = DEVICE_DT_GET(STRIP_CHOSEN);

#if HAS_UNDERGLOW_MAP
    zmk_rgb_underglow_map_init();
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_EXT_POWER)
    if (!device_is_ready(ext_power)) {
        LOG_ERR("External power device \"%s\" is not ready", ext_power->name);
        return -ENODEV;
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
    k_work_init_delayable(&underglow_save_work, zmk_rgb_underglow_save_state_work);
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
    state.on = zmk_usb_is_powered();
#endif

    if (state.on) {
        k_timer_start(&underglow_tick, K_NO_WAIT, K_MSEC(50));
    }

    return 0;
}

int zmk_rgb_underglow_save_state(void) {
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

int zmk_rgb_underglow_on(void) {
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

int zmk_rgb_underglow_off(void) {
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

int zmk_rgb_underglow_toggle(void) {
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
struct rgb_underglow_sleep_state {
    bool is_awake;
    bool rgb_state_before_sleeping;
};

static int rgb_underglow_auto_state(bool target_wake_state) {
    static struct rgb_underglow_sleep_state sleep_state = {
        is_awake : true,
        rgb_state_before_sleeping : false
    };

    // wake up event while awake, or sleep event while sleeping -> no-op
    if (target_wake_state == sleep_state.is_awake) {
        return 0;
    }
    sleep_state.is_awake = target_wake_state;

    if (sleep_state.is_awake) {
        if (sleep_state.rgb_state_before_sleeping) {
            return zmk_rgb_underglow_on();
        } else {
            return zmk_rgb_underglow_off();
        }
    } else {
        sleep_state.rgb_state_before_sleeping = state.on;
        return zmk_rgb_underglow_off();
    }
}

static int rgb_underglow_event_listener(const zmk_event_t *eh) {

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_IDLE)
    if (as_zmk_activity_state_changed(eh)) {
        return rgb_underglow_auto_state(zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE);
    }
#endif

#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW_AUTO_OFF_USB)
    if (as_zmk_usb_conn_state_changed(eh)) {
        return rgb_underglow_auto_state(zmk_usb_is_powered());
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

#if HAS_UNDERGLOW_MAP
static int rgb_underglow_position_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);

    if (ev->state && state.on && state.current_effect == UNDERGLOW_EFFECT_RIPPLE) {
        zmk_rgb_underglow_ripple_start(ev->position);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(rgb_underglow_position, rgb_underglow_position_listener);
ZMK_SUBSCRIPTION(rgb_underglow_position, zmk_position_state_changed);
#endif // HAS_UNDERGLOW_MAP

SYS_INIT(zmk_rgb_underglow_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
