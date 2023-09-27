/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>

struct sensor_key_config {
    const struct gpio_dt_spec pin;
};

struct sensor_key_data {
    const struct device *pin;
    uint8_t state;

#ifdef CONFIG_SENSOR_KEY_TRIGGER
    struct gpio_callback gpio_cb;
    const struct device *dev;

    sensor_trigger_handler_t handler;
    const struct sensor_trigger *trigger;

#if defined(CONFIG_SENSOR_KEY_TRIGGER_OWN_THREAD)
    K_THREAD_STACK_MEMBER(thread_stack, CONFIG_SENSOR_KEY_THREAD_STACK_SIZE);
    struct k_sem gpio_sem;
    struct k_thread thread;
#elif defined(CONFIG_SENSOR_KEY_TRIGGER_GLOBAL_THREAD)
    struct k_work work;
#endif

#endif /* CONFIG_SENSOR_KEY_TRIGGER */
};

#ifdef CONFIG_SENSOR_KEY_TRIGGER

int sensor_key_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                           sensor_trigger_handler_t handler);

int sensor_key_init_interrupt(const struct device *dev);
#endif
