/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <stdint.h>

struct csense_config {
    const char *label;
    const uint32_t pin;
    const uint8_t flags;
    const uint16_t threshold;
};

struct csense_data {
    const struct device *gpio_dev;
    uint8_t ab_state;
    int8_t pulses;
    int8_t ticks;
    int8_t delta;
};

