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
#include <nrfx_saadc.h>
#include <nrfx_lpcomp.h>

struct capsense_config {
    const char *label;
    const uint32_t pin;
    const uint8_t flags;
    const uint16_t threshold;

    const char *adc_label;
    uint8_t adc_channel;
};

struct capsense_data {
    const struct device *dev;
    const struct device *gpio_dev;
    uint32_t value;
    bool presence;

    struct k_delayed_work work;

    const struct device *adc;
    nrfx_lpcomp_config_t comp_cfg;

    // trigger
    sensor_trigger_handler_t handler;
    struct sensor_trigger *trigger;
};
