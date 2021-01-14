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
#include <nrfx_comp.h>

struct capsense_config {
    const char *label;
    const uint32_t pin;
    const uint8_t flags;
    const uint16_t threshold;
    
    const char *adc_label;
    uint8_t adc_channel;
};

struct capsense_data {
    const struct device *gpio_dev;
    uint8_t ab_state;
    int8_t pulses;
    int8_t ticks;
    int8_t delta;
    
    const struct device *adc;
    nrfx_saadc_channel_t ch_cfg;
    nrf_saadc_value_t raw[16];
    
    nrfx_comp_config_t comp_cfg;

    struct k_work work;
};

