/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_capsense

#include <device.h>
#include <nrfx_saadc.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <kernel.h>
#include <logging/log.h>
#include <sys/__assert.h>
#include <sys/util.h>

#include "capsense.h"

#ifdef CONFIG_ADC_NRFX_SAADC
#define GROVE_GAIN ADC_GAIN_1_4
#define GROVE_REF ADC_REF_VDD_1_4
#define GROVE_RESOLUTION 12
#else
#define GROVE_GAIN ADC_GAIN_1
#define GROVE_REF ADC_REF_VDD_1
#define GROVE_RESOLUTION 12
#endif

LOG_MODULE_REGISTER(capsense, CONFIG_SENSOR_LOG_LEVEL);
struct k_timer capsense_timer;

static const struct device *sdev;

static const struct sensor_driver_api capsense_api = {};

// static void capsense_handler(const struct device* dev) {
static int adc_init(const struct device *dev, bool pullup, bool pulldown) {
    struct capsense_data *drv_data = dev->data;
    const struct capsense_config *cfg = dev->config;
    nrfx_err_t err;
    drv_data->ch_cfg = (nrfx_saadc_channel_t)NRFX_SAADC_DEFAULT_CHANNEL_SE(cfg->adc_channel, 0);
    if (pullup)
        drv_data->ch_cfg.channel_config.resistor_p = NRF_SAADC_RESISTOR_PULLUP;
    if (pulldown)
        drv_data->ch_cfg.channel_config.resistor_p = NRF_SAADC_RESISTOR_PULLDOWN;
    nrfx_saadc_uninit();
    if ((err = nrfx_saadc_init(20)) != NRFX_SUCCESS) {
        LOG_WRN("SAADC already initialized (%X)", err);
    }
    if ((err = nrfx_saadc_channels_config(&drv_data->ch_cfg, 1)) != NRFX_SUCCESS) {
        LOG_ERR("Failed to configure SAADC channel (%X)", err);
        return -EIO;
    }
    if ((err = nrfx_saadc_simple_mode_set(1, NRF_SAADC_RESOLUTION_12BIT,
                                          NRF_SAADC_OVERSAMPLE_DISABLED, NULL)) != NRFX_SUCCESS) {
        LOG_ERR("Failed to set SAADC resolution (%X)", err);
        return -EIO;
    }
    if ((err = nrfx_saadc_buffer_set(drv_data->raw, 1)) != NRFX_SUCCESS) {
        LOG_ERR("Failed to set SAADC buffer (%X)", err - NRFX_ERROR_BASE_NUM);
        return -EIO;
    }
    return 0;
}
static void capsense_handler(struct k_timer *exp) {
    static int i = 1;
    struct capsense_data *drv_data = sdev->data;
    LOG_DBG("capsense timer interrupt %d", i++);
    nrfx_err_t err;
    adc_init(sdev, false, false);
    static nrf_saadc_value_t buf[128] = {0};
    for (int j = 0; j < 100; j++) {
        if ((err = nrfx_saadc_mode_trigger()) != NRFX_SUCCESS) {
            LOG_ERR("Could not trigger SAADC: %d", err);
        }
        buf[j] = drv_data->raw[0];
    }
    for (int j = 0; j < 100; j++)
        LOG_DBG("Read from adc: %d", buf[j]);
    adc_init(sdev, true, false);
}

static int capsense_init(const struct device *dev) {
    struct capsense_data *drv_data = dev->data;
    const struct capsense_config *cfg = dev->config;
    sdev = dev;

    // drv_data->adc = device_get_binding(cfg->adc_label);
    // nrfx_err_t err;

    // if (drv_data->adc == NULL) {
    //    LOG_ERR("Failed to get ADC device.");
    //    return -EINVAL;
    //}
    if (adc_init(sdev, true, false)) {
        LOG_ERR("Could not initialize SAADC");
        return -EIO;
    }
    // timers
    k_timer_init(&capsense_timer, capsense_handler, NULL);
    k_timer_start(&capsense_timer, K_SECONDS(10), K_SECONDS(10));

    LOG_DBG("Capsense initialized");
    LOG_DBG("GPIO pin: %d, ADC pin: %d", cfg->pin, cfg->adc_channel);

    return 0;
}

static struct capsense_data capsense_data;
// adc channel +1 for nrfx
static const struct capsense_config capsense_cfg = {.adc_label = DT_INST_IO_CHANNELS_LABEL(0),
                                                    .adc_channel = DT_INST_IO_CHANNELS_INPUT(0) + 1,
                                                    .pin = DT_INST_GPIO_PIN(0, gpios),
                                                    .label = DT_INST_GPIO_LABEL(0, gpios),
                                                    .flags = DT_INST_GPIO_FLAGS(0, gpios)};

DEVICE_AND_API_INIT(capsense_dev, DT_INST_LABEL(0), &capsense_init, &capsense_data, &capsense_cfg,
                    POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &capsense_api);

