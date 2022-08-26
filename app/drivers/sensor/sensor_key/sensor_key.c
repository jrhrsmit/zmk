/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_sensor_key

#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <logging/log.h>

#include "sensor_key.h"

LOG_MODULE_REGISTER(sensor_key, CONFIG_SENSOR_LOG_LEVEL);

static int sensor_key_sample_fetch(const struct device *dev, enum sensor_channel chan) {
    struct sensor_key_data *drv_data = dev->data;
    const struct sensor_key_config *drv_cfg = dev->config;

    __ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL || chan == SENSOR_CHAN_VOLTAGE);

    drv_data->state = gpio_pin_get(drv_data->pin, drv_cfg->pin);

    return 0;
}

static int sensor_key_channel_get(const struct device *dev, enum sensor_channel chan,
                                  struct sensor_value *val) {
    struct sensor_key_data *drv_data = dev->data;

    if (chan != SENSOR_CHAN_VOLTAGE) {
        return -ENOTSUP;
    }

    val->val1 = drv_data->state;

    return 0;
}

static const struct sensor_driver_api sensor_key_driver_api = {
#ifdef CONFIG_SENSOR_KEY_TRIGGER
    .trigger_set = sensor_key_trigger_set,
#endif
    .sample_fetch = sensor_key_sample_fetch,
    .channel_get = sensor_key_channel_get,
};

int sensor_key_init(const struct device *dev) {
    struct sensor_key_data *drv_data = dev->data;
    const struct sensor_key_config *drv_cfg = dev->config;

    LOG_DBG("Sensor key pin: %s %d", drv_cfg->label, drv_cfg->pin);

    drv_data->pin = device_get_binding(drv_cfg->label);
    if (drv_data->pin == NULL) {
        LOG_ERR("Failed to get pointer t oGPIO device");
        return -EINVAL;
    }

    if (gpio_pin_configure(drv_data->pin, drv_cfg->pin, drv_cfg->flags | GPIO_INPUT)) {
        LOG_DBG("Failed to configure pin");
        return -EIO;
    }

#ifdef CONFIG_SENSOR_KEY_TRIGGER
    if (sensor_key_init_interrupt(dev) < 0) {
        LOG_DBG("Failed to initialize interrupt!");
        return -EIO;
    }
#endif

    drv_data->state = gpio_pin_get(drv_data->pin, drv_cfg->pin);

    return 0;
}

#define SENSOR_KEY_INST(n)                                                                         \
    struct sensor_key_data sensor_key_data_##n;                                                    \
    const struct sensor_key_config sensor_key_cfg_##n = {                                          \
        .label = DT_INST_GPIO_LABEL(n, gpios),                                                     \
        .pin = DT_INST_GPIO_PIN(n, gpios),                                                         \
        .flags = DT_INST_GPIO_FLAGS(n, gpios),                                                     \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, sensor_key_init, NULL, &sensor_key_data_##n, &sensor_key_cfg_##n,     \
                          POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &sensor_key_driver_api);

DT_INST_FOREACH_STATUS_OKAY(SENSOR_KEY_INST)
