/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT nrf_csense

#include <device.h>
#include <drivers/gpio.h>
#include <sys/util.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <logging/log.h>
#include <nrf_csense.h>

#include "csense.h"
#include "nrf_csense.h"
#include "nrf_csense_macros.h"

LOG_MODULE_REGISTER(CSENSE, CONFIG_SENSOR_LOG_LEVEL);

static const struct sensor_driver_api csense_driver_api = {
#ifdef CONFIG_EC11_TRIGGER
    .trigger_set = ec11_trigger_set,
#endif
    .sample_fetch = ec11_sample_fetch,
    .channel_get = ec11_channel_get,
};

static void csense_handler(const struct device *dev) {
    static int i = 1;
    LOG_DBG("csense interrupt %d", i++);
}

int csense_init(const struct device *dev) {
    struct ec11_data *drv_data = dev->data;
    const struct ec11_config *drv_cfg = dev->config;

    drv_data->a = device_get_binding(drv_cfg->a_label);
    
    NRF_CSENSE_BUTTON_DEF(button, (drv_cfg->pin, drv_cfg->threshold));
    nrf_csense_init(nrf_csense_handler, 32768 * (1000 / drv_cfg->period));
    nrf_csense_instance_context_set();

    drv_data->ab_state = ec11_get_ab_state(dev);

    return 0;
}

#define EC11_INST(n)                                                                               \
    struct csense_data csense_data_##n;                                                                \
    const struct csense_config csense_cfg_##n = {                                                      \
        .pin = DT_INST_GPIO_PIN(n, a_gpios),                                                     \
        .threshold = DT_INST_GPIO_FLAGS(n, a_gpios),                                                 \
        .period = DT_INST_GPIO_LABEL(n, b_gpios),                                                 \
        COND_CODE_0(DT_INST_NODE_HAS_PROP(n, resolution), (1), (DT_INST_PROP(n, resolution))),     \
    };                                                                                             \
    DEVICE_AND_API_INIT(csense_##n, DT_INST_LABEL(n), csense_init, &csense_data_##n, &csense_cfg_##n,      \
                        POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &csense_driver_api);

DT_INST_FOREACH_STATUS_OKAY(EC11_INST)
