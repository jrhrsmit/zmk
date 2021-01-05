/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT nordic_csense

#include <device.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <kernel.h>
#include <logging/log.h>
#include <sys/__assert.h>
#include <sys/util.h>

#include "csense.h"
#include "nrf_drv_csense.h"

#include <nrfx_comp.h>

LOG_MODULE_REGISTER(CSENSE, CONFIG_SENSOR_LOG_LEVEL);
struct k_timer csense_timer;

static const struct sensor_driver_api csense_driver_api = {
    //#ifdef CONFIG_EC11_TRIGGER
    //    .trigger_set = ec11_trigger_set,
    //#endif
    //    .sample_fetch = ec11_sample_fetch,
    //    .channel_get = ec11_channel_get,
};

//static void csense_handler(const struct device* dev) {
static void csense_handler(struct k_timer* exp) {
    static int i = 1;
    nrfx_err_t err;
    LOG_DBG("csense timer interrupt %d", i++);
    err = nrf_drv_csense_sample();
    if(err != NRFX_SUCCESS) {
        LOG_INF("Csense driver still busy with last sample");
    }
}

static void evt_handler(nrf_drv_csense_evt_t * p_event_struct) {
    static int i=1;
    LOG_DBG("csense evt_handler interrupt %d, value: %d", i++, p_event_struct->read_value);
}

int csense_init(const struct device* dev) {
    struct csense_data* drv_data = dev->data;
    const struct csense_config* drv_cfg = dev->config;

    drv_data->gpio_dev = device_get_binding(drv_cfg->label);
    if (drv_data->gpio_dev == NULL) {
        LOG_ERR("Failed to get pointer to GPIO device");
        return -EINVAL;
    }

    if (gpio_pin_configure(drv_data->gpio_dev, drv_cfg->pin,
                           drv_cfg->flags | GPIO_INPUT)) {
        LOG_ERR("Failed to configure CSENSE pin");
        return -EIO;
    }

    nrf_drv_csense_config_t config = {0};
    if (nrf_drv_csense_init(&config, evt_handler) != NRFX_SUCCESS){
        LOG_ERR("Failed to initialize csense driver!");
    }
    nrf_drv_csense_channels_enable(1 << drv_cfg->pin);
    
    k_timer_init(&csense_timer, csense_handler, NULL);
    k_timer_start(&csense_timer, K_SECONDS(10), K_SECONDS(10));

    LOG_DBG("Initializing CSENSE (dbg)");
    LOG_DBG("csense pin: %d", drv_cfg->pin);
    return 0;
}

#define CSENSE_INST(n)                                                  \
    struct csense_data csense_data_##n;                                 \
    const struct csense_config csense_cfg_##n = {                       \
        .pin = DT_INST_GPIO_PIN(n, gpios),                              \
        .label = DT_INST_GPIO_LABEL(n, gpios),                          \
        .flags = DT_INST_GPIO_FLAGS(n, gpios)};                         \
    DEVICE_AND_API_INIT(csense_##n, DT_INST_LABEL(n), csense_init,      \
                        &csense_data_##n, &csense_cfg_##n, POST_KERNEL, \
                        CONFIG_SENSOR_INIT_PRIORITY, &csense_driver_api);

DT_INST_FOREACH_STATUS_OKAY(CSENSE_INST)
