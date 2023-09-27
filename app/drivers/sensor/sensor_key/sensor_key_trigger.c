/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_sensor_key

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>

#include "sensor_key.h"

extern struct sensor_key_data sensor_key_driver;

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(sensor_key, CONFIG_SENSOR_LOG_LEVEL);

static inline void setup_int(const struct device *dev, bool enable) {
    const struct sensor_key_config *cfg = dev->config;

    LOG_DBG("enabled %s", (enable ? "true" : "false"));

    if (gpio_pin_interrupt_configure_dt(&cfg->pin,
                                        enable ? GPIO_INT_EDGE_RISING : GPIO_INT_DISABLE)) {
        LOG_WRN("Unable to set pin GPIO interrupt");
    }
}

static void sensor_key_gpio_callback(const struct device *dev, struct gpio_callback *cb,
                                     uint32_t pins) {
    struct sensor_key_data *drv_data = CONTAINER_OF(cb, struct sensor_key_data, gpio_cb);

    setup_int(drv_data->dev, false);

#if defined(CONFIG_SENSOR_KEY_TRIGGER_OWN_THREAD)
    k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_SENSOR_KEY_TRIGGER_GLOBAL_THREAD)
    k_work_submit(&drv_data->work);
#endif
}

static void sensor_key_thread_cb(const struct device *dev) {
    struct sensor_key_data *drv_data = dev->data;

    drv_data->handler(dev, drv_data->trigger);

    setup_int(dev, true);
}

#ifdef CONFIG_SENSOR_KEY_TRIGGER_OWN_THREAD
static void sensor_key_thread(int dev_ptr, int unused) {
    const struct device *dev = INT_TO_POINTER(dev_ptr);
    struct sensor_key_data *drv_data = dev->data;

    ARG_UNUSED(unused);

    while (1) {
        k_sem_take(&drv_data->gpio_sem, K_FOREVER);
        sensor_key_thread_cb(dev);
    }
}
#endif

#ifdef CONFIG_SENSOR_KEY_TRIGGER_GLOBAL_THREAD
static void sensor_key_work_cb(struct k_work *work) {
    struct sensor_key_data *drv_data = CONTAINER_OF(work, struct sensor_key_data, work);

    sensor_key_thread_cb(drv_data->dev);
}
#endif

int sensor_key_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                           sensor_trigger_handler_t handler) {
    struct sensor_key_data *drv_data = dev->data;

    setup_int(dev, false);

    k_msleep(5);

    drv_data->trigger = trig;
    drv_data->handler = handler;

    setup_int(dev, true);

    return 0;
}

int sensor_key_init_interrupt(const struct device *dev) {
    struct sensor_key_data *drv_data = dev->data;
    const struct sensor_key_config *drv_cfg = dev->config;

    drv_data->dev = dev;
    /* setup gpio interrupt */

    gpio_init_callback(&drv_data->gpio_cb, sensor_key_gpio_callback, BIT(drv_cfg->pin.pin));

    if (gpio_add_callback(drv_cfg->pin.port, &drv_data->gpio_cb) < 0) {
        LOG_DBG("Failed to set callback!");
        return -EIO;
    }

#if defined(CONFIG_SENSOR_KEY_TRIGGER_OWN_THREAD)
    k_sem_init(&drv_data->gpio_sem, 0, UINT_MAX);

    k_thread_create(&drv_data->thread, drv_data->thread_stack, CONFIG_SENSOR_KEY_THREAD_STACK_SIZE,
                    (k_thread_entry_t)sensor_key_thread, dev, 0, NULL,
                    K_PRIO_COOP(CONFIG_SENSOR_KEY_THREAD_PRIORITY), 0, K_NO_WAIT);
#elif defined(CONFIG_SENSOR_KEY_TRIGGER_GLOBAL_THREAD)
    k_work_init(&drv_data->work, sensor_key_work_cb);
#endif

    return 0;
}
