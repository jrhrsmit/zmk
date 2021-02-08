/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_capsense

#include <device.h>
#include <nrfx_timer.h>
#include <nrfx_ppi.h>
#include <nrfx_lpcomp.h>
#include <hal/nrf_gpio.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <kernel.h>
#include <logging/log.h>
#include <sys/__assert.h>
#include <sys/util.h>

#include "capsense.h"

#define CAPSENSE_STACK_SIZE 128
#define CAPSENSE_PRIORITY 100

LOG_MODULE_REGISTER(capsense, CONFIG_SENSOR_LOG_LEVEL);
K_THREAD_STACK_DEFINE(capsense_stack_area, CAPSENSE_STACK_SIZE);
struct k_work_q capsense_work_q;

nrfx_timer_t capsense_nrfx_timer = NRFX_TIMER_INSTANCE(3);

static void capsense_nrfx_timer_callback(nrf_timer_event_t event_type, void *p_context) {}

static void capsense_nrfx_lpcomp_callback(nrf_lpcomp_event_t event_type) {}

static void gpio_pullup(uint8_t pin, int bank, bool enable) {
    if (bank) {
        if (enable)
            NRF_P1->PIN_CNF[pin] |= GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos;
        else
            NRF_P1->PIN_CNF[pin] &= ~GPIO_PIN_CNF_PULL_Msk;
    } else {
        if (enable)
            NRF_P0->PIN_CNF[pin] |= GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos;
        else
            NRF_P0->PIN_CNF[pin] &= ~GPIO_PIN_CNF_PULL_Msk;
    }
    return;
}

static int comp_init(const struct device *dev) {
    struct capsense_data *drv_data = dev->data;
    const struct capsense_config *cfg = dev->config;
    static nrf_ppi_channel_t ppi_channel;
    nrfx_err_t err;
    IRQ_CONNECT(19, 20, nrfx_isr, nrfx_lpcomp_irq_handler, 0);
    // comparator
    drv_data->comp_cfg = (nrfx_lpcomp_config_t)NRFX_LPCOMP_DEFAULT_CONFIG(cfg->adc_channel);
    drv_data->comp_cfg.hal.reference = NRF_LPCOMP_REF_SUPPLY_2_8;
    drv_data->comp_cfg.hal.detection = NRF_LPCOMP_DETECT_DOWN;
    if ((err = nrfx_lpcomp_init(&drv_data->comp_cfg, capsense_nrfx_lpcomp_callback)) !=
        NRFX_SUCCESS) {
        LOG_ERR("Could not initialize LPCOMP module (%X)", err - NRFX_ERROR_BASE_NUM);
        return err;
    }
    // timer
    nrfx_timer_config_t config = (nrfx_timer_config_t)NRFX_TIMER_DEFAULT_CONFIG;
    if ((err = nrfx_timer_init(&capsense_nrfx_timer, &config, capsense_nrfx_timer_callback)) !=
        NRFX_SUCCESS) {
        LOG_ERR("Could not initialize TIMER module (%X)", err - NRFX_ERROR_BASE_NUM);
        return err;
    }
    // ppi
    if ((err = nrfx_ppi_channel_alloc(&ppi_channel)) != NRFX_SUCCESS) {
        LOG_ERR("Could not allocate PPI channel (%X)", err - NRFX_ERROR_BASE_NUM);
        return err;
    }
    if ((err = nrfx_ppi_channel_assign(
             ppi_channel, nrf_lpcomp_event_address_get(NRF_LPCOMP, NRF_LPCOMP_EVENT_DOWN),
             nrfx_timer_task_address_get(&capsense_nrfx_timer, NRF_TIMER_TASK_STOP))) !=
        NRFX_SUCCESS) {
        LOG_ERR("Could not assign PPI channel (%X)", err - NRFX_ERROR_BASE_NUM);
        return err;
    }
    if ((err = nrfx_ppi_channel_enable(ppi_channel)) != NRFX_SUCCESS) {
        LOG_ERR("Could not enable PPI channel (%X)", err - NRFX_ERROR_BASE_NUM);
        return err;
    }
    return 0;
}

static uint32_t capsense_capacitance(const struct device *dev) {
    // charge capacitor
    gpio_pullup(5, 0, true);
    // start timer
    nrfx_timer_clear(&capsense_nrfx_timer);
    // start comparator
    nrfx_lpcomp_enable();
    // lpcomp needs some time to startup
    k_sleep(K_USEC(50));
    nrfx_timer_enable(&capsense_nrfx_timer);
    // let the cap discharge
    gpio_pullup(5, 0, false);
    // sleep while the LPCOMP event stops the timer via PPI
    k_sleep(K_USEC(500));
    nrfx_lpcomp_disable();
    // stop the timer if it somehow didn't trigger to save power
    nrfx_timer_pause(&capsense_nrfx_timer);
    return nrfx_timer_capture(&capsense_nrfx_timer, NRF_TIMER_CC_CHANNEL0);
}

static int capsense_sample_fetch(const struct device *dev, enum sensor_channel chan) { return 0; }

static void capsense_work(struct k_work *item) {
    struct capsense_data *const drv_data = CONTAINER_OF(item, struct capsense_data, work);
    const struct device *dev = drv_data->dev;
    const struct capsense_config *cfg = dev->config;
    static uint32_t val[DT_INST_PROP(0, buffer_size)] = {0};
    static uint32_t sample[DT_INST_PROP(0, num_samples)] = {0};
    static uint32_t idx = 0;
    uint32_t low = 0xFFFFFFFF, high = 0, sum = 0, avg, sample_avg = 0;
    // maybe use IIR filter?
    for (int i = 0; i < cfg->buffer_size; i++) {
        if (val[i] < low)
            low = val[i];
        else if (val[i] > high)
            high = val[i];
        sum += val[i];
    }
    avg = sum / cfg->buffer_size;
    for (int i = 0; i < cfg->num_samples; i++) {
        sample[i] = capsense_capacitance(dev);
    }
    for (int i = cfg->num_samples - 1; i > 0; i--) {
        for (int j = 0; j < i; j++) {
            if (sample[j] > sample[j + 1]) {
                uint32_t tmp = sample[j + 1];
                sample[j + 1] = sample[j];
                sample[j] = tmp;
            }
        }
    }
    // calculate average excluding top and bottom 30% of samples
    for (int i = cfg->num_samples * 3 / 10; i < cfg->num_samples * 7 / 10; i++) {
        sample_avg += sample[i];
    }
    sample_avg = sample_avg / ((cfg->num_samples * 9 / 10) - (cfg->num_samples / 10));
    val[idx] = sample_avg;
    drv_data->value = sample_avg;
    drv_data->samples_taken++;
    if (drv_data->samples_taken >= cfg->buffer_size && val[idx] > avg + drv_data->threshold) {
        LOG_DBG("Presence detected (%d (avg: %d))", val[idx], avg);
        drv_data->presence = 1;
        drv_data->handler(drv_data->dev, drv_data->trigger);
    } else {
        drv_data->presence = 0;
    }
    if (++idx > cfg->buffer_size - 1)
        idx = 0;
    if (drv_data->period > 0)
        k_delayed_work_submit(&drv_data->work, K_MSEC(drv_data->period));
}

static int capsense_channel_get(const struct device *dev, enum sensor_channel chan,
                                struct sensor_value *val) {
    struct capsense_data *drv_data = dev->data;
    if (chan != SENSOR_CHAN_PROX)
        return -ENOTSUP;
    val->val1 = drv_data->presence;
    val->val2 = 0;
    return 0;
}

static int capsense_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
                                sensor_trigger_handler_t handler) {
    struct capsense_data *drv_data = dev->data;
    drv_data->trigger = (struct sensor_trigger *)trig;
    drv_data->handler = handler;
    return 0;
}

static int capsense_attr_set(const struct device *dev, enum sensor_channel chan,
                             enum sensor_attribute attr, const struct sensor_value *val) {
    struct capsense_data *drv_data = dev->data;
    const struct capsense_config *cfg = dev->config;
    if (chan != SENSOR_CHAN_PROX)
        return -ENOTSUP;
    switch (attr) {
    case SENSOR_ATTR_SAMPLING_FREQUENCY:
        if (val->val1 <= 0) {
            drv_data->period = 0;
            LOG_INF("Disabling capsense work");
            k_delayed_work_cancel(&drv_data->work);
        } else {
            drv_data->period = cfg->sample_period;
            LOG_INF("Resuming capsense work");
            drv_data->samples_taken = 0;
            k_delayed_work_submit(&drv_data->work, K_MSEC(cfg->sample_period));
        }
        break;
    case SENSOR_ATTR_UPPER_THRESH:
        if (val->val1)
            drv_data->threshold = cfg->threshold_grounded;
        else
            drv_data->threshold = cfg->threshold_wireless;
        break;
    default:
        return -ENOTSUP;
    }
    return 0;
}

static const struct sensor_driver_api capsense_driver_api = {.trigger_set = capsense_trigger_set,
                                                             .sample_fetch = capsense_sample_fetch,
                                                             .channel_get = capsense_channel_get,
                                                             .attr_set = capsense_attr_set};

static int capsense_init(const struct device *dev) {
    struct capsense_data *drv_data = dev->data;
    const struct capsense_config *cfg = dev->config;
    drv_data->dev = dev;
    drv_data->threshold = cfg->threshold_grounded;
    // comp
    if (comp_init(dev)) {
        LOG_ERR("Could not initialize COMP");
        return -EIO;
    }
    // work queue
    k_work_q_start(&capsense_work_q, capsense_stack_area,
                   K_THREAD_STACK_SIZEOF(capsense_stack_area), CAPSENSE_PRIORITY);
    k_delayed_work_init(&drv_data->work, capsense_work);
    LOG_DBG("Capsense initialized");
    LOG_DBG("GPIO pin: %d, ADC pin: %d", cfg->pin, cfg->adc_channel);
    LOG_DBG("threshold_grounded: %d", cfg->threshold_grounded);
    LOG_DBG("threshold_wireless: %d", cfg->threshold_wireless);
    return 0;
}

static struct capsense_data capsense_data;
static const struct capsense_config capsense_cfg = {
    .adc_label = DT_INST_IO_CHANNELS_LABEL(0),
    .adc_channel = DT_INST_IO_CHANNELS_INPUT(0),
    .pin = DT_INST_GPIO_PIN(0, gpios),
    .label = DT_INST_GPIO_LABEL(0, gpios),
    .flags = DT_INST_GPIO_FLAGS(0, gpios),
    .threshold_grounded = DT_INST_PROP(0, threshold_grounded),
    .threshold_wireless = DT_INST_PROP(0, threshold_wireless),
    .buffer_size = DT_INST_PROP(0, buffer_size),
    .num_samples = DT_INST_PROP(0, num_samples),
    .sample_period = DT_INST_PROP(0, sample_period)};

DEVICE_AND_API_INIT(capsense_dev, DT_INST_LABEL(0), &capsense_init, &capsense_data, &capsense_cfg,
                    POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &capsense_driver_api);

