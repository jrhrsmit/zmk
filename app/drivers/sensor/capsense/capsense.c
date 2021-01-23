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

nrfx_timer_t capsense_nrfx_timer = NRFX_TIMER_INSTANCE(3);

K_THREAD_STACK_DEFINE(capsense_stack_area, CAPSENSE_STACK_SIZE);
struct k_work_q capsense_work_q;

LOG_MODULE_REGISTER(capsense, CONFIG_SENSOR_LOG_LEVEL);
struct k_timer capsense_timer;

static const struct device *sdev;

static const struct sensor_driver_api capsense_api = {};

static void capsense_nrfx_timer_callback(nrf_timer_event_t event_type, void *p_context) {
    // LOG_DBG("Timer Int: event %d", event_type);
}

static void capsense_nrfx_lpcomp_callback(nrf_lpcomp_event_t event_type) {
    //LOG_DBG("COMP Int: event %d", event_type);
    // while(true){}
    // nrf_lpcomp_event_clear(NRF_LPCOMP, event_type);
}

static void gpio_pullup(uint8_t pin, int bank, bool enable) {
    if (bank) { // pin >= 32
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
    /* Connect GPIOTE_0 IRQ to nrfx_gpiote_irq_handler */
    // IRQ_CONNECT(DT_IRQN(DT_NODELABEL(gpiote)), DT_IRQ(DT_NODELABEL(gpiote), priority), nrfx_isr,
    //            nrfx_lpcomp_irq_handler, 0);
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

static void capsense_handler(struct k_timer *exp) {
    static int i = 1;
    struct capsense_data *drv_data = sdev->data;
    // LOG_DBG("capsense timer interrupt %d", i++);
    k_work_submit(&drv_data->work);
}

static uint32_t capsense_capacitance(struct device *dev) {
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

#define BUF_SZ 8
#define SAMPLE_SZ 64
#define THRESHOLD 10
static void capsense_work(struct k_work *item) {
    // const struct capsense_config *cfg = sdev->config;
    static uint32_t val[BUF_SZ] = {0};
    static uint32_t sample[SAMPLE_SZ] = {0};
    static uint32_t idx = 0;
    uint32_t low = 0xFFFFFFFF, high = 0, sum = 0, avg, sample_avg = 0;
    // maybe use IIR filter?
    for (int i = 0; i < BUF_SZ; i++) {
        if (val[i] < low)
            low = val[i];
        else if (val[i] > high)
            high = val[i];
        sum += val[i];
    }
    avg = sum / BUF_SZ;
    for(int i=0; i<SAMPLE_SZ; i++) {
        sample[i] = capsense_capacitance(sdev);
    }
    for(int i=SAMPLE_SZ-1; i>0; i--) {
        for(int j=0; j<i; j++) {
            if(sample[j] > sample[j+1]){
                uint32_t tmp = sample[j+1];
                sample[j+1] = sample[j];
                sample[j] = tmp;
            }
        }
    }
    // calculate average excluding top and bottom 10% of samples
    for(int i=SAMPLE_SZ/10; i<SAMPLE_SZ*9/10; i++) {
        sample_avg += sample[i];
    }
    sample_avg = sample_avg / ((SAMPLE_SZ*9/10) - (SAMPLE_SZ/10));
    val[idx] = sample_avg;
    if (val[idx] > avg + THRESHOLD)
        LOG_DBG("Presence detected (%d (avg: %d))", val[idx], avg);
    else
        LOG_DBG("(%d (avg: %d))", val[idx], avg);
    if (++idx > BUF_SZ - 1)
        idx = 0;
}

static int capsense_init(const struct device *dev) {
    struct capsense_data *drv_data = dev->data;
    const struct capsense_config *cfg = dev->config;
    sdev = dev;
    // comp
    if (comp_init(dev)) {
        LOG_ERR("Could not initialize COMP");
        return -EIO;
    }
    // work queue
    k_work_q_start(&capsense_work_q, capsense_stack_area,
                   K_THREAD_STACK_SIZEOF(capsense_stack_area), CAPSENSE_PRIORITY);
    k_work_init(&drv_data->work, capsense_work);
    // timers
    k_timer_init(&capsense_timer, capsense_handler, NULL);
    k_timer_start(&capsense_timer, K_SECONDS(3), K_MSEC(250));

    LOG_DBG("Capsense initialized");
    LOG_DBG("GPIO pin: %d, ADC pin: %d", cfg->pin, cfg->adc_channel);
    return 0;
}

static struct capsense_data capsense_data;
static const struct capsense_config capsense_cfg = {.adc_label = DT_INST_IO_CHANNELS_LABEL(0),
                                                    .adc_channel = DT_INST_IO_CHANNELS_INPUT(0),
                                                    .pin = DT_INST_GPIO_PIN(0, gpios),
                                                    .label = DT_INST_GPIO_LABEL(0, gpios),
                                                    .flags = DT_INST_GPIO_FLAGS(0, gpios)};

DEVICE_AND_API_INIT(capsense_dev, DT_INST_LABEL(0), &capsense_init, &capsense_data, &capsense_cfg,
                    POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &capsense_api);

