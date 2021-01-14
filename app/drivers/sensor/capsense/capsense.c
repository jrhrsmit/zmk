/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_capsense

#include <device.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <nrfx_ppi.h>
#include <nrfx_comp.h>
#include <hal/nrf_gpio.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <kernel.h>
#include <logging/log.h>
#include <sys/__assert.h>
#include <sys/util.h>

#include "capsense.h"

#define CAPSENSE_STACK_SIZE 2048
#define CAPSENSE_PRIORITY 1
K_THREAD_STACK_DEFINE(capsense_stack_area, CAPSENSE_STACK_SIZE);
struct k_work_q capsense_work_q;

LOG_MODULE_REGISTER(capsense, CONFIG_SENSOR_LOG_LEVEL);
struct k_timer capsense_timer;

static const struct device *sdev;

static const struct sensor_driver_api capsense_api = {};

nrfx_timer_t capsense_nrfx_timer = NRFX_TIMER_INSTANCE(3);
static uint32_t value;
void capsense_nrfx_timer_callback(nrf_timer_event_t event_type, void *p_context) {}
void capsense_nrfx_comp_callback(nrf_comp_event_t event_type) {
    //LOG_DBG("COMP Int: event %d", event_type);
    //while(true){}
    value = nrfx_comp_sample();
    nrf_comp_event_clear(NRF_COMP, event_type);
}

static void saadc_pullup(uint8_t channel, bool enable) {
    NRF_SAADC->CH[channel].CONFIG &= ~SAADC_CH_CONFIG_RESP_Msk;
    NRF_SAADC->CH[channel].PSELP &= ~SAADC_CH_PSELP_PSELP_Msk;
    if (enable) {
        NRF_SAADC->CH[channel].CONFIG |= (NRF_SAADC_RESISTOR_PULLUP << SAADC_CH_CONFIG_RESP_Pos);
        NRF_SAADC->CH[channel].PSELP |= (SAADC_CH_PSELP_PSELP_AnalogInput0 + channel)
                                        << SAADC_CH_PSELP_PSELP_Pos;
        NRF_SAADC->ENABLE =
            (SAADC_ENABLE_ENABLE_Enabled << SAADC_ENABLE_ENABLE_Pos) & SAADC_ENABLE_ENABLE_Msk;
    } else {
        NRF_SAADC->CH[channel].CONFIG |= (NRF_SAADC_RESISTOR_DISABLED << SAADC_CH_CONFIG_RESP_Pos);
        NRF_SAADC->CH[channel].PSELP |= SAADC_CH_PSELP_PSELP_NC << SAADC_CH_PSELP_PSELP_Pos;
        NRF_SAADC->ENABLE =
            (SAADC_ENABLE_ENABLE_Disabled << SAADC_ENABLE_ENABLE_Pos) & SAADC_ENABLE_ENABLE_Msk;
    }
}

static void gpio_pullup(uint8_t pin, bool enable) {
    //LOG_DBG("old: NRF_P0->PIN_CNF[%d] = 0x%08X", pin, NRF_P0->PIN_CNF[pin]);
    //NRF_P0->PIN_CNF[pin] &= ~GPIO_PIN_CNF_PULL_Msk;
    if(1){//pin >= 32) {
        //pin -= 32;
        if(enable)
            NRF_P1->PIN_CNF[pin] |= GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos;
        else
            NRF_P1->PIN_CNF[pin] &= ~GPIO_PIN_CNF_PULL_Msk;
    } else {
        if(enable)
            NRF_P0->PIN_CNF[pin] |= GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos;
        else
            NRF_P0->PIN_CNF[pin] &= ~GPIO_PIN_CNF_PULL_Msk;
    }
        //NRF_P0->PIN_CNF[pin] = GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos;
    //LOG_DBG("new: NRF_P0->PIN_CNF[%d] = 0x%08X", pin, NRF_P0->PIN_CNF[pin]);
    return;
    //else
    //    NRF_P0->PIN_CNF[pin] |= GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos;
}

static int comp_init(const struct device *dev) {
    struct capsense_data *drv_data = dev->data;
    const struct capsense_config *cfg = dev->config;
    static nrf_ppi_channel_t ppi_channel;
    nrfx_err_t err;
    // comparator
    drv_data->comp_cfg = (nrfx_comp_config_t)NRFX_COMP_DEFAULT_CONFIG(cfg->adc_channel);
    drv_data->comp_cfg.reference = NRF_COMP_REF_Int1V2;
    drv_data->comp_cfg.threshold.th_down = NRFX_VOLTAGE_THRESHOLD_TO_INT(0.9, 1.2);
    drv_data->comp_cfg.threshold.th_up = NRFX_VOLTAGE_THRESHOLD_TO_INT(0.9, 1.2);
    drv_data->comp_cfg.interrupt_priority = 20;
    if ((err = nrfx_comp_init(&drv_data->comp_cfg, capsense_nrfx_comp_callback)) != NRFX_SUCCESS) {
        LOG_ERR("Could not initialize COMP module (%X)", err - NRFX_ERROR_BASE_NUM);
        return err;
    }
    nrfx_comp_pin_select(NRF_COMP_INPUT_0 + cfg->adc_channel);
    // timer
    nrfx_timer_config_t config = (nrfx_timer_config_t)NRFX_TIMER_DEFAULT_CONFIG;
    if ((err = nrfx_timer_init(&capsense_nrfx_timer, &config, capsense_nrfx_timer_callback)) !=
        NRFX_SUCCESS) {
        LOG_ERR("Could not initialize TIMER module (%X)", err - NRFX_ERROR_BASE_NUM);
        return err;
    }
    return 0;
    // ppi
    if ((err = nrfx_ppi_channel_alloc(&ppi_channel)) != NRFX_SUCCESS) {
        LOG_ERR("Could not allocate PPI channel (%X)", err - NRFX_ERROR_BASE_NUM);
        return err;
    }
    if ((err = nrfx_ppi_channel_assign(
             ppi_channel, nrfx_comp_event_address_get(NRFX_COMP_EVT_EN_DOWN_MASK),
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
    //LOG_DBG("capsense timer interrupt %d", i++);
    k_work_submit(&drv_data->work);
}

static void capsense_work(struct k_work *item) {
    // struct capsense_data *drv_data = sdev->data;
    const struct capsense_config *cfg = sdev->config;
    // charge capacitor
    //gpio_pullup(cfg->pin, true);
    nrf_gpio_cfg_input(cfg->pin + 32, NRF_GPIO_PIN_PULLUP);
    k_sleep(K_MSEC(1));
    // start timer
    nrfx_timer_clear(&capsense_nrfx_timer);
    nrfx_timer_enable(&capsense_nrfx_timer);
    // start comparator
    nrfx_comp_start(NRFX_COMP_EVT_EN_DOWN_MASK, NRFX_COMP_SHORT_STOP_AFTER_DOWN_EVT);
    //k_sleep(K_MSEC(1));
    //nrfx_comp_start(0xF, 0);
    //k_sleep(K_MSEC(1));
    //LOG_DBG("val: %d", nrfx_comp_sample());
    // let the cap discharge
    //gpio_pullup(cfg->pin, false);
    k_busy_wait(1000);
    nrf_gpio_cfg_input(cfg->pin + 32, NRF_GPIO_PIN_NOPULL);
    k_busy_wait(1000);
    //k_sleep(K_MSEC(1));
    //LOG_DBG("val: %d", nrfx_comp_sample());
    // report
    nrfx_comp_stop();
    nrfx_timer_pause(&capsense_nrfx_timer);
    LOG_DBG("Timer capture value: %d",
            nrfx_timer_capture(&capsense_nrfx_timer, NRF_TIMER_CC_CHANNEL0));
}

static int capsense_init(const struct device *dev) {
    struct capsense_data *drv_data = dev->data;
    const struct capsense_config *cfg = dev->config;
    sdev = dev;
    return 0;
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
    k_timer_start(&capsense_timer, K_SECONDS(3), K_SECONDS(3));

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

