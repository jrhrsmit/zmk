//#include <stdio.h>
//#include "app_error.h"
//#include "app_util_platform.h"
//#include "nrf_assert.h"
//#include "nrf_gpio.h"
//#include "nrf_peripherals.h"
//#include "string.h"

///*lint -save -e689 */
//#include "arm_math.h"
///*lint -restore */
//#endif

#include "nrf_drv_csense.h"
#include <nrfx_comp.h>
#include <nrfx_ppi.h>
#include <nrfx_timer.h>
#include <string.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(CSENSE_DRV, CONFIG_SENSOR_LOG_LEVEL);

/* Number of channels required by PPI. */
#define PPI_REQUIRED_CHANNELS 3

/* Array of PPI channels. */
static nrf_ppi_channel_t m_ppi_channels[PPI_REQUIRED_CHANNELS];

#define TIMER0_FOR_CSENSE 2
#define TIMER1_FOR_CSENSE 3
#define MEASUREMENT_PERIOD 20

/**
 * @defgroup timer_instances Timer instances.
 * @{
 */
static const nrfx_timer_t m_timer0 = NRFX_TIMER_INSTANCE(TIMER0_FOR_CSENSE);
static const nrfx_timer_t m_timer1 = NRFX_TIMER_INSTANCE(TIMER1_FOR_CSENSE);
/* @} */
// K_TIMER_DEFINE(m_timer0, NULL, NULL);
// K_TIMER_DEFINE(m_timer1, NULL, NULL);
// struct k_timer m_timer0;
// struct k_timer m_timer1;

/* Configuration of the capacitive sensor module. */
typedef struct {
    volatile nrfx_drv_state_t module_state; /**< State of the module. */
    nrf_drv_csense_event_handler_t
        event_handler; /**< Event handler for capacitor sensor events. */
    uint16_t
        analog_values[MAX_ANALOG_INPUTS]; /**< Array containing analog values
                                             measured on the corresponding
                                             COMP/ADC channel. */
    volatile bool busy; /**< Indicates state of module - busy if there are
                           ongoing conversions. */
    volatile uint8_t
        cur_chann_idx; /**< Current channel to be read if enabled. */
    volatile uint8_t adc_channels_input_mask; /**< Enabled channels. */
    uint8_t output_pin; /**< Pin to generate signal charging capacitors. */
    uint8_t channels_to_read; /**< Mask of channels remaining to be read in the
                                 current measurement. */
    volatile bool timers_powered_on; /**< Flag to indicate if timers were
                                        already started. */
} csense_t;

static csense_t m_csense;

/**
 * @brief Function for determining the next analog channel to be read.
 */
__STATIC_INLINE void calculate_next_channel(void) {
    m_csense.cur_chann_idx = 31 - __CLZ(m_csense.channels_to_read);
}

/**
 * @brief Function for handling conversion values.
 *
 * @param[in] val                Value received from ADC or COMP.
 */
static void conversion_handler(uint16_t val) {
    nrf_drv_csense_evt_t event_struct;

    m_csense.analog_values[m_csense.cur_chann_idx] = val;

    event_struct.read_value = val;
    event_struct.analog_channel = m_csense.cur_chann_idx;

    m_csense.channels_to_read &= ~(1UL << m_csense.cur_chann_idx);

    // decide if there will be more conversions
    if (m_csense.channels_to_read == 0) {
        m_csense.busy = false;
    }

    m_csense.event_handler(&event_struct);

    if (m_csense.channels_to_read > 0)  // Start new conversion.
    {
        nrfx_err_t err_code;
        calculate_next_channel();
        err_code = nrf_drv_csense_sample();
        if (err_code != NRFX_SUCCESS) {
            return;
        }
    }
}

/**
 * @brief Timer0 interrupt handler.
 *
 * @param[in] event_type Timer event.
 * @param[in] p_context  General purpose parameter set during initialization of
 *                       the timer. This parameter can be used to pass
 *                       additional information to the handler function, for
 *                       example, the timer ID.
 */
static void counter_compare_handler(nrf_timer_event_t event_type,
                                    void* p_context) {
    static uint32_t i=1;
    LOG_DBG("counter_compare_handler %d", i++);
    if (event_type == NRF_TIMER_EVENT_COMPARE0) {
        uint16_t val = nrfx_timer_capture_get(&m_timer1, NRF_TIMER_CC_CHANNEL1);
        nrfx_timer_pause(&m_timer1);
        nrfx_timer_clear(&m_timer1);

        /* Handle finished measurement. */
        conversion_handler(val);
    }
}

/**
 * @brief Dummy handler.
 *
 * @param[in] event_type Timer event.
 * @param[in] p_context  General purpose parameter set during initialization of
 *                       the timer. This parameter can be used to pass
 *                       additional information to the handler function, for
 *                       example, the timer ID.
 */
static void dummy_handler(nrf_timer_event_t event_type, void* p_context) {}

/**
 * @brief Function for initializing timers.
 *
 * @retval NRFX_ERROR_INTERNAL            If there were error initializing
 * timers.
 * @retval NRFX_SUCCESS                   If timers were initialized
 * successfully.
 */
static nrfx_err_t timer_init(void) {
    nrfx_err_t err_code;

    // k_timer_init(&m_timer1, NULL, NULL);
    // k_timer_init(&m_timer0, counter_compare_handler, NULL);
    // k_timer_start(&m_timer0, K_USEC(610), K_USEC(610));

    // set first timer in timer mode to get period of relaxation oscillator
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG;
    timer_config.mode = NRF_TIMER_MODE_TIMER;
    err_code = nrfx_timer_init(&m_timer1, &timer_config, dummy_handler);
    if (err_code != NRFX_SUCCESS) {
        return NRFX_ERROR_INTERNAL;
    }

    // set second timer in counter mode and generate event on tenth period
    timer_config.mode = NRF_TIMER_MODE_COUNTER;
    err_code =
        nrfx_timer_init(&m_timer0, &timer_config, counter_compare_handler);
    if (err_code != NRFX_SUCCESS) {
        return NRFX_ERROR_INTERNAL;
    }

    // MEASUREMENT_PERIOD here is 20
    nrfx_timer_extended_compare(
        &m_timer0, NRF_TIMER_CC_CHANNEL0, MEASUREMENT_PERIOD,
        (nrf_timer_short_mask_t)(NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK |
                                 NRF_TIMER_SHORT_COMPARE0_STOP_MASK),
        true);

    return NRFX_SUCCESS;
}

/**
 * @brief Function for initializing and enabling PPI channels.
 *
 * @retval NRFX_ERROR_INTERNAL            If there were error initializing or
 * enabling PPI channels.
 * @retval NRFX_SUCCESS                   If PPI channels were initialized and
 * enabled successfully.
 */
static nrfx_err_t ppi_init(void) {
    nrfx_err_t err_code;
    uint8_t i;

    for (i = 0; i < PPI_REQUIRED_CHANNELS; i++) {
        err_code = nrfx_ppi_channel_alloc(&m_ppi_channels[i]);
        if (NRFX_SUCCESS != err_code) {
            return NRFX_ERROR_INTERNAL;
        }
    }

    err_code = nrfx_ppi_channel_assign(
        m_ppi_channels[0], nrfx_comp_event_address_get(NRF_COMP_EVENT_CROSS),
        nrfx_timer_task_address_get(&m_timer0, NRF_TIMER_TASK_COUNT));
    if (NRFX_SUCCESS != err_code) {
        return NRFX_ERROR_INTERNAL;
    }
    err_code = nrfx_ppi_channel_assign(
        m_ppi_channels[1],
        nrfx_timer_event_address_get(&m_timer0, NRF_TIMER_EVENT_COMPARE0),
        nrfx_timer_task_address_get(&m_timer1, NRF_TIMER_TASK_CAPTURE1));
    if (NRFX_SUCCESS != err_code) {
        return NRFX_ERROR_INTERNAL;
    }
    err_code = nrfx_ppi_channel_fork_assign(
        m_ppi_channels[1], nrfx_comp_task_address_get(NRF_COMP_TASK_STOP));
    if (NRFX_SUCCESS != err_code) {
        return NRFX_ERROR_INTERNAL;
    }
    err_code = nrfx_ppi_channel_assign(
        m_ppi_channels[2], nrfx_comp_event_address_get(NRF_COMP_EVENT_READY),
        nrfx_timer_task_address_get(&m_timer0, NRF_TIMER_TASK_CLEAR));
    if (NRFX_SUCCESS != err_code) {
        return NRFX_ERROR_INTERNAL;
    }
    err_code = nrfx_ppi_channel_fork_assign(
        m_ppi_channels[2],
        nrfx_timer_task_address_get(&m_timer1, NRF_TIMER_TASK_CLEAR));
    if (NRFX_SUCCESS != err_code) {
        return NRFX_ERROR_INTERNAL;
    }

    for (i = 0; i < PPI_REQUIRED_CHANNELS; i++) {
        err_code = nrfx_ppi_channel_enable(m_ppi_channels[i]);
        if (NRFX_SUCCESS != err_code) {
            return NRFX_ERROR_INTERNAL;
        }
    }

    return NRFX_SUCCESS;
}

/**
 * @brief Dummy handler for COMP events.
 *
 * @param[in] event         COMP event.
 */
static void comp_event_handler(nrf_comp_event_t event) {}

/**
 * @brief Function for initializing COMP module in relaxation oscillator mode.
 *
 * @note The frequency of the oscillator depends on threshold voltages, current
 * source and capacitance of pad and can be calculated as f_OSC = I_SOURCE /
 * (2CÂ·(VUP-VDOWN) ).
 *
 * @retval NRFX_ERROR_INTERNAL                If there were error while
 * initializing COMP driver.
 * @retval NRFX_SUCCESS                       If the COMP driver initialization
 * was successful.
 */
static nrfx_err_t comp_init(void) {
    nrfx_err_t err_code;
    nrfx_comp_config_t m_comp_config =
        NRFX_COMP_DEFAULT_CONFIG(NRF_COMP_INPUT_0);

    // m_comp_config.isource = NRF_COMP_ISOURCE_Ien10uA;

    err_code = nrfx_comp_init(&m_comp_config, comp_event_handler);
    if (err_code != NRFX_SUCCESS) {
        return NRFX_ERROR_INTERNAL;
    }
    return NRFX_SUCCESS;
}

nrfx_err_t nrf_drv_csense_init(nrf_drv_csense_config_t const* p_config,
                               nrf_drv_csense_event_handler_t event_handler) {
    nrfx_err_t err_code;
    __ASSERT(m_csense.module_state == NRFX_DRV_STATE_UNINITIALIZED, "CSENSE driver already initialized");

    if (p_config == NULL) {
        return NRFX_ERROR_INVALID_PARAM;
    }

    if (event_handler == NULL) {
        return NRFX_ERROR_INVALID_PARAM;
    }
    m_csense.busy = false;
    m_csense.event_handler = event_handler;

    err_code = comp_init();
    if (err_code != NRFX_SUCCESS) {
        LOG_WRN("Comparator intialization failed");
        return err_code;
    }
    LOG_DBG("Successfully initialized comparator");
    err_code = timer_init();
    if (err_code != NRFX_SUCCESS) {
        LOG_WRN("Timer intialization failed");
        return err_code;
    }
    LOG_DBG("Successfully initialized timers");
    err_code = ppi_init();
    if (err_code != NRFX_SUCCESS) {
        LOG_WRN("PPI intialization failed");
        return err_code;
    }
    LOG_DBG("Successfully initialized PPI module");
    m_csense.module_state = NRFX_DRV_STATE_INITIALIZED;
    LOG_DBG("Successfully initialized csense driver");
    return NRFX_SUCCESS;
}

nrfx_err_t nrf_drv_csense_uninit(void) {
    nrfx_err_t err_code;
    uint8_t i;
    __ASSERT(m_csense.module_state != NRFX_DRV_STATE_UNINITIALIZED,
             "Can't uninit driver, driver not initialized in the first place");
    nrf_drv_csense_channels_disable(0xFF);
    nrfx_timer_uninit(&m_timer0);
    nrfx_timer_uninit(&m_timer1);
    nrfx_comp_uninit();
    for (i = 0; i < 3; i++) {
        err_code = nrfx_ppi_channel_free(m_ppi_channels[i]);
        if (err_code != NRFX_SUCCESS) {
            return err_code;
        }
    }
    nrfx_ppi_free_all();
    m_csense.module_state = NRFX_DRV_STATE_UNINITIALIZED;
    memset((void*)&m_csense, 0, sizeof(m_csense));
    return NRFX_SUCCESS;
}

void nrf_drv_csense_channels_enable(uint8_t channels_mask) {
    __ASSERT(m_csense.module_state != NRFX_DRV_STATE_UNINITIALIZED,
             "Can't enable channels, csense driver not initialized");
    m_csense.busy = true;
    m_csense.module_state = NRFX_DRV_STATE_POWERED_ON;
    m_csense.adc_channels_input_mask |= channels_mask;
    m_csense.busy = false;
}

void nrf_drv_csense_channels_disable(uint8_t channels_mask) {
    __ASSERT(m_csense.module_state == NRFX_DRV_STATE_POWERED_ON,
             "Can't disable channels, csense driver not powered on");
    m_csense.adc_channels_input_mask &= ~channels_mask;
    if (m_csense.adc_channels_input_mask == 0) {
        m_csense.module_state = NRFX_DRV_STATE_INITIALIZED;
    }
}

uint16_t nrf_drv_csense_channel_read(uint8_t csense_channel) {
    return m_csense.analog_values[csense_channel];
}

nrfx_err_t nrf_drv_csense_sample(void) {
    __ASSERT(m_csense.module_state == NRFX_DRV_STATE_POWERED_ON,
             "Can't sample channels, csense driver not powered on");
    if (m_csense.adc_channels_input_mask != 0) {
        LOG_DBG("ADC Channels: 0x%0X channels_to_read: 0x%0X", m_csense.adc_channels_input_mask, m_csense.channels_to_read);
        if (m_csense.channels_to_read == 0) {
            if (nrf_drv_csense_is_busy() == true) {
                return NRFX_ERROR_BUSY;
            }
            m_csense.busy = true;
            m_csense.channels_to_read = m_csense.adc_channels_input_mask;
            calculate_next_channel();
        }
        if (!m_csense.timers_powered_on) {
            LOG_DBG("Enabling timers");
            nrfx_timer_enable(&m_timer0);
            nrfx_timer_enable(&m_timer1);
            m_csense.timers_powered_on = true;
        } else {
            LOG_DBG("Starting timers");
            nrfx_timer_resume(&m_timer0);
            nrfx_timer_resume(&m_timer1);
        }
        nrfx_comp_pin_select((nrf_comp_input_t)m_csense.cur_chann_idx);
        nrfx_comp_start(0, 0);
    }
    return NRFX_SUCCESS;
}

bool nrf_drv_csense_is_busy(void) {
    return m_csense.busy;
}
