/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <device.h>
#include <init.h>
#include <kernel.h>

#include <logging/log.h>

#include <drivers/sensor.h>

#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define PROXIMITY_LABEL DT_LABEL(DT_CHOSEN(zmk_proximity))

static const struct device *prox_sensor = NULL;

static int zmk_proximity_init(const struct device *_arg) {
    prox_sensor = device_get_binding(PROXIMITY_LABEL);
    if (prox_sensor) {
        LOG_INF("Found proximity sensor device %s", PROXIMITY_LABEL);
    } else {
        LOG_ERR("Proximity sensor device %s not found", PROXIMITY_LABEL);
        return -EINVAL;
    }
    return 0;
}

#if IS_ENABLED(CONFIG_USB)
static int usb_conn_status_listener(const zmk_event_t *eh) {
    struct zmk_usb_conn_state_changed *ev = as_zmk_usb_conn_state_changed(eh);
    const struct sensor_driver_api *api = (const struct sensor_driver_api *)prox_sensor->api;
    struct sensor_value val;
    // needs device_is_ready(prox_sensor), but is only available zephyr 2.5.0+
    if(k_uptime_get() < 2000)
        return 0;
    if(ev->conn_state == ZMK_USB_CONN_NONE) {
        LOG_INF("USB disconnected, lowering capsense threshold");
        val.val1 = 0;
    } else {
        LOG_INF("USB connected, increasing capsense threshold");
        val.val1 = 1;
    }
    api->attr_set(prox_sensor, SENSOR_CHAN_PROX, SENSOR_ATTR_UPPER_THRESH, &val);
    return 0;
}
#endif


int proximity_event_handler(const zmk_event_t *eh) {
    struct zmk_activity_state_changed *ev = as_zmk_activity_state_changed(eh);
    const struct sensor_driver_api *api = (const struct sensor_driver_api *)prox_sensor->api;
    struct sensor_value val;
    switch (ev->state) {
    case ZMK_ACTIVITY_ACTIVE:
        LOG_INF("ZMK Actitivity event Active: suspending capsense");
        val.val1 = 0;
        break;
    case ZMK_ACTIVITY_IDLE:
    case ZMK_ACTIVITY_SLEEP:
        LOG_INF("ZMK Activity Idle/Sleep: resuming capsense");
        val.val1 = 250;
        break;
    default:
        LOG_WRN("Unhandled activity state: %d", ev->state);
        return -EINVAL;
    }
    api->attr_set(prox_sensor, SENSOR_CHAN_PROX, SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
    return 0;
}

ZMK_LISTENER(proximity, proximity_event_handler);
ZMK_SUBSCRIPTION(proximity, zmk_activity_state_changed);

#if IS_ENABLED(CONFIG_USB)
ZMK_LISTENER(connected_status, usb_conn_status_listener)
ZMK_SUBSCRIPTION(connected_status, zmk_usb_conn_state_changed);
#endif

SYS_INIT(zmk_proximity_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
