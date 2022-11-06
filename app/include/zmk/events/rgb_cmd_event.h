/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr.h>
#include <zmk/event_manager.h>

struct zmk_rgb_cmd_event {
    int cmd;
};

ZMK_EVENT_DECLARE(zmk_rgb_cmd_event);
