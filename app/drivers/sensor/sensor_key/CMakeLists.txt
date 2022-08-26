# Copyright (c) 2020 The ZMK Contributors
# SPDX-License-Identifier: MIT

zephyr_include_directories(.)

zephyr_library()

zephyr_library_sources(sensor_key.c)
zephyr_library_sources_ifdef(CONFIG_SENSOR_KEY_TRIGGER sensor_key_trigger.c)
