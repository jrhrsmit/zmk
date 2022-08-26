# Copyright (c) 2020 The ZMK Contributors
# SPDX-License-Identifier: MIT

target_sources_ifdef(CONFIG_ZMK_WIDGET_CUSTOM_STATUS app PRIVATE bongocat_oled_64x32_frame_0.c)
target_sources_ifdef(CONFIG_ZMK_WIDGET_CUSTOM_STATUS app PRIVATE bongocat_oled_64x32_frame_1.c)
target_sources_ifdef(CONFIG_ZMK_WIDGET_CUSTOM_STATUS app PRIVATE capslock.c)
