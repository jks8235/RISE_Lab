#!/usr/bin/env python
# -*- coding: utf-8 -*-
# author: Jee Kyung sub(RISE)

import VL53L0X_api

tof = VL53L0X_api.VL53L0X(0x29)

tof.setup()

tof.start_ranging(1)  # Start ranging
                      # 0 = Unchanged
                      # 1 = Short Range
                      # 2 = Medium Range
                      # 3 = Long Range

# Grab the range in mm, this function will block until
# a reading is returned.
distance_in_mm = tof.get_distance()

tof.stop_ranging()