#!/usr/bin/env python

from builtins import range

import smbus
import time

bus = smbus.SMBus(1)

for i in range(0x20):
    bus.write_i2c_block_data(0x69, 0x20, [i, 0x20])
    time.sleep(.0100)

#bus.write_i2c_block_data(0x69, 0x1d, [-0x45])
bus.write_i2c_block_data(0x69, 0x1c, [0x40])

for i in range(0x20):
    bus.write_i2c_block_data(0x69, 0x20, [0x00, i])
    time.sleep(.0100)
