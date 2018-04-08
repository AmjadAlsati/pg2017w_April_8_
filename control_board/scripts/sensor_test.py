#!/usr/bin/env python

import smbus
import time

bus = smbus.SMBus(1)


while True:
    sen_vals = bus.read_i2c_block_data(0x69,0x10,13)
    print("\n")
#    print(sen_vals[2:7])
    print("Back Right %d" % sen_vals[2])
    print("Back Left %d" % sen_vals[3])
    print("Front Left %d" % sen_vals[4])
    print("Front Center %d" % sen_vals[5])
    print("Front Right %d" % sen_vals[6])
    time.sleep(0.5)
