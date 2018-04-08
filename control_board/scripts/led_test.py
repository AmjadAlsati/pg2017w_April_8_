#!/usr/bin/env python

import smbus
import time

CCS_I2C_BUS = 0X69

# instruction definitions
CCS_INSTR_SET_LED = 0X20

bus = smbus.SMBus(1)

print("Testing the LEDs")

print("Doing manual test on red LED")
# 0, 0
print(0, 0);
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [0, 0])
time.sleep(.2)
# 15, 0
print(15, 0);
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [15, 0])
time.sleep(.2)
# 31, 0
print(31, 0);
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [31, 0])
time.sleep(.2)
# 63, 0
print(63, 0);
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [63, 0])
time.sleep(.2)

print("Doing manual test on blue LED")
# 0, 0
print(0, 0);
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [0, 0])
time.sleep(.2)
# 0, 15
print(0, 15);
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [0, 15])
time.sleep(.2)
# 0, 31
print(0, 31);
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [0, 31])
time.sleep(.2)
# 0, 63
print(0, 63);
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [0, 63])
time.sleep(.2)

print("Doing manual test on both LEDs")
# 31, 63
print(31, 63);
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [31, 63])
time.sleep(.2)
# 63, 31
print(63, 31);
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [63, 31])
time.sleep(.2)
# 63, 63
print(63, 63);
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [63, 63])
time.sleep(.2)

# test red LED
print("Doing continous fading test on red LED")
for v in range(63):
    print(v)
    bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [v, 0])
    time.sleep(.01)
for v in range(63):
    print(63-v)
    bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [63-v, 0])
    time.sleep(.01)

# test blue LED
print("Doing continous fading test on blue LED")
for v in range(63):
    print(v);
    bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [0, v])
    time.sleep(.01)
for v in range(63):
    print(63-v)
    bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [0, 63-v])
    time.sleep(.01)

print("Finished LED testing")
