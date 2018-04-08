#!/usr/bin/env python
import smbus
import time
###########################
# SETTINGS                #
###########################
CCS_I2C_BUS = 0X69

# instruction definitions
CCS_INSTR_SET_LED = 0X20
CCS_INSTR_SET_VELO = 0X1A

CCS_INSTR_GET_POS = 0X1B
CCS_LEN_GET_POS = 6

CCS_INSTR_GET_SENS = 0X10
CCS_LEN_GET_SENS = 13


bus = smbus.SMBus(1)

print("testing Engine seconds")
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [60,60])
time.sleep(2)
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [0x0,0x0])

time.sleep(5)

print("now left")
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [40,-40])
time.sleep(0.5)
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [0x0,0x0])
time.sleep(3)
print("drive straight a little")
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [40,40])
time.sleep(1)
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [0x0,0x0])
time.sleep(3)
print("Now right")
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [-40,40])
time.sleep(0.5)
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [0x0,0x0])
print("testing Engine for 3 seconds")
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [
    19, 19])
time.sleep(3)
print("drive straight a little")
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [40,40])
time.sleep(1)
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [0x0,0x0])
time.sleep(3)
print("Now 180 turn")
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [40,-40])
time.sleep(0.9)
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [0x0,0x0])
time.sleep(3)
print("now back to place")
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [40,40])
time.sleep(3)
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_VELO, [0x0,0x0])

print("Reading 10 Sensor values with gap of 0.5 seconds")

ticks = 10
while (ticks > 0):
	sen_vals = bus.read_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_GET_SENS, CCS_LEN_GET_SENS)

	print(sen_vals[2:7])
	ticks = ticks - 1
	time.sleep(0.5)

print("testing the LEDs")

bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [60, 60])
time.sleep(1)
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [0, 0])
time.sleep(1)
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [60, 0])
time.sleep(0.5)
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [0, 60])
time.sleep(0.5)
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [60, 0])
time.sleep(0.5)
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [0, 60])
time.sleep(0.5)
bus.write_i2c_block_data(CCS_I2C_BUS, CCS_INSTR_SET_LED, [0, 0])

print("Robot Unit test completed!!")

