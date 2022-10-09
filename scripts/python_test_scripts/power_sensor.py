#!/usr/bin/env python3
import os
import time
from smbus import SMBus

# PowerSensor class
class PowerSensor(object):  

    # Define I2C config
    DEVICE_BUS = 1
    DEVICE_ADDR = 0x40

    # Define reg values
    CONFIG_REG     = 0x00
    SHUNTVOLT_REG  = 0x01
    BUSVOLT_REG    = 0x02
    POWER_REG      = 0x03
    CURRENT_REG    = 0x04
    CALIB_REG      = 0x05

    i2cbus = None

    # Init
    def __init__(self):
        #Main program function
        self.i2cbus = SMBus(self.DEVICE_BUS)
        # Reset configuration
        config_options = [0x8000]
        # Write configuration to the device
        self.i2cbus.write_i2c_block_data(self.DEVICE_ADDR, self.CONFIG_REG, config_options)
        # Give time for the device setup to finish
        time.sleep(0.5)
        # ISL28022 address, 0x40(64)
        # Select configuration register, 0x00(00)
        #		0x799F(31135)   No reset, Bus voltage range = 60V, PGA(Shunt voltage only) = +/- 320mV
        #                       Bus & shunt ADC resolution = 15-bit, Shunt and bus & continuous mode
        config_options = [0x799F]
        # Write configuration to the device
        self.i2cbus.write_i2c_block_data(self.DEVICE_ADDR, self.CONFIG_REG, config_options)
        # Give time for the device setup to finish
        time.sleep(0.5)
        # 5mOhm Shunt with 320mV range
        # Calibration register needs to be set to 4194 (dec) = 1062 (hex)
        calib_value = [0x1062]
        # Write calibration value to the device
        self.i2cbus.write_i2c_block_data(self.DEVICE_ADDR, self.CALIB_REG, calib_value)
        # Give time for the device setup to finish
        time.sleep(0.5)


    def deinit(self):
        pass


    def read_bus_voltage(self):
        # ISL28022 address, 0x40(64)
        # Read data back from 0x01(01), 2 bytes
        data = self.i2cbus.read_i2c_block_data(self.DEVICE_ADDR, self.BUSVOLT_REG, 2)
        # Convert the data
        raw_voltage = data[0] * 256 + data[1]
        #raw_voltage = raw_voltage >> 2
        #raw_voltage = raw_voltage * 0.004
        if raw_voltage > 32767 :
            raw_voltage -= 65536
        return raw_voltage

    def read_shunt_voltage(self):
        # ISL28022 address, 0x40(64)
        # Read data back from 0x01(01), 2 bytes
        data = self.i2cbus.read_i2c_block_data(self.DEVICE_ADDR, self.SHUNTVOLT_REG, 2)
        # Convert the data
        raw_voltage = data[0] * 256 + data[1]
        if raw_voltage > 32767 :
            raw_voltage -= 65536
        return raw_voltage

    def read_current(self):
        # ISL28022 address, 0x40(64)
        # Read data back from 0x01(01), 2 bytes
        data = self.i2cbus.read_i2c_block_data(self.DEVICE_ADDR, self.CURRENT_REG, 2)
        # Convert the data
        raw_voltage = data[0] * 256 + data[1]
        if raw_voltage > 32767 :
            raw_voltage -= 65536
        return raw_voltage

    def read_power(self):
        # ISL28022 address, 0x40(64)
        # Read data back from 0x01(01), 2 bytes
        data = self.i2cbus.read_i2c_block_data(self.DEVICE_ADDR, self.POWER_REG, 2)
        # Convert the data
        raw_voltage = data[0] * 256 + data[1]
        if raw_voltage > 32767 :
            raw_voltage -= 65536
        return raw_voltage


