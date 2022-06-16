#!/usr/bin/env python3
import os
import time
import RPi.GPIO as GPIO


# RelayController class
class RelayController(object):
    # GPIO pin for relay control
    RELAY_GPIO = 4

    # Init relay controller class
    def __init__(self, _gpio):
        # Configure the GPIO for relay control
        self.RELAY_GPIO = _gpio
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.RELAY_GPIO,GPIO.OUT)


    # Denit relay controller class
    def deinit(self):
        # Set GPIO to state LOW
        GPIO.output(self.RELAY_GPIO,False)
        # Clean-up GPIO stack
        GPIO.cleanup()


    # Engage the relay
    def engage_relay(self):
        # Set GPIO to state HIGH
        GPIO.output(self.RELAY_GPIO,True)


    # Disengage the relay
    def disengage_relay(self):
        # Set GPIO to state LOW
        GPIO.output(self.RELAY_GPIO,False)
        
