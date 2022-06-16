#!/usr/bin/env python3

import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
RELAY = 4
GPIO.setup(RELAY,GPIO.OUT)
GPIO.output(RELAY,True)
time.sleep(2)
GPIO.output(RELAY,False)