import network
import time

# load WLAN credentials
from WLAN_PW import SSID, PASSWORD

# define function to start network
def net():
    sta_if = network.WLAN(network.STA_IF)
    sta_if.active(True)

    if not sta_if.isconnected():  # Check if connected
        print('Connecting to WiFi "{}"'.format(SSID))

        sta_if.connect(SSID, PASSWORD) # Connect to an AP

        while not sta_if.isconnected():  # Check for successful connection
            print(".", end='')
            time.sleep(1)

    print()
    print(sta_if.ifconfig())

# load CLI commands
from upysh import *

import os, time, machine, onewire, ds18x20, network
from machine import PWM, Pin, ADC
import urequests as requests