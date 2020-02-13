import network
import time

SSID = "<your-SSID>"
PASSWORD = "<your-password>"
SSID = "vy_guest"
PASSWORD = "#Mercury2017"

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

from upysh import *
