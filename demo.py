import os, time, machine, onewire, ds18x20, network
from machine import PWM, Pin, ADC
from umqtt.simple import MQTTClient
import urequests as requests

# LED an GPIO12
led = machine.Pin(12, Pin.OUT)   # Pin für LED

# OneWire an GPIO19
pin_ow = machine.Pin(19)   # Pin für OneWire-Bus

ow = onewire.OneWire(pin_ow)
ds = ds18x20.DS18X20(ow)

#b'(ad\x11\xb3\x94V\xf5'
#b'(ad\x11\x83\xd6q\x81'

# Netzwerk
net()
mqtt_client = MQTTClient('ich', '10.0.0.2')
mqtt_client.connect()

# Endlosschleife
while 1:
    # Temperatur lesen
    ds.convert_temp()   # Messung starten
    time.sleep_ms(750)   # Messzeit abwarten: 750ms
    t = ds.read_temp(b'(ad\x11\x83\xd6q\x81')
    print(t)

    # MQTT-Broker benachrichtigen
    mqtt_client.publish('/zuhause/temp/ds', str(t))

    # Abhängig von Temperatur Steckdose und LED schalten
    if t < 17:
        requests.get("http://fs200.fritz.box/cm?cmnd=Power%20On")
        led.on()
    elif t > 22:
        requests.get("http://fs200.fritz.box/cm?cmnd=Power%20Off")
        led.off()

    time.sleep(9)
