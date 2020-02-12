# MicroPython-Show

Live-Vortrag über MicroPython für ESP8266 / ESP32

![](https://upload.wikimedia.org/wikipedia/commons/thumb/4/4e/Micropython-logo.svg/200px-Micropython-logo.svg.png) ![](https://avatars2.githubusercontent.com/u/6298560?s=200&v=4)

"MicroPython ist eine Softwareimplementierung einer Programmiersprache, die weitgehend kompatibel mit Python 3 ist, geschrieben in C, die für den Betrieb auf einem Mikrocontroller optimiert ist. MicroPython ist ein vollständiger Python-Compiler und eine Laufzeitumgebung, die auf der Mikrocontroller-Hardware läuft. Enthalten ist eine Auswahl von Python-Kernbibliotheken; MicroPython enthält Module, die dem Programmierer Zugriff auf Low-Level-Hardware ermöglichen. Der Quellcode für das Projekt ist auf GitHub unter der MIT-Lizenz verfügbar." 
*Quelle: [Wikipedia.de](https://de.wikipedia.org/wiki/MicroPython) / [Wikipedia.com](https://en.wikipedia.org/wiki/MicroPython)*

##### PyBoard
![](http://micropython.org/static/home/img/pybv11-persp.jpg)
*Quelle: [micropython.org](http://micropython.org/)*

Entwickelt von [Damien P. George](http://dpgeorge.net/) in 2014. Sourcen auf [Github](https://github.com/micropython/micropython)

---

### Größenordnung

Typische Größen:

| Rechner | Bits | CPU | RAM | Disk/Flash | P | Features |
| - | - | - | - | - | - | - |
| PC / Laptop | 64 | ~3 GHz | ~8 GB | ~1 TB | >50 W | Betriebssystem, (W)LAN, FPU, GPU, ...
| Raspberry Pi 3 | 32 | 1 GHz | 512 MB | 4 GB ... 32 GB | 5 W | Betriebssystem, GPIO, (W)LAN
| __ESP32__ | 32 | 240 MHz | 160 kB (4 MB) | 4 MB | <1 W | GPIO, WLAN
| __ESP8266__ | 32 | 80 MHz | 80 kB | 512 kB ... 4 MB | <1 W | GPIO, WLAN
| Arduino (AVR) | 8 | 16 MHz| 2 Kb | 16 kB | <100 mW | GPIO

> => Für Hardware-Spielereien: **ESP**

---

### Entwicklungsumgebung für ESP

##### Arduino-IDE

https://www.arduino.cc/

- Programmiersprache (fast) C / C++
- Compiler
- Programmier-Tool (speichern im Flash)
- Kein Debugger

##### MicroPython

http://micropython.org/
https://docs.micropython.org/

- Programmiersprache MicroPython - entspricht CPython 3.4
- REPL (Read–eval–print loop)
- WebREPL
- Flash-Dateisystem für *.py und Daten
- 'Boot-Loader' für ESP
- Standard-Module (fast) wie CPython
- Hardware-Module für ESP

---

### Installation

##### Einmalige Flashen auf dem ESP

http://micropython.org/download

Anleitung folgen...

    esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash
    esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x1000 esp32-20190125-v1.10.bin

##### Kommunikation mit ESP / IDE

- Terminal (TTY/COM 115200 Buad 8N1)
- "[uPyCraft](https://github.com/DFRobot/uPyCraft)" (GUI, Windows, Mac)
- "[mpfshell](https://github.com/wendlers/mpfshell)" (CLI, alle)
- "rshell" (CLI, Linux)
- "pycom"-Plugin für VS-Code (GUI, alle)

Antwort vom ESP:

    MicroPython v1.12-164-g7679e3be9 on 2020-02-12; ESP module with ESP8266
    Type "help()" for more information.
    >>> 

Spielen im REPL:

```python
>>> 1 + 2
3
>>> "Hallo" + " Welt"
'Hallo Welt'
>>> a = "Hallo"
>>> print(a)
Hallo
>>> a
'Hallo'
>>>
```

REPL mit Tab-Completion ...

WebREPL: http://micropython.org/webrepl/

---

### Module (Libs)

Anzeige aller 'eingebauten' Module:

```python
>>> help('modules')
__main__          inisetup          ucollections      uselect
_boot             lwip              ucryptolib        usocket
_onewire          machine           uctypes           ussl
_webrepl          math              uerrno            ustruct
apa102            micropython       uhashlib          utime
btree             neopixel          uheapq            utimeq
builtins          network           uio               uwebsocket
dht               ntptime           ujson             uzlib
ds18x20           onewire           uos               webrepl
esp               port_diag         upip              webrepl_setup
flashbdev         sys               upip_utarfile     websocket_helper
framebuf          uarray            urandom
gc                ubinascii         ure
Plus any modules on the filesystem
>>>
```

##### Übersicht Standard-Module (wie CPython)

- cmath – mathematical functions for complex numbers
- gc – control the garbage collector
- math – mathematical functions
- sys – system specific functions
- uarray – arrays of numeric data
- ubinascii – binary/ASCII conversions
- ucollections – collection and container types
- uerrno – system error codes
- uhashlib – hashing algorithms
- uheapq – heap queue algorithm
- uio – input/output streams
- ujson – JSON encoding and decoding
- uos – basic “operating system” services
- ure – simple regular expressions
- uselect – wait for events on a set of streams
- usocket – socket module
- ussl – SSL/TLS module
- ustruct – pack and unpack primitive data types
- utime – time related functions
- uzlib – zlib decompression
- _thread – multithreading support

##### Übersicht MicroPython-Module

- btree – simple BTree database
- framebuf — frame buffer manipulation
- machine — functions related to the hardware
- micropython – access and control MicroPython internals
- network — network configuration
- ubluetooth — low-level Bluetooth
- ucryptolib – cryptographic ciphers
- uctypes – access binary data in a structured way

##### Übersicht ESP-Module

- ESP — functions related to the ESP8266 and ESP32 
    sleep...
- ESP32 — functionality specific to the ESP32 
    Flash partitions...
    RMT
    Ultra-Low-Power co-processor
    hall-sensor

##### Regeln für Modulnamen

Mit __import__ werden Module geladen.
Wird ein Name ohne führendem 'u' angegeben wird zuerst ein Modul mit diesem Namen gesucht. Ist kein Modul mit diesem Namen vorhanden, wird ein Modul mit vorangestelltem 'u' gesucht und unter dem Originalnamen geladen.

```python
>>> import os                 # entspricht: import uos as os
>>> os.uname()
(sysname='esp8266', nodename='esp8266', release='2.0.0(5a875ba)', version='v1.12-164-g7679e3be9 on 2020-02-12', machine='ESP module with ESP8266')
>>>
```

Die Module mit 'u' sind fast komplett implementiert!

Spielen:

```python
>>> import sys
>>> sys.                      # <<<TAB>>>
__class__       __name__        argv            byteorder
exit            implementation  maxsize         modules
path            platform        print_exception
stderr          stdin           stdout          version
version_info
>>> sys.std                   # <<<TAB>>>
stderr          stdin           stdout
>>> sys.std
```

---

### Zugriff auf die Hardware des ESP

Alle Klassen für den Zugriff auf Hardware sind im Modul "machine"

##### GPIO als Ausgang (LED)

```python
>>> from machine import Pin
>>> pin2 = Pin(2, Pin.OUT)
>>> pin2
Pin(2)
>>> pin2.
__class__       value           IN              IRQ_FALLING
IRQ_RISING      OPEN_DRAIN      OUT             PULL_UP
init            irq             off             on
>>> pin2.on()
>>> pin2.off()
>>> pin2.value(1)
>>> pin2.value(0)
>>>
```

##### GPIO als Eingang (Button)

```python
>>> from machine import Pin
>>> pin0 = Pin(0, Pin.IN, Pin.PULL_UP)
>>> pin0.value()
1
>>> pin0.value()                   # gedrückter Button
0
>>>
```

##### PWM (LED)

```python
>>> from machine import PWM, Pin
>>> pin2 = Pin(2, Pin.OUT)
>>> pwm2 = PWM(pin2)
>>> pwm2.duty(512)
>>> pwm2.duty(0)
>>> pwm2.duty(1023)
>>>
```

##### I2C-Bus

```python
>>> from machine import I2C, Pin
>>> i2c = I2C(scl=Pin(5), sda=Pin(4), freq=100000)
>>> i2c.readfrom(0x3a, 4)   # read 4 bytes from slave device 
>>> i2c.writeto(0x3a, '12') # write '12' to slave device with 
>>>
```

##### Neopixel (WS2812)

```python
>>> from machine import Pin
>>> from neopixel import NeoPixel
>>> pin = Pin(0, Pin.OUT)   # set GPIO0 to output to drive NeoPixels
>>> np = NeoPixel(pin, 8)   # create NeoPixel driver on GPIO0 for 8 pixels
>>> np[0] = (255, 255, 255) # set the first pixel to white
>>> np.write()              # write data to all pixels
>>> r, g, b = np[0]         # get first pixel colour
```

##### ADC (Analog-Digital-Wandler)

ESP8266:
```python
>>> from machine import ADC
>>> adc = ADC(0)            # create ADC object on ADC pin
>>> adc.read()              # read value, 0-1024
```
ESP32:
```python
rom machine import ADC

adc = ADC(Pin(32))          # create ADC object on ADC pin
adc.read()                  # read value, 0-4095 across voltage range 0.0v - 1.0v

adc.atten(ADC.ATTN_11DB)    # set 11dB input attenuation (voltage range roughly 0.0v - 3.6v)
adc.width(ADC.WIDTH_9BIT)   # set 9 bit return values (returned range 0-511)
adc.read()                  # read value using the newly configured attenuation and width
```

##### Weitere...

- UART
- SPI
- DAC
- RTC
- RMT (ESP32)
- OneWire
- Capacitive touch (ESP32)
- DHT
- Timer
- Sleep

---

### Netzwerk

##### Verbindung zu einem Access-Point (AP)

```python
import network
import time

SSID = "<your-SSID>"
PWD = "<your-password>"

sta_if = network.WLAN(network.STA_IF)   # WLAN-Objekt als _Client_ erzeugen
sta_if.active(True)                     # Modem einschalten
sta_if.scan()                           # Scannen nach APs
sta_if.connect(SSID, PWD)               # Verbindung herstellen
while not sta_if.isconnected():         # Verbindung prüfen
    pass
print(sta_if.ifconfig())                # IP ausgeben
```

> siehe main.py

##### Verbindung als Access-Point (AP)

TODO

```python
station = network.WLAN(network.STA_IF)

station.active(True)
station.connect(ssid, password)

while not station.isconnected():
    pass

print(station.ifconfig())
```

##### Uhrzeit über Netzwerk holen (NTP)

```python
>>> import ntptime
>>> ntptime.settime()
>>>
>>> rtc = machine.RTC()
>>> print('Date Time:', rtc.datetime())
Date Time: (2020, 2, 12, 2, 11, 8, 38, 481)
>>>
```

##### MQTT-Client

> Nur auf ESP32 als Standard-Modul

```python
from umqttsimple import MQTTClient

client = MQTTClient(client_id, '192.168.1.123')
client.set_callback(sub_cb)
client.connect()
client.subscribe('/zuhause/licht/#')

client.publish('/zuhause/licht/schalter', 'ON')

def sub_cb(topic, msg):
    print(topic, msg)
    if topic == b'/zuhause/licht/schalter' and msg == b'ON':
        print('Es werde Licht!')
```

##### HTTP-Request (REST-API)

> Nur auf ESP32 als Standard-Modul

```python
import urequests as requests

>>> r = requests.get("https://www.nerd2nerd.org")
>>> r
<Response object at 3f826240>
>>> r.   # <<<TAB>>>
__class__       __init__        __module__      __qualname__
close           __dict__        encoding        text
json            content         raw             _cached
status_code     reason
>>> r.text
'<!DOCTYPE html>\n<html xmlns="http://www.w3.org/1999/xhtml" lang="de" xml:lang="de">\n  <head><meta ...
...
```

---

### Flash-Dateisystem

```python
with open('hello.txt', 'w') as f:
    f.write('Hello world')

with open('hello.txt', 'r') as f:
    print(f.read())
```

Weitere Funktionen im Modul "os"

```python
>>> import os
>>> os.listdir('/')
['boot.py', 'main.py']
>>>
```

Hack auf ESP32:

```python
>>> from upysh import *

upysh is intended to be imported using:
from upysh import *

To see this help text again, type "man".

upysh commands:
pwd, cd("new_dir"), ls, ls(...), head(...), cat(...)
newfile(...), mv("old", "new"), rm(...), mkdir(...), rmdir(...),
clear

>>> ls
    139 boot.py
    342 debug.json
    <dir> lib
    822 main.py
    413 networks.json
    <dir> www

>>> cd("lib")
>>>
```

---

### Demo: WebServer

[Projekt auf Github](https://github.com/jczic/MicroWebSrv2)

---

### The End
