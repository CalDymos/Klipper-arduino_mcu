# Klipper-arduino_mcu
extends Klipper (klippy/extra) by the possibility to communicate with an Arduino via the serial interface 

## G-Code Command

`SEND_ARDUINO TARGET=<mcu_name> COMMAND="<command>"`

#### example
`SEND_ARDUINO TARGET=airfilter COMMAND="SET_LED 128"`

## Configuration

```
[arduino_mcu mcu_name]
port:
#   The serial port to connect to the MCU. If unsure (or if it
#   changes) use
#   ls /dev/tty*
#   This shows a list of all serial devices. Common ports for the Arduino Nano are e.g. /dev/ttyUSB0 or /dev/ttyACM0.
#   The default is /dev/ttyUSB0
baud: 115200
#   The baud rate to use. The default is 115200.
timeout: 1
#   Timout for receiving data. The default is 1.
retries_on_timeout: 2
#   Number of retries if a timeout has occurred. The default is 0.
```


