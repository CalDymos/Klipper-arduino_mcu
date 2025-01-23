# Klipper arduino_mcu
![Alpha Status](https://img.shields.io/badge/status-alpha-red)
[![Project Maintenance](https://img.shields.io/maintenance/yes/2024.svg)](https://github.com/CalDymos/Klipper-arduino_mcu 'GitHub Repository')
[![License](https://img.shields.io/github/license/CalDymos/Klipper-arduino_mcu.svg)](https://github.com/CalDymos/Klipper-arduino_mcu/blob/main/LICENSE 'License')


🚧 **Alpha-Status**: This project is in the alpha phase. It is not yet stable and may change drastically.


extends Klipper (klippy/extra) by the possibility to communicate with some ATmega/ESP/SAMD/RP2040/ATtiny MCUs via the serial interface (UART), which are not directly supported by klipper. (I use it for the Arduino Nano Every)
The firmware for the MCU can/must then be programmed in the Arduino IDE.

arduino_mcu is intended as an extension to the main mcu, e.g. to realize a spool manager, a controller for air filters or similar.
It is not intended to control time-critical outputs or to query time-critical inputs.

## Install

Clone this repository from git and run the install script:

```sh
cd ~
git clone https://github.com/CalDymos/Klipper-arduino_mcu.git
./Klipper-arduino_mcu/klippy/extras/install.sh
```

## compile and flash firmware

- Install [Arduino IDE](https://www.arduino.cc/en/software)
- open `Arduino_MCU.ino`, modify, compile and upload it to mcu

## Usage

### Configuration

First, add an `[arduino_mcu mcu_name]` section to your `printer.cfg` to define the `arduino_mcu`:

```ini
[arduino_mcu mcu_name]
port:
#   The serial port to connect to the MCU. If unsure (or if it
#   changes) use
#   ls /dev/tty*
#   This shows a list of all serial devices. Common ports for the Arduino Nano are e.g. /dev/ttyUSB0 or /dev/ttyACM0.
#   The default is /dev/ttyUSB0
baud: 115200
#   The baud rate to use. The default is 115200.
```


### G-Code Command

You can use this G-code command to send commands to the Arduino_MCU. A handler must be set up on the MCU for the command.

`SEND_ARDUINO TARGET=<mcu_name> COMMAND="<command>"`
