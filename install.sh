#!/bin/bash

#Define the path to the Klipper directory (assuming Klipper is installed in the user's home directory).
KLIPPER_DIR="${HOME}/klipper"

# Determine the directory where the current script is located.
# $(dirname "$0") gives the directory of the script.
# 'cd --' changes to that directory, and 'pwd -P' returns the absolute path of that directory.
ARDUINO_MCU_DIR="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# Check if the Klipper directory exists.
# If it doesn't exist, output an error message and exit the script with error code 1.
if [ ! -d "$KLIPPER_DIR" ]; then
    echo "arduino_mcu: klipper doesn't exist"
    exit 1
fi

echo "arduino_mcu: linking arduino_mcu.py to klippy/extra folder"

# Check if the file 'arduino_mcu.py' already exists in the target directory.
# If the file exists, remove it.
if [ -e "${KLIPPER_DIR}/klippy/extras/arduino_mcu.py" ]; then
    rm "${KLIPPER_DIR}/klippy/extras/arduino_mcu.py"
fi
# Create a symbolic link (symlink) from 'arduino_mcu.py'
# to the target directory ${KLIPPER_DIR}/klippy/extras.
ln -s "${ARDUINO_MCU_DIR}/klippy/extra/arduino_mcu.py" "${KLIPPER_DIR}/klippy/extras/arduino_mcu.py"

# Check if the file 'klippy/extras/arduino_mcu.py' is already in the Git exclusion list.
# The exclusion list is stored in the .git/info/exclude file.
# If the entry is not present, append it to the file.
if ! grep -q "klippy/extras/arduino_mcu.py" "${KLIPPER_DIR}/.git/info/exclude"; then
    echo "klippy/extras/arduino_mcu.py" >> "${KLIPPER_DIR}/.git/info/exclude"
fi

echo "arduino_mcu: installation successful."