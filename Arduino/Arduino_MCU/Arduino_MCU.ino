// Example sketch for an Arduino Nano Every mcu, which is used for an air filter.
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <avr/sleep.h>
#include <EEPROM.h>

#define _PWM_LOGLEVEL_ 0
#if defined(ARDUINO_AVR_NANO)
#include "AVR_PWM.h"
#elif defined(ARDUINO_AVR_NANO_EVERY)
#include "megaAVR_PWM.h"
#endif

#define MCU_NAME "AIRFILTER"  // Identifier for the MCU

// EEPROM addresses for stored data
// Note: An EEPROM memory has a specified life of 100,000 write/erase cycles, 
// so you may need to be careful about how often you write to it. 
#define EEPROM_I2C_ADDR 0  // Memory address for the I2C address
#define EEPROM_FAN_PIN 1   // Memory address for Fan Pin
#define EEPROM_PWM_FREQ 2  // Memory address for PWM-Frequenz
#define EEPROM_TACH_PIN 3  // Memory address for Tacho signal pin

// Create an instance of the Adafruit BME680 library
Adafruit_BME680 bme(&Wire);  // I2C

//creates pwm instance
#if defined(ARDUINO_AVR_NANO)
AVR_PWM* PWM_Instance;
#elif defined(ARDUINO_AVR_NANO_EVERY)
megaAVR_PWM* PWM_Instance;
#endif

unsigned long previousMillis = 0;     // Tracks the last time a sensor reading was made
const unsigned long interval = 2000;  // Interval for sensor readings (2 seconds)


int fanPin = 3;                    // default Pin for fan control (PWM)
int tachoPin = 4;                  // default tacho pin for fan
int pwmFreq = 62500;               // default PWM frequency
int fanSpeed = 0;                  // Current fan speed (PWM value, 0-255)
uint8_t i2cAddress = 0x76;         // Default I2C address
volatile unsigned int fanRPM = 0;  // Calculated fan RPM
unsigned long lastFanUpdate = 0;   // Tracks the last RPM calculation time
bool bme68x_ready = false;

float temp = 0.0;
float humidity = 0.0;
float voc = 0.0;

// Function prototypes
void loadConfig();
void saveConfig();
bool validateChecksum(String command, int receivedChecksum);
void setI2CAddress(uint8_t address);
void setFanPin(int pin);
void setTachoPin(uint8_t pin);
void setPWMFrequency(int frequency);
void enterSleep();
void updateFanRPM();
void startFan();
void stopFan();
void setFanSpeed(int percent);
void SensorStatus();

void setup() {
  Serial.begin(115200);  // Start serial communication for debugging and commands

  // Load configuration from EEPROM
  loadConfig();

  pinMode(fanPin, OUTPUT);  // Set fan pin as an output for PWM control

  // Initialize the PWM for Fan
#if defined(ARDUINO_AVR_NANO)
  PWM_Instance = new AVR_PWM(fanPin, pwmFreq, 0);
#elif defined(ARDUINO_AVR_NANO_EVERY)
  PWM_Instance = new megaAVR_PWM(fanPin, pwmFreq, 0);
#endif

  // Initialize the BME680 sensor
  if (bme.begin(0x76)) {  // Default I2C address is 0x76
    bme68x_ready = true;
  } else {
    //Serial.println("ERROR: Could not find BME680 sensor!");
  }

  if (bme68x_ready) {
    // Configure BME680
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);  // 320°C for 150ms
  }
}

void (*resetFunc)(void) = 0;  //declare reset function @ address 0

void loop() {
  unsigned long currentMillis = millis();  // Get the current time

  // update / read data from the BSEC sensor
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    updateFanRPM();
    if (bme68x_ready) {
      if (bme.performReading()) {
        temp = bme.temperature;             // Temperature in °C
        humidity = bme.humidity;            // Humidity in %
        voc = bme.gas_resistance / 1000.0;  // VOC resistance in kOhms
      }
    }
  }

  // Process serial commands
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read the full command
    int splitIndex = input.indexOf('*');          // Separate the command and checksum
    if (splitIndex == -1) {
      Serial.println("ERROR: NO CHECKSUM");
      return;
    }
    String command = input.substring(0, splitIndex);
    int checksum = input.substring(splitIndex + 1).toInt();

    if (!validateChecksum(command, checksum)) {
      Serial.println("ERROR: CHECKSUM MISMATCH");
      return;
    }

    // Execute commands based on input
    if (command == "IDENTIFY") {
      Serial.println(String(MCU_NAME));
    } else if (command.startsWith("SET_I2C ")) {
      uint8_t address = command.substring(8).toInt();
      setI2CAddress(address);
    } else if (command.startsWith("SET_FANPIN ")) {
      int pin = command.substring(11).toInt();
      setFanPin(pin);
    } else if (command.startsWith("SET_TACHPIN ")) {
      int pin = command.substring(12).toInt();
      setTachoPin(pin);
    } else if (command.startsWith("SET_PWM ")) {
      int frequency = command.substring(8).toInt();
      setPWMFrequency(frequency);
    } else if (command == "TEMP") {
      Serial.println(String(temp));  // °C
    } else if (command == "VOC") {
      Serial.println(String(voc));  // kOhm
    } else if (command == "HUMIDITY") {
      Serial.println(String(humidity));  // %
    } else if (command == "START") {
      startFan();
      Serial.println("FAN STARTED");
    } else if (command == "STOP") {
      stopFan();
      Serial.println("FAN STOPPED");
    } else if (command.startsWith("PWM ")) {
      int percent = command.substring(4).toInt();
      setFanSpeed(percent);
      Serial.println("FAN SPEED SET TO " + String(percent) + "%");
    } else if (command == "SLEEP") {
      Serial.println("MCU '" + String(MCU_NAME) + "' POWERED DOWN");
      delay(100);
      enterSleep();
    } else if (command == "REBOOT") {
      Serial.println("MCU '" + String(MCU_NAME) + "' IS REBOOTING");
      delay(100);
      resetFunc();
    } else if (command == "SENSOR_STATUS") {
      SensorStatus();
    } else {
      Serial.println("ERROR: UNKNOWN COMMAND");
    }
  }
}

// Load configuration from EEPROM
void loadConfig() {
  i2cAddress = EEPROM.read(EEPROM_I2C_ADDR);
  if (i2cAddress < 0x03 || i2cAddress > 0x77) {
    i2cAddress = 0x76;  // Set default address
  }

  fanPin = EEPROM.read(EEPROM_FAN_PIN);
  if (fanPin < 0 || fanPin > 13) {
    fanPin = 3;  // Set default pin
  }

  pwmFreq = EEPROM.read(EEPROM_PWM_FREQ);
  if (pwmFreq < 30 || pwmFreq > 78125) {
    pwmFreq = 62500;  // Set default frequency
  }
}

// Save configuration to EEPROM
void saveConfig() {
  EEPROM.update(EEPROM_I2C_ADDR, i2cAddress);
  EEPROM.update(EEPROM_FAN_PIN, fanPin);
  EEPROM.update(EEPROM_PWM_FREQ, pwmFreq);
  EEPROM.update(EEPROM_TACH_PIN, tachoPin);
}

// Set tacho pin
void setTachoPin(uint8_t pin) {
  if (pin >= 0 && pin <= 13) {
    tachoPin = pin;
    pinMode(tachoPin, INPUT);
    saveConfig();
    Serial.println("Tacho pin set to: " + String(pin) + ". Need MCU Reboot !");
  } else {
    Serial.println("ERROR: Invalid Tacho pin!");
  }
}

// Set I2C address
void setI2CAddress(uint8_t address) {
  if (address >= 0x03 && address <= 0x77) {
    i2cAddress = address;
    saveConfig();
    Serial.println("I2C address set to: 0x" + String(address, HEX) + ". Need MCU Reboot !");
  } else {
    Serial.println("ERROR: Invalid I2C address!");
  }
}

// Set fan pin
void setFanPin(int pin) {
  if (pin >= 0 && pin <= 13) {
    fanPin = pin;
    pinMode(fanPin, OUTPUT);
    saveConfig();
    Serial.println("Fan pin set to: " + String(pin) + ". Need MCU Reboot !");
  } else {
    Serial.println("ERROR: Invalid fan pin!");
  }
}

// Set PWM frequency
void setPWMFrequency(int frequency) {
  if (frequency >= 30 && frequency <= 5000) {
    pwmFreq = frequency;
    saveConfig();
    Serial.println("PWM frequency set to: " + String(frequency) + ". Need MCU Reboot !");
  } else {
    Serial.println("ERROR: Invalid PWM frequency!");
  }
}

void SensorStatus() {
  if (bme68x_ready)
    Serial.println("READY");
  else
    Serial.println("NOT READY");
}
// Validate checksum to ensure command integrity
bool validateChecksum(String command, int receivedChecksum) {
  int calculatedChecksum = 0;
  for (int i = 0; i < command.length(); i++) {
    calculatedChecksum += command[i];
  }
  return (calculatedChecksum % 256) == receivedChecksum;
}

// Update fan RPM
void updateFanRPM() {
  unsigned long pulseDuration = pulseIn(tachoPin, LOW);  // Measure the duration of a pulse (in microseconds)
  if (pulseDuration > 0) {
    fanRPM = 60000000 / (pulseDuration * 2);  // 2 pulses per revolution
  }
}

// Start the fan at full speed
void startFan() {
  fanSpeed = 100;  // Full PWM value
  PWM_Instance->setPWM(fanPin, pwmFreq, fanSpeed);
}

// Stop the fan
void stopFan() {
  fanSpeed = 0;  // Zero PWM value
  PWM_Instance->setPWM(fanPin, pwmFreq, fanSpeed);
}

// Set fan speed based on a percentage (0-100%)
void setFanSpeed(int percent) {
  fanSpeed = percent;
  PWM_Instance->setPWM(fanPin, pwmFreq, fanSpeed);
}

// Put the MCU into deep sleep mode
void enterSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Deepest sleep mode available
  sleep_enable();
  sleep_cpu();  // Enter sleep mode
}



