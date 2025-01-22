#include "KlipperComm.h"

#define MCU_NAME "AIRFILTER"
#define MAX_COMMANDS 10

KlipperComm klipperComm(MCU_NAME, MAX_COMMANDS);

#define PIN_TYPE_DOUT 0  // Digital out
#define PIN_TYPE_PWM 1   // PWM
#define PIN_TYPE_AIN 3   // Analog in
#define PIN_TYPE_DIN 4  // Digital in

struct PinConfig {
  int type;         // Pin Type 0=Digital OUT 1=PWM 2=Analog IN 3=Digital IN
  int cycleTime;    // PWM cycle time (only relevant for PWM)
  int startValue;   // Start value (only relevant for PWM and Digital Out)
  bool pullUp;      // Pull-up resistor (only for digital input pins)
  bool invert;      // Inverted logic (only for digital out pins)
};


// Map for pin configurations
Map<int, PinConfig> pinConfigurations(10);

void configAnalogInHandler(const String& command) {
  int pin = klipperComm.getParamValue(command, "PIN").toInt();
  PinConfig config;
  config.type = PIN_TYPE_AIN;

  pinConfigurations[pin](pin, config);
  pinMode(pin, INPUT);

  // Send response
  int checksum = klipperComm.calculateChecksum("ok");
  klipperComm.sendResponse("ok*" + String(checksum));
}

void configPwmOutHandler(const String& command) {
  int pin = klipperComm.getParamValue(command, "PIN").toInt();
  int cycleTime = klipperComm.getParamValue(command, "CYCLE_TIME").toInt();
  int startValue = klipperComm.getParamValue(command, "START_VALUE").toInt();

  PinConfig config;
  config.type = PIN_TYPE_PWM;
  config.cycleTime = cycleTime;
  config.startValue = startValue;

  pinConfigurations[pin](pin, config);

  pinMode(pin, OUTPUT);
  delay(10);
  analogWrite(pin, startValue);

  // Send response
  int checksum = klipperComm.calculateChecksum("ok");
  klipperComm.sendResponse("ok*" + String(checksum));
}

void configDigitalOutHandler(const String& command) {
  int pin = klipperComm.getParamValue(command, "PIN").toInt();
  int invert = klipperComm.getParamValue(command, "INVERT").toInt();

  PinConfig config;
  config.type = PIN_TYPE_DOUT;
  config.invert = invert;

  pinConfigurations[pin](pin, config);
  pinMode(pin, OUTPUT);


  // Send response
  int checksum = klipperComm.calculateChecksum("ok");
  klipperComm.sendResponse("ok*" + String(checksum));
}

void configDigitalInHandler(const String& command) {
  int pin = klipperComm.getParamValue(command, "PIN").toInt();
  int pullUp = klipperComm.getParamValue(command, "PULL_UP").toInt();

  PinConfig config;
  config.type = PIN_TYPE_DOUT;
  config.pullUp = pullUp;

  pinConfigurations[pin](pin, config);
  pinMode(pin, pullUp ? INPUT_PULLUP : INPUT);


  // Send response
  int checksum = klipperComm.calculateChecksum("ok");
  klipperComm.sendResponse("ok*" + String(checksum));
}

void setPinHandler(const String& command) {
  int pin = klipperComm.getParamValue(command, "PIN").toInt();
  int value = klipperComm.getParamValue(command, "VALUE").toInt();

  if (pinConfigurations.indexOf(pin) == pinConfigurations.getSize()) {
    int checksum = klipperComm.calculateChecksum("error: PIN NOT CONFIGURED");
    klipperComm.sendResponse("error: PIN NOT CONFIGURED*" + String(checksum));
    return;
  }

  PinConfig config = pinConfigurations[pin].getValue();

  switch (config.type) {

      case PIN_TYPE_DOUT:
        digitalWrite(pin, value ? HIGH : LOW);
        break;
      case PIN_TYPE_PWM:
        analogWrite(pin, value);
        break;
    }

    // Send response
    int checksum = klipperComm.calculateChecksum("ok");
    klipperComm.sendResponse("ok*"+ String(checksum));
}

void setup() {
    klipperComm.begin();

    // Register commands
    klipperComm.registerCommand("CONFIG_ANALOG_IN", configAnalogInHandler);
    klipperComm.registerCommand("CONFIG_PWM_OUT", configPwmOutHandler);
    klipperComm.registerCommand("SET_PIN", setPinHandler);
    klipperComm.registerCommand("CONFIG_DIGITAL_OUT", configDigitalOutHandler);
    klipperComm.registerCommand("CONFIG_DIGITAL_IN", configDigitalInHandler);
}

void loop() {
    if (klipperComm.available()) {
        String input = klipperComm.readCommand();
        klipperComm.handleCommand(input);
    }
}
