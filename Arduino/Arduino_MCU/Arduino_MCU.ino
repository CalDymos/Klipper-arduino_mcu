#include "KlipperComm.h"

#define MCU_NAME "airfilter"
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
  int pin = klipperComm.getParamValue(command, "pin").toInt();
  int nid = klipperComm.getParamValue(command, "nid").toInt();

  PinConfig config;
  config.type = PIN_TYPE_AIN;

  pinConfigurations[pin](pin, config);
  pinMode(pin, INPUT);

  // Send response
  klipperComm.sendResponse("ok nid=" + String(nid));
}

void configPwmOutHandler(const String& command) {
  int pin = klipperComm.getParamValue(command, "pin").toInt();
  int cycleTime = klipperComm.getParamValue(command, "cycle_time").toInt();
  int startValue = klipperComm.getParamValue(command, "start_value").toInt();
  int nid = klipperComm.getParamValue(command, "nid").toInt();

  PinConfig config;
  config.type = PIN_TYPE_PWM;
  config.cycleTime = cycleTime;
  config.startValue = startValue;

  pinConfigurations[pin](pin, config);

  pinMode(pin, OUTPUT);
  delay(10);
  analogWrite(pin, startValue);

  // Send response
  klipperComm.sendResponse("ok nid=" + String(nid));
}

void configDigitalOutHandler(const String& command) {
  int pin = klipperComm.getParamValue(command, "pin").toInt();
  int invert = klipperComm.getParamValue(command, "invert").toInt();
  int nid = klipperComm.getParamValue(command, "nid").toInt();

  PinConfig config;
  config.type = PIN_TYPE_DOUT;
  config.invert = invert;

  pinConfigurations[pin](pin, config);
  pinMode(pin, OUTPUT);


  // Send response
  klipperComm.sendResponse("ok nid=" + String(nid));
}

void configDigitalInHandler(const String& command) {
  int pin = klipperComm.getParamValue(command, "pin").toInt();
  int pullUp = klipperComm.getParamValue(command, "pullup").toInt();
  int nid = klipperComm.getParamValue(command, "nid").toInt();

  PinConfig config;
  config.type = PIN_TYPE_DOUT;
  config.pullUp = pullUp;

  pinConfigurations[pin](pin, config);
  pinMode(pin, pullUp ? INPUT_PULLUP : INPUT);


  // Send response
  klipperComm.sendResponse("ok nid=" + String(nid));
}

void setPinHandler(const String& command) {
  int pin = klipperComm.getParamValue(command, "pin").toInt();
  int value = klipperComm.getParamValue(command, "value").toInt();

  if (pinConfigurations.indexOf(pin) == pinConfigurations.getSize()) {
    klipperComm.sendResponse("error: pin not configured");
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
}

void setup() {
    klipperComm.begin();

    // Register commands
    klipperComm.registerCommand("config_analog_in", configAnalogInHandler);
    klipperComm.registerCommand("config_pwm_out", configPwmOutHandler);
    klipperComm.registerCommand("set_pin", setPinHandler);
    klipperComm.registerCommand("config_digital_out", configDigitalOutHandler);
    klipperComm.registerCommand("config_digital_in", configDigitalInHandler);
}

void loop() {
    if (klipperComm.available()) {
        String input = klipperComm.readCommand();
        klipperComm.handleCommand(input);
    }
}
