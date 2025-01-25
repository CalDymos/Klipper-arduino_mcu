// 9245 Bytes
#include "KlipperComm.h"

#define MCU_NAME "airfilter"
#define MAX_COMMANDS 10

KlipperComm klipperComm(MCU_NAME, MAX_COMMANDS);

enum PinType {
    PIN_TYPE_DOUT, // Digital out
    PIN_TYPE_PWM,  // PWM
    PIN_TYPE_AIN,  // Analog in
    PIN_TYPE_DIN   // Digital in
};

struct PinConfig {
  uint8_t type;         // Pin Type 0=Digital OUT 1=PWM 2=Analog IN 3=Digital IN
  uint32_t cycleTime;   // PWM cycle time (only relevant for PWM)
  uint16_t startValue;  // Start value (only relevant for PWM and Digital Out)
  bool pullUp;          // Pull-up resistor (only for digital input pins)
  bool invert;          // Inverted logic (only for digital out pins)
  uint32_t updateTime;  // Update interval in milliseconds
  uint32_t lastUpdate;  // Last time the value was sent
};


// Map for pin configurations 9736 Bytes
Map<int, PinConfig> pinConfigurations(10);

void configurePin(const String& command, int type) {
  int pin = klipperComm.getParamValue(command, "pin").toInt();
  int nid = klipperComm.getParamValue(command, "nid").toInt();
  int updateTime = klipperComm.getParamValue(command, "update_time").toInt();
  if (updateTime <= 0) updateTime = 2000;

  PinConfig config;
  config.type = type;
  config.updateTime = updateTime;
  config.lastUpdate = 0;

  if (type == PIN_TYPE_DOUT) {
    config.invert = klipperComm.getParamValue(command, "invert").toInt();
    pinMode(pin, OUTPUT);
  } else if (type == PIN_TYPE_PWM) {
    config.cycleTime = klipperComm.getParamValue(command, "cycle_time").toInt();
    config.startValue = klipperComm.getParamValue(command, "start_value").toInt();
    pinMode(pin, OUTPUT);
    analogWrite(pin, config.startValue);
  } else if (type == PIN_TYPE_AIN || type == PIN_TYPE_DIN) {
    config.pullUp = klipperComm.getParamValue(command, "pullup").toInt();
    pinMode(pin, config.pullUp ? INPUT_PULLUP : INPUT);
  }

  pinConfigurations[pin](pin, config);
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

  if (config.type == PIN_TYPE_DOUT) {
    digitalWrite(pin, value ? HIGH : LOW);
  } else if (config.type == PIN_TYPE_PWM) {
    analogWrite(pin, value);
  }
}

void sendPeriodicUpdates() {
  unsigned long currentTime = millis();

  // Iterate over the registered pins
  for (uint8_t i = 0; i < pinConfigurations.getSize(); i++) {
    HashType<int, PinConfig> pinEntry = pinConfigurations[i];
    int pin = pinEntry.getHash();
    PinConfig config = pinEntry.getValue();

    // Check whether the update time for the pin has been reached
    if (currentTime - config.lastUpdate >= config.updateTime) {
      config.lastUpdate = currentTime;

      // Send the value based on the pin type
      if (config.type == PIN_TYPE_AIN) {
        int analogValue = analogRead(pin);
        klipperComm.sendResponse("pin=" + String(pin) + " value=" + String(analogValue));
      } else if (config.type == PIN_TYPE_DIN) {
        int digitalValue = digitalRead(pin);
        klipperComm.sendResponse("pin=" + String(pin) + " value=" + String(digitalValue));
      }

      // Update the configuration in the map
      pinConfigurations[pin].setValue(config);
    }
  }
}

void setup() {
  klipperComm.begin();

  // Register commands
  klipperComm.registerCommand("config_analog_in", [](const String& cmd) { configurePin(cmd, PIN_TYPE_AIN); });
  klipperComm.registerCommand("config_pwm_out", [](const String& cmd) { configurePin(cmd, PIN_TYPE_PWM); });
  klipperComm.registerCommand("config_digital_out", [](const String& cmd) { configurePin(cmd, PIN_TYPE_DOUT); });
  klipperComm.registerCommand("config_digital_in", [](const String& cmd) { configurePin(cmd, PIN_TYPE_DIN); });
  klipperComm.registerCommand("set_pin", setPinHandler);
}

void loop() {
  if (klipperComm.available()) {
    String input = klipperComm.readCommand();
    klipperComm.handleCommand(input);
  }

  // Periodic Updates for Inputs
  sendPeriodicUpdates();
}
