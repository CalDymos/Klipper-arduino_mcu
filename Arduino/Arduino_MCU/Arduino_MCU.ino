// #define USE_WIFI
// #define USE_ETHERNET
#define MCU_NAME "airfilter"        // mcu name
#define MCU_FIRMWARE_VERSION "1.0"  // Version
#define MCU_ADC_MAX 1023            // ADC resolution (default 10Bit)
#define MCU_ADC_SAMPLE_COUNT 1      // Totals of the measured values that are sent in summary form (for analog inputs)
#define MAX_COMMANDS 6              // Max number of commands processed by the MCU (handlers)
#define MAX_PIN_CONFIGURATIONS 10   // Max number of pin configurations
#include "KlipperComm.h"

// is only required for communication via LAN or WLAN
//***************************************************
#if defined(USE_ETHERNET) || defined(USE_WIFI)
#define SERVER_IP "192.6.1.124"      // <- Klipper server IP adress
#define SERVER_PORT 45800            // <- port for Arduino_mcu
#define LOCAL_IP "192.6.1.123"       // ip adress for MCU in LAN
#define GATEWAY "192.6.1.1"          // Gateway for LAN
#define SUBNET_MASK "225.255.255.0"  // subnet mask for LAN
#endif
#ifdef USE_ETHERNET
#define MAC_ADDR \
  { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED }  // mac adress for MCU
#endif
#ifdef USE_WIFI
#define SSID "<ssid>"     // ssid for WLAN
#define PWD "<password>"  // password for WLAN
#endif
//***************************************************

KlipperComm klipperComm;

enum PinType {
  PIN_TYPE_DOUT,  // Digital out
  PIN_TYPE_PWM,   // PWM
  PIN_TYPE_AIN,   // Analog in
  PIN_TYPE_DIN    // Digital in
};

// pin configurations
#define DEFAULT_REPORT_TIME 2000 // ms
struct PinConfig {
  uint8_t type;         // Pin Type 0=Digital OUT 1=PWM 2=Analog IN 3=Digital IN
  uint32_t cycleTime;   // PWM cycle time (only relevant for PWM)
  uint16_t startValue;  // Start value (only relevant for PWM and Digital Out)
  bool pullUp;          // Pull-up resistor (only for digital input pins)
  bool invert;          // Inverted logic (only for digital out pins)
  uint32_t reportTime;  // report interval in milliseconds
  uint32_t lastReportTime;  // Last time the value was sent
};

// watchdog definitions
#define WATCHDOG_INTERVAL 5000 // ms
unsigned long lastWatchdogTime = 0;  // Speichert die letzte Sendezeit

// Map for pin configurations
Map<int, PinConfig> pinConfigurations(MAX_PIN_CONFIGURATIONS);
uint8_t pinCount = 0;

void configurePin(const String& command, int type) {
  if (pinCount <= pinConfigurations.getSize()) {

    int pin = klipperComm.getParamValue(command, "pin").toInt();
    int nid = klipperComm.getParamValue(command, "nid").toInt();
    int reportTime = klipperComm.getParamValue(command, "report_time").toInt();
    if (reportTime <= 0) reportTime = DEFAULT_REPORT_TIME;

    PinConfig config;
    config.type = type;
    config.reportTime = reportTime;
    config.lastReportTime = 0;

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
    pinCount++;
    klipperComm.sendResponse("ok nid=" + String(nid));

  } else {
    klipperComm.sendResponse(F("error: Pin list full"));
  }
}

void setPinHandler(const String& command) {
  int pin = klipperComm.getParamValue(command, "pin").toInt();
  int value = klipperComm.getParamValue(command, "value").toInt();

  if (pinConfigurations.indexOf(pin) == pinConfigurations.getSize()) {
    klipperComm.sendResponse(F("error: pin not configured"));
    return;
  }

  PinConfig config = pinConfigurations[pin].getValue();

  if (config.type == PIN_TYPE_DOUT) {
    digitalWrite(pin, value ? HIGH : LOW);
  } else if (config.type == PIN_TYPE_PWM) {
    analogWrite(pin, value);
  }
}

void sendWatchdogMsg() {
  // Send watchdog message every 5 seconds
  unsigned long currentTime = millis();
  if (currentTime - lastWatchdogTime >= WATCHDOG_INTERVAL) {
    klipperComm.sendResponse("mcu_watchdog=1");
    lastWatchdogTime = currentTime;
  }
}

void sendPeriodicReports() {
  unsigned long currentTime = millis();

  // Iterate over the registered pins
  for (uint8_t i = 0; i < pinConfigurations.getSize(); i++) {
    HashType<int, PinConfig> pinEntry = pinConfigurations[i];
    int pin = pinEntry.getHash();
    PinConfig config = pinEntry.getValue();

    // Check whether the report time for the pin has been reached
    if (currentTime - config.lastReportTime >= config.reportTime) {
      config.lastReportTime = currentTime;

      // Send the value based on the pin type
      if (config.type == PIN_TYPE_AIN) {
        int analogValue = analogRead(pin);
        klipperComm.sendResponse("analog_in_state pin=" + String(pin) + " value=" + String(analogValue));
      } else if (config.type == PIN_TYPE_DIN) {
        int digitalValue = digitalRead(pin);
        klipperComm.sendResponse("digital_in_state pin=" + String(pin) + " value=" + String(digitalValue));
      }

      // Update the configuration in the map
      pinConfigurations[pin].setValue(config);
    }
  }
}

void setup() {
#if defined(USE_ETHERNET)
  byte mac[] = MAC_ADDR;
  klipperComm.begin(SERVER_IP, SERVER_PORT, mac, LOCAL_IP, GATEWAY, SUBNET_MASK);
#elif defined(USE_WIFI)
  klipperComm.begin(SERVER_IP, SERVER_PORT, SSID, PWD, LOCAL_IP, GATEWAY, SUBNET_MASK);
#else
  klipperComm.begin();
#endif


  // Register commands
  klipperComm.registerCommand(F("config_analog_in"), [](const String& cmd) {
    configurePin(cmd, PIN_TYPE_AIN);
  });
  klipperComm.registerCommand(F("config_pwm_out"), [](const String& cmd) {
    configurePin(cmd, PIN_TYPE_PWM);
  });
  klipperComm.registerCommand(F("config_digital_out"), [](const String& cmd) {
    configurePin(cmd, PIN_TYPE_DOUT);
  });
  klipperComm.registerCommand(F("config_digital_in"), [](const String& cmd) {
    configurePin(cmd, PIN_TYPE_DIN);
  });
  klipperComm.registerCommand(F("set_pin"), setPinHandler);
}

void loop() {
  if (klipperComm.available()) {
    String input = klipperComm.readCommand();
    klipperComm.handleCommand(input);
  }

  // Periodic Reports for Inputs
  sendPeriodicReports();

  // watchdog
  sendWatchdogMsg();
}
