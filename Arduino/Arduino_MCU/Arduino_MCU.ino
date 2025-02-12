// #define USE_WIFI
// #define USE_ETHERNET

const char* MCU_NAME = "airfilter";        // mcu name
const char* MCU_FIRMWARE_VERSION = "1.0";  // Version
const char* MCU_ADC_MAX = "1023";          // ADC resolution (default 10Bit)
const char* MCU_ADC_SAMPLE_COUNT = "1";    // Totals of the measured values that are sent in summary form (for analog inputs)
const char* MCU_CHIP = "ATmega328P";       // MCU chip

#define MAX_CMDS 8                 // Max number of commands processed by the MCU (handlers)
#define MAX_PIN_CONFIGURATIONS 10  // Max number of pin configurations

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

enum PinReturnType {
  RETURN_TYPE_TEMP,  // temperature (°C)
  RETURN_TYPE_HUM,   // humidity (%)
  RETURN_TYPE_PRES,  // pressure (hPa)
  RETURN_TYPE_GAS,   // gas
  RETURN_TYPE_FREQ   // frequency (Hz)
};

// pin configurations
#define DEFAULT_REPORT_TIME 2000  // ms
struct PinConfig {
  uint8_t type;             // Pin Type 0=Digital OUT 1=PWM 2=Analog IN 3=Digital IN
  uint32_t cycleTime;       // PWM cycle time (only relevant for PWM)
  uint16_t startValue;      // Start value (only relevant for PWM and Digital Out)
  bool pullUp;              // Pull-up resistor (only for digital input pins)
  bool invert;              // Inverted logic (only for digital out pins)
  uint32_t reportTime;      // report interval in milliseconds
  uint32_t lastReportTime;  // Last time the value was sent
};

// watchdog definitions
#define WATCHDOG_INTERVAL 5000       // ms
unsigned long lastWatchdogTime = 0;  // Speichert die letzte Sendezeit

// Map for pin configurations
Map<int, PinConfig> pinConfigurations(MAX_PIN_CONFIGURATIONS);
uint8_t pinCount = 0;

// Received message size
uint8_t recvMsgSize;

void handleConfigPin(const char* params, int type) {
  if (pinCount <= pinConfigurations.getSize()) {

    int pin = atoi(klipperComm.getParamValue(params, PARAM_PIN));
    int nid = atoi(klipperComm.getParamValue(params, PARAM_NID));
    int reportTime = atoi(klipperComm.getParamValue(params, PARAM_REPORT_TIME));
    if (reportTime <= 0) reportTime = DEFAULT_REPORT_TIME;

    PinConfig config;
    config.type = type;
    config.reportTime = reportTime;
    config.lastReportTime = 0;

    if (type == PIN_TYPE_DOUT) {
      config.invert = atoi(klipperComm.getParamValue(params, PARAM_INVERT));
      pinMode(pin, OUTPUT);
    } else if (type == PIN_TYPE_PWM) {
      config.cycleTime = atoi(klipperComm.getParamValue(params, PARAM_CYCLE_TIME));
      config.startValue = atoi(klipperComm.getParamValue(params, PARAM_START_VALUE));
      pinMode(pin, OUTPUT);
      analogWrite(pin, config.startValue);
    } else if (type == PIN_TYPE_AIN || type == PIN_TYPE_DIN) {
      config.pullUp = atoi(klipperComm.getParamValue(params, PARAM_PULLUP));
      pinMode(pin, config.pullUp ? INPUT_PULLUP : INPUT);
    }

    pinConfigurations[pin](pin, config);
    pinCount++;
    snprintf(klipperComm.msgBuffer, MAX_BUFFER_LEN, "%c %c%u", RESP_ACK, PARAM_NID, nid);

  } else {
    snprintf(klipperComm.msgBuffer, MAX_BUFFER_LEN, "%c %c%s", RESP_ERROR_MSG, PARAM_VALUE, "Pin list full");
  }
  klipperComm.sendResponse();
}

void handleSetPin(const char* params) {
  int pin = atoi(klipperComm.getParamValue(params, PARAM_PIN));
  int value = atoi(klipperComm.getParamValue(params, PARAM_VALUE));

  if (pinConfigurations.indexOf(pin) == pinConfigurations.getSize()) {
    snprintf(klipperComm.msgBuffer, MAX_BUFFER_LEN, "%c %c%s", RESP_ERROR_MSG, PARAM_VALUE, "Pin list full");
    klipperComm.sendResponse();
    return;
  }

  PinConfig config = pinConfigurations[pin].getValue();

  if (config.type == PIN_TYPE_DOUT) {
    digitalWrite(pin, value ? HIGH : LOW);
  } else if (config.type == PIN_TYPE_PWM) {
    analogWrite(pin, value);
  }
}

void handleRestart(const char* params) {
  klipperComm.isConnected = false;
  // handle Restart of MCU
}

void handleUserdefined(const char* params) {
  // User-defined commands can be processed here.
  // use option userParams in getParamValue => klipperComm.getParamValue(params, "Pin", true);
}

/*
 * Send watchdog message every 5 seconds to klipper
 *
 */
void sendWatchdogMsg() {
  if (!klipperComm.isConnected) return;  // If no connection → cancel
  
  unsigned long currentTime = millis();
  if (currentTime - lastWatchdogTime >= WATCHDOG_INTERVAL) {
    snprintf(klipperComm.msgBuffer, MAX_BUFFER_LEN, "%c %c1", RESP_WATCHDOG, PARAM_MCU_WATCHDOG);
    klipperComm.sendResponse();
    lastWatchdogTime = currentTime;
  }
}

void sendPeriodicReports() {
  if (!klipperComm.isConnected) return;  // If no connection → cancel

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
        snprintf(klipperComm.msgBuffer, MAX_BUFFER_LEN, "%c %c%u;%c%u", RESP_ANALOG_IN_STATE, PARAM_PIN, pin, PARAM_VALUE, analogValue);
        klipperComm.sendResponse();
      } else if (config.type == PIN_TYPE_DIN) {
        int digitalValue = digitalRead(pin);
        snprintf(klipperComm.msgBuffer, MAX_BUFFER_LEN, "%c %c%u;%c%u", RESP_BUTTONS_STATE, PARAM_PIN, pin, PARAM_VALUE, digitalValue);
        klipperComm.sendResponse();
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
  klipperComm.registerCommand(CMD_CONFIG_ANALOG_IN, [](const char* params) {
    handleConfigPin(params, PIN_TYPE_AIN);
  });
  klipperComm.registerCommand(CMD_CONFIG_PWM_OUT, [](const char* params) {
    handleConfigPin(params, PIN_TYPE_PWM);
  });
  klipperComm.registerCommand(CMD_CONFIG_DIGITAL_OUT, [](const char* params) {
    handleConfigPin(params, PIN_TYPE_DOUT);
  });
  klipperComm.registerCommand(CMD_CONFIG_BUTTONS, [](const char* params) {
    handleConfigPin(params, PIN_TYPE_DIN);
  });
  klipperComm.registerCommand(CMD_SET_PIN, handleSetPin);
  klipperComm.registerCommand(CMD_RESTART, handleRestart);
  klipperComm.registerCommand(CMD_USERDEFINED, handleUserdefined);
}

void loop() {
  if (klipperComm.available()) {
    klipperComm.readMsg(recvMsgSize);
    klipperComm.handleCommand();
  }

  // Periodic Reports for Inputs
  sendPeriodicReports();

  // watchdog
  sendWatchdogMsg();
}
