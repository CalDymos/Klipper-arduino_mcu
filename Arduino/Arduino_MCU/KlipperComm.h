#ifndef KLIPPER_COMM_H
#define KLIPPER_COMM_H

// MCU definitions (Konstants)
// --------------------------------------------------
extern const char* MCU_NAME;          // mcu name
extern const char* MCU_FIRMWARE_VERSION;  // Version
extern const char* MCU_ADC_MAX;           // ADC resolution (default 10Bit)
extern const char* MCU_ADC_SAMPLE_COUNT;     // Totals of the measured values that are sent in summary form (for analog inputs)
extern const char* MCU_CHIP;    // MCU chip
// --------------------------------------------------

#define MAX_BUFFER_LEN 128

#ifndef MAX_CMDS
#define MAX_CMDS  6              // Max number of commands processed by the MCU (handlers)
#elif MAX_CMDS < 6
#undef MAX_CMDS
#define MAX_CMDS 6
#endif

// Default Commands and Parameter Mapping
// !! If these definitions are changed, arduino_mcu.py must also be adjusted. !!

// Input Commands
#define CMD_CONFIG_ANALOG_IN   "\x80"
#define CMD_CONFIG_PWM_OUT     "\x81"
#define CMD_CONFIG_DIGITAL_OUT "\x82"
#define CMD_CONFIG_BUTTONS     "\x83"
#define CMD_SET_PIN            "\x84"
#define CMD_IDENTIFY           "\x85"

// Responses
#define RE_ACK                 "\xA0"
#define RE_WATCHDOG            "\xA1"
#define RE_ANALOG_IN_STATE     "\xA2"
#define RE_BUTTONS_STATE       "\xA3"
#define RE_COUNTER_STATE       "\xA4"

// Parameters
#define PARAM_MCU_WATCHDOG     "\xB0"
#define PARAM_OID              "\xB1"
#define PARAM_PIN              "\xB2"
#define PARAM_PULLUP           "\xB3"
#define PARAM_INVERT           "\xB4"
#define PARAM_VALUE            "\xB5"
#define PARAM_VALUE_2          "\xB6"
#define PARAM_VALUE_3          "\xB7"
#define PARAM_VALUE_4          "\xB8"
#define PARAM_MCU              "\xB9"
#define PARAM_FREQ             "\xBA"
#define PARAM_CHIP             "\xBB"
#define PARAM_VERSION          "\xBC"
#define PARAM_ADC_MAX          "\xBD"
#define PARAM_ADC_SAMPLE_COUNT "\xBE"
#define PARAM_NID              "\xBF"
#define PARAM_REPORT_TIME      "\xC0"
#define PARAM_CYCLE_TIME       "\xC1"
#define PARAM_START_VALUE      "\xC2"
// --------------------------------------------------

#include <Arduino.h>
#include "map.h"

#ifdef USE_WIFI
#include <WiFi.h>
#include <WiFiClient.h>
#endif

#ifdef USE_ETHERNET
#include <Ethernet.h>
#include <EthernetClient.h>
#endif


class KlipperComm {
private:
    Map<const char*, void (*)(const char*)> commandMap;  // Map for command callbacks

#ifdef USE_WIFI
    WiFiClient client;                                // WiFi client for network communicatio
    IPAddress serverIP;                               // IP address of the Klipper server
    uint16_t serverPort;                              // Port number for Arduino_mcu on the klipper server
    IPAddress local_IP;                               // static IP of MCU
    IPAddress gateway;                                // 
    IPAddress subnet;                                 //
    char ssid[32];
    char password[32];

#endif

#ifdef USE_ETHERNET
    EthernetClient client;                            // Ethernet client for LAN communication
    IPAddress serverIP;                               // IP address of the Klipper server
    uint16_t serverPort;                              // Port number for Arduino_mcu on the klipper server
    IPAddress local_IP;                               // static IP of MCU
    byte mac[6];                                      //
    IPAddress gateway;                                //
    IPAddress subnet;                                 //

#endif

#if defined(USE_WIFI) or defined(USE_ETHERNET)
    bool connectToServer() {
        if (client.connect(serverIP, serverPort)) {
            return true;
        } else {
            return false;
        }
    }
#endif

public:
    char msgBuffer[MAX_BUFFER_LEN];            // buffer for in/out messages
    KlipperComm() : commandMap(MAX_CMDS) {
      msgBuffer[0] = '\0';
    }

#if !defined(USE_ETHERNET) && !defined(USE_WIFI)

    void begin(unsigned long baudRate = 115200) {
        Serial.begin(baudRate);
    }

    bool available() {
        return Serial.available();
    }

    char* readMsg(uint8_t& length) {
        static uint8_t bufferIndex = 0;
        while (Serial.available()) {
            char c = Serial.read();
            if (c == '\n') {
                msgBuffer[bufferIndex] = '\0';
                length = bufferIndex;
                return msgBuffer;
            } else if (bufferIndex < sizeof(msgBuffer) - 1) {
                msgBuffer[bufferIndex++] = c;
            }
        }
        length = 0;
        return nullptr;
    }

    void sendResponse() {
      sendResponse(msgBuffer);
    }

    void sendResponse(const char* response) {
        uint8_t checksum = calculateChecksum(response);
        size_t len = strlen(response);

        // Check whether response is already msgBuffer and space for checksum "*123\n" = 6 Byte
        if (response == msgBuffer && len + 6 < sizeof(msgBuffer)) {
            snprintf(msgBuffer + len, sizeof(msgBuffer) - len, "*%d\n", checksum);
        } else if (len + 6 < sizeof(msgBuffer)) { // If 'response' is a different string, format normally
            snprintf(msgBuffer, sizeof(msgBuffer), "%s*%d\n", response, checksum);
        } else {
            sendResponse("error: message too long");
            return;          
        }

        Serial.print(msgBuffer);
    }

#endif

#ifdef USE_WIFI

    void begin(const String& serverIP, uint16_t serverPort, const char* ssid, const char* password, const String& local_IP, const String& gateway, const String& subnet) {
        if (!this->serverIP.fromString(serverIP))
          Serial.println(F("invalid klipper server IP adress"));

        this->serverPort = serverPort;

        if (!this->local_IP.fromString(local_IP))
          Serial.println(F("invalid MCU IP adress"));
        
        if (!this->gateway.fromString(gateway))
          Serial.println(F("invalid gateway IP adress"));
        
        if (!this->subnet.fromString(subnet))
          Serial.println(F("invalid subnet mask IP adress"));

        this->ssid = ssid;
        this->password = password;

        if (!WiFi.config(this->local_IP, this->gateway, this->subnet)) {
          Serial.println(F("Error when configuring the static IP address"));
        }
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.begin(this->ssid, this->password);
        }

        if (WiFi.waitForConnectResult() == WL_CONNECTED) {
            connectToServer();
        }
    }

    bool available() {
        return client.available();
    }

    char* readMsg(uint8_t& length) {
        static uint8_t bufferIndex = 0;
        while (client.available()) {
            char c = client.read();
            if (c == '\n') {
                msgBuffer[bufferIndex] = '\0';
                length = bufferIndex;
                return msgBuffer;
            } else if (bufferIndex < sizeof(msgBuffer) - 1) {
                msgBuffer[bufferIndex++] = c;
            }
        }
        length = 0;
        return nullptr;
    }

    void sendResponse() {
      sendResponse(msgBuffer);
    }

    void sendResponse(const char* response) {
        uint8_t checksum = calculateChecksum(response);
        size_t len = strlen(response);

        // Check whether response is already msgBuffer and space for checksum "*123\n" = 6 Byte
        if (response == msgBuffer && len + 6 < sizeof(msgBuffer)) {
            snprintf(msgBuffer + len, sizeof(msgBuffer) - len, "*%d\n", checksum);
        } else if (len + 6 < sizeof(msgBuffer)) { // If 'response' is a different string, format normally
            snprintf(msgBuffer, sizeof(msgBuffer), "%s*%d\n", response, checksum);
        } else {
            sendResponse("error: message too long");
            return;          
        }

        client.print(msgBuffer);
    }
#endif

#ifdef USE_ETHERNET

    void begin(const String& serverIP, uint16_t serverPort, byte* mac, const String& local_IP, const String& gateway, const String& subnet) {
        if (!this->serverIP.fromString(serverIP))
          Serial.println(F("invalid klipper server IP adress"));

        this->serverPort = serverPort;

        if (!this->local_IP.fromString(local_IP))
          Serial.println(F("invalid MCU IP adress"));
        
        if (!this->gateway.fromString(gateway))
          Serial.println(F("invalid gateway IP adress"));
        
        if (!this->subnet.fromString(subnet))
          Serial.println(F("invalid subnet mask IP adress"));
        
        for (int i = 0; i < 6; i++) this->mac[i] = mac[i];

        Ethernet.begin(this->mac, this->local_IP, this->gateway, this->gateway, this->subnet);
        connectToServer();
    }

    bool available() {
        return client.available();
    }

    char* readMsg(uint8_t& length) {
        static uint8_t bufferIndex = 0;
        while (client.available()) {
            char c = client.read();
            if (c == '\n') {
                msgBuffer[bufferIndex] = '\0';
                length = bufferIndex;
                bufferIndex = 0;
                return msgBuffer;
            } else if (bufferIndex < sizeof(msgBuffer) - 1) {
                msgBuffer[bufferIndex++] = c;
            } else {
                bufferIndex = 0;
            }
        }
        length = 0;
        return nullptr;
    }

    void sendResponse() {
      sendResponse(msgBuffer);
    }

    void sendResponse(const char* response) {
        uint8_t checksum = calculateChecksum(response);
        size_t len = strlen(response);

        // Check whether response is already msgBuffer and space for checksum "*123\n" = 6 Byte
        if (response == msgBuffer && len + 6 < sizeof(msgBuffer)) {
            snprintf(msgBuffer + len, sizeof(msgBuffer) - len, "*%d\n", checksum);
        } else if (len + 6 < sizeof(msgBuffer)) { // If 'response' is a different string, format normally
            snprintf(msgBuffer, sizeof(msgBuffer), "%s*%d\n", response, checksum);
        } else {
            sendResponse("error: message too long");
            return;          
        }

        client.print(msgBuffer);
    }
#endif

    uint8_t calculateChecksum(const char* command) {
        int checksum = 0;
        while (*command) {
            checksum += (unsigned char)*command++;  // Summiere die Byte-Werte der Zeichen
        }
        return (uint8_t)(checksum % 256);  // Limit the result to 8 bits
    }


    void registerCommand(const char* command, void (*func)(const char*)) {
        uint8_t index = commandMap.indexOf(command);
        if (index < commandMap.getSize()) {
            commandMap[index](command, func);
        } else {
            for (uint8_t i = 0; i < commandMap.getSize(); i++) {
                if (commandMap[i].getHash()[0] == '\0') {  // Empty slot
                    commandMap[i](command, func);
                    return;
                }
            }
            sendResponse("error: Command list full");
        }
    }

    char* getParamValue(const char* msg, const char* paramKey) {
        char* start = strstr(msg, paramKey);
        if (!start) return 0;  // Parameter not found
        start += 1; // Move past paramKey
        char* end = strchr(start, ';');
        if (!end) end = start + strlen(start);
        static char value[32];
        strncpy(value, start, end - start);
        value[end - start] = '\0';
        return value;
    }


    void handleCommand() {
        char* splitPtr = strchr(msgBuffer, '*');
        if (!splitPtr) {
            sendResponse("error: no checksum");
            return;
        }

        int splitIndex = splitPtr - msgBuffer;
        char* checksumStr = &msgBuffer[splitIndex + 1];

        char* endPtr;
        int checksum = strtol(checksumStr, &endPtr, 10);
        if (endPtr == checksumStr) { 
            sendResponse("error: invalid checksum");
            return;
        }

        msgBuffer[splitIndex] = '\0'; // Separate main message in the message buffer

        if (checksum != calculateChecksum(msgBuffer)) {
            sendResponse("error: checksum mismatch");
            return;
        }
      
        if ((uint8_t)msgBuffer[0] == (uint8_t)CMD_IDENTIFY[0]) {
            snprintf(msgBuffer, sizeof(msgBuffer), "%c%s %c%lu;%c%s;%c%s;%c%d;%c%d", 
                     PARAM_MCU, MCU_NAME, PARAM_FREQ, F_CPU, PARAM_CHIP, MCU_CHIP, 
                     PARAM_VERSION, MCU_FIRMWARE_VERSION, PARAM_ADC_MAX, MCU_ADC_MAX, 
                     PARAM_ADC_SAMPLE_COUNT, MCU_ADC_SAMPLE_COUNT);
            sendResponse();
        } else {
            splitPtr = strchr(msgBuffer, ' ');
            if (splitPtr) {
                *splitPtr = '\0'; // Separate command from Message
                char* command = msgBuffer;
                char* params = splitPtr + 1;

                void (*func)(const char*) = commandMap.valueOf(command, nullptr);
                if (func) {
                    func(params);
                    return;
                }
            }
        }

        sendResponse("error: unknown command");
    }
};

#endif
