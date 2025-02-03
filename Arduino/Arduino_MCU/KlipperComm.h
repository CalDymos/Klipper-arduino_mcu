#ifndef KLIPPER_COMM_H
#define KLIPPER_COMM_H

// default MCU Konstants
// --------------------------------------------------
#ifndef MCU_NAME 
#define MCU_NAME "Unknown"          // mcu name
#endif

#ifndef MCU_FIRMWARE_VERSION
#define MCU_FIRMWARE_VERSION "1.0"  // Version
#endif

#ifndef MCU_ADC_MAX
#define MCU_ADC_MAX 1023           // ADC resolution (default 10Bit)
#endif

#ifndef MCU_ADC_SAMPLE_COUNT       
#define MCU_ADC_SAMPLE_COUNT 1     // Totals of the measured values that are sent in summary form (for analog inputs)
#endif

#define MCU_CHIP "Unknown"    // MCU chip (not implementet yet)
// --------------------------------------------------

#ifndef MAX_COMMANDS
#define MAX_COMMANDS  10              // Max number of commands processed by the MCU (handlers)
#endif

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
    Map<String, void (*)(const String&)> commandMap;  // Map for command callbacks

#ifdef USE_WIFI
    WiFiClient client;                                // WiFi client for network communicatio
    IPAddress serverIP;                               // IP address of the Klipper server
    uint16_t serverPort;                              // Port number for Arduino_mcu on the klipper server
    IPAddress local_IP;                               // static IP of MCU
    IPAddress gateway;                                // 
    IPAddress subnet;                                 //
    String ssid;
    String password;

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
    KlipperComm() : commandMap(MAX_COMMANDS) {}

#if !defined(USE_ETHERNET) && !defined(USE_WIFI)

    void begin(unsigned long baudRate = 115200) {
        Serial.begin(baudRate);
    }

    bool available() {
        return Serial.available();
    }

    String readCommand() {
        return Serial.readStringUntil('\n');
    }

    void sendResponse(const String& response) {
        int checksum = calculateChecksum(response);
        Serial.print(response + "*" + String(checksum) + "\n");
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

    String readCommand() {
        return client.readStringUntil('\n');
    }

    void sendResponse(const String& response) {
        int checksum = calculateChecksum(response);
        client.print(response + "*" + String(checksum) + "\n");
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

    String readCommand() {
        return client.readStringUntil('\n');
    }

    void sendResponse(const String& response) {
        int checksum = calculateChecksum(response);
        client.print(response + "*" + String(checksum) + "\n");
    }
#endif

    int calculateChecksum(const String& command) {
        int calculatedChecksum = 0;
        for (int i = 0; i < command.length(); i++) {
            calculatedChecksum += command[i];
        }
        return calculatedChecksum % 256;
    }

    void registerCommand(const String& command, void (*func)(const String&)) {
        uint8_t index = commandMap.indexOf(command);
        if (index < commandMap.getSize()) {
            commandMap[index](command, func);
        } else {
            for (uint8_t i = 0; i < commandMap.getSize(); i++) {
                if (commandMap[i].getHash() == "") {  // Empty slot
                    commandMap[i](command, func);
                    return;
                }
            }
            sendResponse(F("error: Command list full"));
        }
    }

    String getParamValue(const String& command, const String& paramName) {
        int start = command.indexOf(paramName + "=");
        if (start == -1) return "";  // Parameter not found
        int end = command.indexOf(' ', start);
        if (end == -1) end = command.length();
        return command.substring(start + paramName.length() + 1, end);
    }

    void handleCommand(const String& input) {
        int splitIndex = input.indexOf('*');
        if (splitIndex == -1) {
            sendResponse("error: no checksum");
            return;
        }

        String command = input.substring(0, splitIndex);
        int checksum = input.substring(splitIndex + 1).toInt();

        if (checksum != calculateChecksum(command)) {
            sendResponse("error: checksum mismatch");
            return;
        }

        if (command == "identify") {
            String response = "mcu=" + String(MCU_NAME) + 
                              " freq=" + String(F_CPU) + 
                              " chip=" + String(MCU_CHIP) + 
                              " version=" + String(MCU_FIRMWARE_VERSION) + 
                              " adc_max=" + String(MCU_ADC_MAX) + 
                              " adc_sample_count=" + String(MCU_ADC_SAMPLE_COUNT);

            sendResponse(response);
            return;
        } else {
            splitIndex = input.indexOf(' ');
            if (splitIndex != -1) {
                String funcName = input.substring(0, splitIndex);
                uint8_t index = commandMap.indexOf(funcName);
                if (index < commandMap.getSize()) {
                    void (*func)(const String&) = commandMap[index].getValue();
                    if (func) {
                        func(command);
                        return;
                    }
                }
            }
        }

        sendResponse("error: unknown command");
    }
};

#endif
