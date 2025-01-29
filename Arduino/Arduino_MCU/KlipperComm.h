#ifndef KLIPPER_COMM_H
#define KLIPPER_COMM_H

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
    String mcuName;                                   // Name of MCU
    String mcuFirmwareVersion;                        // FirmwareVersion of MCU
    String mcuChip;                                   // Chip
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

    bool connectToServer() {
        if (client.connect(serverIP, serverPort)) {
            return true;
        } else {
            return false;
        }
    }
#endif

#ifdef USE_ETHERNET
    EthernetClient client;                            // Ethernet client for LAN communication
    IPAddress serverIP;                               // IP address of the Klipper server
    uint16_t serverPort;                              // Port number for Arduino_mcu on the klipper server
    IPAddress local_IP;                               // static IP of MCU
    byte mac[6];                                      //
    IPAddress gateway;                                //
    IPAddress subnet;                                 //

    bool connectToServer() {
        if (client.connect(serverIP, serverPort)) {
            return true;
        } else {
            return false;
        }
    }
#endif

public:
    KlipperComm(const String& mcuName, const String& mcuFirmwareVersion, uint8_t maxCommands)
        : mcuName(mcuName), mcuFirmwareVersion(mcuFirmwareVersion), commandMap(maxCommands) {
          mcuChip = "Unknown"; // TODO: get MCU chip with method
        }

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
          Serial.println("invalid klipper server IP adress");

        this->serverPort = serverPort;

        if (!this->local_IP.fromString(local_IP))
          Serial.println("invalid MCU IP adress");
        
        if (!this->gateway.fromString(gateway))
          Serial.println("invalid gateway IP adress");
        
        if (!this->subnet.fromString(subnet))
          Serial.println("invalid subnet mask IP adress");

        this->ssid = ssid;
        this->password = password;

        if (!WiFi.config(this->local_IP, this->gateway, this->subnet)) {
          Serial.println("Error when configuring the static IP address");
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
          Serial.println("invalid klipper server IP adress");

        this->serverPort = serverPort;

        if (!this->local_IP.fromString(local_IP))
          Serial.println("invalid MCU IP adress");
        
        if (!this->gateway.fromString(gateway))
          Serial.println("invalid gateway IP adress");
        
        if (!this->subnet.fromString(subnet))
          Serial.println("invalid subnet mask IP adress");
        
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
            sendResponse("error: Command list full");
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
            sendResponse("mcu=" + mcuName + " freq=" + F_CPU + " chip=" + mcuChip + " version=" + mcuFirmwareVersion);
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
