#ifndef KLIPPER_COMM_H
#define KLIPPER_COMM_H

#include <Arduino.h>
#include "map.h"

#ifdef USE_WIFI
#include <WiFi.h>
#include <WiFiClient.h>
#endif

#ifdef USE_LAN
#include <Ethernet.h>
#include <EthernetClient.h>
#endif

class KlipperComm {
private:
    const String mcuName;                             // Name der MCU
    Map<String, void (*)(const String&)> commandMap;  // Map for command callbacks

#ifdef USE_WIFI
    WiFiClient client;                                // WiFiClient für die Netzwerkkommunikation
    String serverIP;                                  // IP-Adresse des Servers
    uint16_t serverPort;                              // Portnummer des Servers
    String commandBuffer;                             // Puffer für empfangene Daten

    bool connectToServer() {
        if (client.connect(serverIP.c_str(), serverPort)) {
            return true;
        } else {
            return false;
        }
    }
#endif

#ifdef USE_LAN
    EthernetClient client;                            // EthernetClient für die LAN-Kommunikation
    IPAddress serverIP;                               // IP-Adresse des Servers
    uint16_t serverPort;                              // Portnummer des Servers

    bool connectToServer() {
        if (client.connect(serverIP, serverPort)) {
            return true;
        } else {
            return false;
        }
    }
#endif

public:
#ifdef USE_SERIAL
    KlipperComm(const String& mcuName)
        : mcuName(mcuName), commandBuffer("") {}

    void begin(unsigned long baudRate = 115200) {
        Serial.begin(baudRate);
    }

    bool available() {
        return Serial.available() > 0;
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
    KlipperComm(const String& mcuName, const String& ip, uint16_t port)
        : mcuName(mcuName), serverIP(ip), serverPort(port) {}

    void begin(const char* ssid, const char* password) {
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.begin(ssid, password);
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

#ifdef USE_LAN
    KlipperComm(const String& mcuName, IPAddress ip, uint16_t port)
        : mcuName(mcuName), serverIP(ip), serverPort(port) {}

    void begin(byte* mac) {
        Ethernet.begin(mac);
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
        if (start == -1) return "";  // Parameter nicht gefunden
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
            sendResponse("mcu=" + mcuName);
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
