#ifndef KLIPPER_COMM_H
#define KLIPPER_COMM_H

#include <Arduino.h>
#include "map.h"

class KlipperComm {

public:
  KlipperComm(const String& mcuName, uint8_t maxCommands);

  void begin(unsigned long baudRate = 115200);
  bool available();
  String readCommand();
  void sendResponse(const String& response);
  int calculateChecksum(const String& command);
  void handleCommand(const String& input);

  void registerCommand(const String& command, void (*func)(const String&));
  String getParamValue(const String& command, const String& paramName);

private:
  String mcuName;
  Map<String, void (*)(const String&)> commandMap;
};

#endif