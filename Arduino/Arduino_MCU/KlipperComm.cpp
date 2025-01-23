#include "KlipperComm.h"

KlipperComm::KlipperComm(const String& mcuName, uint8_t maxCommands)
  : mcuName(mcuName), commandMap(maxCommands) {}

void KlipperComm::begin(unsigned long baudRate) {
  Serial.begin(baudRate);
}

bool KlipperComm::available() {
  return Serial.available() > 0;
}

String KlipperComm::readCommand() {
  return Serial.readStringUntil('\n');
}

void KlipperComm::sendResponse(const String& response) {
  int checksum = calculateChecksum(response);
  Serial.print(response + "*" + String(checksum) + '\n');
}

int KlipperComm::calculateChecksum(const String& command) {
  int calculatedChecksum = 0;
  for (int i = 0; i < command.length(); i++) {
    calculatedChecksum += command[i];
  }
  // Serial.println(command);
  // Serial.println(calculatedChecksum);
  return calculatedChecksum % 256;
}

void KlipperComm::registerCommand(const String& command, void (*func)(const String&)) {
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

String KlipperComm::getParamValue(const String& command, const String& paramName) {
  int start = command.indexOf(paramName + "=");
  if (start == -1) return "";  // Parameter not found
  int end = command.indexOf(' ', start);
  if (end == -1) end = command.length();
  return command.substring(start + paramName.length() + 1, end);
}

void KlipperComm::handleCommand(const String& input) {
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
    // Send response
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
