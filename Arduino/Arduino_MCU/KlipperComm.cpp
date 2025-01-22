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
  Serial.println(response);
}

bool KlipperComm::calculateChecksum(const String& command) {
  int calculatedChecksum = 0;
  for (int i = 0; i < command.length(); i++) {
    calculatedChecksum += command[i];
  }
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
    sendResponse("ERROR: Command list full");
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
    sendResponse("ERROR: NO CHECKSUM");
    return;
  }

  String command = input.substring(0, splitIndex);
  int checksum = input.substring(splitIndex + 1).toInt();

  if (checksum != calculateChecksum(command)) {
    sendResponse("ERROR: CHECKSUM MISMATCH");
    return;
  }

  uint8_t index = commandMap.indexOf(command);
  if (index < commandMap.getSize()) {
    void (*func)(const String&) = commandMap[index].getValue();
    if (func) {
      func(command);
      return;
    }
  }

  sendResponse("ERROR: UNKNOWN COMMAND");
}
