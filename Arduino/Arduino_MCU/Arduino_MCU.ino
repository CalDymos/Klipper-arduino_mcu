#include <avr/sleep.h>

#define MCU_NAME "AIRFILTER"

void setup() {
  Serial.begin(115200); // Start serielle Kommunikation
}

bool validateChecksum(String command, int receivedChecksum) {
  int calculatedChecksum = 0;
  for (int i = 0; i < command.length(); i++) {
    calculatedChecksum += command[i];
  }
  return (calculatedChecksum % 256) == receivedChecksum;
}

void enterSleep() {
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Tiefster Schlafmodus
  sleep_enable();
  sleep_cpu(); // Schlafmodus aktivieren
  // Der Arduino bleibt hier, bis ein Reset oder Interrupt ihn aufweckt
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Gesamter Befehl
    int splitIndex = input.indexOf('*'); // Trenne Befehl und Checksumme
    if (splitIndex == -1) {
      Serial.println("ERROR: NO CHECKSUM");
      return;
    }
    String command = input.substring(0, splitIndex);
    int checksum = input.substring(splitIndex + 1).toInt();

    if (!validateChecksum(command, checksum)) {
      Serial.println("ERROR: CHECKSUM MISMATCH");
      return;
    }

    // Befehl validieren und ausführen
    if (command == "IDENTIFY") {
      Serial.println("MCU: " + String(MCU_NAME));
    } else if (command.startsWith("SET_LED ")) {
      int value = command.substring(8).toInt();
      analogWrite(LED_BUILTIN, value);
      Serial.println("LED SET TO " + String(value));
    } else if (command == "SLEEP"){
      enterSleep();
    } else {
      Serial.println("ERROR: UNKNOWN COMMAND");
    }
  }
}


