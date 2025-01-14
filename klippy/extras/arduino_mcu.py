import serial
import logging

class ArduinoMCU:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.gcode = self.printer.lookup_object('gcode')
        self.port = config.get('port', '/dev/ttyUSB0') # Standardport
        self.baudrate = config.getint('baudrate', 115200) # Standardbaudrate

        # Serielle Verbindung einrichten
        self.serial = serial.Serial(self.port, self.baudrate, timeout=1)

        # G-Code-Befehl registrieren
        self.gcode.register_command("SEND_ARDUINO", self.cmd_send_arduino)

    def calculate_checksum(self, command):
        """Berechnet eine einfache Checksumme für die Datenintegrität."""
        return sum(ord(c) for c in command) % 256

    def cmd_send_arduino(self, params):
        """Sendet einen Befehl an die Ziel-MCU."""
        target = params.get('TARGET', '').lower()
        if target != self.name:
            return  # Ignoriere Befehle, die nicht an diese MCU gerichtet sind
        cmd = params.get('COMMAND', '')
        if not cmd:
            raise self.gcode.error("COMMAND parameter is required")
        
        # Checksumme berechnen und Befehl formatieren
        checksum = self.calculate_checksum(cmd)
        full_command = f"{cmd}*{checksum}\n"  # Format: Befehl*Checksumme
                                              # Fügt einen Zeilenumbruch hinzu, um sicherzustellen, dass der Arduino die Eingabe korrekt liest
        
        # Senden und Antwort lesen
        self.serial.write(full_command.encode())
        response = self.serial.readline().decode().strip()

        # Fehlerbehandlung basierend auf der Antwort
        if "ERROR" in response:
            self.gcode.respond_info(f"{self.name} Error: {response}")
            return
        
        self.gcode.respond_info(f"{self.name} responded: {response}")

def load_config_prefix(config):
    return ArduinoMCU(config)