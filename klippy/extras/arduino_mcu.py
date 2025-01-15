import serial
import logging

class ArduinoMCU:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.gcode = self.printer.lookup_object('gcode')
        self.port = config.get('port', '/dev/ttyUSB0') # deault port
        self.baudrate = config.getint('baud', 115200) # default baudrate

        # Set up serial connection
        self.serial = serial.Serial(self.port, self.baudrate, timeout=1)

        # Register G-code command
        self.gcode.register_command("SEND_ARDUINO", self.cmd_send_arduino)

    def calculate_checksum(self, command):
        """Calculates a simple checksum for data integrity."""
        return sum(ord(c) for c in command) % 256

    def cmd_send_arduino(self, params):
        """Sends a command to the target MCU."""
        target = params.get('TARGET', '').lower()
        if target != self.name:
            return  # Ignore commands not addressed to this MCU
        cmd = params.get('COMMAND', '')
        if not cmd:
            raise self.gcode.error("COMMAND parameter is required")
        
        # Calculate checksum and format command
        checksum = self.calculate_checksum(cmd)
        full_command = f"{cmd}*{checksum}\n"  # Format: Command*Checksum
                                              # Adds a line break to ensure that the Arduino reads the input correctly
        
        # Send and read reply
        self.serial.write(full_command.encode())
        response = self.serial.readline().decode().strip()

        # Error handling based on the response
        if "ERROR" in response:
            self.gcode.respond_info(f"{self.name} Error: {response}")
            return
        
        self.gcode.respond_info(f"{self.name} responded: {response}")

def load_config_prefix(config):
    return ArduinoMCU(config)
