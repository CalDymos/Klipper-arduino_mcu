import serial
import logging
import time

class ArduinoMCU:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.gcode = self.printer.lookup_object('gcode')
        self.port = config.get('port', '/dev/ttyUSB0') # Default port
        self.baudrate = config.getint('baud', 115200) # Default baudrate
        self.timeout = config.getint('timeout', 1) # Default timout
        self.retries_on_timeout = config.getint('retries_on_timeout', 0)  # Default retries

        # init a Virtual MCU
        ppins = self.printer.lookup_object('pins')
        ppins.register_chip(f"{self.name}_pin", self)
        self._pins = {}
        self._oid_count = 0
        self._config_callbacks = []
        
        # Set up serial connection
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            logging.info(f"Connected to {self.name} on {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            raise config.error(f"Failed to connect to {self.name} on {self.port}: {e}\n please check connection!")

        # Register G-code command
        self.gcode.register_command("SEND_ARDUINO", self.cmd_send_arduino)

    def setup_pin(self, pin_type, pin_params):
        ppins = self.printer.lookup_object('pins')
        name = pin_params['pin']
        if name in self._pins:
            return self._pins[name]
        if pin_type == 'digital_out':
            pin = None #pin = DigitalOutVirtualPin(self, pin_params)  ##TODO:add class
        elif pin_type == 'pwm':
            pin = None #pin = PwmVirtualPin(self, pin_params) ##TODO:add class
        elif pin_type == 'adc':
            pin = None #pin = AdcVirtualPin(self, pin_params) ##TODO:add class
        elif pin_type == 'digital_in':
            pin = None #pin = EndstopVirtualPin(self, pin_params) ##TODO:add class
        else:
            raise ppins.error("unable to create virtual pin of type %s" % (
                pin_type,))
        self._pins[name] = pin
        return pin
    
    def calculate_checksum(self, command):
        """Calculates a simple checksum for data integrity."""
        return sum(ord(c) for c in command) % 256

    def _respond_error(self, msg):
        logging.warning(msg)
        lines = msg.strip().split('\n')
        if len(lines) > 1:
            self.gcode.respond_info("\n".join(lines), log=False)
        self.gcode.respond_raw('!! %s' % (lines[0].strip(),))

    def cmd_send_arduino(self, params):
        """Sends a command to the target MCU."""
        target = params.get('TARGET', '').lower()
        if not target:
            raise self.gcode.error("TARGET parameter is missing")
        if target != self.name:
            return  # Ignore commands not addressed to this MCU
        cmd = params.get('COMMAND', '')
        if not cmd:
            raise self.gcode.error("COMMAND parameter is required")
        
        # Calculate checksum and format command
        checksum = self.calculate_checksum(cmd)
        full_command = f"{cmd}*{checksum}\n"  # Format: Command*Checksum
                                              # Adds a line break to ensure that the Arduino reads the input correctly
        
        # clear input buffer
        self.serial.reset_input_buffer()   

        # Retry logic
        max_attempts = self.retries_on_timeout + 1 # 1 initial attempt + retries

        for attempt in range(max_attempts):
            # Send command
            self.serial.write(full_command.encode())
            logging.debug(f"{self.name}: Sending command (Attempt {attempt + 1}): {full_command.strip()}")

            # Wait for response
            start_time = time.time()
            while time.time() - start_time < self.timeout:
                if self.serial.in_waiting > 0:  # Check if new data is available
                    response = self.serial.readline().decode().strip()
                    if "ERROR" in response:
                        self._respond_error(f"{self.name} Error: {response}")
                    else:
                        self.gcode.respond_info(f"{self.name} responded: {response}")
                    return

            # Log timeout for this attempt
            logging.warning(f"{self.name}: Timeout on attempt {attempt + 1}")
            if attempt == max_attempts - 1:  # Last attempt
                self._respond_error(f"{self.name} Error: No response after {max_attempts} attempts")

def load_config_prefix(config):
    return ArduinoMCU(config)