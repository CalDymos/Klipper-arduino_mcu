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
        
        self._config_callbacks = []
        self._printer.register_event_handler("klippy:connect",
                                        self.handle_connect)

    def handle_connect(self):
        for cb in self._config_callbacks:
            cb()

    def setup_pin(self, pin_type, pin_params):
        ppins = self.printer.lookup_object('pins')
        name = pin_params['pin']
        if name in self._pins:
            return self._pins[name]
        if pin_type == 'digital_out':
            pin = DigitalOutVirtualPin(self, pin_params)
        elif pin_type == 'pwm':
            pin = PwmVirtualPin(self, pin_params)
        elif pin_type == 'adc':
            pin = AdcVirtualPin(self, pin_params)
        else:
            raise ppins.error("unable to create virtual pin of type %s" % (
                pin_type,))
        self._pins[name] = pin
        return pin

    def create_oid(self):
        self._oid_count += 1
        return self._oid_count - 1

    def register_config_callback(self, cb):
        self._config_callbacks.append(cb)

    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        pass

    def get_query_slot(self, oid):
        return 0

    def seconds_to_clock(self, time):
        return 0

    def get_printer(self):
        return self._printer

    def register_response(self, cb, msg, oid=None):
        pass

    def alloc_command_queue(self):
        pass

    def lookup_command(self, msgformat, cq=None):
        return VirtualCommand()

    def lookup_query_command(self, msgformat, respformat, oid=None,
                             cq=None, is_async=False):
        return VirtualCommandQuery(respformat, oid)

    def get_enumerations(self):
        return {}

    def print_time_to_clock(self, print_time):
        return 0

    def estimated_print_time(self, eventtime):
        return 0

    def register_stepqueue(self, stepqueue):
        pass

    def request_move_queue_slot(self):
        pass

    def get_status(self, eventtime):
        return {
            'pins': {
                name : pin.get_status(eventtime)
                    for name, pin in self._pins.items()
            }
        }
        
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

class VirtualCommand:
    def send(self, data=(), minclock=0, reqclock=0):
        pass

    def get_command_tag(self):
        pass

class VirtualCommandQuery:
    def __init__(self, respformat, oid):
        entries = respformat.split()
        self._response = {}
        for entry in entries[1:]:
            key, _ = entry.split('=')
            self._response[key] = oid if key == 'oid' else 1

    def send(self, data=(), minclock=0, reqclock=0):
        return self._response

    def send_with_preface(self, preface_cmd, preface_data=(), data=(),
                          minclock=0, reqclock=0):
        return self._response
    
class VirtualPin:
    def __init__(self, mcu, pin_params):
        self._mcu = mcu
        self._name = pin_params['pin']
        self._pullup = pin_params['pullup']
        self._invert = pin_params['invert']
        self._value = self._pullup
        printer = self._mcu.get_printer()
        self._real_mcu = printer.lookup_object('mcu')
        gcode = printer.lookup_object('gcode')
        self.cmd_SET_VIRTUAL_PIN_help = f"Set the value of an output pin on mcu '{self._mcu.name}'"
        gcode.register_mux_command(f"SET_{self._mcu.name.upper()}_MCU_PIN", "PIN", self._name,
                                   self.cmd_SET_VIRTUAL_PIN,
                                   desc=self.cmd_SET_VIRTUAL_PIN_help)

    def cmd_SET_VIRTUAL_PIN(self, gcmd):
        self._value = gcmd.get_float('VALUE', minval=0., maxval=1.)

    def get_mcu(self):
        return self._real_mcu
    
class DigitalOutVirtualPin(VirtualPin):
    def __init__(self, mcu, pin_params):
        VirtualPin.__init__(self, mcu, pin_params)

    def setup_max_duration(self, max_duration):
        pass

    def setup_start_value(self, start_value, shutdown_value):
        self._value = start_value

    def set_digital(self, print_time, value):
        self._value = value

    def get_status(self, eventtime):
        return {
            'value': self._value,
            'type': 'digital_out'
        }

class PwmVirtualPin(VirtualPin):
    def __init__(self, mcu, pin_params):
        VirtualPin.__init__(self, mcu, pin_params)

    def setup_max_duration(self, max_duration):
        pass

    def setup_start_value(self, start_value, shutdown_value):
        self._value = start_value

    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        pass

    def set_pwm(self, print_time, value, cycle_time=None):
        self._value = value

    def get_status(self, eventtime):
        return {
            'value': self._value,
            'type': 'pwm'
        }

class AdcVirtualPin(VirtualPin):
    def __init__(self, mcu, pin_params):
        VirtualPin.__init__(self, mcu, pin_params)
        self._callback = None
        self._min_sample = 0.
        self._max_sample = 0.
        printer = self._mcu.get_printer()
        printer.register_event_handler("klippy:connect",
                                            self.handle_connect)

    def handle_connect(self):
        reactor = self._mcu.get_printer().get_reactor()
        reactor.register_timer(self._raise_callback, reactor.monotonic() + 2.)

    def setup_adc_callback(self, report_time, callback):
        self._callback = callback

    def setup_adc_sample(self, sample_time, sample_count,
                         minval=0., maxval=1., range_check_count=0):

        self._min_sample = minval
        self._max_sample = maxval

    def _raise_callback(self, eventtime):
        range = self._max_sample - self._min_sample
        sample_value = (self._value * range) + self._min_sample
        self._callback(eventtime, sample_value)
        return eventtime + 2.

    def get_status(self, eventtime):
        return {
            'value': self._value,
            'type': 'adc'
        }
        
def load_config_prefix(config):
    return ArduinoMCU(config)