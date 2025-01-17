import logging
import threading
import serial
import time
import reactor
from queue import Queue, Empty

class SerialHandler:
    def __init__(self, port, baudrate=115200, retries_on_timeout=3):
        """
        Initializes the serial handler for UART communication.

        Args:
            port (str): The serial port to connect to (e.g., '/dev/ttyUSB0').
            baudrate (int): The baud rate for communication (default: 115200).
            timeout (float): Timeout for serial read operations (default: 1 second).
            retries_on_timeout (int): Number of retries for sending messages on timeout.
        """
        self.reactor = reactor
        self.port = port
        self.baudrate = baudrate
        self.retries_on_timeout = retries_on_timeout
        
        self.serial_dev = None
        self.lock = threading.Lock()
        self.response_handlers = {}  # Handlers for different message types
        self.running = False
        self.read_thread = None
        self.write_thread = None

        # Queues for incoming and outgoing messages
        self.incoming_queue = Queue()
        self.outgoing_queue = Queue()

    def connect(self):
        """Establishes a serial connection."""
        try:
            self.serial_dev = serial.Serial(self.port, self.baudrate, timeout=0) # Non-blocking mode
            logging.info(f"Connected to {self.port} at {self.baudrate} baud.")
            self.running = True

            # Start threads for reading and writing
            self.read_thread = threading.Thread(target=self._read_thread)
            self.write_thread = threading.Thread(target=self._write_thread)
            self.read_thread.start()
            self.write_thread.start()
        except serial.SerialException as e:
            logging.error(f"Failed to connect to {self.port}: {e}")
            raise

    def disconnect(self):
        """Closes the serial connection."""
        self.running = False
        if self.read_thread:
            self.read_thread.join()
        if self.write_thread:
            self.write_thread.join()
        if self.serial_dev:
            self.serial_dev.close()
            self.serial_dev = None
        logging.info("Serial connection closed.")

    def _read_thread(self):
        """Background thread for reading serial data."""
        while self.running:
            try:
                if self.serial_dev.in_waiting > 0:
                    line = self.serial_dev.readline().decode('utf-8').strip()
                    self.incoming_queue.put(line)  # Add to queue
                    self._handle_message(line)  # Dispatch to handlers
            except Exception as e:
                logging.error(f"Error reading from serial: {e}")
                self.running = False

    def _write_thread(self):
        """Background thread for sending serial data from the queue."""
        while self.running:
            try:
                # Get the next message to send
                message = self.outgoing_queue.get(timeout=0.1)  # Non-blocking mode
                retries = self.retries_on_timeout
                retry_delay = 0.01  # Initial delay for exponential backoff

                while retries > 0:
                    try:
                        checksum = self._calculate_checksum(message)
                        full_message = f"{message}*{checksum}\n"# Format: Command*Checksum
                                                                # Adds a line break to ensure that the Arduino reads the input correctly
                        self.serial_dev.write(full_message.encode('utf-8'))
                        logging.debug(f"Sent message: {full_message.strip()}")
                        break
                    except Exception as e:
                        retries -= 1
                        logging.warning(f"Retrying message ({self.retries_on_timeout - retries}/{self.retries_on_timeout}): {e}")
                        time.sleep(retry_delay)
                        retry_delay *= 2  # Exponential backoff

                        if retries == 0:
                            logging.error(f"Failed to send message after {self.retries_on_timeout} retries.")
            except Empty:
                # No messages to send; continue waiting
                pass

    def _handle_message(self, message):
        """
        Processes incoming messages and dispatches them to the appropriate handler.

        Args:
            message (str): The received message.
        """
        logging.debug(f"Received message: {message}")
        with self.lock:
            parts = message.split(' ', 1)
            cmd = parts[0]
            data = parts[1] if len(parts) > 1 else ""

            # Check for registered command handlers
            handler = self.response_handlers.get(cmd, self._default_handler)
            handler(data)
   
    def _calculate_checksum(self, command):
        """
        Calculates a simple checksum for data integrity.

        Args:
            command (str): The command string to calculate the checksum for.

        Returns:
            int: The calculated checksum value.
        """
        return sum(ord(c) for c in command) % 256

    def send_message(self, command, expect_ack=False):
        """
        Sends a command over the serial connection with optional ACK handling.

        Args:
            command (str): The command to send.
            expect_ack (bool): If True, retries on timeout will be applied.
        """
        if expect_ack:
            retry_delay = 0.01
            retries = self.retries_on_timeout

            while retries > 0:
                try:
                    checksum = self._calculate_checksum(command)
                    full_message = f"{command}*{checksum}\\n"
                    self.serial_dev.write(full_message.encode('utf-8'))
                    logging.debug(f"Sent message: {full_message.strip()}")

                    # Wait for an ACK response
                    start_time = time.time()
                    while time.time() - start_time < self.timeout:
                        try:
                            response = self.incoming_queue.get(timeout=self.timeout)
                            if "ACK" in response:  # Replace with actual ACK check
                                logging.info(f"ACK received for: {command}")
                                return
                        except Empty:
                            pass

                    # No ACK received; retry
                    retries -= 1
                    reactor.pause(reactor.monotonic() + retry_delay)  # Wartezeit
                    retry_delay *= 2
                except Exception as e:
                    logging.error(f"Error sending message: {e}")
                    retries -= 1

            logging.error(f"Failed to send message after {self.retries_on_timeout} retries.")
        else:
            # Send message without retry logic
            self.outgoing_queue.put(command)
            logging.debug(f"Queued message: {command}")

    def register_response(self, command, handler):
        """
        Registers a handler for a specific command.

        Args:
            command (str): The command to handle.
            handler (callable): The function to handle the command.
        """
        with self.lock:
            self.response_handlers[command] = handler

    def _default_handler(self, data):
        """
        Default handler for unrecognized commands.

        Args:
            data (str): The data associated with the command.
        """
        logging.warning(f"Unhandled command: {data}")

class ArduinoMCU:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.gcode = self.printer.lookup_object('gcode')
        self.port = config.get('port', '/dev/ttyUSB0') # Default port
        self.baudrate = config.getint('baud', 115200) # Default baudrate
        self.retries_on_timeout = config.getint('retries_on_timeout', 3)  # Default retries

        # init a Virtual MCU
        ppins = self.printer.lookup_object('pins')
        ppins.register_chip(f"{self.name}_pin", self)
        self._pins = {}
        self._oid_count = 0
        self._config_callbacks = []
        
        # Initialize SerialHandler
        self.serial = SerialHandler(self.port, self.baudrate, self.retries_on_timeout)
        # TODO: Ersetze alle HAndler durch Allgemeine Handler z.B. PIND_VAL / PINA_VAL etc.
        #self.serial.register_response("TEMP", self.handle_temp)
        #self.serial.register_response("HUMIDITY", self.handle_humidity)
        self.serial.register_response("ERROR", self.handle_error)

        # Register G-code command
        self.gcode.register_mux_command("SEND_ARDUINO", "TARGET", self.name, self.cmd_SEND_ARDUINO, desc=self.cmd_SEND_ARDUINO_help)
        
        self._config_callbacks = []
       # Register event handlers
        self.printer.register_event_handler("klippy:connect", self.on_klippy_connect)
        self.printer.register_event_handler("klippy:disconnect", self.on_klippy_disconnect)

    def on_klippy_connect(self):
        """Handler called when Klipper connects."""
        logging.info(f"Establishing {self.name} connection...")
        self.connect()
        for cb in self._config_callbacks:
            cb()

    def on_klippy_disconnect(self):
        """Handler called when Klipper disconnects."""
        logging.info(f"Closing {self.name} connection...")
        self.connect()

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
    
    def _respond_error(self, msg):
        logging.warning(msg)
        lines = msg.strip().split('\n')
        if len(lines) > 1:
            self.gcode.respond_info("\n".join(lines), log=False)
        self.gcode.respond_raw('!! %s' % (lines[0].strip(),))

    def connect(self):
        """Connect to the Arduino using SerialHandler."""
        try:
            self.serial.connect()
            logging.info(f"Connected to Arduino on {self.port}.")
        except Exception as e:
            raise self._respond_error(f"Failed to connect to Arduino: {e}")

    def disconnect(self):
        """Disconnect from the Arduino."""
        self.serial.disconnect()
        logging.info("Disconnected from Arduino.")

    cmd_SEND_ARDUINO_help = f"Sends a command to the target arduino_mcu."
    def cmd_SEND_ARDUINO(self, gcmd):
        """Sends a command to the target MCU."""
        cmd = gcmd.get('COMMAND', '')
        if not cmd:
            raise self._respond_error("COMMAND parameter is required")
                
        # Send the command to the Arduino
        self.serial.send_message(cmd)

    def get_status(self, eventtime):
        return {
            'name': self.name,
            'port': self.port,
            'baudrate': self.baudrate,
            'retries_on_timeout': self.retries_on_timeout,
        }
    def handle_error(self, data):
        """Handle error messages from the Arduino."""
        self._respond_error(f"Error from Arduino: {data}")

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
        gcode.register_mux_command(f"SET_ARDUINO_PIN", "TARGET", self._mcu.name,
                                   self.cmd_SET_ARDUINO_PIN,
                                   desc=self.cmd_SET_ARDUINO_PIN_help)
    
    cmd_SET_ARDUINO_PIN_help = f"Set the value of an output pin on arduino_mcu"
    def cmd_SET_ARDUINO_PIN(self, gcmd):
        _pin = gcmd.get('PIN', '')
        if not _pin:
            raise self.gcode.error("PIN parameter is missing")
        elif _pin != self.name:
            return
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