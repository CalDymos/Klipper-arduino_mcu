import logging
import threading
import serial
import time
import reactor
from queue import Queue, Empty

class SerialHandler:
    def __init__(self, reactor):
        """
        Initializes the serial handler for UART communication.

        Args:
            reactor (reactor): Event reactor instance (reactor.py)
        """
        self.reactor = reactor
        self.port = ''
        self.baudrate = 115200
        
        self.serial_dev = None
        self.lock = threading.Lock()
        self.response_handlers = {}  # Handlers for different message types
        self.running = False
        self.read_thread = None
        self.write_thread = None
        self.process_incoming_thread = None

        # Queues for incoming and outgoing messages
        self.incoming_queue = Queue()
        self.outgoing_queue = Queue()

        self.last_notify_id = 0
        self.pending_notifications = {}

    def connect(self, port, baudrate=115200):
        """Establishes a serial connection.

        Args:
            port (str): The serial port to connect to (e.g., '/dev/ttyUSB0').
            baudrate (int): The baud rate for communication (default: 115200).
        """
        self.port = port
        self.baudrate = baudrate

        try:
            self.serial_dev = serial.Serial(self.port, self.baudrate, timeout=0) # Non-blocking mode
            logging.info(f"Connected to {self.port} at {self.baudrate} baud.")
            self.running = True

            # Start threads for reading and writing
            self.read_thread = threading.Thread(target=self._read_thread)
            self.write_thread = threading.Thread(target=self._write_thread)
            self.process_incoming_thread = threading.Thread(target=self._process_incoming_thread)
            self.read_thread.start()
            self.write_thread.start()
            self.process_incoming_thread.start()
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
        if self.process_incoming_thread:
            self.process_incoming_thread.join()
        if self.serial_dev:
            self.serial_dev.close()
            self.serial_dev = None
        logging.info("Serial connection closed.")

    def _read_thread(self):
        """
        Reads serial data and stores it in the incoming queue.
        """
        while self.running:
            try:
                if self.serial_dev.in_waiting > 0:
                    # Read a line from the serial device
                    line = self.serial_dev.readline().decode('utf-8').strip()
                    logging.debug(f"Received raw message: {line}")
                    
                    # Store the line in the incoming queue
                    self.incoming_queue.put(line)
                    
            except Exception as e:
                logging.error(f"Error reading from serial: {e}")
                self.running = False

    def _process_incoming_thread(self):
        """
        Processes messages from the incoming queue.
        """
        while self.running:
            try:
                # Get the next message from the incoming queue
                line = self.incoming_queue.get(timeout=0.1)  # Wait for a message
                logging.debug(f"Processing message: {line}")

                # Parse the message and CRC
                if "*" in line:
                    msg, crc = line.rsplit("*", 1)

                    # Validate the CRC
                    try:
                        expected_crc = self._calculate_checksum(msg)
                        if int(crc) != expected_crc:
                            logging.error(f"CRC mismatch for message: {msg}, expected: {expected_crc}, got: {crc}")
                            continue
                    except ValueError:
                        logging.error(f"Invalid CRC in message: {line}")
                        continue

                    # Check for notification ID (NID)
                    if "NID=" in msg:
                        parts = msg.split("NID=")
                        if len(parts) > 1:
                            try:
                                nid = int(parts[1])
                                if nid in self.pending_notifications:
                                    # Complete the associated notification
                                    completion = self.pending_notifications.pop(nid)
                                    self.reactor.async_complete(completion, line)
                                    continue
                            except ValueError:
                                logging.error(f"Invalid NID in message: {msg}")
                                continue

                # If no matching notification, process the message
                self._handle_message(line)

            except Empty:
                # No messages in the queue, continue
                pass
            except Exception as e:
                logging.error(f"Error processing message: {e}")
                          
    def _write_thread(self):
        """Background thread for sending serial data from the queue."""
        while self.running:
            try:
                # Get the next message to send
                message = self.outgoing_queue.get(timeout=0.1)  # Non-blocking mode
                try:
                    checksum = self._calculate_checksum(message)
                    full_message = f"{message}*{checksum}\n"# Format: Command*Checksum
                                                            # Adds a line break to ensure that the Arduino reads the input correctly
                    self.serial_dev.write(full_message.encode('utf-8'))
                    logging.debug(f"Sent message: {full_message.strip()}")
                    break
                except Exception as e:
                    logging.error(f"Failed to send message.")
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

    def send_message(self, command, expect_ack=False, wait_for_response=False):
        """
        Sends a command over the serial connection with optional ACK handling.

        Args:
            command (str): The command to send.
            expect_ack (bool): If True, wait asynchronously for an ACK.
            wait_for_response (bool): If True, wait for a response using retries.
        Returns:
            str: The response received if `wait_for_response` is True, otherwise None.
        """
        if expect_ack:
            timeout = 5
            try:
                # Generate a unique ID for this message
                self.last_notify_id += 1
                nid = self.last_notify_id

                # Create a completion instance for this message
                completion = self.reactor.completion()
                self.pending_notifications[nid] = completion

                # Send the command with the notification ID
                command = f"{command} NID={nid}"
                checksum = self._calculate_checksum(command)
                full_message = f"{command}*{checksum}\\n"
                self.serial_dev.write(full_message.encode('utf-8'))
                logging.debug(f"Sent message: {full_message.strip()}")

                # Wait for an ACK response
                completion = self.reactor.completion()
                self.pending_notifications[nid] = completion
                # Calculate the waketime
                waketime = self.reactor.monotonic() + timeout
                logging.debug(f"waketime = {waketime}")
                response = completion.wait(waketime=waketime, waketime_result=None) # wait for response
                if response is None:
                    del self.pending_notifications[nid]  # Cleanup pending notification
                    raise TimeoutError(f"No ACK received for: {command} / {self.reactor.monotonic()}")

                return response
            except Exception as e:
                logging.error(f"Error sending message: {e}")
                raise

        elif wait_for_response:
            retries = 5
            retry_delay = 0.01
            while retries > 0:
                try:
                    checksum = self._calculate_checksum(command)
                    full_message = f"{command}*{checksum}\n"
                    self.serial_dev.write(full_message.encode('utf-8'))
                    logging.debug(f"Sent message: {full_message.strip()}")

                    # Wait for a response
                    start_time = time.time()
                    while time.time() - start_time < retry_delay * 2:
                        try:
                            response = self.incoming_queue.get(timeout=retry_delay)
                            if response:
                                logging.debug(f"Response received: {response}")
                                return response
                        except Empty:
                            pass

                    # No response; retry
                    retries -= 1
                    logging.warning(f"Retrying message ({5 - retries}/5): {command}")
                    retry_delay *= 2  # Exponential backoff
                except Exception as e:
                    logging.error(f"Error sending message: {e}")
                    retries -= 1

            logging.error(f"Failed to receive response after retries for: {command}")
            raise TimeoutError(f"No response received for: {command}")

        else:
            # Send message without ACK, over queue
            self.outgoing_queue.put(command)
            logging.debug(f"Queued message: {command}")

    def register_response(self, command, handler, oid=None):
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
        self._printer = config.get_printer()
        self._reactor = self._printer.get_reactor()
        self._name = config.get_name().split()[-1]
        self._gcode = self._printer.lookup_object('gcode')
        self._port = config.get('port', '/dev/ttyUSB0') # Default port
        self._baudrate = config.getint('baud', 115200) # Default baudrate
        self._debuglevel = config.get('debug', False)
        if self._debuglevel:
            logging.getLogger().setLevel(logging.DEBUG) #  generally activates the debug mode for logging

        # init Arduino MCU
        ppins = self._printer.lookup_object('pins')
        ppins.register_chip(f"{self._name}_pin", self) # registers the mcu_name + the suffix '_pin' as a new chip, 
                                                       # so the pin must be specified in the config with {mcu_name}_pin:{pin_name}.
        self._pins = {}
        self._oid_count = 0
        self._config_callbacks = []
        
        # Initialize SerialHandler
        self._serial = SerialHandler(self._reactor)
        self.register_response("ERROR", self.handle_error)

        # Register G-code command
        self._gcode.register_mux_command("SEND_ARDUINO", "TARGET", self._name, self.cmd_SEND_ARDUINO, desc=self.cmd_SEND_ARDUINO_help)
        
        self._config_callbacks = []
       # Register event handlers
        self._printer.register_event_handler("klippy:mcu_identify", self._mcu_identify)
        self._printer.register_event_handler("klippy:connect", self._connect)
        self._printer.register_event_handler("klippy:disconnect", self._disconnect)

    def setup_pin(self, pin_type, pin_params):
        ppins = self._printer.lookup_object('pins')
        name = pin_params['pin']
        if name in self._pins:
            return self._pins[name]
        if pin_type == 'digital_out':
            pin = ArduinoMCU_digital_out(self, pin_params)
        elif pin_type == 'pwm':
            pin = ArduinoMCU_pwm(self, pin_params)
        elif pin_type == 'adc':
            pin = ArduinoMCU_adc(self, pin_params)
        else:
            raise ppins.error("Arduino_MCU does not support pin of type %s" % (
                pin_type,))
        self._pins[name] = pin
        return pin

    def create_oid(self):
        self._oid_count += 1
        return self._oid_count - 1

    def register_config_callback(self, cb):
        logging.debug(f"Registering config callback: {cb}")
        self._config_callbacks.append(cb)

    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        pass

    def get_query_slot(self, oid):
        return 0

    def seconds_to_clock(self, time):
        return 0

    def get_printer(self):
        return self._printer

    def register_response(self, command, handler, oid=None):
        self._serial.register_response(command, handler, oid)

    def alloc_command_queue(self):
        pass

    def lookup_command(self, msgformat, cq=None):
        return CommandWrapper()

    def lookup_query_command(self, msgformat, respformat, oid=None,
                             cq=None, is_async=False):
        return CommandQueryWrapper(respformat, oid)

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

    def _mcu_identify(self):

        logging.info(f"Establishing {self._name} connection...")
        try:
            self._serial.connect(self._port, self._baudrate)
            logging.info(f"Connected to Arduino on {self._port}.")
        except Exception as e:
            self._respond_error(f"Failed to connect to Arduino: {e}\n check the connection to the MCU ", True)

    def get_status(self, eventtime):
        return {
            'pins': {
                name : pin.get_status(eventtime)
                    for name, pin in self._pins.items()
            }
        }
    
    def _respond_error(self, msg, exception=False):
        """
        Logs an error message and raises a custom exception.
        """
        logging.warning(msg)
        lines = msg.strip().split('\n')
        if len(lines) > 1:
            self._gcode.respond_info("\n".join(lines), log=False)
        self._gcode.respond_raw('!! %s' % (lines[0].strip(),))
        if exception : 
            raise RuntimeError(msg)  # Trigger RuntimeError exception

    def _connect(self):
        """Connect to the Arduino using SerialHandler."""
        for cb in self._config_callbacks:
            cb()

    def _disconnect(self):
        """Disconnect from the Arduino."""
        logging.info(f"Closing {self._name} connection...")
        self._serial.disconnect()
        logging.info(f"Disconnected from {self._name}.")

    cmd_SEND_ARDUINO_help = f"Sends a command to the target arduino_mcu."
    def cmd_SEND_ARDUINO(self, gcmd):
        """Sends a command to the target MCU."""
        cmd = gcmd.get('COMMAND', '')
        if not cmd:
            self._respond_error("COMMAND parameter is required", True)
                
        # Send the command to the Arduino
        self.serial.send_message(cmd)

    def get_status(self, eventtime):
        return {
            'name': self._name,
            'port': self._port,
            'baudrate': self._baudrate,
        }
    
    def handle_error(self, data):
        """Handle error messages from the Arduino."""
        self._respond_error(f"Error from Arduino: {data}")

class CommandWrapper:
    def send(self, data=(), minclock=0, reqclock=0):
        pass

    def get_command_tag(self):
        pass

class CommandQueryWrapper:
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
    
class ArduinoMCUPin:
    def __init__(self, mcu, pin_params):
        self._mcu = mcu
        self._name = pin_params['pin']
        self._pullup = pin_params['pullup']
        self._invert = pin_params['invert']
        self._value = self._pullup
        printer = self._mcu.get_printer()
        self._real_mcu = printer.lookup_object('mcu')

    def get_mcu(self):
        return self._real_mcu
    
class ArduinoMCU_digital_out(ArduinoMCUPin):
    def __init__(self, mcu, pin_params):
        ArduinoMCUPin.__init__(self, mcu, pin_params)
        self._mcu.register_config_callback(self._send_pin_config)

    def setup_max_duration(self, max_duration):
        pass

    def setup_start_value(self, start_value, shutdown_value):
        self._value = start_value

    def set_digital(self, print_time, value):
        """Set the pin value and send the command to Arduino MCU."""
        self._value = value
        command = f"SET_PIN PIN={self._name} VALUE={int(value)}"
        self._mcu._serial.send_message(command)
        logging.debug(f"sends to {self._mcu._name} SET_PIN PIN={self._name} VALUE={int(value)}")

    def _send_pin_config(self):
        """ Sends the configuration for the digital pin to the Arduino MCU """
        command = f"CONFIG_DIGITAL_OUT PIN={self._name} PULL_UP={self._pullup} INVERT={self._invert}"
        self._mcu._serial.send_message(command, True)
        logging.debug(f"sends to {self._mcu._name} CONFIG_DIGITAL_OUT PIN={self._name} PULL_UP={self._pullup} INVERT={self._invert}")

    def get_status(self, eventtime):
        return {
            'value': self._value,
            'type': 'digital_out'
        }

class ArduinoMCU_pwm(ArduinoMCUPin):
    def __init__(self, mcu, pin_params):
        ArduinoMCUPin.__init__(self, mcu, pin_params)
        self._cycle_time = 0.100
        self._mcu.register_config_callback(self._send_pin_config)

    def setup_max_duration(self, max_duration):
        pass

    def setup_start_value(self, start_value, shutdown_value):
        self._value = start_value

    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        self._cycle_time = cycle_time

    def set_pwm(self, print_time, value, cycle_time=None):
        """Set the PWM value and send the command to Arduino MCU."""
        self._value = value
        cycle_time = cycle_time or self._cycle_time
        command = f"SET_PIN PIN={self._name} VALUE={value}"
        self._mcu._serial.send_message(command)
        logging.debug(f"sends to {self._mcu._name} SET_PIN PIN={self._name} VALUE={value}")

    def _send_pin_config(self):
        """ Sends the configuration for the pwm pin to the Arduino MCU """
        command = f"CONFIG_PWM_OUT PIN={self._name} CYCLE_TIME={self._cycle_time} START_VALUE={self._value}"
        self._mcu._serial.send_message(command, True)
        logging.debug(f"sends to {self._mcu._name} CONFIG_PWM_OUT PIN={self._name} CYCLE_TIME={self._cycle_time} START_VALUE={self._value}")

    def get_status(self, eventtime):
        return {
            'value': self._value,
            'type': 'pwm'
        }

class ArduinoMCU_adc(ArduinoMCUPin):
    def __init__(self, mcu, pin_params):
        ArduinoMCUPin.__init__(self, mcu, pin_params)
        self._oid = None
        self._callback = None
        self._min_sample = 0.
        self._max_sample = 0.
        self._sample_time = 0
        self._report_time = 0.
        self._sample_count = 0
        self._range_check_count = 0
        printer = self._mcu.get_printer()
        self._mcu.register_config_callback(self._send_pin_config)
        printer.register_event_handler("klippy:connect",
                                            self.handle_connect)

    def handle_connect(self):
        reactor = self._mcu.get_printer().get_reactor()
        reactor.register_timer(self._raise_callback, reactor.monotonic() + 2.)
        self._mcu.register_response(self._handle_analog_in_state,
                                    "analog_in_state", self._oid)

    def setup_adc_callback(self, report_time, callback):
        self._report_time = report_time
        self._callback = callback

    def setup_adc_sample(self, sample_time, sample_count,
                         minval=0., maxval=1., range_check_count=0):
        self._sample_time = sample_time
        self._sample_count = sample_count
        self._min_sample = minval
        self._max_sample = maxval
        self._range_check_count = range_check_count

    def _raise_callback(self, eventtime):
        range = self._max_sample - self._min_sample
        sample_value = (self._value * range) + self._min_sample
        self._callback(eventtime, sample_value)
        return eventtime + 2.
    
    def _send_pin_config(self):
        #logging.debug(f"{self._sample_time} / {self._sample_count} / {self._min_sample} / {self._max_sample} / {self._range_check_count}")
        """ Sends the configuration for the analog input pin to the Arduino MCU """
        command = f"CONFIG_ANALOG_IN PIN={self._name}"
        self._mcu._serial.send_message(command, True)
        logging.debug(f"sends to {self._mcu._name} CONFIG_ANALOG_IN PIN={self._name}") 

    def _handle_analog_in_state(self, params):
        pass

    def get_status(self, eventtime):
        return {
            'value': self._value,
            'type': 'adc'
        }
        
def load_config_prefix(config):
    return ArduinoMCU(config)