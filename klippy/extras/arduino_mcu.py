import logging
import threading
import serial, socket
import time, select, selectors
from queue import Queue, Empty
from abc import ABC, abstractmethod

# imports for type checking
from typing import TYPE_CHECKING, cast, Any

if TYPE_CHECKING:
    from klippy.klippy import Printer  
    from klippy.reactor import SelectReactor as Reactor
    from klippy.configfile import PrinterConfig as Config
    from klippy.extras.heaters import PrinterHeaters as Heater
    from klippy.gcode import GCodeDispatch as GCode
    from klippy.toolhead import ToolHead
    from klippy.pins import PrinterPins as Pin
    from klippy.extras.pulse_counter import MCU_counter
    from klippy.extras.pulse_counter import FrequencyCounter
    from klippy.extras.buttons import MCU_buttons
    from klippy.extras.gcode_button import GCodeButton

class logger:
    """helper class for timestamped debug logging."""

    @staticmethod
    def debug(message : str):
        """
        Logs a debug message with a timestamp.

        Args:
            message (str): The message to log.
        """
        if logging.getLogger().level == logging.DEBUG:
            current_time = time.time()
            local_time = time.localtime(current_time)
            milliseconds = int((current_time - int(current_time)) * 1000)
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S', local_time)
            logging.debug(f"{timestamp}.{milliseconds} {message}")


class MessageParser:
    def __init__(self, mcu_name: str = ""):
        """
        class to parse the data getting from device.
        """
        self._mcu_name = mcu_name

    def _calculate_checksum(self, command: str) -> int:
        """
        Calculates a simple checksum for data integrity.

        Args:
            command (str): The command string to calculate the checksum for.

        Returns:
            int: The calculated checksum value as utf-8 string.
        """
        return sum(command.encode("utf-8")) % 256

    def validate(self, data: str) -> str:
        """
        Validates a message by checking its checksum.

        Args:
            data (str): The raw message string received.

        Returns:
            str : The message if valid, otherwise empty str.
        """
        logger.debug(f"{self._mcu_name}: Received data to validate: {data}")
        if "*" in data:
            msg, crc = data.rsplit("*", 1)
            try:
                if int(crc) == self._calculate_checksum(msg):
                    return msg
                else:
                    logging.warning(f"{self._mcu_name}: Checksum mismatch. Message '{msg}' discarded.")
            except ValueError:
                logging.warning(f"{self._mcu_name}: Invalid checksum format in data '{data}'.")

        logging.error(f"{self._mcu_name}: checksum for data '{data}' missing")
        return ""

    def get_msg(self, data: str) -> str:
        """
        return message without checksum part.

        Args:
            data (str): The raw message string received.

        Returns:
            str : The message without crc, otherwise empty str.
        """  
        if "*" in data:
            msg = data.rsplit("*", 1)
            return msg[0]
        return ""     
             
    def parse(self, msg: str) -> dict:
        """
        Parses a valid message, and extracts parameters.

        Args:
            msg (str): The message string received without CRC.

        Returns:
            dict: Parsed key-value parameters
        """
        logger.debug(f"{self._mcu_name}: Received msg to parse: {msg}")

        if msg:
            # Parse parameters from msg
            parts = msg.split()
            params = {}
            for part in parts:
                if "=" in part:
                    key, value = part.split("=")
                    params[key] = value
            return params

        return {}
    
    def create_cmd(self, command: str) -> bytes:
        """
        Creates a properly formatted command with checksum and newline.

        Args:
            command (str): The base command string to send.

        Returns:
            bytes: The full command string with checksum and newline, encoded asa UTF-8.
        """
        checksum = self._calculate_checksum(command)
        return f"{command}*{checksum}\n".encode("utf-8")
    
    def get_cmd(self, msg: str) -> str:
        """
        Extracts the command part from a valid message.

        Args:
            data (str): The (raw) message string received.

        Returns:
            str : The command if valid, otherwise empty str.
        """

        if msg:
            cmd = msg.split(" ", 1)[0]
            return cmd.rstrip(":") #removes trailing ':' (if exists) and return command

        return ""

MAX_POLL_TIMEOUT =  2500 # ms  (WATCHDOG_INTERVAL / 2)
MIN_POLL_TIMEOUT =  250  # ms   
DEFAULT_POLL_TIMEOUT = 1000 # ms (DEFAULT_REPORT_TIME / 2)

class CommunicationHandler(ABC):
    def __init__(self, reactor: Reactor, mcu_name, mcu_const: dict):
        """
        Base class for communication handlers.

        Args:
            reactor (reactor): Event reactor instance.
            mcu_name (str): Name of the MCU to identify.
            mcu_const (dict) : Dictionary for MCU constants that is received when connecting to the MCU for the first time
        """
        self._reactor = reactor
        self._mcu_name = mcu_name
        
        self._msgparser = MessageParser(self._mcu_name)
        self._partial_data = ""  # Buffer für unvollständige Nachrichten

        # dict for MCU constants
        self._mcu_const = mcu_const

        self._lock = threading.Lock()
        self._response_handlers = {}
        self._running = False

        # Threads
        self._read_thread_inst = None
        self._write_thread_inst = None
        self._write_condition = threading.Condition()
        self._process_incoming_thread_inst = None

        # Watchdog (checked in mcu -> check_active())
        self.last_response_time = 0

        # Queues for incoming and outgoing messages
        self._incoming_queue = Queue()
        self._outgoing_queue = Queue()

        self._poll_timeout  = DEFAULT_POLL_TIMEOUT # ms 
        
        self._last_notify_id = 0
        self._pending_notifications = {}

        self._identify_cmd = self._msgparser.create_cmd("identify")

    @abstractmethod
    def _read_data(self) -> str:
        """Abstract method to read data from the connection."""
        pass

    @abstractmethod
    def _send_data(self, data):
        """Abstract method to send data over the connection."""
        pass

    @abstractmethod
    def _reset_device_input_buffer(self):
        """Abstract method to reset the input buffer from the connection."""
        pass 

    @abstractmethod
    def _setup_connection(self):
        """Abstract method to set up the connection."""
        pass

    @abstractmethod
    def _cleanup_connection(self):
        """Abstract method to clean up the connection."""
        pass

    def _get_identify_data(self):
        """
        Checks whether the connection is ready by attempting to send an identification request to the device.

        Returns:
            bool: True if identifier is received, indicating the connection is ready; False otherwise.
        """
        try:

            self._send_data(self._identify_cmd)
            logger.debug(f"{self._mcu_name}: Sent init command: {self._identify_cmd.decode('utf-8').strip()}")

            # Check if there is any data available in the input buffer
            data = self._read_data()
            if data:
                
                # validate CRC and Parse the message
                params = self._msgparser.parse(self._msgparser.validate(data))
                if params:
                    if params.get('mcu', '') == self._mcu_name:
                        self._mcu_const['chip'] = params.get('chip', 'Unknown')
                        self._mcu_const['version'] = params.get('version', 'Unknown')
                        self._mcu_const['freq'] = params.get('freq', 0)
                        self._mcu_const['adc_max'] = params.get('adc_max', 1)
                        self._mcu_const['adc_sample_count'] = params.get('adc_sample_count', 1)
                        self._reactor.pause(self._reactor.monotonic() + 0.1)
                        self._reset_device_input_buffer()
                        return True

            # No valid data received, connection is not ready yet
            return False

        except Exception as e:
            # Log the error if any issue occurs during the readiness check
            logging.error(f"{self._mcu_name}: Error while checking connection readiness: {e}")
            return False

    def _read_thread(self):
        """Reads data from device and stores it in the queue."""
        while self._running:
            try:
                if not self._running:
                    return
                
                # read data from device
                data = self._read_data()
                if data:
                    logger.debug(f"{self._mcu_name}: Received raw message: {data}")

                    # store data in the incoming queue
                    self._incoming_queue.put(data)

            except Exception as e:
                logging.error(f"{self._mcu_name}: Error reading data: {e}")
                self._running = False

    def _write_thread(self):
        """Background thread for Sending data from the outgoing queue."""
        while self._running:
            if not self._running:
                return
            with self._write_condition:
                try:
                    if self._outgoing_queue.empty():
                        # Put thread to sleep and wait for data
                        self._write_condition.wait()  # Thread sleeps infinitely until it is woken up again via notify (send_message())                
                        #self._write_condition.wait(timeout=1)  # Thread sleeps and waits until it is woken up again via notify or 1 second has passed  
                        
                    # Get the next message to send
                    message = self._outgoing_queue.get()

                    full_msg = self._msgparser.create_cmd(message)
                    self._send_data(full_msg)
                    logger.debug(f"{self._mcu_name}: Sent message: {full_msg.strip()}")

                except Exception as e:
                    logging.error(f"{self._mcu_name}: Error sending message: {e}")

    def connect(self):
        """Sets up the connection and starts communication threads."""
        try:
            self._setup_connection()

            # Test whether the connection is ready 
            start_time = self._reactor.monotonic()
            timeout = 5  # Waiting time for the connection to be ready

            while self._reactor.monotonic() - start_time < timeout:
                if self._get_identify_data():
                    logging.info(f"{self._mcu_name}: connection is ready.")
                    self._running = True
                    break
                self._reactor.pause(self._reactor.monotonic() + 0.1)  # simple dirty solutions to avoid blocking the main thread completely

            if not self._running:
                self._cleanup_connection()
                raise TimeoutError(f"{self._mcu_name}: connection not ready within timeout period.")

            # Start threads for reading and writing
            self._read_thread_inst = threading.Thread(target=self._read_thread)
            self._write_thread_inst = threading.Thread(target=self._write_thread)
            self._process_incoming_thread_inst = threading.Thread(target=self._process_incoming_thread)
            self._read_thread_inst.start()
            self._write_thread_inst.start()
            self._process_incoming_thread_inst.start()

            # save last response time for watchdog
            self.last_response_time = self._reactor.monotonic()

        except Exception as e:
            logging.error(f"{self._mcu_name}: Failed to establish connection: {e}")
            raise

    def disconnect(self):
        """Stops communication and cleans up resources."""
        if self._running:
            self._running = False

            # Stop all threads
            for thd in [self._read_thread_inst, self._write_thread_inst, self._process_incoming_thread_inst]:
                if thd and thd.is_alive() and thd != threading.current_thread():
                    thd.join()


            # Clean up the specific connection
            self._cleanup_connection()

    def _process_incoming_thread(self):
        """Processes messages from the incoming queue."""
        while self._running:
            if not self._running:
                return
            try:
                # Get the next message from the incoming queue
                data = self._incoming_queue.get()# Wait for a message in queue
                logger.debug(f"{self._mcu_name}: Processing message: {data}")

                # CRC check
                msg = self._msgparser.validate(data)
                if not msg:
                    continue
                # Parse the message
                params = self._msgparser.parse(msg)
                if not params:
                    continue

                # simple check for the watchdog confirmation message
                if "mcu_watchdog" in params:
                    self.last_response_time = self._reactor.monotonic()  # reset Watchdog zurücksetzen
                    logger.debug(f"{self._mcu_name}: Watchdog-Reset durch MCU")
                    continue

                # Check for notification ID (NID)
                nid = int(params.get("nid", 0))
                if nid:
                    if nid in self._pending_notifications:
                        # Complete the associated notification
                        completion = self._pending_notifications.pop(nid)
                        self._reactor.async_complete(completion, data)
                        continue
                    else:
                        logging.error(f"{self._mcu_name}: Invalid nid in message: {data}")
                        continue

                # If no matching notification, process the message
                self._handle_message(msg)

            except Exception as e:
                logging.error(f"{self._mcu_name}: Error processing message: {e}")

    def _handle_message(self, msg: str):
        """
        Processes incoming messages and dispatches them to the appropriate handler.

        Args:
            message (str): The received validated message.
        """
        logger.debug(f"{self._mcu_name}: Received message: {msg}")
        with self._lock:
            cmd = self._msgparser.get_cmd(msg)
            params = self._msgparser.parse(msg)

            handler = (cmd, params.get('oid'))
 
            # Check for registered command handlers
            handler = self._response_handlers.get(handler, self._default_handler)
            handler(params)

    def _default_handler(self, params: dict):
        """
        Default handler for unrecognized commands.

        Args:
            params (dict): The data associated with the command.
        """
        logging.warning(f"{self._mcu_name}: Unhandled command")

    def send_message(self, command, expect_ack: bool=False, wait_for_response: bool=False):
        """
        Sends a command over the connection with optional ACK handling.

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
                self._last_notify_id += 1
                nid = self._last_notify_id

                # Create a completion instance for this message
                completion = self._reactor.completion()
                self._pending_notifications[nid] = completion

                # Send the command with the notification ID
                full_msg = self._msgparser.create_cmd(f"{command} nid={nid}")
                self._send_data(full_msg)
                logger.debug(f"{self._mcu_name}: Sent message: {full_msg.strip()}")

                # Wait for an ACK response
                completion = self._reactor.completion()
                self._pending_notifications[nid] = completion
                # Calculate the waketime
                waketime = self._reactor.monotonic() + timeout

                response = completion.wait(waketime=waketime, waketime_result=None) # wait for response
                if response is None:
                    del self._pending_notifications[nid]  # Cleanup pending notification
                    raise TimeoutError(f"{self._mcu_name}: No ACK received for: {command}")

                return response
            except Exception as e:
                logging.error(f"{self._mcu_name}: Error sending message: {e}")
                raise

        elif wait_for_response:
            retries = 5
            retry_delay = 0.01
            while retries > 0:
                try:
                    full_msg = self._msgparser.create_cmd(command)
                    self._send_data(full_msg)
                    logger.debug(f"{self._mcu_name}: Sent message: {full_msg.strip()}")

                    # Wait for a response
                    start_time = time.time()
                    while time.time() - start_time < retry_delay * 2:
                        try:
                            response = self._incoming_queue.get(timeout=retry_delay)
                            if response:
                                logger.debug(f"{self._mcu_name}: Response received: {response}")
                                return response
                        except Empty:
                            pass

                    # No response; retry
                    retries -= 1
                    logging.warning(f"{self._mcu_name}: Retrying message ({5 - retries}/5): {command}")
                    retry_delay *= 2  # Exponential backoff
                except Exception as e:
                    logging.error(f"{self._mcu_name}: Error sending message: {e}")
                    retries -= 1

            logging.error(f"{self._mcu_name}: Failed to receive response after retries for: {command}")
            raise TimeoutError(f"{self._mcu_name}: No response received for: {command}")

        else: # Send message without ACK, over queue
            with self._write_condition:
                self._outgoing_queue.put(command)
                self._write_condition.notify()  # Wake up the writing thread
                logger.debug(f"{self._mcu_name}: Queued message: {command}")

    def register_response(self, handler, command, oid=None):
        """
        Registers a handler for a specific command.

        Args:
            handler (callable): The function to handle the command.
            command (str): The command to handle.
            oid (int): Object-ID for the Hardware object 
        """
        with self._lock:
            if handler is None:
                del self._response_handlers[command, oid]
            else:
                self._response_handlers[command, oid] = handler

class NetworkHandler(CommunicationHandler):
    def __init__(self, reactor, mcu_name, mcu_const: dict, hostIp=None, port=45800):
        """
        Initializes the Network handler for TCP communication.

        Args:
            reactor (reactor): Event reactor instance (reactor.py).
            mcu_name (str): Name of the MCU to identify.
            mcu_const (dict) : Dictionary for MCU constants that is received when connecting to the MCU for the first time
            hostIp (str): IP address to bind the server. (default: current IP)
            port (int): Port number for the server to listen on. (default: 45800)
        """
        super().__init__(reactor, mcu_name, mcu_const)

        # Get the current IP if hostIp is not specified
        self._hostIp = hostIp or self._get_local_ip()

        # Validate and set the port
        self._port = self._validate_port(port)

        self._server_socket = None
        self._client_socket = None
        self._client_address = None

        self._selector = selectors.DefaultSelector()

    def _get_local_ip(self):
        """Gets the local IP address of the current machine."""
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception as e:
            logging.error(f"{self._mcu_name}: Failed to get local IP address: {e}")
            return "127.0.0.1"

    def _validate_port(self, port):
        """
        Validates the port to ensure it does not conflict with common services.

        Args:
            port (int): The requested port number.

        Returns:
            int: A valid port number.
        """
        # List of standard "klipper" ports to avoid        
        reserved_ports = {1883, 5000, 7125, 8080, 8819, 8883}
        if port in reserved_ports or port < 1024 or port > 65535:
            logging.warning(f"{self._mcu_name}: Port {port} is reserved or invalid. Assigning default port.")
            return 45800
        return port

    def _setup_connection(self):
        self._server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket.bind((self._hostIp, self._port))
        self._server_socket.listen(1) # Accept one client at a time
        logging.info(f"{self._mcu_name}: Server started on {self._hostIp}:{self._port}")

        # Accept a client connection
        self._client_socket, self._client_address = self._server_socket.accept()
        logging.info(f"{self._mcu_name}: Client connected from {self._client_address}")

        self._selector.register(self._client_socket, selectors.EVENT_READ)

    def _cleanup_connection(self):
        if self._client_socket:
            self._client_socket.close()
            self._client_socket = None
        if self._server_socket:
            self._server_socket.close()
            self._server_socket = None
        logging.info(f"{self._mcu_name}: Server stopped.")

    def _read_data(self):
        """Reads network data using polling."""
        if self._client_socket:
            try:
                events = self._selector.select(timeout=(self._poll_timeout / 1000))
                if events:
                    # Receive new data
                    new_data = self._client_socket.recv(1024).decode("utf-8")
                    if not new_data:
                        return ""

                    # Extend buffer
                    self._partial_data += new_data

                    # Check whether a complete message is available
                    if "\n" in self._partial_data:
                        lines = self._partial_data.split("\n")
                        complete_msg = lines[0]  # First complete line
                        self._partial_data = "\n".join(lines[1:])  # Rest back into the buffer

                        return complete_msg.strip()  # Return the complete message
                
                return ""  # No complete message yet, keep waiting
            except Exception as e:
                logging.error(f"{self._mcu_name}: Error when reading the data: {e}")
                return ""

    def _send_data(self, data):
        if self._client_socket is not None:
            self._client_socket.sendall(data)
    
    def _reset_device_input_buffer(self):
        """Clears the input buffer by reading all available data."""
        if self._client_socket:
            try:
                blockingMode = self._client_socket.getblocking()
                self._client_socket.setblocking(False)  # Nicht blockierender Modus
                while True:
                    data = self._client_socket.recv(8192)
                    if not data:
                        break  # Keine weiteren Daten mehr
            except BlockingIOError:
                pass  # Kein Fehler, wenn keine Daten mehr vorhanden sind
            finally:
                self._client_socket.setblocking(blockingMode)  # Ursprünglichen Modus wiederherstellen        


class SerialHandler(CommunicationHandler):
    def __init__(self, reactor, mcu_name, mcu_const: dict, port="", baudrate=115200):
        """
        Initializes the serial handler for UART communication.

        Args:
            reactor (reactor): Event reactor instance (reactor.py)
            mcu_name (str): Name of the MCU to identify.
            mcu_const (dict) : Dictionary for MCU constants that is received when connecting to the MCU for the first time
            port (str): The serial port to connect to (e.g., '/dev/ttyUSB0').
            baudrate (int): The baud rate for communication (default: 115200).
        """
        super().__init__(reactor, mcu_name, mcu_const)
        self._port = port
        self._baudrate = baudrate
        self._serial_dev = None
        self._poll_obj = select.poll()

    def _setup_connection(self):
        start_time = self._reactor.monotonic()
        while 1:
            try:
                if self._reactor.monotonic() > start_time + 90.:
                    raise RuntimeError("Unable to connect")

                self._serial_dev = serial.Serial(self._port, self._baudrate, timeout=0, exclusive=True)

                # Registers the file descriptor of the serial interface with the poll object.
                self._poll_obj.register(self._serial_dev.fileno(), select.POLLIN)       

            except (OSError, IOError, serial.SerialException) as e:
                logging.warning("%s: Unable to open serial port: %s",
                             self._mcu_name, e)
                self._reactor.pause(self._reactor.monotonic() + 5.)
                continue

            logging.info(f"{self._mcu_name}: Connected to {self._port} at {self._baudrate} baud.")
            break
    def _cleanup_connection(self):
        if self._serial_dev:
            self._serial_dev.close()
            self._serial_dev = None

    def _read_data(self):
        """Reads complete lines from the serial port using polling"""
        if self._serial_dev:
            try:
                events = self._poll_obj.poll(self._poll_timeout) # wait before checking again
                if events:
                    new_data = self._serial_dev.read(self._serial_dev.in_waiting).decode("utf-8")
                    self._partial_data += new_data

                    if "\n" in self._partial_data:
                        lines = self._partial_data.split("\n")
                        complete_msg = lines[0]  # Erste vollständige Zeile
                        self._partial_data = "\n".join(lines[1:])  # Rest in Buffer speichern
                        return complete_msg.strip()

                return ""  # No complete line yet
            
            except (OSError, IOError, serial.SerialException) as e:
                logging.error(f"{self._mcu_name}: Error reading from the serial port: {e}")
                self.disconnect()  # Sicherstellen, dass die Verbindung korrekt getrennt wird
                return ""

    def _send_data(self, data):
        if self._serial_dev is not None:
            self._serial_dev.write(data)
    
    def _reset_device_input_buffer(self):
        if self._serial_dev is not None:
            self._serial_dev.reset_input_buffer()

class ArduinoMCU:
    def __init__(self, config):
        self._printer: Printer = config.get_printer()
        self._reactor: Reactor = self._printer.get_reactor()
        self._name = config.get_name().split()[-1]
        self._gcode: GCode = cast("GCode", self._printer.lookup_object('gcode'))
        self._commType = config.get('comm', 'serial')
        if self._commType == 'lan':
            self._port = config.getint('port', 0) # Default port for network communication
            self._ip = config.get('ip', None)
        else:
            self._commType = 'serial'
            self._port = config.get('port', '/dev/ttyUSB0') # Default port for serial
            self._baudrate = config.getint('baud', 115200) # Default baudrate   
        
        self._debuglevel = config.get('debug', False)
        if self._debuglevel:
            logging.getLogger().setLevel(logging.DEBUG) 

        self._config_messages = []  # List for configuration messages

        self._freq_counter_instances = {} # oid -> FrequencyCounter instances
        self._button_instances = {} # oid -> MCU_buttons instances

        self._is_shutdown = self._is_timeout = False
        
        # init Arduino MCU
        ppins = cast("Pin", self._printer.lookup_object('pins'))
        ppins.register_chip(f"{self._name}_pin", self) # registers the mcu_name + the suffix '_pin' as a new chip, 
                                                       # so the pin must be specified in the config with {mcu_name}_pin:{pin_name}.
        self._pins = {}
        self._oid_count = 0
        self._config_callbacks = []

        self._mcu_const = {}
        
        # Initialize communication
        if self._commType == 'serial':
            self._comm = SerialHandler(self._reactor, self._name, self._mcu_const, self._port, self._baudrate)
        elif self._commType == 'lan':
            self._comm = NetworkHandler(self._reactor, self._name, self._mcu_const, self._ip, self._port)
        self.register_response(self.handle_error, "error")

        # Register G-code command
        self._gcode.register_mux_command("SEND_ARDUINO", "TARGET", self._name, self.cmd_SEND_ARDUINO, desc=self.cmd_SEND_ARDUINO_help)
        
        self._config_callbacks = []
       # Register event handlers
        self._printer.register_event_handler("klippy:firmware_restart", self._firmware_restart)
        self._printer.register_event_handler("klippy:mcu_identify", self._mcu_identify)
        self._printer.register_event_handler("klippy:connect", self._connect)
        self._printer.register_event_handler("klippy:disconnect", self._disconnect)

    def store_frequency_counter_instances(self):
        # Get all "pulse_counter" objects that loaded by Klipper
        pulse_counters = self._printer.lookup_objects(module="pulse_counter")
        
        for name, instance in pulse_counters:
            if isinstance(instance, FrequencyCounter): 
                self._freq_counter_instances[instance._counter._oid] = instance
                logger.debug(f"{self._name}: stored FrequencyCounter instance with oid {instance._counter._oid}")

    def store_button_instances(self):
        # Get all "buttons" objects that loaded by Klipper
        buttons = self._printer.lookup_objects(module="buttons")
        
        for name, instance in buttons:
            if isinstance(instance, MCU_buttons):
                self._button_instances[instance.oid] = instance
                logger.debug(f"{self._name}: stored MCU_buttons instance with oid {instance.oid}")

    def setup_pin(self, pin_type, pin_params):
        """
        Sets up the specified pin type and returns the appropriate pin object.

        Args:
            pin_type (str): The type of pin ('digital_out', 'pwm', 'adc').
            pin_params (dict): The parameters associated with the pin.

        Returns:
            object: The initialized pin object.
        """
        ppins = cast("Pin", self._printer.lookup_object('pins'))
        name = pin_params['pin']
        logger.debug(f"{self._name}: setup_pin for pin_type: {pin_type}")
        if name in self._pins:
            return self._pins[name]
        
        pcs = {'digital_out': ArduinoMCU_digital_out, 'pwm': ArduinoMCU_pwm, 'adc': ArduinoMCU_adc}
        if pin_type not in pcs:
            raise ppins.error("arduino_mcu does not support pin of type %s" % (pin_type,))
        pin = pcs[pin_type](self, pin_params) 
        self._pins[name] = pin
        
        return pin

    def create_oid(self):
        self._oid_count += 1
        return self._oid_count - 1

    def register_config_callback(self, cb):
        logger.debug(f"{self._name}: Registering config callback: {cb}")
        self._config_callbacks.append(cb)

    def _send_config(self):
        """
        Sends all captured button configuration commands to the Arduino MCU.
        """
        for cb in self._config_callbacks:
            cb()
        if not self._config_messages:
            logging.info(f"{self._name}: no configuration to send.")
            return

        logger.debug(f"{self._name}: Sending configuration messages to Arduino MCU...")
        for msg in self._config_messages:
            # Parse command and params from message
            self._comm.send_message(msg, True)                 

        # Clear the list after sending
        self._config_messages.clear()
        
    def add_config_cmd(self, cmd: str, is_init=False, on_restart=False):
        """
        store all configuration commands.
        """
        logger.debug(f"{self._name}: config command: {cmd}")
        # Capture and save all relevant commands that relate to buttons (captured by buttons.py)
        if cmd.startswith("buttons_add"):
            params = self._comm._msgparser.parse(cmd)
            if params:
                oid = int(params.get("oid", 0))
                pos = int(params.get("pos", 0))
                pin = params.get("pin", "")
                pull_up = int(params.get("pull_up", 0))
                snd_msg = f"config_buttons oid={oid} pos={pos} pin={pin} pullup={pull_up}"
                self._config_messages.append(snd_msg)

        # Capture and save all relevant commands that relate to pulse counter (captured by pulse_counter.py)
        elif cmd.startswith("config_counter"):
            params = self._comm._msgparser.parse(cmd)
            if params:
                oid = int(params.get("oid", 0))
                pin = params.get("pin", "")
                pull_up = int(params.get("pull_up", 0))
                snd_msg = f"config_counter oid={oid} pin={pin} pullup={pull_up}"
                self._config_messages.append(snd_msg)             
        else:
            self._config_messages.append(cmd)

    def get_constant(self, name: str, parser: type = str):
        if name not in self._mcu_const:
            RuntimeError("Firmware constant '%s' not found", name)
        try:
            value = parser(self._mcu_const[name])
        except:
            RuntimeError("Unable to parse firmware constant %s: %s",
                        name, self._mcu_const[name])
        return value
    def get_query_slot(self, oid):
        return 0

    def seconds_to_clock(self, time):
        return 0

    def get_printer(self):
        return self._printer

    def register_response(self, handler, command: str, oid=None):
        logger.debug(f"{self._name}: Registering response handler: {command}")
        # Check if we need to replace the handlers with the custom one
        if command == "buttons_state":
            self._comm.register_response(self._handle_buttons_state, command, oid)
        elif command == "counter_state":
            self._comm.register_response(self._handle_counter_state, command, oid)
        else:    
            self._comm.register_response(handler, command, oid)

    def alloc_command_queue(self):
        pass

    def lookup_command(self, msgformat, cq=None):
        pass

    def lookup_query_command(self, msgformat, respformat, oid=None,
                             cq=None, is_async=False):
        pass
    
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

    def _handle_buttons_state(self, params):
        oid = int(params['oid'])
        state = int(params['state'])  # 1 = pressed, 0 = released
        if oid not in self._button_instances:
            logging.warning(f"{self._name}: Received button_state for unknown oid {oid}")
            return  
        
        button: MCU_buttons = self._button_instances[oid]

        # Update the button status
        button.last_button = state 

        # Retrieve GCode module
        gcode_buttons = self._printer.lookup_objects(module="gcode_button")

        for _, gbutton in gcode_buttons:
            gbutton: GCodeButton = gbutton  # only for type casting   
            # Check whether one of the pins from 'pin_list' matches 'gbutton.pin'
            if any(gbutton.pin == pin[0] for pin in button.pin_list): 
                template = gbutton.press_template if state else gbutton.release_template
                try:
                    self._gcode.run_script(template.render())
                    logger.debug(f"{self._name}: Executed {'press' if state else 'release'} GCode for button {oid}")
                except Exception as e:
                    logging.error(f"{self._name}: Error executing GCode for button {oid}: {e}")
                break # cancel if button matches

    def _handle_counter_state(self, params):
        oid = int(params['oid'])
        freq = float(params['freq'])
        if oid not in self._freq_counter_instances:
            logging.warning(f"{self._name}: Received counter_state for unknown oid {oid}")
            return
        
        # Set the frequency directly, as the MCU (must) return(s) the frequency directly.
        freq_counter: FrequencyCounter = self._freq_counter_instances[oid]
        freq_counter._freq = freq

    def _mcu_identify(self):

        logging.info(f"{self._name}: Establishing connection...")
        try:
            self._comm.connect()
            logging.info(f"{self._name}: Connected to Arduino on {self._port}.")

            # register Arduino_mcu as 'mcu' to show it in SystemLoad panel
            mcu_name = 'mcu ' + self._name
            self._printer.add_object(mcu_name, self)

            # register ArduinoMCU in self.all_mcus so that check_active is called. 
            toolhead: ToolHead = cast("ToolHead", self._printer.lookup_object('toolhead'))
            toolhead.all_mcus.append(self)
            
        except Exception as e:
            self._respond_error(f"{self._name}: Failed to connect to Arduino: {e}\n check the connection to the MCU ", True)

    def get_status(self, eventtime):
        status = {
            'mcu_version': self.get_constant('version'),
            #'mcu_build_versions': 'gcc: 10.2.1 binutils: 2.35.2',
            'mcu_constants': {
                'CLOCK_FREQ': self.get_constant('freq'),
                'MCU': self.get_constant('chip'),
                #'ADC_MAX': 1023,
                #'PWM_MAX': 255,
            },
            'last_stats': {
            #    'mcu_awake': 1,
            #    'mcu_task_avg': 0.0001,
            #    'mcu_task_stddev': 0.0001,
            #    'bytes_write': 5000,
            #    'bytes_read': 6000,
            #    'bytes_retransmit': 0,
            #    'bytes_invalid': 0,
                'freq': self.get_constant('freq'),
            },
            'pins': {
                name: pin.get_status(eventtime)
                    for name, pin in self._pins.items()
            }
        }
        #logging.debug(f"get_status() is called {status}")
        return status
    
    def _respond_error(self, msg:str , exception=False):
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
        """Connect to the Arduino"""

        self.store_frequency_counter_instances()
        self.store_button_instances()         
        
        self._send_config()
        
    def _disconnect(self):
        """Disconnect from the Arduino."""
        logging.info(f"{self._name}: Closing connection...")
        self._comm.disconnect()
        logging.info(f"{self._name}: Disconnected from.")

    def _firmware_restart(self):
        # Attempt reset via reset command
        logging.info("%s: Attempting Arduino MCU reset command", self._name)
        self._comm.send_message("restart")
        self._reactor.pause(self._reactor.monotonic() + 0.015)
        self._disconnect()

    def check_active(self, print_time, eventtime):
        """
        Checks whether the Arduino MCU is still active.
        If there is no response, Klipper is stopped.
        """
        if self._is_timeout:  # If a timeout has already been detected, do nothing
            return
        
        elapsed_time = self._reactor.monotonic() - self._comm.last_response_time
        timeout = 6  # Timeout after 6 seconds

        if elapsed_time > timeout:
            logging.error(f"{self._name}: Watchdog timeout! No response from the MCU.")
            
            self._comm.disconnect()
            
            # Prevent `check_active()` from being triggered multiple times
            self._is_timeout = True
            
            self._printer.invoke_shutdown(f"Lost communication with MCU '{self._name}'")

    cmd_SEND_ARDUINO_help = f"Sends a command to the target arduino_mcu."
    def cmd_SEND_ARDUINO(self, gcmd):
        """Sends a command to the target MCU."""
        cmd = gcmd.get('COMMAND', '')
        if not cmd:
            self._respond_error("COMMAND parameter is required", True)
                
        # Send the command to the Arduino
        self._comm.send_message(cmd)
    
    def handle_error(self, data):
        """Handle error messages from the Arduino."""
        self._respond_error(f"Error from Arduino: {data}")
        
        
class ArduinoMCUPin:
    def __init__(self, mcu: ArduinoMCU, pin_params: dict):
        self._mcu: ArduinoMCU = mcu
        self._name: str = pin_params['pin']
        self._chip_name: str = pin_params['chip_name']
        self._pullup: int = pin_params['pullup']
        self._invert: int = pin_params['invert']
        self._oid = None
        self._value: float = self._pullup
        printer: Printer = self._mcu.get_printer()
        self._real_mcu = printer.lookup_object('mcu')

        # Get sensor_type from the configuration
        configfile: Any = printer.lookup_object('configfile')
        config_data: dict[str, dict[str, Any]] = configfile.get_status(0).get("config", {})
        for section, params in config_data.items():
            section: str
            params: dict[str, Any]
            if section.startswith("temperature_sensor") and params.get("sensor_pin", "") == f"{self._chip_name}:{self._name}":
                self._sensor_type = params.get("sensor_type", "Unknown")
                break

        
    def get_mcu(self):
        return self._real_mcu
    
class ArduinoMCU_digital_out(ArduinoMCUPin):
    def __init__(self, mcu, pin_params):
        ArduinoMCUPin.__init__(self, mcu, pin_params)
        self._mcu.register_config_callback(self._build_pin_config)

    def setup_max_duration(self, max_duration):
        pass

    def setup_start_value(self, start_value, shutdown_value):
        self._value = start_value

    def set_digital(self, print_time, value):
        """Set the pin value and send the command to Arduino MCU."""
        self._value = value
        command = f"set_pin oid={self._oid} value={int(value)}"
        self._mcu._comm.send_message(command)
        logging.debug(f"sends to {self._mcu._name}: {command}")

    def _build_pin_config(self):
        """ Sends the configuration for the digital pin to the Arduino MCU """
        self._oid = self._mcu.create_oid()
        command = f"config_digital_out oid={self._oid} pin={self._name} pullup={self._pullup} invert={self._invert}"
        self._mcu.add_config_cmd(command)
        logging.debug(f"{self._mcu._name}: Build config: {command}")

    def get_status(self, eventtime):
        return {
            'value': self._value,
            'type': 'digital_out'
        }

class ArduinoMCU_pwm(ArduinoMCUPin):
    def __init__(self, mcu, pin_params):
        ArduinoMCUPin.__init__(self, mcu, pin_params)
        self._cycle_time = 0.100
        self._mcu.register_config_callback(self._build_pin_config)

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
        command = f"set_pin oid={self._oid} value={value}"
        self._mcu._comm.send_message(command)
        logging.debug(f"sends to {self._mcu._name}: {command}")

    def _build_pin_config(self):
        """ Sends the configuration for the pwm pin to the Arduino MCU """
        self._oid = self._mcu.create_oid()
        command = f"config_pwm_out oid={self._oid} pin={self._name} cycle_time={self._cycle_time} start_value={self._value}"
        self._mcu.add_config_cmd(command)
        logging.debug(f"{self._mcu._name}: Build config: {command}")

    def get_status(self, eventtime):
        return {
            'value': self._value,
            'type': 'pwm'
        }

REPORT_TIME_TEMP = 2
REPORT_TIME_HUMIDITY = 5 
REPORT_TIME_LIGHT = 1
REPORT_TIME_AIR_QUALITY = 10
REPORT_TIME_PRESSURE = 5
REPORT_TIME_FAN_SPEED = 0.5
REPORT_TIME_DEFAULT = DEFAULT_POLL_TIMEOUT * 2

class ArduinoMCU_adc(ArduinoMCUPin):
    def __init__(self, mcu, pin_params):
        ArduinoMCUPin.__init__(self, mcu, pin_params)
        self._callback = None
        self._number_of_values = 1 # default is one value, 4 are currently supported,
                                   # The sensor values received are interpreted as follows.
                                   # value = temperature
                                   # value2 = humidity
                                   # value3 = pressure
                                   # value4 = gas
        self._min_sample = 0.
        self._max_sample = 0.
        self._sample_time = 0
        self._report_time = 0. # Klipper report time for the respective adc sensor
        self._min_report_time = REPORT_TIME_DEFAULT # minimum reporting time for the respective adc sensor of the arduino mcu
                                                    # This defines the poll timout for _read_data() at the end
        self._adc_sample_count = 0
        self._mcu_sample_count = 0
        self._range_check_count = 0
        self._last_state = (0., 0.)
        self._inv_max_adc = 0.
        self._mcu.register_config_callback(self._build_pin_config)

    def setup_adc_callback(self, report_time, callback):
        self._report_time = report_time
        self._callback = callback

    def setup_adc_sample(self, sample_time, sample_count,
                         minval=0., maxval=1., range_check_count=0):
        self._sample_time = sample_time
        self._adc_sample_count = sample_count
        self._min_sample = minval
        self._max_sample = maxval
        self._range_check_count = range_check_count
    
    def _build_pin_config(self):
        """ build the configuration for the analog input pin for the Arduino MCU """
        self._oid = self._mcu.create_oid()
        self._mcu_sample_count = self._mcu.get_constant('adc_sample_count', parser=float)
        mcu_adc_max = self._mcu.get_constant('adc_max', parser=float)
        max_adc = self._mcu_sample_count * mcu_adc_max
        self._inv_max_adc = 1.0 / max_adc
        command = f"config_analog_in oid={self._oid} pin={self._name} nval={self._number_of_values}"
        self._mcu.add_config_cmd(command)
        logging.debug(f"{self._mcu._name}: Build config: {command}")
        self._mcu.register_response(self._handle_analog_in_state,
                                    "analog_in_state", self._oid)
    def _handle_analog_in_state(self, params):
        reactor: Reactor = self._mcu.get_printer().get_reactor()
        last_value = int(params['value']) * self._inv_max_adc 
        last_read_time = reactor.monotonic()
        self._last_state = (last_value, last_read_time)
        self._value = last_value
        if self._callback is not None:
            self._callback(last_read_time, last_value)        

    def get_status(self, eventtime):
        return {
            'value': self._value,
            'type': 'adc'
        }
                
def load_config_prefix(config):
    return ArduinoMCU(config)