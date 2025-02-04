import logging
import threading
import serial, socket
import time, reactor, select, selectors
from queue import Queue, Empty
from abc import ABC, abstractmethod

class logger:
    """helper class for timestamped debug logging."""

    @staticmethod
    def _log_debug(message):
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
        logger._log_debug(f"{self._mcu_name}: Received data to validate: {data}")
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
        logger._log_debug(f"{self._mcu_name}: Received msg to parse: {msg}")

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
    
class CommunicationHandler(ABC):
    def __init__(self, reactor, mcu_name, mcu_const: dict):
        """
        Base class for communication handlers.

        Args:
            reactor (reactor): Event reactor instance.
            mcu_name (str): Name of the MCU to identify.
            mcu_const (dict) : Dictionary for MCU constants that is received when connecting to the MCU for the first time
        """
        self.reactor = reactor
        self._mcu_name = mcu_name
        
        self.msgparser = MessageParser(self._mcu_name)
        self.partial_data = ""  # Buffer für unvollständige Nachrichten

        # dict for MCU constants
        self._mcu_const = mcu_const

        self.lock = threading.Lock()
        self.response_handlers = {}
        self.running = False

        # Threads
        self.read_thread = None
        self.write_thread = None
        self.write_condition = threading.Condition()
        self.process_incoming_thread = None

        # Watchdog (checked in mcu -> check_active())
        self.last_response_time = 0

        # Queues for incoming and outgoing messages
        self.incoming_queue = Queue()
        self.outgoing_queue = Queue()

        self.last_notify_id = 0
        self.pending_notifications = {}

        self._identify_cmd = self.msgparser.create_cmd("identify")

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
            logger._log_debug(f"{self._mcu_name}: Sent init command: {self._identify_cmd.decode('utf-8').strip()}")

            # Check if there is any data available in the input buffer
            data = self._read_data()
            if data:
                
                # validate CRC and Parse the message
                params = self.msgparser.parse(self.msgparser.validate(data))
                if params:
                    if params.get('mcu', '') == self._mcu_name:
                        self._mcu_const['chip'] = params.get('chip', 'Unknown')
                        self._mcu_const['version'] = params.get('version', 'Unknown')
                        self._mcu_const['freq'] = params.get('freq', 0)
                        self._mcu_const['adc_max'] = params.get('adc_max', 1)
                        self._mcu_const['adc_sample_count'] = params.get('adc_sample_count', 1)
                        self.reactor.pause(self.reactor.monotonic() + 0.1)
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
        while self.running:
            try:
                if not self.running:
                    return
                
                # read data from device
                data = self._read_data()
                if data:
                    logger._log_debug(f"{self._mcu_name}: Received raw message: {data}")

                    # store data in the incoming queue
                    self.incoming_queue.put(data)

            except Exception as e:
                logging.error(f"{self._mcu_name}: Error reading data: {e}")
                self.running = False

    def _write_thread(self):
        """Background thread for Sending data from the outgoing queue."""
        while self.running:
            if not self.running:
                return
            with self.write_condition:
                try:
                    if self.outgoing_queue.empty():
                        self.write_condition.wait()  # Wait until data is available                   

                    # Get the next message to send
                    message = self.outgoing_queue.get()

                    full_msg = self.msgparser.create_cmd(message)
                    self._send_data(full_msg)
                    logger._log_debug(f"{self._mcu_name}: Sent message: {full_msg.strip()}")

                except Exception as e:
                    logging.error(f"{self._mcu_name}: Error sending message: {e}")

    def connect(self):
        """Sets up the connection and starts communication threads."""
        try:
            self._setup_connection()

            # Test whether the connection is ready 
            start_time = self.reactor.monotonic()
            timeout = 5  # Waiting time for the connection to be ready

            while self.reactor.monotonic() - start_time < timeout:
                if self._get_identify_data():
                    logging.info(f"{self._mcu_name}: connection is ready.")
                    self.running = True
                    break
                self.reactor.pause(self.reactor.monotonic() + 0.1)  # simple dirty solutions to avoid blocking the main thread completely

            if not self.running:
                self._cleanup_connection()
                raise TimeoutError(f"{self._mcu_name}: connection not ready within timeout period.")

            # Start threads for reading and writing
            self.read_thread = threading.Thread(target=self._read_thread)
            self.write_thread = threading.Thread(target=self._write_thread)
            self.process_incoming_thread = threading.Thread(target=self._process_incoming_thread)
            self.read_thread.start()
            self.write_thread.start()
            self.process_incoming_thread.start()

            # save last response time for watchdog
            self.last_response_time = self.reactor.monotonic()

        except Exception as e:
            logging.error(f"{self._mcu_name}: Failed to establish connection: {e}")
            raise

    def disconnect(self):
        """Stops communication and cleans up resources."""
        if self.running:
            self.running = False

            # Stop all threads
            for thd in [self.read_thread, self.write_thread, self.process_incoming_thread]:
                if thd and thd.is_alive() and thd != threading.current_thread():
                    thd.join()


            # Clean up the specific connection
            self._cleanup_connection()

    def _process_incoming_thread(self):
        """Processes messages from the incoming queue."""
        while self.running:
            if not self.running:
                return
            try:
                # Get the next message from the incoming queue
                data = self.incoming_queue.get()# Wait for a message in queue
                logger._log_debug(f"{self._mcu_name}: Processing message: {data}")

                # CRC check
                msg = self.msgparser.validate(data)
                if not msg:
                    continue
                # Parse the message
                params = self.msgparser.parse(msg)
                if not params:
                    continue

                # simple check for the watchdog confirmation message
                if "mcu_watchdog" in params:
                    self.last_response_time = self.reactor.monotonic()  # reset Watchdog zurücksetzen
                    logger._log_debug(f"{self._mcu_name}: Watchdog-Reset durch MCU")
                    continue

                # Check for notification ID (NID)
                nid = int(params.get("nid", 0))
                if nid:
                    if nid in self.pending_notifications:
                        # Complete the associated notification
                        completion = self.pending_notifications.pop(nid)
                        self.reactor.async_complete(completion, data)
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
        logger._log_debug(f"{self._mcu_name}: Received message: {msg}")
        with self.lock:
            cmd = self.msgparser.get_cmd(msg)
            params = self.msgparser.parse(msg)
 
            # Check for registered command handlers
            handler = self.response_handlers.get(cmd, self._default_handler)
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
                self.last_notify_id += 1
                nid = self.last_notify_id

                # Create a completion instance for this message
                completion = self.reactor.completion()
                self.pending_notifications[nid] = completion

                # Send the command with the notification ID
                full_msg = self.msgparser.create_cmd(f"{command} nid={nid}")
                self._send_data(full_msg)
                logger._log_debug(f"{self._mcu_name}: Sent message: {full_msg.strip()}")

                # Wait for an ACK response
                completion = self.reactor.completion()
                self.pending_notifications[nid] = completion
                # Calculate the waketime
                waketime = self.reactor.monotonic() + timeout

                response = completion.wait(waketime=waketime, waketime_result=None) # wait for response
                if response is None:
                    del self.pending_notifications[nid]  # Cleanup pending notification
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
                    full_msg = self.msgparser.create_cmd(command)
                    self._send_data(full_msg)
                    logger._log_debug(f"{self._mcu_name}: Sent message: {full_msg.strip()}")

                    # Wait for a response
                    start_time = time.time()
                    while time.time() - start_time < retry_delay * 2:
                        try:
                            response = self.incoming_queue.get(timeout=retry_delay)
                            if response:
                                logger._log_debug(f"{self._mcu_name}: Response received: {response}")
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
            with self.write_condition:
                self.outgoing_queue.put(command)
                self.write_condition.notify()  # Wake up the writing thread
                logger._log_debug(f"{self._mcu_name}: Queued message: {command}")

    def register_response(self, handler, command, oid):
        """
        Registers a handler for a specific command.

        Args:
            handler (callable): The function to handle the command.
            command (str): The command to handle.
        """
        with self.lock:
            self.response_handlers[command] = handler


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
        self.hostIp = hostIp or self._get_local_ip()

        # Validate and set the port
        self.port = self._validate_port(port)

        self.server_socket = None
        self.client_socket = None
        self.client_address = None

        self.selector = selectors.DefaultSelector()

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
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.hostIp, self.port))
        self.server_socket.listen(1) # Accept one client at a time
        logging.info(f"{self._mcu_name}: Server started on {self.hostIp}:{self.port}")

        # Accept a client connection
        self.client_socket, self.client_address = self.server_socket.accept()
        logging.info(f"{self._mcu_name}: Client connected from {self.client_address}")

        self.selector.register(self.client_socket, selectors.EVENT_READ)

    def _cleanup_connection(self):
        if self.client_socket:
            self.client_socket.close()
            self.client_socket = None
        if self.server_socket:
            self.server_socket.close()
            self.server_socket = None
        logging.info(f"{self._mcu_name}: Server stopped.")

    def _read_data(self):
        """Reads network data using polling."""
        if self.client_socket:
            try:
                events = self.selector.select(timeout=1.0)
                if events:
                    # Receive new data
                    new_data = self.client_socket.recv(1024).decode("utf-8")
                    if not new_data:
                        return ""

                    # Extend buffer
                    self.partial_data += new_data

                    # Check whether a complete message is available
                    if "\n" in self.partial_data:
                        lines = self.partial_data.split("\n")
                        complete_msg = lines[0]  # First complete line
                        self.partial_data = "\n".join(lines[1:])  # Rest back into the buffer

                        return complete_msg.strip()  # Return the complete message
                
                return ""  # No complete message yet, keep waiting
            except Exception as e:
                logging.error(f"{self._mcu_name}: Error when reading the data: {e}")
                return ""

    def _send_data(self, data):
        if self.client_socket is not None:
            self.client_socket.sendall(data)
    
    def _reset_device_input_buffer(self):
        """Clears the input buffer by reading all available data."""
        if self.client_socket:
            try:
                blockingMode = self.client_socket.getblocking()
                self.client_socket.setblocking(False)  # Nicht blockierender Modus
                while True:
                    data = self.client_socket.recv(8192)
                    if not data:
                        break  # Keine weiteren Daten mehr
            except BlockingIOError:
                pass  # Kein Fehler, wenn keine Daten mehr vorhanden sind
            finally:
                self.client_socket.setblocking(blockingMode)  # Ursprünglichen Modus wiederherstellen        


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
        self.port = port
        self.baudrate = baudrate
        self.serial_dev = None
        self.poll_obj = select.poll()

    def _setup_connection(self):
        start_time = self.reactor.monotonic()
        while 1:
            try:
                if self.reactor.monotonic() > start_time + 90.:
                    raise RuntimeError("Unable to connect")

                self.serial_dev = serial.Serial(self.port, self.baudrate, timeout=0, exclusive=True)

                # Registers the file descriptor of the serial interface with the poll object.
                self.poll_obj.register(self.serial_dev.fileno(), select.POLLIN)       

            except (OSError, IOError, serial.SerialException) as e:
                logging.warning("%s: Unable to open serial port: %s",
                             self._mcu_name, e)
                self.reactor.pause(self.reactor.monotonic() + 5.)
                continue

            logging.info(f"{self._mcu_name}: Connected to {self.port} at {self.baudrate} baud.")
            break
    def _cleanup_connection(self):
        if self.serial_dev:
            self.serial_dev.close()
            self.serial_dev = None

    def _read_data(self):
        """Reads complete lines from the serial port using polling"""
        if self.serial_dev:
            try:
                events = self.poll_obj.poll(1000) # Timout 1000ms
                if events:
                    new_data = self.serial_dev.read(self.serial_dev.in_waiting).decode("utf-8")
                    self.partial_data += new_data

                    if "\n" in self.partial_data:
                        lines = self.partial_data.split("\n")
                        complete_msg = lines[0]  # Erste vollständige Zeile
                        self.partial_data = "\n".join(lines[1:])  # Rest in Buffer speichern
                        return complete_msg.strip()

                return ""  # No complete line yet
            
            except (OSError, IOError, serial.SerialException) as e:
                logging.error(f"{self._mcu_name}: Error reading from the serial port: {e}")
                self.disconnect()  # Sicherstellen, dass die Verbindung korrekt getrennt wird
                return ""

    def _send_data(self, data):
        if self.serial_dev is not None:
            self.serial_dev.write(data)
    
    def _reset_device_input_buffer(self):
        if self.serial_dev is not None:
            self.serial_dev.reset_input_buffer()

class ArduinoMCU:
    def __init__(self, config):
        self._printer = config.get_printer()
        self._reactor = self._printer.get_reactor()
        self._name = config.get_name().split()[-1]
        self._gcode = self._printer.lookup_object('gcode')
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

        self._is_shutdown = self._is_timeout = False
        
        # init Arduino MCU
        ppins = self._printer.lookup_object('pins')
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

    def setup_pin(self, pin_type, pin_params):
        ppins = self._printer.lookup_object('pins')
        name = pin_params['pin']
        logging.debug(f"{self._name}: setup_pin for pin_type: {pin_type}")
        if name in self._pins:
            return self._pins[name]
        if pin_type == 'digital_out':
            pin = ArduinoMCU_digital_out(self, pin_params)
        elif pin_type == 'pwm':
            pin = ArduinoMCU_pwm(self, pin_params)
        elif pin_type == 'adc':
            pin = ArduinoMCU_adc(self, pin_params)
        else:
            raise ppins.error("arduino_mcu does not support pin of type %s" % (
                pin_type,))
        self._pins[name] = pin
        return pin

    def create_oid(self):
        self._oid_count += 1
        return self._oid_count - 1

    def register_config_callback(self, cb):
        logging.debug(f"{self._name}: Registering config callback: {cb}")
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

        logging.debug(f"{self._name}: Sending configuration messages to Arduino MCU...")
        for msg in self._config_messages:
            # Parse command and params from message
            self._comm.send_message(msg, True)                 

        # Clear the list after sending
        self._config_messages.clear()
        
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        """
        store all configuration commands.
        """
        logging.debug(f"{self._name}: config command: {cmd}")
        # Capture and save all relevant commands that relate to buttons (captured by buttons.py)
        if cmd.startswith("buttons_add"):
            params = self._comm.msgparser.parse(cmd)
            if params:
                oid = int(params.get("oid", 0))
                pos = int(params.get("pos", 0))
                pin = params.get("pin", "")
                pull_up = int(params.get("pull_up", 0))
                snd_msg = f"config_digital_in oid={oid} pos={pos} pin={pin} pullup={pull_up}"
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

    def register_response(self, handler, command, oid=None):
        logging.debug(f"{self._name}: Registering response handler: {command}")
        # Check if we need to replace the 'button_state' handler with the custom one
        if command == "buttons_state":
            # Replace with custom handler
            self._comm.register_response(self._handle_buttons_state, command, oid)
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
        logging.debug(f"{self._name}: custom handle_button_state")

    def _mcu_identify(self):

        logging.info(f"{self._name}: Establishing connection...")
        try:
            self._comm.connect()
            logging.info(f"{self._name}: Connected to Arduino on {self._port}.")

            # register Arduino_mcu as 'mcu' to show it in SystemLoad panel
            mcu_name = 'mcu ' + self._name
            self._printer.add_object(mcu_name, self)

            # register ArduinoMCU in self.all_mcus so that check_active is called. 
            toolhead = self._printer.lookup_object('toolhead')
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
        """Connect to the Arduino"""
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
    def __init__(self, mcu: ArduinoMCU, pin_params):
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
        self._mcu.register_config_callback(self._build_pin_config)

    def setup_max_duration(self, max_duration):
        pass

    def setup_start_value(self, start_value, shutdown_value):
        self._value = start_value

    def set_digital(self, print_time, value):
        """Set the pin value and send the command to Arduino MCU."""
        self._value = value
        command = f"set_pin pin={self._name} value={int(value)}"
        self._mcu._comm.send_message(command)
        logging.debug(f"sends to {self._mcu._name}: {command}")

    def _build_pin_config(self):
        """ Sends the configuration for the digital pin to the Arduino MCU """
        command = f"config_digital_out pin={self._name} pullup={self._pullup} invert={self._invert}"
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
        command = f"set_pin pin={self._name} value={value}"
        self._mcu._comm.send_message(command)
        logging.debug(f"sends to {self._mcu._name}: {command}")

    def _build_pin_config(self):
        """ Sends the configuration for the pwm pin to the Arduino MCU """
        command = f"config_pwm_out pin={self._name} cycle_time={self._cycle_time} start_value={self._value}"
        self._mcu.add_config_cmd(command)
        logging.debug(f"{self._mcu._name}: Build config: {command}")

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
        self._mcu_sample_count = self._mcu.get_constant('adc_sample_count', parser=float)
        mcu_adc_max = self._mcu.get_constant('adc_max', parser=float)
        max_adc = self._mcu_sample_count * mcu_adc_max
        self._inv_max_adc = 1.0 / max_adc
        command = f"config_analog_in pin={self._name}"
        self._mcu.add_config_cmd(command)
        logging.debug(f"{self._mcu._name}: Build config: {command}")
        self._mcu.register_response(self._handle_analog_in_state,
                                    "analog_in_state", self._oid)
    def _handle_analog_in_state(self, params):
        reactor = self._mcu.get_printer().get_reactor()
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