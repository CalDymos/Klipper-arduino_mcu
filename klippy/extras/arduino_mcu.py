import logging
import threading
import serial
import time
import reactor
from queue import Queue, Empty
import socket

class NetworkHandler:
    def __init__(self, reactor, mcu_name, hostIp=None, port=45800):
        """
        Initializes the Network handler for TCP communication.

        Args:
            reactor (reactor): Event reactor instance (reactor.py).
            mcu_name (str): Name of the MCU to identify.
            hostIp (str): IP address to bind the server. (default: current IP)
            port (int): Port number for the server to listen on. (default: 45800)
        """
        self.reactor = reactor
        self._mcu_name = mcu_name
        
        # Get the current IP if hostIp is not specified
        self.hostIp = hostIp or self._get_local_ip()
		
        # Validate and set the port
        self.port = self._validate_port(port)

        self.server_socket = None
        self.client_socket = None
        self.client_address = None
        self.lock = threading.Lock()
        self.response_handlers = {}
        self.running = False
        self.read_thread = None
        self.write_thread = None
        self.process_incoming_thread = None

        # Queues for incoming and outgoing messages
        self.incoming_queue = Queue()
        self.outgoing_queue = Queue()

        self.last_notify_id = 0
        self.pending_notifications = {}
        
        self._init_cmd = self._create_cmd("identify")
        
    def _get_local_ip(self):
        """Gets the local IP address of the current machine."""
        try:
            return socket.gethostbyname(socket.gethostname())
        except Exception as e:
            logging.error(f"Failed to get local IP address: {e}")
            return "127.0.0.1"

    def _validate_port(self, port):
        """
        Validates the port to ensure it does not conflict with common services.

        Args:
            port (int): The requested port number.

        Returns:
            int: A valid port number.
        """
        # List of standard ports to avoid
        reserved_ports = {1883, 5000, 7125, 8080, 8819, 8883}

        if port in reserved_ports or port < 1024 or port > 65535:
            logging.warning(f"Port {port} is reserved or invalid. Assigning a random free port.")
            return 45800  # use default Port

        return port
    
    def _create_cmd(self, command):
        """
        Creates a properly formatted command with checksum and newline.

        Args:
            command (str): The base command string to send.

        Returns:
            str: The full command string with checksum and newline appended.
        """
        checksum = self._calculate_checksum(command)
        full_command = f"{command}*{checksum}\n".encode('utf-8')
        return full_command

    def _calculate_checksum(self, command):
        """Calculates a simple checksum for data integrity."""
        return sum(command.encode('utf-8')) % 256
    
    def _is_connection_ready(self):
        """
        Checks if the connection is ready by sending an identification command.

        Returns:
            bool: True if the connection is ready; False otherwise.
        """
        try:
            self.client_socket.sendall(self._init_cmd)
            logging.debug(f"Sent init command: {self._init_cmd.decode('utf-8').strip()}")

            data = self.client_socket.recv(1024).decode('utf-8').strip()
            if "*" in data:
                msg, crc = data.rsplit("*", 1)
                if int(crc) == int(self._calculate_checksum(msg)):
                    if msg == f"mcu={self._mcu_name}":
                        return True
            return False
        except Exception as e:
            logging.error(f"Error while checking connection readiness: {e}")
            return False

    def connect(self):
        """Starts the TCP server and listens for incoming connections."""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.bind((self.hostIp, self.port))
            self.server_socket.listen(1)  # Accept one client at a time
            logging.info(f"Server started on {self.hostIp}:{self.port}")

            # Accept a client connection
            self.client_socket, self.client_address = self.server_socket.accept()
            logging.info(f"Client connected from {self.client_address}")
            
            # Check if the connection is ready and the MCU identifies itself correctly
            if not self._is_connection_ready():
                self.client_socket.close()
                raise TimeoutError("Connection not ready.")

            self.running = True
            
            # Start threads for communication            
            self.read_thread = threading.Thread(target=self._read_thread)
            self.write_thread = threading.Thread(target=self._write_thread)
            self.process_incoming_thread = threading.Thread(target=self._process_incoming_thread)
            self.read_thread.start()
            self.write_thread.start()
            self.process_incoming_thread.start()

        except Exception as e:
            logging.error(f"Failed to start server: {e}")
            raise

    def disconnect(self):
        """Stops the server and disconnects the client."""
        self.running = False
        if self.read_thread:
            self.read_thread.join()
        if self.write_thread:
            self.write_thread.join()
        if self.process_incoming_thread:
            self.process_incoming_thread.join()
        if self.client_socket:
            self.client_socket.close()
            self.client_socket = None
        if self.server_socket:
            self.server_socket.close()
            self.server_socket = None
        logging.info("Server stopped.")

    def _read_thread(self):
        """Reads data from the network socket and puts it into the incoming queue."""
        while self.running:
            try:
                data = self.client_socket.recv(1024).decode('utf-8').strip()
                if data:
                    logging.debug(f"Received raw message: {data}")
                    self.incoming_queue.put(data)
            except socket.timeout:
                continue
            except Exception as e:
                logging.error(f"Error reading from socket: {e}")
                self.running = False

    def _process_incoming_thread(self):
        """Processes messages from the incoming queue."""
        while self.running:
            try:
                line = self.incoming_queue.get(timeout=0.1)
                logging.debug(f"Processing message: {line}")

                if "*" in line:
                    msg, crc = line.rsplit("*", 1)
                    if int(crc) == self._calculate_checksum(msg):
                        self._handle_message(msg)
                    else:
                        logging.error(f"CRC mismatch for message: {msg}")
            except Empty:
                pass
            except Exception as e:
                logging.error(f"Error processing message: {e}")

    def _write_thread(self):
        """Sends data from the outgoing queue over the network connection."""
        while self.running:
            try:
                message = self.outgoing_queue.get(timeout=0.1)
                full_msg = self._create_cmd(message)
                self.client_socket.sendall(full_msg)
                logging.debug(f"Sent message: {full_msg.strip()}")
            except Empty:
                pass
            except Exception as e:
                logging.error(f"Error sending message: {e}")

    def _handle_message(self, message):
        """Processes incoming messages and dispatches them to the appropriate handler."""
        logging.debug(f"Received message: {message}")
        with self.lock:
            parts = message.split(' ', 1)
            cmd = parts[0]
            data = parts[1] if len(parts) > 1 else ""

            handler = self.response_handlers.get(cmd, self._default_handler)
            handler(data)

    def send_message(self, command):
        """Sends a command over the network connection."""
        self.outgoing_queue.put(command)
        logging.debug(f"Queued message: {command}")

    def register_response(self, handler, command):
        """Registers a handler for a specific command."""
        with self.lock:
            self.response_handlers[command] = handler

    def _default_handler(self, data):
        """Default handler for unrecognized commands."""
        logging.warning(f"Unhandled command: {data}")


class SerialHandler:
    def __init__(self, reactor, mcu_name):
        """
        Initializes the serial handler for UART communication.

        Args:
            reactor (reactor): Event reactor instance (reactor.py)
        """
        self.reactor = reactor
        self.port = ''
        self.baudrate = 115200
        self._mcu_name = mcu_name
                
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
        
        self._init_cmd = self._create_cmd("identify")

    def _create_cmd(self, command):
        """
        Creates a properly formatted (utf-8) command with checksum and newline.

        Args:
            command (str): The base command string to send.

        Returns:
            str: The full command string with checksum and newline appended.
        """
        checksum = self._calculate_checksum(command)
        full_command = f"{command}*{checksum}\n".encode('utf-8')
        return full_command

    def _is_connection_ready(self):
        """
        Checks if the connection is ready by attempting to read data from the serial port.

        Returns:
            bool: True if data is received, indicating the connection is ready; False otherwise.
        """
        try:

            self.serial_dev.write(self._init_cmd)
            logging.debug(f"Sent init command: {self._init_cmd.decode('utf-8').strip()}")

            # Check if there is any data available in the serial input buffer
            if self.serial_dev.in_waiting > 0:
                line = self.serial_dev.readline().decode('utf-8').strip()
                logging.debug(f"Received during readiness check: {line}")
                
                # Parse the message and CRC
                if "*" in line:
                    msg, crc = line.rsplit("*", 1)
                    if int(crc) == int(self._calculate_checksum(msg)): 
                        if msg == f"mcu={self._mcu_name}":
                            self.reactor.pause(self.reactor.monotonic() + 0.1)
                            self.serial_dev.reset_input_buffer()  # clear buffer
                            return True

            # No data received, connection is not ready yet
            return False

        except Exception as e:
            # Log the error if any issue occurs during the readiness check
            logging.error(f"Error while checking connection readiness: {e}")
            return False
    
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

            # Test whether the connection is ready 
            start_time = self.reactor.monotonic()
            timeout = 5  # Waiting time for the connection to be ready

            while self.reactor.monotonic() - start_time < timeout:
                if self._is_connection_ready():
                    logging.info("Serial connection is ready.")
                    self.running = True
                    break
                self.reactor.pause(self.reactor.monotonic() + 0.1)  # simple dirty solutions to avoid blocking the main thread completely

            if not self.running:
                self.serial_dev.close()
                raise TimeoutError("Connection not ready within timeout period.")
            
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

    # Temporärer Logger mit Zeitstempel
    def _log_debug(self, message):
        if logging.getLogger().level == logging.DEBUG:
            current_time = time.time()
            local_time = time.localtime(current_time)
            milliseconds = int((current_time - int(current_time)) * 1000)
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S', local_time)
            logging.debug(f"{timestamp}.{milliseconds} {message}")


    def _read_thread(self):
        """
        Reads serial data and stores it in the incoming queue.
        """
        while self.running:
            try:
                if self.serial_dev.in_waiting > 0:
                    # Read a line from the serial device
                    line = self.serial_dev.readline().decode('utf-8').strip()
                    self._log_debug(f"Received raw message: {line}")
                    
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
                self._log_debug(f"Processing message: {line}")

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
                    if "nid=" in msg:
                        parts = msg.split("nid=")
                        if len(parts) > 1:
                            try:
                                nid = int(parts[1])
                                if nid in self.pending_notifications:
                                    # Complete the associated notification
                                    completion = self.pending_notifications.pop(nid)
                                    self.reactor.async_complete(completion, line)
                                    continue
                            except ValueError:
                                logging.error(f"Invalid nid in message: {msg}")
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
                    full_msg = self._create_cmd(message) 
                    self.serial_dev.write(full_msg)
                    self._log_debug(f"Sent message: {full_msg.strip()}")
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
        self._log_debug(f"Received message: {message}")
        with self.lock:
            parts = message.split(' ', 1)
            cmd = parts[0]
            if cmd.endswith(":"):
                cmd = cmd[:-1]
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
        return sum(command.encode('utf-8')) % 256

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
                full_msg = self._create_cmd(f"{command} nid={nid}")
                self.serial_dev.write(full_msg)
                self._log_debug(f"Sent message: {full_msg.strip()}")

                # Wait for an ACK response
                completion = self.reactor.completion()
                self.pending_notifications[nid] = completion
                # Calculate the waketime
                waketime = self.reactor.monotonic() + timeout

                response = completion.wait(waketime=waketime, waketime_result=None) # wait for response
                if response is None:
                    del self.pending_notifications[nid]  # Cleanup pending notification
                    raise TimeoutError(f"No ACK received for: {command}")

                return response
            except Exception as e:
                logging.error(f"Error sending message: {e}")
                raise

        elif wait_for_response:
            retries = 5
            retry_delay = 0.01
            while retries > 0:
                try:
                    full_msg = self._create_cmd(command)
                    self.serial_dev.write(full_msg)
                    self._log_debug(f"Sent message: {full_msg.strip()}")

                    # Wait for a response
                    start_time = time.time()
                    while time.time() - start_time < retry_delay * 2:
                        try:
                            response = self.incoming_queue.get(timeout=retry_delay)
                            if response:
                                self._log_debug(f"Response received: {response}")
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
            self._log_debug(f"Queued message: {command}")

    def register_response(self, handler, command, oid=None):
        """
        Registers a handler for a specific command.

        Args:
            handler (callable): The function to handle the command.
            command (str): The command to handle.
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
        self._commType = config.get('comm', 'serial')
        if self._commType == 'lan':
            self._port = config.getint('port', 0) # Default port for network communication
            self._ip = config.get('ip', None)
        else:
            self._commType = 'serial'
            self._port = config.get('port', '/dev/ttyUSB0') # Default port for serial
            self._baudrate = config.getint('baud', 115200) # Default baudrate
            
        # vars for mcu stats
        self._mcu_version = "Unknown"
        self._mcu_freq = 0
        self._mcu_chip = 'Unknown'       
        
        self._debuglevel = config.get('debug', False)
        if self._debuglevel:
            logging.getLogger().setLevel(logging.DEBUG) 

        # Store captured configuration commands for buttons
        self._button_config_commands = []  # List to track button configuration commands
        self._buttons_config_complete = False  # Flag to indicate when config is ready

        # init Arduino MCU
        ppins = self._printer.lookup_object('pins')
        ppins.register_chip(f"{self._name}_pin", self) # registers the mcu_name + the suffix '_pin' as a new chip, 
                                                       # so the pin must be specified in the config with {mcu_name}_pin:{pin_name}.
        self._pins = {}
        self._oid_count = 0
        self._config_callbacks = []
        
        # Initialize communication
        if self._commType == 'serial':
            self._comm = SerialHandler(self._reactor, self._name)
        elif self._commType == 'lan':
            self._comm = NetworkHandler(self._reactor, self._name, self._ip, self._port)
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
        logging.debug(f"setup_pin for pin_type: {pin_type}")
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
        logging.debug(f"Registering config callback: {cb}")
        self._config_callbacks.append(cb)

    def _send_buttons_config(self):
        """
        Sends all captured button configuration commands to the Arduino MCU.
        """
        if not self._buttons_config_complete:
            logging.warning("Button configuration is not yet complete.")
            return

        logging.debug("Sending button configuration commands to Arduino MCU...")
        for cmd in self._button_config_commands:
            # Parse button_count from command
            parts = cmd.split()
            params = {}
            for part in parts:
                if "=" in part:
                    key, value = part.split("=")
                    params[key] = value
           
            if cmd.startswith("buttons_add"):
                # Extracted parameters
                oid = int(params.get("oid", 0))
                pos = int(params.get("pos", 0))
                pin = params.get("pin", "")
                pull_up = int(params.get("pull_up", 0))
                command = f"config_digital_in oid={oid} pos={pos} pin={pin} pullup={pull_up}"
                self._comm.send_message(command, True)                 

        # Clear the list after sending
        self._button_config_commands.clear()
        self._buttons_config_complete = False  # Reset the flag
        
    def add_config_cmd(self, cmd, is_init=False, on_restart=False):
        """
        Overridden method to capture all button-related configuration commands.
        """
        logging.debug(f"Captured config command: {cmd}")
        # Store commands related to buttons
        if cmd.startswith("config_buttons") or cmd.startswith("buttons_add") or cmd.startswith("buttons_query"):
            self._button_config_commands.append(cmd)

        # Check if the final command ("buttons_query") is captured
        if cmd.startswith("buttons_query"):
            self._buttons_config_complete = True
            self._send_buttons_config()

    def get_query_slot(self, oid):
        return 0

    def seconds_to_clock(self, time):
        return 0

    def get_printer(self):
        return self._printer

    def register_response(self, handler, command, oid=None):
        logging.debug(f"Registering response handler: {command}")
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
        logging.debug("custom handle_button_state")

    def _mcu_identify(self):

        logging.info(f"Establishing {self._name} connection...")
        try:
            self._comm.connect(self._port, self._baudrate)
            logging.info(f"Connected to Arduino on {self._port}.")

            # register Arduino_mcu as 'mcu' to show it in SystemLoad panel
            mcu_name = 'mcu ' + self._name
            self._printer.add_object(mcu_name, self)
            
        except Exception as e:
            self._respond_error(f"Failed to connect to Arduino: {e}\n check the connection to the MCU ", True)

    def get_status(self, eventtime):
        status = {
            'mcu_version': self._mcu_version,
            #'mcu_build_versions': 'gcc: 10.2.1 binutils: 2.35.2',
            'mcu_constants': {
                'CLOCK_FREQ': self._mcu_freq,
                'MCU': self._mcu_chip,
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
                'freq': self._mcu_freq,
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
        for cb in self._config_callbacks:
            cb()

    def _disconnect(self):
        """Disconnect from the Arduino."""
        logging.info(f"Closing {self._name} connection...")
        self._comm.disconnect()
        logging.info(f"Disconnected from {self._name}.")

    def _firmware_restart(self):
        # Attempt reset via reset command
        logging.info("Attempting Arduino MCU '%s' reset command", self._name)
        self._comm.send_message("restart")
        self._reactor.pause(self._reactor.monotonic() + 0.015)
        self._disconnect()

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
        command = f"set_pin pin={self._name} value={int(value)}"
        self._mcu._comm.send_message(command)
        logging.debug(f"sends to {self._mcu._name}: {command}")

    def _send_pin_config(self):
        """ Sends the configuration for the digital pin to the Arduino MCU """
        command = f"config_digital_out pin={self._name} pullup={self._pullup} invert={self._invert}"
        self._mcu._comm.send_message(command, True)
        logging.debug(f"sends to {self._mcu._name}: {command}")

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
        command = f"set_pin pin={self._name} value={value}"
        self._mcu._comm.send_message(command)
        logging.debug(f"sends to {self._mcu._name}: {command}")

    def _send_pin_config(self):
        """ Sends the configuration for the pwm pin to the Arduino MCU """
        command = f"config_pwm_out pin={self._name} cycle_time={self._cycle_time} start_value={self._value}"
        self._mcu._comm.send_message(command, True)
        logging.debug(f"sends to {self._mcu._name}: {command}")

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
        command = f"config_analog_in pin={self._name}"
        self._mcu._comm.send_message(command, True)
        logging.debug(f"sends to {self._mcu._name}: {command}") 

    def _handle_analog_in_state(self, params):
        pass

    def get_status(self, eventtime):
        return {
            'value': self._value,
            'type': 'adc'
        }
                
def load_config_prefix(config):
    return ArduinoMCU(config)