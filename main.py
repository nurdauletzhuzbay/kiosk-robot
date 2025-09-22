#!/usr/bin/env python3

import json
import time
import threading
import signal
import sys
import argparse
from datetime import datetime
import logging

# MQTT imports
import paho.mqtt.client as mqtt

# Serial communication for Arduinos
import serial
import serial.tools.list_ports

# Import our servo motor controller
from servo_motor_controller import ServoMotorController

# ========================================================================================
# CONFIGURATION
# ========================================================================================

class Config:
    """Configuration settings for the MQTT servo controller"""
    
    # MQTT Settings
    MQTT_BROKER = "localhost"  # Change to your MQTT broker IP
    MQTT_PORT = 1883
    MQTT_KEEPALIVE = 60
    MQTT_QOS = 1
    
    # Client ID for this RPi (Robot/Servo Controller)
    CLIENT_ID = "kiosk_robot_controller"
    
    # MQTT Topics
    TOPICS = {
        # Robot status publishing
        'robot_status': 'kiosk/robot/status',
        'robot_position': 'kiosk/robot/position',
        'robot_heartbeat': 'kiosk/robot/heartbeat',
        
        # Robot command subscriptions
        'robot_commands': 'kiosk/robot/commands',
        'robot_move': 'kiosk/robot/move',
        'robot_move_vertical': 'kiosk/robot/move_vertical',
        'robot_gripper': 'kiosk/robot/gripper',
        'robot_home': 'kiosk/robot/home',
        'robot_stop': 'kiosk/robot/stop',
        'robot_connect': 'kiosk/robot/connect',
        
        # Storage and retrieval commands
        'robot_store_box': 'kiosk/robot/store_box',
        'robot_retrieve_box': 'kiosk/robot/retrieve_box',
        'robot_move_to_shelf': 'kiosk/robot/move_to_shelf',
        
        # Drone Port Topics
        'drone_ready_to_land': 'kiosk/drone/ready_to_land',
        'drone_port_sequence_start': 'kiosk/drone_port/sequence_start',
        'drone_port_sequence_complete': 'kiosk/drone_port/sequence_complete',
        'box_ready_for_pickup': 'kiosk/drone_port/box_ready',
        'box_retrieved': 'kiosk/robot/box_retrieved',
        'drone_port_status': 'kiosk/drone_port/status',
        
        # Robot operational states for drone coordination
        'robot_ready_for_box': 'kiosk/robot/ready_for_box',
        'robot_at_pickup_position': 'kiosk/robot/at_pickup_position',
        'robot_box_secured': 'kiosk/robot/box_secured',
        'robot_at_delivery_position': 'kiosk/robot/at_delivery_position',
        
        # QR/Pickup point topics
        'qr_scan': 'kiosk/pickup/qr_scan',
        'pickup_request': 'kiosk/pickup/request',
        'pickup_ready': 'kiosk/pickup/ready',
        'pickup_complete': 'kiosk/pickup/complete',
        
        # System-wide topics
        'system_status': 'kiosk/system/status',
        'drone_status': 'kiosk/drone/status',
        'screen_status': 'kiosk/screen/status',
        'emergency_stop': 'kiosk/emergency/stop'
    }
    
    # Motor Settings
    MOTOR_INTERFACE = "eth0"
    DEFAULT_MOTION_PARAMS = {
        'acceleration': 200000,
        'deceleration': 200000,
        'max_velocity': 100000
    }
    
    # Predefined positions for drone operations (in degrees)
    POSITIONS = {
        # Horizontal positions (Servo 1 - EtherCAT)
        'home': 0,              # Home/rest position
        'pickup': 90,           # Position to pickup box from drone port
        'delivery': 180,        # Position to deliver box to customer pickup point
        'standby': 45,          # Standby position when ready for box
        'shelf_1': 270,         # Shelf 1 position
        'shelf_2': 315,         # Shelf 2 position
        'shelf_3': 225,         # Shelf 3 position
    }
    
    # Vertical positions (Servo 2 - Arduino)
    VERTICAL_POSITIONS = {
        'bottom': 0,            # Ground level / pickup level
        'shelf_1': 30,          # Shelf 1 height
        'shelf_2': 60,          # Shelf 2 height  
        'shelf_3': 90,          # Shelf 3 height
        'top': 120,             # Maximum height
        'delivery_height': 45   # Customer pickup height
    }
    
    # Gripper states (Stepper motors - Arduino)
    GRIPPER_STATES = {
        'open': 0,              # Gripper fully open
        'closed': 1,            # Gripper closed around box
        'partial': 2            # Gripper partially closed
    }
    
    # Arduino communication settings
    ARDUINO_SETTINGS = {
        'servo_arduino': {
            'port': '/dev/ttyUSB0',      # Arduino controlling servo 2 (vertical)
            'baudrate': 9600,
            'timeout': 1
        },
        'gripper_arduino': {
            'port': '/dev/ttyUSB1',      # Arduino controlling gripper steppers
            'baudrate': 9600,
            'timeout': 1
        }
    }
    
    # Status reporting intervals (seconds)
    STATUS_INTERVAL = 2.0
    POSITION_INTERVAL = 1.0
    HEARTBEAT_INTERVAL = 5.0

# ========================================================================================
# MQTT SERVO CONTROLLER CLASS
# ========================================================================================

class MQTTServoController:
    """
    MQTT-enabled servo motor controller for drone kiosk system
    """
    
    def __init__(self, config):
        self.config = config
        self.mqtt_client = None
        self.servo_controller = None
        self.running = False
        
        # Arduino connections
        self.servo_arduino = None      # Arduino controlling vertical servo
        self.gripper_arduino = None    # Arduino controlling gripper steppers
        
        # Status tracking
        self.last_position = None
        self.last_status_time = 0
        self.last_position_time = 0
        self.last_heartbeat_time = 0
        
        # Robot state tracking
        self.horizontal_position = 0.0    # Servo 1 position (degrees)
        self.vertical_position = 0.0      # Servo 2 position (degrees)
        self.gripper_state = "open"       # Gripper state
        self.box_in_gripper = False       # Whether robot is holding a box
        
        # Shelf inventory (simple tracking)
        self.shelf_inventory = {
            'shelf_1': [],          # List of box IDs on shelf 1
            'shelf_2': [],          # List of box IDs on shelf 2  
            'shelf_3': []           # List of box IDs on shelf 3
        }
        
        # Operational state for drone coordination
        self.operational_state = "idle"  # idle, ready_for_box, moving_to_pickup, at_pickup, 
                                        # retrieving_box, storing_box, moving_to_delivery, 
                                        # at_delivery, delivering, moving_to_shelf
        self.box_secured = False
        self.current_box_id = None       # ID of currently handled box
        
        # Setup logging
        self._setup_logging()
        
    def _setup_logging(self):
        """Setup logging configuration"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler('/var/log/kiosk_robot.log'),
                logging.StreamHandler(sys.stdout)
            ]
        )
        self.logger = logging.getLogger('KioskRobot')

    # ========================================================================================
    # MQTT CONNECTION AND CALLBACKS
    # ========================================================================================
    
    def setup_mqtt(self):
        """Initialize and configure MQTT client"""
        self.logger.info("Setting up MQTT connection...")
        
        self.mqtt_client = mqtt.Client(self.config.CLIENT_ID)
        self.mqtt_client.on_connect = self._on_mqtt_connect
        self.mqtt_client.on_disconnect = self._on_mqtt_disconnect
        self.mqtt_client.on_message = self._on_mqtt_message
        
        try:
            self.mqtt_client.connect(self.config.MQTT_BROKER, self.config.MQTT_PORT, self.config.MQTT_KEEPALIVE)
            self.mqtt_client.loop_start()
            return True
        except Exception as e:
            self.logger.error(f"MQTT connection failed: {e}")
            return False

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback for MQTT connection"""
        if rc == 0:
            self.logger.info("Connected to MQTT broker")
            self._subscribe_to_topics()
            self._publish_startup_message()
        else:
            self.logger.error(f"MQTT connection failed with code {rc}")

    def _on_mqtt_disconnect(self, client, userdata, rc):
        """Callback for MQTT disconnection"""
        self.logger.warning(f"Disconnected from MQTT broker (code: {rc})")

    def _subscribe_to_topics(self):
        """Subscribe to all relevant MQTT topics"""
        topics_to_subscribe = [
            # Robot control topics
            self.config.TOPICS['robot_commands'],
            self.config.TOPICS['robot_move'],
            self.config.TOPICS['robot_move_vertical'],
            self.config.TOPICS['robot_gripper'],
            self.config.TOPICS['robot_home'],
            self.config.TOPICS['robot_stop'],
            self.config.TOPICS['robot_connect'],
            self.config.TOPICS['robot_store_box'],
            self.config.TOPICS['robot_retrieve_box'],
            self.config.TOPICS['robot_move_to_shelf'],
            
            # Pickup point topics
            self.config.TOPICS['qr_scan'],
            self.config.TOPICS['pickup_request'],
            
            # Drone coordination topics
            self.config.TOPICS['drone_ready_to_land'],
            self.config.TOPICS['drone_port_sequence_complete'],
            self.config.TOPICS['box_ready_for_pickup'],
            self.config.TOPICS['drone_port_status'],
            
            # System topics
            self.config.TOPICS['emergency_stop']
        ]
        
        for topic in topics_to_subscribe:
            self.mqtt_client.subscribe(topic, self.config.MQTT_QOS)
            self.logger.info(f"Subscribed to: {topic}")

    def _on_mqtt_message(self, client, userdata, msg):
        """Handle incoming MQTT messages"""
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            self.logger.info(f"Received message on {topic}: {payload}")
            
            # Parse JSON payload if possible
            try:
                data = json.loads(payload)
            except json.JSONDecodeError:
                data = {'raw_message': payload}
            
            # Route message to appropriate handler
            self._handle_mqtt_command(topic, data)
            
        except Exception as e:
            self.logger.error(f"Error processing MQTT message: {e}")

    # ========================================================================================
    # COMMAND HANDLERS
    # ========================================================================================
    
    def _handle_mqtt_command(self, topic, data):
        """Route MQTT commands to appropriate handlers"""
        
        # Robot control commands
        if topic == self.config.TOPICS['robot_move']:
            self._handle_move_command(data)
        elif topic == self.config.TOPICS['robot_move_vertical']:
            self._handle_move_vertical_command(data)
        elif topic == self.config.TOPICS['robot_gripper']:
            self._handle_gripper_command(data)
        elif topic == self.config.TOPICS['robot_home']:
            self._handle_home_command(data)
        elif topic == self.config.TOPICS['robot_stop']:
            self._handle_stop_command(data)
        elif topic == self.config.TOPICS['robot_connect']:
            self._handle_connect_command(data)
        elif topic == self.config.TOPICS['robot_commands']:
            self._handle_general_command(data)
        
        # Storage and retrieval commands
        elif topic == self.config.TOPICS['robot_store_box']:
            self._handle_store_box_command(data)
        elif topic == self.config.TOPICS['robot_retrieve_box']:
            self._handle_retrieve_box_command(data)
        elif topic == self.config.TOPICS['robot_move_to_shelf']:
            self._handle_move_to_shelf_command(data)
        
        # Pickup point commands
        elif topic == self.config.TOPICS['qr_scan']:
            self._handle_qr_scan(data)
        elif topic == self.config.TOPICS['pickup_request']:
            self._handle_pickup_request(data)
        
        # Drone coordination commands
        elif topic == self.config.TOPICS['drone_ready_to_land']:
            self._handle_drone_ready_to_land(data)
        elif topic == self.config.TOPICS['drone_port_sequence_complete']:
            self._handle_drone_port_sequence_complete(data)
        elif topic == self.config.TOPICS['box_ready_for_pickup']:
            self._handle_box_ready_for_pickup(data)
        elif topic == self.config.TOPICS['drone_port_status']:
            self._handle_drone_port_status(data)
        
        # System commands
        elif topic == self.config.TOPICS['emergency_stop']:
            self._handle_emergency_stop(data)
        else:
            self.logger.warning(f"Unknown topic: {topic}")

    def _handle_move_command(self, data):
        """Handle move position commands"""
        try:
            if not self.servo_controller or not self.servo_controller.is_initialized:
                self._publish_error("Motor not connected")
                return
            
            # Extract position from command
            degrees = data.get('degrees', data.get('position', 0))
            wait = data.get('wait', False)
            timeout = data.get('timeout', 30.0)
            
            self.logger.info(f"Moving to {degrees} degrees (wait: {wait})")
            
            # Execute move command
            if wait:
                success = self.servo_controller.move_to_position(degrees, True, timeout)
                self._publish_move_result(degrees, success)
            else:
                # Non-blocking move
                threading.Thread(
                    target=self._async_move, 
                    args=(degrees, timeout),
                    daemon=True
                ).start()
                
        except Exception as e:
            self.logger.error(f"Move command failed: {e}")
            self._publish_error(f"Move failed: {str(e)}")

    def _handle_home_command(self, data):
        """Handle home position commands"""
        try:
            if not self.servo_controller or not self.servo_controller.is_initialized:
                self._publish_error("Motor not connected")
                return
            
            self.logger.info("Returning to home position")
            
            # Execute home command in separate thread
            threading.Thread(
                target=self._async_home,
                daemon=True
            ).start()
            
        except Exception as e:
            self.logger.error(f"Home command failed: {e}")
            self._publish_error(f"Home failed: {str(e)}")

    def _handle_stop_command(self, data):
        """Handle stop commands"""
        try:
            if not self.servo_controller or not self.servo_controller.is_initialized:
                return
            
            self.logger.info("Stopping motor")
            success = self.servo_controller.emergency_stop()
            
            self._publish_command_result("stop", success)
            
        except Exception as e:
            self.logger.error(f"Stop command failed: {e}")
            self._publish_error(f"Stop failed: {str(e)}")

    def _handle_connect_command(self, data):
        """Handle motor connection commands"""
        try:
            interface = data.get('interface', self.config.MOTOR_INTERFACE)
            
            if self.servo_controller and self.servo_controller.is_initialized:
                self._publish_error("Motor already connected")
                return
            
            self.logger.info(f"Connecting to motor on {interface}")
            success = self._connect_motor(interface)
            
            self._publish_command_result("connect", success)
            
        except Exception as e:
            self.logger.error(f"Connect command failed: {e}")
            self._publish_error(f"Connect failed: {str(e)}")

    def _handle_general_command(self, data):
        """Handle general commands"""
        try:
            command = data.get('command', '').lower()
            
            if command == 'status':
                self._publish_status_now()
            elif command == 'position':
                self._publish_position_now()
            elif command == 'disconnect':
                self._disconnect_motor()
            elif command == 'set_params':
                self._set_motion_parameters(data.get('params', {}))
            else:
                self.logger.warning(f"Unknown command: {command}")
                
        except Exception as e:
            self.logger.error(f"General command failed: {e}")
            self._publish_error(f"Command failed: {str(e)}")

    def _handle_emergency_stop(self, data):
        """Handle system-wide emergency stop"""
        self.logger.critical("EMERGENCY STOP RECEIVED!")
        try:
            if self.servo_controller and self.servo_controller.is_initialized:
                self.servo_controller.emergency_stop()
            self._publish_emergency_response()
        except Exception as e:
            self.logger.error(f"Emergency stop failed: {e}")

    def _handle_move_vertical_command(self, data):
        """Handle vertical movement commands (Servo 2 - Arduino)"""
        try:
            position = data.get('position', data.get('degrees', 0))
            self.logger.info(f"Moving vertically to {position} degrees")
            
            success = self._move_vertical_servo(position)
            self._publish_command_result("move_vertical", success)
            
        except Exception as e:
            self.logger.error(f"Vertical move command failed: {e}")
            self._publish_error(f"Vertical move failed: {str(e)}")

    def _handle_gripper_command(self, data):
        """Handle gripper control commands (Stepper motors - Arduino)"""
        try:
            action = data.get('action', 'open')  # open, close, partial
            self.logger.info(f"Gripper action: {action}")
            
            success = self._control_gripper(action)
            self._publish_command_result("gripper", success)
            
        except Exception as e:
            self.logger.error(f"Gripper command failed: {e}")
            self._publish_error(f"Gripper control failed: {str(e)}")

    def _handle_store_box_command(self, data):
        """Handle box storage commands"""
        try:
            box_id = data.get('box_id', f'BOX_{int(time.time())}')
            shelf = data.get('shelf', 'shelf_1')
            
            self.logger.info(f"Storing box {box_id} on {shelf}")
            
            # Execute storage sequence asynchronously
            threading.Thread(
                target=self._execute_box_storage_sequence,
                args=(box_id, shelf),
                daemon=True
            ).start()
            
        except Exception as e:
            self.logger.error(f"Store box command failed: {e}")
            self._publish_error(f"Box storage failed: {str(e)}")

    def _handle_retrieve_box_command(self, data):
        """Handle box retrieval commands"""
        try:
            box_id = data.get('box_id')
            if not box_id:
                self._publish_error("Box ID required for retrieval")
                return
            
            self.logger.info(f"Retrieving box {box_id}")
            
            # Find which shelf the box is on
            shelf = self._find_box_shelf(box_id)
            if not shelf:
                self._publish_error(f"Box {box_id} not found in inventory")
                return
            
            # Execute retrieval sequence asynchronously
            threading.Thread(
                target=self._execute_box_retrieval_sequence,
                args=(box_id, shelf),
                daemon=True
            ).start()
            
        except Exception as e:
            self.logger.error(f"Retrieve box command failed: {e}")
            self._publish_error(f"Box retrieval failed: {str(e)}")

    def _handle_move_to_shelf_command(self, data):
        """Handle move to shelf commands"""
        try:
            shelf = data.get('shelf', 'shelf_1')
            
            self.logger.info(f"Moving to {shelf}")
            
            # Execute shelf movement asynchronously
            threading.Thread(
                target=self._move_to_shelf_position,
                args=(shelf,),
                daemon=True
            ).start()
            
        except Exception as e:
            self.logger.error(f"Move to shelf command failed: {e}")
            self._publish_error(f"Move to shelf failed: {str(e)}")

    def _handle_qr_scan(self, data):
        """Handle QR code scan from pickup point"""
        try:
            qr_data = data.get('qr_code', '')
            customer_id = data.get('customer_id', '')
            
            self.logger.info(f"QR scan received: {qr_data}")
            
            # Process QR code to find box
            box_id = self._process_qr_code(qr_data)
            if box_id:
                # Initiate box retrieval for customer
                self._initiate_customer_pickup(box_id, customer_id)
            else:
                self._publish_error("Invalid QR code or box not found")
                
        except Exception as e:
            self.logger.error(f"QR scan handling failed: {e}")
            self._publish_error(f"QR scan failed: {str(e)}")

    def _handle_pickup_request(self, data):
        """Handle customer pickup requests"""
        try:
            box_id = data.get('box_id')
            customer_id = data.get('customer_id')
            
            self.logger.info(f"Pickup request: {box_id} for customer {customer_id}")
            
            self._initiate_customer_pickup(box_id, customer_id)
            
        except Exception as e:
            self.logger.error(f"Pickup request handling failed: {e}")
            self._publish_error(f"Pickup request failed: {str(e)}")

    # ========================================================================================
    # DRONE COORDINATION HANDLERS
    # ========================================================================================
    
    def _handle_drone_ready_to_land(self, data):
        """Handle drone ready to land notification"""
        try:
            self.logger.info("Drone ready to land - preparing robot for box pickup")
            
            # Update operational state
            self.operational_state = "ready_for_box"
            
            # Move to standby position to prepare for box pickup
            if self.servo_controller and self.servo_controller.is_initialized:
                standby_pos = self.config.POSITIONS['standby']
                self.logger.info(f"Moving to standby position ({standby_pos} degrees)")
                
                # Move to standby position asynchronously
                threading.Thread(
                    target=self._async_move_for_drone_operation,
                    args=(standby_pos, "standby"),
                    daemon=True
                ).start()
            
            # Notify that robot is getting ready
            self._publish_robot_operational_state("preparing_for_box")
            
        except Exception as e:
            self.logger.error(f"Handle drone ready to land failed: {e}")
            self._publish_error(f"Drone preparation failed: {str(e)}")

    def _handle_drone_port_sequence_complete(self, data):
        """Handle drone port landing sequence completion"""
        try:
            self.logger.info("Drone port sequence complete")
            
            # Robot should be in standby, ready to receive box pickup notification
            if self.operational_state == "ready_for_box":
                self.logger.info("Robot ready and waiting for box pickup notification")
            else:
                self.logger.warning(f"Robot in unexpected state: {self.operational_state}")
            
        except Exception as e:
            self.logger.error(f"Handle drone port sequence complete failed: {e}")

    def _handle_box_ready_for_pickup(self, data):
        """Handle box ready for pickup notification from drone port"""
        try:
            self.logger.info("Box ready for pickup - initiating pickup sequence")
            
            if not self.servo_controller or not self.servo_controller.is_initialized:
                self._publish_error("Cannot pickup box - motor not connected")
                return
            
            # Update operational state
            self.operational_state = "moving_to_pickup"
            
            # Move to pickup position
            pickup_pos = self.config.POSITIONS['pickup']
            self.logger.info(f"Moving to pickup position ({pickup_pos} degrees)")
            
            # Execute pickup sequence asynchronously
            threading.Thread(
                target=self._execute_box_pickup_sequence,
                daemon=True
            ).start()
            
        except Exception as e:
            self.logger.error(f"Handle box ready for pickup failed: {e}")
            self._publish_error(f"Box pickup failed: {str(e)}")

    def _handle_drone_port_status(self, data):
        """Handle drone port status updates"""
        try:
            port_status = data.get('status', 'unknown')
            self.logger.info(f"Drone port status: {port_status}")
            
            # React to specific drone port statuses if needed
            if port_status == "error":
                self.logger.warning("Drone port reported error")
            elif port_status == "maintenance":
                self.logger.info("Drone port in maintenance mode")
            
        except Exception as e:
            self.logger.error(f"Handle drone port status failed: {e}")

    # ========================================================================================
    # ARDUINO COMMUNICATION METHODS
    # ========================================================================================
    
    def _setup_arduino_connections(self):
        """Setup serial connections to Arduino controllers"""
        try:
            self.logger.info("Setting up Arduino connections...")
            
            # Connect to servo Arduino (vertical movement)
            try:
                servo_port = self.config.ARDUINO_SETTINGS['servo_arduino']['port']
                servo_baud = self.config.ARDUINO_SETTINGS['servo_arduino']['baudrate']
                self.servo_arduino = serial.Serial(servo_port, servo_baud, timeout=1)
                time.sleep(2)  # Wait for Arduino to initialize
                self.logger.info(f"Servo Arduino connected on {servo_port}")
            except Exception as e:
                self.logger.error(f"Failed to connect servo Arduino: {e}")
            
            # Connect to gripper Arduino (stepper motors)
            try:
                gripper_port = self.config.ARDUINO_SETTINGS['gripper_arduino']['port']
                gripper_baud = self.config.ARDUINO_SETTINGS['gripper_arduino']['baudrate']
                self.gripper_arduino = serial.Serial(gripper_port, gripper_baud, timeout=1)
                time.sleep(2)  # Wait for Arduino to initialize
                self.logger.info(f"Gripper Arduino connected on {gripper_port}")
            except Exception as e:
                self.logger.error(f"Failed to connect gripper Arduino: {e}")
                
        except Exception as e:
            self.logger.error(f"Arduino setup failed: {e}")

    def _move_vertical_servo(self, degrees):
        """
        Move vertical servo (Servo 2) via Arduino
        
        Args:
            degrees (float): Target vertical position in degrees
            
        Returns:
            bool: True if successful
        """
        try:
            if not self.servo_arduino:
                self.logger.error("Servo Arduino not connected")
                return False
            
            # Send command to Arduino
            command = f"SERVO2:{degrees}\n"
            self.servo_arduino.write(command.encode())
            
            # Wait for response
            response = self.servo_arduino.readline().decode().strip()
            
            if response == "OK":
                self.vertical_position = degrees
                self.logger.info(f"Vertical servo moved to {degrees} degrees")
                return True
            else:
                self.logger.error(f"Servo Arduino error: {response}")
                return False
                
        except Exception as e:
            self.logger.error(f"Vertical servo move failed: {e}")
            return False

    def _control_gripper(self, action):
        """
        Control gripper via stepper motors (Arduino)
        
        Args:
            action (str): 'open', 'close', or 'partial'
            
        Returns:
            bool: True if successful
        """
        try:
            if not self.gripper_arduino:
                self.logger.error("Gripper Arduino not connected")
                return False
            
            # Map action to gripper state
            if action not in self.config.GRIPPER_STATES:
                self.logger.error(f"Invalid gripper action: {action}")
                return False
            
            # Send command to Arduino
            command = f"GRIPPER:{action.upper()}\n"
            self.gripper_arduino.write(command.encode())
            
            # Wait for response
            response = self.gripper_arduino.readline().decode().strip()
            
            if response == "OK":
                self.gripper_state = action
                if action == "close":
                    self.box_in_gripper = True
                elif action == "open":
                    self.box_in_gripper = False
                    
                self.logger.info(f"Gripper {action} completed")
                return True
            else:
                self.logger.error(f"Gripper Arduino error: {response}")
                return False
                
        except Exception as e:
            self.logger.error(f"Gripper control failed: {e}")
            return False

    def _get_arduino_status(self):
        """
        Get status from both Arduino controllers
        
        Returns:
            dict: Status of both Arduino systems
        """
        status = {
            'servo_arduino_connected': self.servo_arduino is not None,
            'gripper_arduino_connected': self.gripper_arduino is not None,
            'vertical_position': self.vertical_position,
            'gripper_state': self.gripper_state,
            'box_in_gripper': self.box_in_gripper
        }
        
        # Try to get live status from Arduinos
        try:
            if self.servo_arduino:
                self.servo_arduino.write(b"STATUS\n")
                servo_response = self.servo_arduino.readline().decode().strip()
                status['servo_status'] = servo_response
        except:
            status['servo_status'] = 'error'
            
        try:
            if self.gripper_arduino:
                self.gripper_arduino.write(b"STATUS\n")
                gripper_response = self.gripper_arduino.readline().decode().strip()
                status['gripper_status'] = gripper_response
        except:
            status['gripper_status'] = 'error'
            
        return status

    # ========================================================================================
    # ROBOT OPERATION SEQUENCES
    # ========================================================================================
    
    def _execute_box_storage_sequence(self, box_id, shelf):
        """
        Execute complete box storage sequence
        
        Args:
            box_id (str): ID of the box to store
            shelf (str): Target shelf (shelf_1, shelf_2, shelf_3)
        """
        try:
            self.logger.info(f"Starting storage sequence for {box_id} on {shelf}")
            self.operational_state = "storing_box"
            self.current_box_id = box_id
            
            # Step 1: Move to shelf horizontal position
            shelf_horizontal_pos = self.config.POSITIONS[shelf]
            success = self.servo_controller.move_to_position(shelf_horizontal_pos, True, 30.0)
            if not success:
                raise Exception(f"Failed to reach {shelf} horizontal position")
            
            # Step 2: Move to shelf vertical position
            shelf_vertical_pos = self.config.VERTICAL_POSITIONS[shelf]
            success = self._move_vertical_servo(shelf_vertical_pos)
            if not success:
                raise Exception(f"Failed to reach {shelf} vertical position")
            
            # Step 3: Release box (open gripper)
            success = self._control_gripper("open")
            if not success:
                raise Exception("Failed to open gripper")
            
            # Step 4: Move away from shelf (lower vertical position)
            success = self._move_vertical_servo(self.config.VERTICAL_POSITIONS['bottom'])
            if not success:
                raise Exception("Failed to move away from shelf")
            
            # Step 5: Update inventory
            self.shelf_inventory[shelf].append(box_id)
            
            # Step 6: Return to home position
            success = self.servo_controller.move_to_position(self.config.POSITIONS['home'], True, 30.0)
            
            self.operational_state = "idle"
            self.current_box_id = None
            self.box_secured = False
            
            self.logger.info(f"Box {box_id} stored successfully on {shelf}")
            self._publish_storage_complete(box_id, shelf, True)
            
        except Exception as e:
            self.logger.error(f"Storage sequence failed: {e}")
            self.operational_state = "error"
            self._publish_storage_complete(box_id, shelf, False, str(e))

    def _execute_box_retrieval_sequence(self, box_id, shelf):
        """
        Execute complete box retrieval sequence
        
        Args:
            box_id (str): ID of the box to retrieve
            shelf (str): Source shelf
        """
        try:
            self.logger.info(f"Starting retrieval sequence for {box_id} from {shelf}")
            self.operational_state = "retrieving_box"
            self.current_box_id = box_id
            
            # Step 1: Move to shelf horizontal position
            shelf_horizontal_pos = self.config.POSITIONS[shelf]
            success = self.servo_controller.move_to_position(shelf_horizontal_pos, True, 30.0)
            if not success:
                raise Exception(f"Failed to reach {shelf} horizontal position")
            
            # Step 2: Move to shelf vertical position
            shelf_vertical_pos = self.config.VERTICAL_POSITIONS[shelf]
            success = self._move_vertical_servo(shelf_vertical_pos)
            if not success:
                raise Exception(f"Failed to reach {shelf} vertical position")
            
            # Step 3: Grab box (close gripper)
            success = self._control_gripper("close")
            if not success:
                raise Exception("Failed to close gripper")
            
            # Step 4: Move away from shelf
            success = self._move_vertical_servo(self.config.VERTICAL_POSITIONS['bottom'])
            if not success:
                raise Exception("Failed to move away from shelf")
            
            # Step 5: Move to delivery position
            delivery_pos = self.config.POSITIONS['delivery']
            success = self.servo_controller.move_to_position(delivery_pos, True, 30.0)
            if not success:
                raise Exception("Failed to reach delivery position")
            
            # Step 6: Move to delivery height
            delivery_height = self.config.VERTICAL_POSITIONS['delivery_height']
            success = self._move_vertical_servo(delivery_height)
            if not success:
                raise Exception("Failed to reach delivery height")
            
            # Step 7: Release box for customer
            success = self._control_gripper("open")
            if not success:
                raise Exception("Failed to release box")
            
            # Step 8: Update inventory
            if box_id in self.shelf_inventory[shelf]:
                self.shelf_inventory[shelf].remove(box_id)
            
            # Step 9: Return to home
            self._move_vertical_servo(self.config.VERTICAL_POSITIONS['bottom'])
            self.servo_controller.move_to_position(self.config.POSITIONS['home'], True, 30.0)
            
            self.operational_state = "idle"
            self.current_box_id = None
            self.box_secured = False
            
            self.logger.info(f"Box {box_id} delivered successfully")
            self._publish_pickup_ready(box_id)
            
        except Exception as e:
            self.logger.error(f"Retrieval sequence failed: {e}")
            self.operational_state = "error"
            self._publish_error(f"Retrieval failed: {str(e)}")

    def _move_to_shelf_position(self, shelf):
        """
        Move robot to shelf position (both horizontal and vertical)
        
        Args:
            shelf (str): Target shelf
        """
        try:
            # Move horizontally to shelf
            horizontal_pos = self.config.POSITIONS[shelf]
            success = self.servo_controller.move_to_position(horizontal_pos, True, 30.0)
            if not success:
                raise Exception(f"Failed to reach {shelf} horizontal position")
            
            # Move vertically to shelf height
            vertical_pos = self.config.VERTICAL_POSITIONS[shelf]
            success = self._move_vertical_servo(vertical_pos)
            if not success:
                raise Exception(f"Failed to reach {shelf} vertical position")
            
            self.logger.info(f"Robot positioned at {shelf}")
            self._publish_command_result("move_to_shelf", True)
            
        except Exception as e:
            self.logger.error(f"Move to shelf failed: {e}")
            self._publish_command_result("move_to_shelf", False)

    # ========================================================================================
    # INVENTORY AND QR PROCESSING
    # ========================================================================================
    
    def _find_box_shelf(self, box_id):
        """
        Find which shelf contains the specified box
        
        Args:
            box_id (str): Box ID to search for
            
        Returns:
            str: Shelf name or None if not found
        """
        for shelf, boxes in self.shelf_inventory.items():
            if box_id in boxes:
                return shelf
        return None

    def _process_qr_code(self, qr_data):
        """
        Process QR code data to extract box ID
        
        Args:
            qr_data (str): Raw QR code data
            
        Returns:
            str: Box ID or None if invalid
        """
        try:
            # TODO: Implement actual QR processing logic
            # For now, assume QR contains box ID directly
            
            # Example QR format: "BOX:PKG_12345" or just "PKG_12345"
            if "BOX:" in qr_data:
                box_id = qr_data.split("BOX:")[1]
            else:
                box_id = qr_data
            
            # Verify box exists in inventory
            if self._find_box_shelf(box_id):
                return box_id
            else:
                self.logger.warning(f"Box {box_id} not found in inventory")
                return None
                
        except Exception as e:
            self.logger.error(f"QR processing failed: {e}")
            return None

    def _initiate_customer_pickup(self, box_id, customer_id):
        """
        Initiate customer pickup sequence
        
        Args:
            box_id (str): Box ID to retrieve
            customer_id (str): Customer identifier
        """
        try:
            self.logger.info(f"Initiating pickup for {box_id}, customer: {customer_id}")
            
            # Verify box exists
            shelf = self._find_box_shelf(box_id)
            if not shelf:
                self._publish_error(f"Box {box_id} not found in inventory")
                return
            
            # Start retrieval sequence
            threading.Thread(
                target=self._execute_box_retrieval_sequence,
                args=(box_id, shelf),
                daemon=True
            ).start()
            
        except Exception as e:
            self.logger.error(f"Customer pickup initiation failed: {e}")
            self._publish_error(f"Pickup initiation failed: {str(e)}")

    def _get_inventory_status(self):
        """
        Get current inventory status
        
        Returns:
            dict: Complete inventory information
        """
        return {
            'shelf_inventory': self.shelf_inventory,
            'total_boxes': sum(len(boxes) for boxes in self.shelf_inventory.values()),
            'shelf_capacity': {
                'shelf_1': {'current': len(self.shelf_inventory['shelf_1']), 'max': 10},
                'shelf_2': {'current': len(self.shelf_inventory['shelf_2']), 'max': 10},
                'shelf_3': {'current': len(self.shelf_inventory['shelf_3']), 'max': 10}
            }
        }

    # ========================================================================================
    # DRONE OPERATION SEQUENCES
    # ========================================================================================
    
    def _execute_box_pickup_sequence(self):
        """Execute the complete box pickup sequence"""
        try:
            pickup_pos = self.config.POSITIONS['pickup']
            delivery_pos = self.config.POSITIONS['delivery']
            
            # Step 1: Move to pickup position
            self.logger.info("Step 1: Moving to pickup position")
            success = self.servo_controller.move_to_position(pickup_pos, True, 30.0)
            
            if not success:
                raise Exception("Failed to reach pickup position")
            
            self.operational_state = "at_pickup"
            self._publish_robot_operational_state("at_pickup_position")
            
            # Step 2: Simulate box securing (you can add actual hardware control here)
            self.logger.info("Step 2: Securing box with gripper")
            
            # Close gripper to secure box
            success = self._control_gripper("close")
            if not success:
                raise Exception("Failed to secure box with gripper")
            
            time.sleep(1.0)  # Allow time for gripper to secure box
            self.box_secured = True
            self.box_in_gripper = True
            self.operational_state = "retrieving_box"
            
            # Extract box ID from drone delivery data if available
            # TODO: Get actual box ID from drone/drone port
            self.current_box_id = f"DRONE_BOX_{int(time.time())}"
            
            # Notify drone port that box has been retrieved
            self._publish_box_retrieved()
            
            # Step 3: Determine storage shelf (simple logic - use first available)
            target_shelf = self._find_available_shelf()
            if not target_shelf:
                raise Exception("No available shelf space")
            
            # Step 4: Execute storage sequence
            self.logger.info(f"Step 4: Storing box on {target_shelf}")
            self.operational_state = "storing_box"
            
            # Store the box
            self._execute_box_storage_sequence(self.current_box_id, target_shelf)
            
        except Exception as e:
            self.logger.error(f"Box pickup sequence failed: {e}")
            self.operational_state = "error"
            self._publish_error(f"Pickup sequence failed: {str(e)}")

    def _async_move_for_drone_operation(self, degrees, operation_type):
        """Execute move operation for drone coordination"""
        try:
            success = self.servo_controller.move_to_position(degrees, True, 30.0)
            
            if success:
                if operation_type == "standby":
                    self._publish_robot_operational_state("ready_for_box")
                    self.logger.info("Robot in standby position, ready for box")
                elif operation_type == "pickup":
                    self._publish_robot_operational_state("at_pickup_position")
                    self.logger.info("Robot at pickup position")
                elif operation_type == "delivery":
                    self._publish_robot_operational_state("at_delivery_position")
                    self.logger.info("Robot at delivery position")
            else:
                self.operational_state = "error"
                self._publish_error(f"Failed to reach {operation_type} position")
                
        except Exception as e:
            self.logger.error(f"Async move for drone operation failed: {e}")
            self.operational_state = "error"
            self._publish_error(f"Move for {operation_type} failed: {str(e)}")

    def _find_available_shelf(self):
        """
        Find an available shelf for box storage
        
        Returns:
            str: Available shelf name or None if all full
        """
        max_capacity = 10  # Maximum boxes per shelf
        
        for shelf in ['shelf_1', 'shelf_2', 'shelf_3']:
            if len(self.shelf_inventory[shelf]) < max_capacity:
                return shelf
        
        self.logger.warning("All shelves are full!")
        return None

    # ========================================================================================
    # ENHANCED PUBLISHING METHODS
    # ========================================================================================
    
    def _publish_storage_complete(self, box_id, shelf, success, error=None):
        """Publish box storage completion status"""
        message = {
            'timestamp': datetime.now().isoformat(),
            'box_id': box_id,
            'shelf': shelf,
            'success': success,
            'error': error,
            'inventory': self._get_inventory_status()
        }
        
        self._publish_json(self.config.TOPICS['robot_status'], message)
        self.logger.info(f"Storage complete notification sent for {box_id}")

    def _publish_pickup_ready(self, box_id):
        """Notify that box is ready for customer pickup"""
        message = {
            'timestamp': datetime.now().isoformat(),
            'box_id': box_id,
            'pickup_ready': True,
            'position': 'delivery_point'
        }
        
        self._publish_json(self.config.TOPICS['pickup_ready'], message)
        self.logger.info(f"Pickup ready notification sent for {box_id}")

    def _publish_enhanced_status(self):
        """Publish enhanced status including Arduino systems"""
        if self.servo_controller:
            system_info = self.servo_controller.get_system_info()
        else:
            system_info = {
                'connected': False,
                'interface': None,
                'slave_name': None,
                'current_position': None,
                'motor_status': None,
                'is_moving': False,
                'last_error': 'Not connected'
            }
        
        # Get Arduino status
        arduino_status = self._get_arduino_status()
        
        status_message = {
            'timestamp': datetime.now().isoformat(),
            'client_id': self.config.CLIENT_ID,
            'operational_state': self.operational_state,
            'box_secured': self.box_secured,
            'current_box_id': self.current_box_id,
            'positions': {
                'horizontal': self.horizontal_position,
                'vertical': self.vertical_position
            },
            'gripper': {
                'state': self.gripper_state,
                'box_in_gripper': self.box_in_gripper
            },
            'arduino_systems': arduino_status,
            'inventory': self._get_inventory_status(),
            'ethercat_servo': system_info
        }
        
        self._publish_json(self.config.TOPICS['robot_status'], status_message)

    def _publish_robot_operational_state(self, state):
        """Publish robot operational state for drone coordination"""
        state_message = {
            'timestamp': datetime.now().isoformat(),
            'client_id': self.config.CLIENT_ID,
            'operational_state': state,
            'position': self.servo_controller.get_current_position() if self.servo_controller else None,
            'box_secured': self.box_secured
        }
        
        # Publish to specific operational state topic
        if state == "ready_for_box":
            self._publish_json(self.config.TOPICS['robot_ready_for_box'], state_message)
        elif state == "at_pickup_position":
            self._publish_json(self.config.TOPICS['robot_at_pickup_position'], state_message)
        elif state == "at_delivery_position":
            self._publish_json(self.config.TOPICS['robot_at_delivery_position'], state_message)
        
        # Also publish to general status
        self._publish_json(self.config.TOPICS['robot_status'], state_message)

    def _publish_box_retrieved(self):
        """Notify drone port that box has been retrieved"""
        retrieval_message = {
            'timestamp': datetime.now().isoformat(),
            'client_id': self.config.CLIENT_ID,
            'box_retrieved': True,
            'position': self.servo_controller.get_current_position() if self.servo_controller else None,
            'operational_state': self.operational_state
        }
        
        self._publish_json(self.config.TOPICS['box_retrieved'], retrieval_message)
        self.logger.info("Notified drone port: Box retrieved")

    def _publish_box_secured(self):
        """Notify that box has been secured by robot"""
        secured_message = {
            'timestamp': datetime.now().isoformat(),
            'client_id': self.config.CLIENT_ID,
            'box_secured': True,
            'position': self.servo_controller.get_current_position() if self.servo_controller else None
        }
        
        self._publish_json(self.config.TOPICS['robot_box_secured'], secured_message)
        self.logger.info("Box secured by robot")

    # ========================================================================================
    # MOTOR CONTROL METHODS
    # ========================================================================================
    
    def _connect_motor(self, interface):
        """Connect to servo motor and setup Arduino connections"""
        try:
            # Connect EtherCAT servo (horizontal movement)
            self.servo_controller = ServoMotorController(interface)
            success = self.servo_controller.connect()
            
            if success:
                # Set default motion parameters
                self.servo_controller.set_motion_parameters(
                    **self.config.DEFAULT_MOTION_PARAMS
                )
                self.logger.info("EtherCAT servo connected and configured")
                
                # Setup Arduino connections
                self._setup_arduino_connections()
                
                return True
            
            return False
            
        except Exception as e:
            self.logger.error(f"Motor connection failed: {e}")
            return False

    def _disconnect_motor(self):
        """Disconnect from all motor systems"""
        try:
            # Disconnect EtherCAT servo
            if self.servo_controller:
                self.servo_controller.close()
                self.servo_controller = None
            
            # Disconnect Arduino systems
            if self.servo_arduino:
                self.servo_arduino.close()
                self.servo_arduino = None
                
            if self.gripper_arduino:
                self.gripper_arduino.close()
                self.gripper_arduino = None
            
            self.logger.info("All motor systems disconnected")
            self._publish_command_result("disconnect", True)
            
        except Exception as e:
            self.logger.error(f"Motor disconnect failed: {e}")

    def _set_motion_parameters(self, params):
        """Set motor motion parameters"""
        try:
            if not self.servo_controller or not self.servo_controller.is_initialized:
                self._publish_error("Motor not connected")
                return
            
            # Merge with defaults
            motion_params = {**self.config.DEFAULT_MOTION_PARAMS, **params}
            
            success = self.servo_controller.set_motion_parameters(
                motion_params['acceleration'],
                motion_params['deceleration'],
                motion_params['max_velocity']
            )
            
            self._publish_command_result("set_params", success)
            
        except Exception as e:
            self.logger.error(f"Set parameters failed: {e}")
            self._publish_error(f"Set parameters failed: {str(e)}")

    # ========================================================================================
    # ASYNC OPERATION HANDLERS
    # ========================================================================================
    
    def _async_move(self, degrees, timeout):
        """Execute move operation asynchronously"""
        try:
            success = self.servo_controller.move_to_position(degrees, True, timeout)
            self._publish_move_result(degrees, success)
        except Exception as e:
            self.logger.error(f"Async move failed: {e}")
            self._publish_error(f"Async move failed: {str(e)}")

    def _async_home(self):
        """Execute home operation asynchronously"""
        try:
            success = self.servo_controller.go_home(True)
            self._publish_move_result(0, success, "home")
        except Exception as e:
            self.logger.error(f"Async home failed: {e}")
            self._publish_error(f"Home failed: {str(e)}")

    # ========================================================================================
    # MQTT PUBLISHING METHODS
    # ========================================================================================
    
    def _publish_startup_message(self):
        """Publish startup message"""
        message = {
            'timestamp': datetime.now().isoformat(),
            'client_id': self.config.CLIENT_ID,
            'status': 'online',
            'message': 'Robot controller started'
        }
        self._publish_json(self.config.TOPICS['robot_status'], message)

    def _publish_status_now(self):
        """Immediately publish current status"""
        if self.servo_controller:
            system_info = self.servo_controller.get_system_info()
        else:
            system_info = {
                'connected': False,
                'interface': None,
                'slave_name': None,
                'current_position': None,
                'motor_status': None,
                'is_moving': False,
                'last_error': 'Not connected'
            }
        
        status_message = {
            'timestamp': datetime.now().isoformat(),
            'client_id': self.config.CLIENT_ID,
            'operational_state': self.operational_state,
            'box_secured': self.box_secured,
            **system_info
        }
        
        self._publish_json(self.config.TOPICS['robot_status'], status_message)

    def _publish_position_now(self):
        """Immediately publish current position"""
        if self.servo_controller and self.servo_controller.is_initialized:
            position = self.servo_controller.get_current_position()
        else:
            position = None
        
        position_message = {
            'timestamp': datetime.now().isoformat(),
            'position': position,
            'connected': self.servo_controller.is_initialized if self.servo_controller else False
        }
        
        self._publish_json(self.config.TOPICS['robot_position'], position_message)

    def _publish_move_result(self, target_degrees, success, move_type="absolute"):
        """Publish movement result"""
        current_position = None
        if self.servo_controller and self.servo_controller.is_initialized:
            current_position = self.servo_controller.get_current_position()
        
        result_message = {
            'timestamp': datetime.now().isoformat(),
            'move_type': move_type,
            'target_degrees': target_degrees,
            'success': success,
            'final_position': current_position,
            'error': self.servo_controller.last_error if self.servo_controller else None
        }
        
        self._publish_json(self.config.TOPICS['robot_status'], result_message)

    def _publish_command_result(self, command, success):
        """Publish general command result"""
        result_message = {
            'timestamp': datetime.now().isoformat(),
            'command': command,
            'success': success,
            'error': self.servo_controller.last_error if self.servo_controller else None
        }
        
        self._publish_json(self.config.TOPICS['robot_status'], result_message)

    def _publish_error(self, error_message):
        """Publish error message"""
        error_msg = {
            'timestamp': datetime.now().isoformat(),
            'error': error_message,
            'client_id': self.config.CLIENT_ID
        }
        
        self._publish_json(self.config.TOPICS['robot_status'], error_msg)

    def _publish_emergency_response(self):
        """Publish emergency stop response"""
        response_message = {
            'timestamp': datetime.now().isoformat(),
            'client_id': self.config.CLIENT_ID,
            'emergency_stop': True,
            'status': 'stopped'
        }
        
        self._publish_json(self.config.TOPICS['robot_status'], response_message)

    def _publish_heartbeat(self):
        """Publish heartbeat message"""
        heartbeat_message = {
            'timestamp': datetime.now().isoformat(),
            'client_id': self.config.CLIENT_ID,
            'uptime': time.time() - self.start_time,
            'connected': self.servo_controller.is_initialized if self.servo_controller else False
        }
        
        self._publish_json(self.config.TOPICS['robot_heartbeat'], heartbeat_message)

    def _publish_json(self, topic, data):
        """Helper method to publish JSON data"""
        try:
            json_string = json.dumps(data, indent=2)
            self.mqtt_client.publish(topic, json_string, self.config.MQTT_QOS)
        except Exception as e:
            self.logger.error(f"Failed to publish to {topic}: {e}")

    # ========================================================================================
    # MAIN CONTROL LOOP
    # ========================================================================================
    
    def run(self):
        """Main application loop"""
        self.logger.info("Starting MQTT Servo Controller...")
        self.start_time = time.time()
        self.running = True
        
        # Setup MQTT connection
        if not self.setup_mqtt():
            self.logger.error("Failed to setup MQTT, exiting...")
            return False
        
        # Main loop
        try:
            while self.running:
                current_time = time.time()
                
                # Periodic status publishing (now includes Arduino status)
                if current_time - self.last_status_time >= self.config.STATUS_INTERVAL:
                    self._publish_enhanced_status()
                    self.last_status_time = current_time
                
                # Periodic position publishing (if connected)
                if (current_time - self.last_position_time >= self.config.POSITION_INTERVAL and
                    self.servo_controller and self.servo_controller.is_initialized):
                    self._publish_position_now()
                    self.last_position_time = current_time
                
                # Periodic heartbeat
                if current_time - self.last_heartbeat_time >= self.config.HEARTBEAT_INTERVAL:
                    self._publish_heartbeat()
                    self.last_heartbeat_time = current_time
                
                # Short sleep to prevent CPU spinning
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            self.logger.info("Keyboard interrupt received")
        except Exception as e:
            self.logger.error(f"Unexpected error in main loop: {e}")
        finally:
            self._shutdown()

    def _shutdown(self):
        """Clean shutdown procedure"""
        self.logger.info("Shutting down...")
        self.running = False
        
        # Disconnect motor systems
        if self.servo_controller:
            try:
                self.servo_controller.close()
            except:
                pass
                
        # Disconnect Arduino systems
        if self.servo_arduino:
            try:
                self.servo_arduino.close()
            except:
                pass
                
        if self.gripper_arduino:
            try:
                self.gripper_arduino.close()
            except:
                pass
        
        # Disconnect MQTT
        if self.mqtt_client:
            try:
                # Send offline message
                offline_message = {
                    'timestamp': datetime.now().isoformat(),
                    'client_id': self.config.CLIENT_ID,
                    'status': 'offline',
                    'message': 'Robot controller shutting down'
                }
                self._publish_json(self.config.TOPICS['robot_status'], offline_message)
                
                time.sleep(0.5)  # Give time for message to send
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except:
                pass
        
        self.logger.info("Shutdown complete")


# ========================================================================================
# SIGNAL HANDLERS
# ========================================================================================

def signal_handler(signum, frame):
    """Handle system signals for graceful shutdown"""
    print(f"\nReceived signal {signum}, shutting down...")
    global mqtt_servo_controller
    if mqtt_servo_controller:
        mqtt_servo_controller.running = False


# ========================================================================================
# MAIN FUNCTION
# ========================================================================================

def main():
    """Main function"""
    global mqtt_servo_controller
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='MQTT Servo Motor Controller for Drone Kiosk')
    parser.add_argument('--interface', '-i', default='eth0', 
                       help='EtherCAT network interface (default: eth0)')
    parser.add_argument('--broker', '-b', default='localhost', 
                       help='MQTT broker address (default: localhost)')
    parser.add_argument('--port', '-p', type=int, default=1883, 
                       help='MQTT broker port (default: 1883)')
    parser.add_argument('--client-id', '-c', default='kiosk_robot_controller', 
                       help='MQTT client ID (default: kiosk_robot_controller)')
    parser.add_argument('--auto-connect', '-a', action='store_true',
                       help='Automatically connect to motor on startup')
    
    args = parser.parse_args()
    
    # Update configuration with command line arguments
    config = Config()
    config.MQTT_BROKER = args.broker
    config.MQTT_PORT = args.port
    config.CLIENT_ID = args.client_id
    config.MOTOR_INTERFACE = args.interface
    
    print("Drone Kiosk - Robot Controller")
    print("=" * 50)
    print(f"MQTT Broker: {config.MQTT_BROKER}:{config.MQTT_PORT}")
    print(f"Client ID: {config.CLIENT_ID}")
    print(f"Motor Interface: {config.MOTOR_INTERFACE}")
    print(f"Auto-connect: {args.auto_connect}")
    print()
    
    # Setup signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and run the controller
    mqtt_servo_controller = MQTTServoController(config)
    
    # Auto-connect to motor if requested
    if args.auto_connect:
        print("Auto-connecting to motor...")
        if mqtt_servo_controller._connect_motor(config.MOTOR_INTERFACE):
            print("Motor connected successfully")
        else:
            print("Auto-connect failed, motor can be connected via MQTT command")
    
    # Run the main application
    try:
        mqtt_servo_controller.run()
    except Exception as e:
        print(f"Fatal error: {e}")
        return 1
    
    return 0


# ========================================================================================
# ENTRY POINT
# ========================================================================================

if __name__ == "__main__":
    # Global variable for signal handler
    mqtt_servo_controller = None
    
    # Run main function
    exit_code = main()
    sys.exit(exit_code)