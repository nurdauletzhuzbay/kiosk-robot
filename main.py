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

# Serial communication for Arduino
import serial
import serial.tools.list_ports

# Import servo motor controller (EtherCAT for horizontal movement)
from servo_motor_controller import ServoMotorController

# ========================================================================================
# CONFIGURATION
# ========================================================================================

class Config:
    """Configuration settings for the MQTT servo controller"""
    
    # MQTT Settings
    MQTT_BROKER = "localhost"
    MQTT_PORT = 1883
    MQTT_KEEPALIVE = 60
    MQTT_QOS = 1
    
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
        'robot_home': 'kiosk/robot/home',
        'robot_stop': 'kiosk/robot/stop',
        'robot_connect': 'kiosk/robot/connect',
        'robot_demo': 'kiosk/robot/demo',  # ADDED FOR DEMO
        
        # Robot gripper control topics
        'robot_gripper_home': 'kiosk/robot/gripper/home',
        'robot_gripper_open': 'kiosk/robot/gripper/open',
        'robot_gripper_close': 'kiosk/robot/gripper/close',
        'robot_gripper_slide_forward': 'kiosk/robot/gripper/slide_forward',
        'robot_gripper_slide_backward': 'kiosk/robot/gripper/slide_backward',
        'robot_gripper_rotate_left': 'kiosk/robot/gripper/rotate_left',
        'robot_gripper_rotate_center': 'kiosk/robot/gripper/rotate_center',
        'robot_gripper_rotate_right': 'kiosk/robot/gripper/rotate_right',
        'robot_gripper_vertical': 'kiosk/robot/gripper/vertical',
        'robot_gripper_slide_disable': 'kiosk/robot/gripper/slide_disable',
        
        # QR/Pickup point topics - SIMPLIFIED
        'pickup_box_request': 'kiosk/pickup/box_request',  # Receive QR scan requests
        'pickup_box_delivered': 'kiosk/pickup/box_delivered',  # Publish when box delivered
        
        # Drone coordination topics
        'drone_ready_to_land': 'kiosk/drone/ready_to_land',
        'box_ready_for_pickup': 'kiosk/drone_port/box_ready',
        'box_retrieved': 'kiosk/robot/box_retrieved',  # Publish when box stored
        
        # System-wide topics
        'emergency_stop': 'kiosk/emergency/stop'
    }
    
    # Motor Settings
    MOTOR_INTERFACE = "eth0"
    MOTOR_HOME_POSITION = 0  # Absolute home position in encoder ticks
    DEFAULT_MOTION_PARAMS = {
        'velocity': 500000,
        'acceleration': 500000
    }
    
    # Predefined positions (in degrees) - PLACEHOLDERS
    POSITIONS = {
        'home': 0,              # Home/rest position
        'pickup_shelf': 500,     # Position to pickup box from shelf (PLACEHOLDER)
        'delivery': 1500,        # Position to deliver box to customer (PLACEHOLDER)
        'drone_pickup': 200,    # Position to pickup box from drone port (PLACEHOLDER)
        'storage_shelf': 135,   # Position to store box on shelf (PLACEHOLDER)
    }
    
    # Arduino communication settings
    ARDUINO_SETTINGS = {
        'port': '/dev/ttyACM0',
        'baudrate': 9600,
        'timeout': 1
    }
    
    # Gripper movement degrees (PLACEHOLDERS - adjust based on your system)
    GRIPPER_VERTICAL_DEFAULT = 0  # Default vertical movement degrees
    
    # Status reporting intervals (seconds)
    STATUS_INTERVAL = 2.0
    POSITION_INTERVAL = 1.0
    HEARTBEAT_INTERVAL = 5.0

# ========================================================================================
# MQTT ROBOT CONTROLLER CLASS
# ========================================================================================

class MQTTRobotController:
    """MQTT-enabled robot controller for kiosk system"""
    
    def __init__(self, config):
        self.config = config
        self.mqtt_client = None
        self.servo_controller = None
        self.running = False
        
        # Arduino connection for gripper control
        self.arduino = None
        
        # Status tracking
        self.last_status_time = 0
        self.last_position_time = 0
        self.last_heartbeat_time = 0
        
        # Robot state
        self.horizontal_position = 0.0
        self.operational_state = "idle"
        self.current_box_id = None
        
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
        
        # Check paho-mqtt version and create client accordingly
        try:
            # For paho-mqtt >= 2.0.0
            self.mqtt_client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION1,
                client_id=self.config.CLIENT_ID
            )
        except (AttributeError, TypeError):
            # For paho-mqtt < 2.0.0
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
            self.config.TOPICS['robot_commands'],
            self.config.TOPICS['robot_move'],
            self.config.TOPICS['robot_home'],
            self.config.TOPICS['robot_stop'],
            self.config.TOPICS['robot_connect'],
            self.config.TOPICS['robot_demo'],  # ADDED FOR DEMO
            
            # Gripper control topics
            self.config.TOPICS['robot_gripper_home'],
            self.config.TOPICS['robot_gripper_open'],
            self.config.TOPICS['robot_gripper_close'],
            self.config.TOPICS['robot_gripper_slide_forward'],
            self.config.TOPICS['robot_gripper_slide_backward'],
            self.config.TOPICS['robot_gripper_rotate_left'],
            self.config.TOPICS['robot_gripper_rotate_center'],
            self.config.TOPICS['robot_gripper_rotate_right'],
            self.config.TOPICS['robot_gripper_vertical'],
            self.config.TOPICS['robot_gripper_slide_disable'],

            
            # Pickup and drone topics
            self.config.TOPICS['pickup_box_request'],
            self.config.TOPICS['drone_ready_to_land'],
            self.config.TOPICS['box_ready_for_pickup'],
            
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
            
            try:
                data = json.loads(payload)
            except json.JSONDecodeError:
                data = {'raw_message': payload}
            
            self._handle_mqtt_command(topic, data)
            
        except Exception as e:
            self.logger.error(f"Error processing MQTT message: {e}")

    # ========================================================================================
    # COMMAND HANDLERS
    # ========================================================================================
    
    def _handle_mqtt_command(self, topic, data):
        """Route MQTT commands to appropriate handlers"""
        
        if topic == self.config.TOPICS['robot_move']:
            self._handle_move_command(data)
        elif topic == self.config.TOPICS['robot_home']:
            self._handle_home_command(data)
        elif topic == self.config.TOPICS['robot_stop']:
            self._handle_stop_command(data)
        elif topic == self.config.TOPICS['robot_connect']:
            self._handle_connect_command(data)
        elif topic == self.config.TOPICS['robot_commands']:
            self._handle_general_command(data)
        elif topic == self.config.TOPICS['robot_demo']:  # ADDED FOR DEMO
            self._handle_demo_sequence(data)
        
        # Gripper control commands
        elif topic == self.config.TOPICS['robot_gripper_home']:
            self._handle_gripper_home(data)
        elif topic == self.config.TOPICS['robot_gripper_open']:
            self._handle_gripper_open(data)
        elif topic == self.config.TOPICS['robot_gripper_close']:
            self._handle_gripper_close(data)
        elif topic == self.config.TOPICS['robot_gripper_slide_forward']:
            self._handle_gripper_slide_forward(data)
        elif topic == self.config.TOPICS['robot_gripper_slide_backward']:
            self._handle_gripper_slide_backward(data)
        elif topic == self.config.TOPICS['robot_gripper_rotate_left']:
            self._handle_gripper_rotate_left(data)
        elif topic == self.config.TOPICS['robot_gripper_rotate_center']:
            self._handle_gripper_rotate_center(data)
        elif topic == self.config.TOPICS['robot_gripper_rotate_right']:
            self._handle_gripper_rotate_right(data)
        elif topic == self.config.TOPICS['robot_gripper_vertical']:
            self._handle_gripper_vertical(data)
        elif topic == self.config.TOPICS['robot_gripper_slide_disable']:
            self._handle_gripper_slide_disable(data)
        
        # Pickup and drone workflows
        elif topic == self.config.TOPICS['pickup_box_request']:
            self._handle_pickup_box_request(data)
        elif topic == self.config.TOPICS['drone_ready_to_land']:
            self._handle_drone_ready_to_land(data)
        elif topic == self.config.TOPICS['box_ready_for_pickup']:
            self._handle_box_ready_for_pickup(data)
        
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
            
            degrees = data.get('degrees', data.get('position', 0))
            wait = data.get('wait', False)
            timeout = data.get('timeout', 30.0)
            
            self.logger.info(f"Moving to {degrees} degrees (wait: {wait})")
            
            if wait:
                success = self.servo_controller.move_to_position(degrees, True, timeout)
                self._publish_move_result(degrees, success)
            else:
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
            threading.Thread(target=self._async_home, daemon=True).start()
            
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

    def _handle_connect_command(self, data):
        """Handle motor connection commands"""
        try:
            interface = data.get('interface', self.config.MOTOR_INTERFACE)
            home_position = data.get('home_position', self.config.MOTOR_HOME_POSITION)
            
            if self.servo_controller and self.servo_controller.is_initialized:
                self._publish_error("Motor already connected")
                return
            
            self.logger.info(f"Connecting to motor on {interface} with home at {home_position}")
            success = self._connect_motor(interface, home_position)
            self._publish_command_result("connect", success)
            
        except Exception as e:
            self.logger.error(f"Connect command failed: {e}")

    def _handle_general_command(self, data):
        """Handle general commands"""
        try:
            command = data.get('command', '').lower()
            
            if command == 'status':
                self._publish_status_now()
            elif command == 'position':
                self._publish_position_now()
            elif command == 'encoder':
                if self.servo_controller and self.servo_controller.is_initialized:
                    self.servo_controller.show_encoder_info()
            elif command == 'disconnect':
                self._disconnect_motor()
            else:
                self.logger.warning(f"Unknown command: {command}")
                
        except Exception as e:
            self.logger.error(f"General command failed: {e}")

    def _handle_emergency_stop(self, data):
        """Handle system-wide emergency stop"""
        self.logger.critical("EMERGENCY STOP RECEIVED!")
        try:
            if self.servo_controller and self.servo_controller.is_initialized:
                self.servo_controller.emergency_stop()
            self._send_arduino_command("robot_gripper_home")
            self._publish_emergency_response()
        except Exception as e:
            self.logger.error(f"Emergency stop failed: {e}")

    # ========================================================================================
    # DEMO SEQUENCE HANDLER - NEW
    # ========================================================================================
    
    def _handle_demo_sequence(self, data):
        """Handle demo sequence request"""
        try:
            self.logger.info("Demo sequence requested")
            self.operational_state = "demo_running"
            
            threading.Thread(
                target=self._execute_demo_sequence,
                daemon=True
            ).start()
            
        except Exception as e:
            self.logger.error(f"Demo sequence handling failed: {e}")
            self._publish_error(f"Demo sequence failed: {str(e)}")

    def _execute_demo_sequence(self):
        """Execute complete demo sequence"""
        try:
            self.logger.info("[DEMO] === Starting demo sequence ===")
            
            # Step 1: Gripper home
            self.logger.info("[DEMO] Step 1: Gripper home")
            self._send_arduino_command("robot_gripper_home")
            
            # Step 2: Wait 10 seconds
            self.logger.info("[DEMO] Step 2: Waiting 10 seconds")
            time.sleep(65.0)
            
            # Step 3: Move to position 1500
            self.logger.info("[DEMO] Step 3: Moving to position 1500")
            if not self.servo_controller or not self.servo_controller.is_initialized:
                raise Exception("Motor not connected")
            
            success = self.servo_controller.move_to_position(1500, True, 30.0)
            if not success:
                raise Exception("Failed to move to position 1500")
            
            # Step 4: Return home
            self.logger.info("[DEMO] Step 4: Returning to home position")
            home_pos = self.config.POSITIONS['home']
            success = self.servo_controller.move_to_position(home_pos, True, 30.0)
            if not success:
                raise Exception("Failed to return home")
            
            # Step 5: Publish box delivered
            self.logger.info("[DEMO] Step 5: Publishing box delivered notification")
            demo_box_id = f"DEMO_BOX_{int(time.time())}"
            self._publish_box_delivered(demo_box_id)
            
            self.operational_state = "idle"
            self.logger.info("[DEMO] === Demo sequence completed successfully ===")
            
        except Exception as e:
            self.logger.error(f"[DEMO] Demo sequence failed: {e}")
            self.operational_state = "error"
            self._publish_error(f"Demo sequence failed: {str(e)}")

    # ========================================================================================
    # GRIPPER CONTROL HANDLERS (MQTT Topics)
    # ========================================================================================
    
    def _handle_gripper_home(self, data):
        """Handle gripper home command"""
        self._send_arduino_command("robot_gripper_home")
    
    def _handle_gripper_open(self, data):
        """Handle gripper open command"""
        self._send_arduino_command("robot_gripper_open")
    
    def _handle_gripper_close(self, data):
        """Handle gripper close command"""
        self._send_arduino_command("robot_gripper_close")
    
    def _handle_gripper_slide_forward(self, data):
        """Handle gripper slide forward command"""
        self._send_arduino_command("robot_gripper_slide_forward")
    
    def _handle_gripper_slide_backward(self, data):
        """Handle gripper slide backward command"""
        self._send_arduino_command("robot_gripper_slide_backward")
    
    def _handle_gripper_rotate_left(self, data):
        """Handle gripper rotate left command"""
        self._send_arduino_command("robot_gripper_rotate_left")
    
    def _handle_gripper_rotate_center(self, data):
        """Handle gripper rotate center command"""
        self._send_arduino_command("robot_gripper_rotate_center")
    
    def _handle_gripper_rotate_right(self, data):
        """Handle gripper rotate right command"""
        self._send_arduino_command("robot_gripper_rotate_right")
    
    def _handle_gripper_vertical(self, data):
        """Handle gripper vertical movement command with absolute degrees"""
        degrees = data.get('degrees', self.config.GRIPPER_VERTICAL_DEFAULT)
        self._send_arduino_command(f"robot_vertical_{degrees}")
        
    def _handle_gripper_slide_disable(self, data):
        """Handle gripper vertical movement command with absolute degrees"""
        self._send_arduino_command(f"robot_gripper_slide_disable")

    # ========================================================================================
    # DRONE WORKFLOW HANDLERS
    # ========================================================================================

    def _handle_drone_ready_to_land(self, data):
        """Handle drone ready to land notification"""
        try:
            self.logger.info("Drone ready to land - moving to pickup position")
            self.operational_state = "preparing_for_drone"
            
            threading.Thread(
                target=self._execute_drone_preparation_sequence,
                daemon=True
            ).start()
            
        except Exception as e:
            self.logger.error(f"Drone ready to land handling failed: {e}")
            self._publish_error(f"Drone preparation failed: {str(e)}")
    
    def _execute_drone_preparation_sequence(self):
        """Execute drone preparation sequence"""
        try:
            self.logger.info("Starting drone preparation sequence")
            
            # Step 2: Gripper vertical to pickup height
            self.logger.info("Step 2: Gripper vertical to pickup height")
            if not self._send_arduino_command_and_wait("robot_vertical_7700"): return
            time.sleep(1.0)
            
            self.operational_state = "ready_for_box"
            self.logger.info("Robot ready for box pickup from drone")
            
        except Exception as e:
            self.logger.error(f"Drone preparation sequence failed: {e}")
            self.operational_state = "error"
            self._publish_error(f"Drone preparation failed: {str(e)}")
    
    def _handle_box_ready_for_pickup(self, data):
        """Handle box ready for pickup from drone port"""
        try:
            box_id = data.get('box_id', f'DRONE_BOX_{int(time.time())}')
            self.logger.info(f"Box ready for pickup from drone: {box_id}")
            self.current_box_id = box_id
            self.operational_state = "picking_up_from_drone"
            
            threading.Thread(
                target=self._execute_drone_pickup_storage_sequence,
                args=(box_id,),
                daemon=True
            ).start()
            
        except Exception as e:
            self.logger.error(f"Box ready for pickup handling failed: {e}")
            self._publish_error(f"Box pickup failed: {str(e)}")
    
    def _execute_drone_pickup_storage_sequence(self, box_id):
        """Execute complete drone box pickup and storage sequence"""
        try:
            self.logger.info(f"Starting drone pickup-storage sequence for {box_id}")
            
            # Step 1: Move to drone pickup position
            self.logger.info("Step 1: Moving to drone pickup position")
            drone_pos = self.config.POSITIONS['drone_pickup']
            success = self.servo_controller.move_to_position(drone_pos, True, 30.0)
            if not success:
                raise Exception("Failed to reach drone pickup position")
            time.sleep(0.5)
            
            # Full sequence as before
            if not self._send_arduino_command_and_wait("robot_gripper_slide_forward"): return
            time.sleep(1.0)
            if not self._send_arduino_command_and_wait("robot_gripper_close"): return
            time.sleep(1.0)
            if not self._send_arduino_command_and_wait("robot_gripper_slide_backward"): return
            time.sleep(1.0)
            
            self.servo_controller.move_to_position(self.config.POSITIONS['home'], True, 30.0)
            self._publish_box_retrieved(box_id)
            self.logger.info(f"[SEQ] Box {box_id} retrieved")
            
            if not self._send_arduino_command_and_wait(f"robot_vertical_0"): return  # Return to home position
            time.sleep(8.0)
            
            delivery_pos = self.config.POSITIONS['delivery']
            self.logger.info(f"[SEQ] Moving to delivery position → {delivery_pos}")
            self.servo_controller.move_to_position(delivery_pos, True, 30.0)

            if not self._send_arduino_command_and_wait("robot_vertical_450"): return
            time.sleep(1)
            if not self._send_arduino_command_and_wait("robot_gripper_rotate_right"): return
            time.sleep(1)
            if not self._send_arduino_command_and_wait("robot_gripper_slide_forward"): return
            time.sleep(1)
            if not self._send_arduino_command_and_wait("robot_gripper_open"): return
            time.sleep(10)

            if not self._send_arduino_command_and_wait("robot_gripper_slide_backward"): return
            time.sleep(10)
            if not self._send_arduino_command_and_wait("robot_gripper_rotate_center"): return
            time.sleep(10)
            if not self._send_arduino_command_and_wait("robot_vertical_0"): return

            # self.logger.info(f"[SEQ] Returning to home position")
            self.servo_controller.move_to_position(self.config.POSITIONS['home'], True, 30.0)
            self._publish_box_delivered(box_id)
            self.logger.info(f"[SEQ] Box {box_id} delivered")
            
            self.operational_state = "idle"
            self.current_box_id = None
            self.logger.info(f"Drone pickup-storage sequence completed for {box_id}")
            
        except Exception as e:
            self.logger.error(f"Drone pickup-storage sequence failed: {e}")
            self.operational_state = "error"
            self._publish_error(f"Drone pickup-storage failed: {str(e)}")

    # ========================================================================================
    # QR SCAN / BOX PICKUP WORKFLOW
    # ========================================================================================
    
    def _handle_pickup_box_request(self, data):
        """Handle box pickup request from QR scan at pickup point"""
        try:
            box_id = data.get('box_id')
            if not box_id:
                self._publish_error("Box ID required for pickup request")
                return
            
            self.logger.info(f"Pickup request received: {box_id}")
            self.current_box_id = box_id
            self.operational_state = "processing_pickup"
            
            threading.Thread(
                target=self._execute_pickup_delivery_sequence,
                args=(box_id,),
                daemon=True
            ).start()
            
        except Exception as e:
            self.logger.error(f"Pickup request handling failed: {e}")
            self._publish_error(f"Pickup request failed: {str(e)}")

    def _execute_pickup_delivery_sequence(self, box_id):
        """Execute complete pickup and delivery sequence"""
        try:
            self.logger.info(f"[SEQ] === Starting pickup-delivery sequence for box {box_id} ===")

            # Move to pickup position
            pickup_pos = self.config.POSITIONS['pickup_shelf']
            self.logger.info(f"[SEQ] Moving to pickup position → {pickup_pos}")
            self.servo_controller.move_to_position(pickup_pos, True, 30.0)

            # Sequence with confirmations
            if not self._send_arduino_command_and_wait("robot_vertical_830"): return
            if not self._send_arduino_command_and_wait("robot_gripper_rotate_left"): return
            if not self._send_arduino_command_and_wait("robot_gripper_slide_forward"): return
            time.sleep(1)
            if not self._send_arduino_command_and_wait("robot_gripper_close"): return
            time.sleep(1)
            if not self._send_arduino_command_and_wait("robot_gripper_slide_backward"): return
            time.sleep(1)
            if not self._send_arduino_command_and_wait("robot_gripper_rotate_center"): return
            time.sleep(1)

            if not self._send_arduino_command_and_wait("robot_vertical_0"): return

            # # Move to delivery
            delivery_pos = self.config.POSITIONS['delivery']
            self.logger.info(f"[SEQ] Moving to delivery position → {delivery_pos}")
            self.servo_controller.move_to_position(delivery_pos, True, 30.0)

            if not self._send_arduino_command_and_wait("robot_vertical_350"): return
            time.sleep(1)
            if not self._send_arduino_command_and_wait("robot_gripper_rotate_right"): return
            time.sleep(1)
            if not self._send_arduino_command_and_wait("robot_gripper_slide_forward"): return
            time.sleep(1)
            if not self._send_arduino_command_and_wait("robot_gripper_open"): return
            time.sleep(10)

            if not self._send_arduino_command_and_wait("robot_gripper_slide_backward"): return
            time.sleep(10)
            if not self._send_arduino_command_and_wait("robot_gripper_rotate_center"): return
            time.sleep(10)
            if not self._send_arduino_command_and_wait("robot_vertical_0"): return

            # self.logger.info(f"[SEQ] Returning to home position")
            self.servo_controller.move_to_position(self.config.POSITIONS['home'], True, 30.0)
            self._publish_box_delivered(box_id)
            self.logger.info(f"[SEQ] Box {box_id} delivered")
            

            # self.operational_state = "idle"
            # self.current_box_id = None
            # self.logger.info(f"[SEQ] === Pickup-delivery sequence completed for box {box_id} ===")

        except Exception as e:
            self.logger.error(f"[SEQ] Pickup-delivery sequence failed: {e}")
            self.operational_state = "error"
            self._publish_error(f"Pickup-delivery failed: {str(e)}")



    # ========================================================================================
    # ARDUINO COMMUNICATION
    # ========================================================================================
    
    def _setup_arduino_connection(self):
        """Setup serial connection to Arduino"""
        try:
            self.logger.info("Setting up Arduino connection...")
            port = self.config.ARDUINO_SETTINGS['port']
            baudrate = self.config.ARDUINO_SETTINGS['baudrate']
            
            self.arduino = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)
            self.logger.info(f"Arduino connected on {port}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to connect Arduino: {e}")
            return False

    def _send_arduino_command(self, command):
        """Send command to Arduino via serial"""
        try:
            if not self.arduino:
                self.logger.error("Arduino not connected")
                return False
            
            command_string = f"{command}\n"
            self.arduino.write(command_string.encode())
            self.logger.info(f"Sent to Arduino: {command}")
            
            response = self.arduino.readline().decode().strip()
            if response:
                self.logger.info(f"Arduino response: {response}")
            
            return True
                
        except Exception as e:
            self.logger.error(f"Arduino command failed: {e}")
            return False
        
    def _send_arduino_command_and_wait(self, command, expected_response="true", timeout=60.0):
        """Send command to Arduino and wait until expected response is received"""
        try:
            if not self.arduino:
                self.logger.error("Arduino not connected")
                return False

            command_string = f"{command}\n"
            self.arduino.write(command_string.encode())
            self.logger.info(f"[SEQ] Command sent → {command}")

            start_time = time.time()
            while time.time() - start_time < timeout:
                response = self.arduino.readline().decode().strip()
                if response:
                    self.logger.info(f"[SEQ] Response received ← {response}")
                    if expected_response.lower() in response.lower():
                        self.logger.info(f"[SEQ] Confirmation received for {command}")
                        return True

            self.logger.error(f"[SEQ] Timeout waiting for confirmation on {command}")
            return False
        except Exception as e:
            self.logger.error(f"[SEQ] Arduino command wait failed: {e}")
            return False



    # ========================================================================================
    # MOTOR CONTROL METHODS
    # ========================================================================================
    
    def _connect_motor(self, interface, home_position=0):
        """Connect to servo motor and setup Arduino"""
        # if self._setup_arduino_connection():
        #     # Perform homing sequence
        #     self.logger.info("Performing initialization homing sequence...")
        #     # self._perform_homing_sequence()
        
        # return True
            
        try:
            # Connect EtherCAT servo with absolute home position
            self.servo_controller = ServoMotorController(interface, home_position)
            success = self.servo_controller.connect()
            
            if success:
                self.servo_controller.set_motion_parameters(
                    **self.config.DEFAULT_MOTION_PARAMS
                )
                self.logger.info("EtherCAT servo connected and configured")
                
                # Setup Arduino connection
                if self._setup_arduino_connection():
                    # Perform homing sequence
                    self.logger.info("Performing initialization homing sequence...")
                    self._perform_homing_sequence()
                
                return True
            
            return False
            
        except Exception as e:
            self.logger.error(f"Motor connection failed: {e}")
            return False

    def _perform_homing_sequence(self):
        """Perform homing sequence for gripper and horizontal axis"""
        try:
            self.logger.info("Starting homing sequence...")
            
            # Step 1: Home gripper (Arduino)
            self.logger.info("Step 1: Homing gripper")
            self._send_arduino_command("robot_gripper_home")
            time.sleep(65)  # Wait for gripper to home
            
            # Step 2: Home horizontal axis (EtherCAT servo)
            self.logger.info("Step 2: Homing horizontal axis")
            success = self.servo_controller.go_home(wait_for_completion=True)
            
            if success:
                self.logger.info("Homing sequence completed successfully")
                self.operational_state = "idle"
            else:
                self.logger.warning("Horizontal homing may have failed")
                self.operational_state = "error"
                
        except Exception as e:
            self.logger.error(f"Homing sequence failed: {e}")
            self.operational_state = "error"

    def _disconnect_motor(self):
        """Disconnect from all motor systems"""
        try:
            if self.servo_controller:
                self.servo_controller.close()
                self.servo_controller = None
            
            if self.arduino:
                self._send_arduino_command("robot_gripper_slide_disable")
                self._send_arduino_command("robot_gripper_rotate_disable")
                self.arduino.close()
                self.arduino = None
            
            self.logger.info("All motor systems disconnected")
            self._publish_command_result("disconnect", True)
            
        except Exception as e:
            self.logger.error(f"Motor disconnect failed: {e}")

    def _async_move(self, degrees, timeout):
        """Execute move operation asynchronously"""
        try:
            success = self.servo_controller.move_to_position(degrees, True, timeout)
            self._publish_move_result(degrees, success)
        except Exception as e:
            self.logger.error(f"Async move failed: {e}")

    def _async_home(self):
        """Execute home operation asynchronously"""
        try:
            success = self.servo_controller.go_home(True)
            self._publish_move_result(0, success, "home")
        except Exception as e:
            self.logger.error(f"Async home failed: {e}")

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

    def _publish_box_delivered(self, box_id):
        """Publish box delivered notification"""
        message = {
            'timestamp': datetime.now().isoformat(),
            'box_id': box_id,
            'status': 'delivered',
            'position': 'delivery_point'
        }
        self._publish_json(self.config.TOPICS['pickup_box_delivered'], message)
        self.logger.info(f"Box delivered notification sent for {box_id}")
    
    def _publish_box_retrieved(self, box_id):
        """Publish box stored notification"""
        message = {
            'timestamp': datetime.now().isoformat(),
            'box_id': box_id,
            'status': 'stored',
            'position': 'storage_shelf'
        }
        self._publish_json(self.config.TOPICS['box_retrieved'], message)
        self.logger.info(f"Box stored notification sent for {box_id}")

    def _publish_status_now(self):
        """Immediately publish current status"""
        if self.servo_controller:
            system_info = self.servo_controller.get_system_info()
        else:
            system_info = {'connected': False}
        
        status_message = {
            'timestamp': datetime.now().isoformat(),
            'client_id': self.config.CLIENT_ID,
            'operational_state': self.operational_state,
            'current_box_id': self.current_box_id,
            **system_info
        }
        
        self._publish_json(self.config.TOPICS['robot_status'], status_message)

    def _publish_position_now(self):
        """Immediately publish current position"""
        position = None
        if self.servo_controller and self.servo_controller.is_initialized:
            position = self.servo_controller.get_current_position()
        
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
            'final_position': current_position
        }
        
        self._publish_json(self.config.TOPICS['robot_status'], result_message)

    def _publish_command_result(self, command, success):
        """Publish general command result"""
        result_message = {
            'timestamp': datetime.now().isoformat(),
            'command': command,
            'success': success
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
        self.logger.info("Starting MQTT Robot Controller...")
        self.start_time = time.time()
        self.running = True
        
        if not self.setup_mqtt():
            self.logger.error("Failed to setup MQTT, exiting...")
            return False
        
        try:
            while self.running:
                current_time = time.time()
                
                if current_time - self.last_status_time >= self.config.STATUS_INTERVAL:
                    self._publish_status_now()
                    self.last_status_time = current_time
                
                if (current_time - self.last_position_time >= self.config.POSITION_INTERVAL and
                    self.servo_controller and self.servo_controller.is_initialized):
                    self._publish_position_now()
                    self.last_position_time = current_time
                
                if current_time - self.last_heartbeat_time >= self.config.HEARTBEAT_INTERVAL:
                    self._publish_heartbeat()
                    self.last_heartbeat_time = current_time
                
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
        
        if self.servo_controller:
            try:
                self.servo_controller.close()
            except:
                pass
                
        if self.arduino:
            try:
                self.arduino.close()
            except:
                pass
        
        if self.mqtt_client:
            try:
                offline_message = {
                    'timestamp': datetime.now().isoformat(),
                    'client_id': self.config.CLIENT_ID,
                    'status': 'offline',
                    'message': 'Robot controller shutting down'
                }
                self._publish_json(self.config.TOPICS['robot_status'], offline_message)
                time.sleep(0.5)
                self.mqtt_client.loop_stop()
                self.mqtt_client.disconnect()
            except:
                pass
        
        self.logger.info("Shutdown complete")


# ========================================================================================
# SIGNAL HANDLERS & MAIN
# ========================================================================================

def signal_handler(signum, frame):
    """Handle system signals for graceful shutdown"""
    print(f"\nReceived signal {signum}, shutting down...")
    global mqtt_robot_controller
    if mqtt_robot_controller:
        mqtt_robot_controller.running = False


def main():
    """Main function"""
    global mqtt_robot_controller
    
    parser = argparse.ArgumentParser(description='MQTT Robot Controller for Kiosk')
    parser.add_argument('--interface', '-i', default='eth0', 
                       help='EtherCAT network interface (default: eth0)')
    parser.add_argument('--home-position', '-hp', type=int, default=0,
                       help='Absolute home position in encoder ticks (default: 0)')
    parser.add_argument('--broker', '-b', default='localhost', 
                       help='MQTT broker address (default: localhost)')
    parser.add_argument('--port', '-p', type=int, default=1883, 
                       help='MQTT broker port (default: 1883)')
    parser.add_argument('--auto-connect', '-a', action='store_true',
                       help='Automatically connect to motor on startup')
    
    args = parser.parse_args()
    
    config = Config()
    config.MQTT_BROKER = args.broker
    config.MQTT_PORT = args.port
    config.MOTOR_INTERFACE = args.interface
    config.MOTOR_HOME_POSITION = args.home_position
    
    print("Kiosk Robot Controller")
    print("=" * 50)
    print(f"MQTT Broker: {config.MQTT_BROKER}:{config.MQTT_PORT}")
    print(f"Client ID: {config.CLIENT_ID}")
    print(f"Motor Interface: {config.MOTOR_INTERFACE}")
    print(f"Motor Home Position: {config.MOTOR_HOME_POSITION} ticks")
    print(f"Auto-connect: {args.auto_connect}")
    print()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    mqtt_robot_controller = MQTTRobotController(config)
    
    if args.auto_connect:
        print("Auto-connecting to motor...")
        if mqtt_robot_controller._connect_motor(config.MOTOR_INTERFACE, config.MOTOR_HOME_POSITION):
            print("Motor connected successfully")
        else:
            print("Auto-connect failed, motor can be connected via MQTT command")
    
    try:
        mqtt_robot_controller.run()
    except Exception as e:
        print(f"Fatal error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    mqtt_robot_controller = None
    exit_code = main()
    sys.exit(exit_code)