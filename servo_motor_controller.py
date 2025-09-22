#!/usr/bin/env python3

import pysoem
import struct
import time
import math

class ServoMotorController:
    """
    EtherCAT servo motor controller with degree-based positioning
    
    Features:
    - Absolute position control in degrees
    - Configurable acceleration, deceleration, and velocity
    - Home position reference
    - Status monitoring and diagnostics
    """
    
    def __init__(self, interface="eth0"):
        """
        Initialize the servo motor controller
        
        Args:
            interface (str): Network interface name (e.g., "eth0", "enp3s0")
        """
        self.interface = interface
        self.master = None
        self.slave = None
        
        # Motor configuration
        self.encoder_resolution = 262144  # 18-bit encoder counts per revolution
        self.gear_ratio = 1.0            # Gearbox ratio (1.0 = direct drive)
        
        # State tracking
        self.is_initialized = False
        self.home_position = 0           # Encoder counts at "zero" position
        self.is_moving = False           # Movement status flag
        self.last_error = None           # Last error message
        
    
    def connect(self):
        """
        Connect to EtherCAT network and initialize motor
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        print(f"Connecting to EtherCAT on interface: {self.interface}")
        
        try:
            # Open EtherCAT master
            self.master = pysoem.Master()
            self.master.open(self.interface)

            # Discover EtherCAT slaves
            if self.master.config_init() <= 0:
                raise Exception("No EtherCAT slaves found")

            self.slave = self.master.slaves[0]
            print(f"Connected to slave: {self.slave.name}")
            
            # Initialize motor for position control
            self._setup_motor()
            self.is_initialized = True
            self.last_error = None
            print("Motor controller ready for position control!")
            return True
            
        except Exception as e:
            error_msg = f"Connection failed: {str(e)}"
            print(f"{error_msg}")
            self.last_error = error_msg
            if self.master:
                self.master.close()
            return False

    def _setup_motor(self):
        """Setup motor for Profile Position mode operation"""
        print("Setting up motor...")
        
        # Check current operation mode
        current_mode = self._read_register(0x6061, "Operation Mode Display", signed=True)
        print(f"   Current operation mode: {current_mode}")
        
        # Try to set Profile Position mode (mode 1)
        print("   Setting Profile Position mode...")
        try:
            self._write_register(0x6060, struct.pack('<b', 1), "Operation Mode")
            time.sleep(0.1)
        except Exception as e:
            print(f"Could not set operation mode: {e}")
            print("Motor may already be in correct mode")
        
        # Verify operation mode
        final_mode = self._read_register(0x6061, "Operation Mode Display", signed=True)
        print(f"   Final operation mode: {final_mode}")
        if final_mode != 1:
            print(f"   Warning: Motor is in mode {final_mode}, not Profile Position (1)")

        # Execute state machine to enable motor
        self._enable_motor()
        
        # Set home position (current position becomes "zero degrees")
        self.home_position = self._read_register(0x6064, "Position Actual", signed=True)
        print(f"   Home position set to: {self.home_position} counts")

    def _enable_motor(self):
        """Execute the EtherCAT state machine to enable motor operation"""
        print("   Enabling motor operation...")
        
        print("      → Shutdown")
        self._write_register(0x6040, struct.pack('<H', 0x0006), "Control Word")
        time.sleep(0.1)
        self._print_motor_state()
        
        print("      → Switch On")
        self._write_register(0x6040, struct.pack('<H', 0x0007), "Control Word") 
        time.sleep(0.1)
        self._print_motor_state()
        
        print("      → Operation Enabled")
        self._write_register(0x6040, struct.pack('<H', 0x000F), "Control Word")
        time.sleep(0.1)
        self._print_motor_state()
        
        # Verify motor is ready
        status = self.get_motor_status()
        if status and status['operation_enabled']:
            print("      Motor enabled and ready!")
        else:
            print("      Motor may not be properly enabled")

    # ========================================================================================
    # MOTION CONTROL
    # ========================================================================================
    
    def set_motion_parameters(self, acceleration=200000, deceleration=200000, max_velocity=100000):
        """
        Configure motion parameters for the motor
        
        Args:
            acceleration (int): Acceleration in counts/sec²
            deceleration (int): Deceleration in counts/sec²
            max_velocity (int): Maximum velocity in counts/sec
            
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.is_initialized:
            self.last_error = "Motor not initialized"
            return False
        
        try:
            print(f"Setting motion parameters:")
            print(f"   Acceleration: {acceleration:,}")
            print(f"   Deceleration: {deceleration:,}")
            print(f"   Max velocity: {max_velocity:,}")
            
            self._write_register(0x6083, struct.pack('<I', acceleration), "Profile Acceleration")
            self._write_register(0x6084, struct.pack('<I', deceleration), "Profile Deceleration")
            self._write_register(0x6081, struct.pack('<I', max_velocity), "Profile Velocity")
            time.sleep(0.1)
            self.last_error = None
            return True
            
        except Exception as e:
            error_msg = f"Failed to set motion parameters: {str(e)}"
            print(f"{error_msg}")
            self.last_error = error_msg
            return False

    def move_to_position(self, degrees, wait_for_completion=True, timeout=30.0):
        """
        Move motor to absolute position in degrees
        
        Args:
            degrees (float): Target position in degrees (0 = home position)
            wait_for_completion (bool): Wait for movement to complete
            timeout (float): Maximum time to wait in seconds
            
        Returns:
            bool: True if movement completed successfully
        """
        if not self.is_initialized:
            self.last_error = "Motor not initialized"
            return False
        
        try:
            self.is_moving = True
            
            # Calculate target position in encoder counts
            target_counts = self._degrees_to_counts(degrees)
            absolute_target = self.home_position + target_counts
            
            print(f"Moving to {degrees}° ({target_counts:+,} counts from home)")
            
            # Show current position
            current_pos = self._read_register(0x6064, "Position Actual", signed=True)
            current_degrees = self._counts_to_degrees(current_pos - self.home_position)
            print(f"   Current: {current_degrees:.2f}° ({current_pos:,} counts)")
            print(f"   Target:  {degrees}° ({absolute_target:,} counts)")
            
            # Verify motor is ready
            status = self.get_motor_status()
            if not (status and status['operation_enabled']):
                print("   Warning: Motor is not in operation enabled state!")
                self._print_motor_state()
            
            # Set target position and trigger motion
            self._write_register(0x607A, struct.pack('<i', absolute_target), "Target Position")
            time.sleep(0.05)
            
            # Trigger motion by setting "new set-point" bit
            print("   Starting motion...")
            self._write_register(0x6040, struct.pack('<H', 0x000F), "Control Word")  # Clear bit 4
            time.sleep(0.05)
            self._write_register(0x6040, struct.pack('<H', 0x001F), "Control Word")  # Set bit 4
            time.sleep(0.1)
            
            if wait_for_completion:
                success = self._wait_for_motion_complete(absolute_target, timeout)
                self.is_moving = False
                if success:
                    final_pos = self._read_register(0x6064, "Position Actual", signed=True)
                    final_degrees = self._counts_to_degrees(final_pos - self.home_position)
                    print(f"Movement completed! Final position: {final_degrees:.2f}°")
                self.last_error = None
                return success
            else:
                # For non-blocking moves, assume success for now
                self.last_error = None
                return True
                
        except Exception as e:
            error_msg = f"Movement failed: {str(e)}"
            print(f"{error_msg}")
            self.last_error = error_msg
            self.is_moving = False
            return False

    def move_relative(self, degrees, wait_for_completion=True, timeout=30.0):
        """
        Move motor relative to current position
        
        Args:
            degrees (float): Relative movement in degrees
            wait_for_completion (bool): Wait for movement to complete
            timeout (float): Maximum time to wait in seconds
            
        Returns:
            bool: True if movement completed successfully
        """
        current_degrees = self.get_current_position()
        if current_degrees is None:
            return False
        target_degrees = current_degrees + degrees
        return self.move_to_position(target_degrees, wait_for_completion, timeout)

    def go_home(self, wait_for_completion=True):
        """
        Return motor to home position (0 degrees)
        
        Returns:
            bool: True if successful
        """
        return self.move_to_position(0, wait_for_completion)

    # ========================================================================================
    # STATUS AND MONITORING
    # ========================================================================================
    
    def get_current_position(self):
        """
        Get current motor position in degrees relative to home
        
        Returns:
            float: Current position in degrees, None if error
        """
        if not self.is_initialized:
            return None
        try:
            current_counts = self._read_register(0x6064, "Position Actual", signed=True)
            relative_counts = current_counts - self.home_position
            return self._counts_to_degrees(relative_counts)
        except:
            return None

    def get_motor_status(self):
        """
        Get detailed motor status information
        
        Returns:
            dict: Motor status with boolean flags, None if error
        """
        if not self.is_initialized:
            return None
        try:
            status_word = self._read_register(0x6041, "Status Word")
            return {
                'status_word': status_word,
                'ready_to_switch_on': bool(status_word & 0x0001),
                'switched_on': bool(status_word & 0x0002),
                'operation_enabled': bool(status_word & 0x0004),
                'fault': bool(status_word & 0x0008),
                'quick_stop': bool(status_word & 0x0020),
                'switch_on_disabled': bool(status_word & 0x0040),
                'target_reached': bool(status_word & 0x0400),
                'is_moving': self.is_moving
            }
        except:
            return None

    def get_system_info(self):
        """
        Get comprehensive system information for status reporting
        
        Returns:
            dict: Complete system status
        """
        return {
            'connected': self.is_initialized,
            'interface': self.interface,
            'slave_name': self.slave.name if self.slave else None,
            'home_position': self.home_position,
            'current_position': self.get_current_position(),
            'motor_status': self.get_motor_status(),
            'is_moving': self.is_moving,
            'last_error': self.last_error
        }

    # ========================================================================================
    # SAFETY AND CLEANUP
    # ========================================================================================
    
    def emergency_stop(self):
        """
        Emergency stop - immediately disable motor
        
        Returns:
            bool: True if successful
        """
        print("EMERGENCY STOP!")
        if not self.is_initialized:
            return False
        try:
            self._write_register(0x6040, struct.pack('<H', 0x0002), "Control Word (Quick Stop)")
            self.is_moving = False
            self.last_error = None
            return True
        except Exception as e:
            error_msg = f"Emergency stop failed: {str(e)}"
            print(f"{error_msg}")
            self.last_error = error_msg
            return False

    def disable_motor(self):
        """
        Safely disable motor operation
        
        Returns:
            bool: True if successful
        """
        print("Disabling motor...")
        if not self.is_initialized:
            return True
        try:
            self._write_register(0x6040, struct.pack('<H', 0x0006), "Control Word (Shutdown)")
            time.sleep(0.05)
            self.is_moving = False
            return True
        except Exception as e:
            error_msg = f"Failed to disable motor: {str(e)}"
            print(f"{error_msg}")
            self.last_error = error_msg
            return False

    def close(self):
        """
        Clean shutdown of EtherCAT connection
        
        Returns:
            bool: True if successful
        """
        if self.master:
            try:
                if self.is_initialized:
                    self.disable_motor()
                self.master.close()
                print("EtherCAT connection closed")
                self.is_initialized = False
                return True
            except Exception as e:
                print(f"Error during close: {e}")
                return False
        return True

    # ========================================================================================
    # PRIVATE HELPER METHODS
    # ========================================================================================
    
    def _degrees_to_counts(self, degrees):
        """Convert degrees to encoder counts"""
        return int((degrees / 360.0) * self.encoder_resolution / self.gear_ratio)

    def _counts_to_degrees(self, counts):
        """Convert encoder counts to degrees"""
        return (counts * 360.0 * self.gear_ratio) / self.encoder_resolution

    def _read_register(self, address, name="Register", sub=0, signed=False):
        """
        Read SDO register with error handling
        
        Args:
            address (int): Register address (e.g., 0x6064)
            name (str): Human-readable register name for errors
            sub (int): Sub-index
            signed (bool): Interpret as signed integer
            
        Returns:
            int or None: Register value or None if error
        """
        try:
            data = self.slave.sdo_read(address, sub)
            return int.from_bytes(data, 'little', signed=signed)
        except Exception as e:
            print(f"Error reading {name} (0x{address:04X}:{sub}): {e}")
            return None

    def _write_register(self, address, data, name="Register", sub=0):
        """
        Write SDO register with error handling
        
        Args:
            address (int): Register address
            data (bytes): Data to write
            name (str): Human-readable register name
            sub (int): Sub-index
        """
        try:
            self.slave.sdo_write(address, sub, data)
        except Exception as e:
            print(f"Error writing {name} (0x{address:04X}:{sub}): {e}")
            raise

    def _print_motor_state(self):
        """Print brief motor state for debugging"""
        status_word = self._read_register(0x6041, "Status Word")
        if status_word:
            ready = bool(status_word & 0x0001)
            enabled = bool(status_word & 0x0004)
            fault = bool(status_word & 0x0008)
            print(f"         State: 0x{status_word:04X} (Ready:{ready} Enabled:{enabled} Fault:{fault})")

    def _wait_for_motion_complete(self, target_position, timeout=30.0, tolerance=500):
        """
        Wait for motor to reach target position
        
        Args:
            target_position (int): Target position in counts
            timeout (float): Maximum wait time in seconds
            tolerance (int): Position tolerance in counts
            
        Returns:
            bool: True if target reached within timeout
        """
        print(f"   Waiting for target: {target_position:,} counts...")
        start_time = time.time()
        last_position = None
        
        while time.time() - start_time < timeout:
            # Read current position
            current_pos = self._read_register(0x6064, "Position Actual", signed=True)
            if current_pos is None:
                continue
            
            # Show progress only when position changes significantly
            if last_position is None or abs(current_pos - last_position) > 1000:
                error = abs(target_position - current_pos)
                print(f"      Position: {current_pos:,} (error: {error:,})")
                last_position = current_pos
                
            # Check if within tolerance
            if abs(target_position - current_pos) <= tolerance:
                print("   Target position reached!")
                return True
                
            # Check status word for target reached flag
            status_word = self._read_register(0x6041, "Status Word")
            if status_word and (status_word & 0x0400):  # bit 10: target reached
                print("   Motor reports target reached!")
                return True
                
            time.sleep(0.1)
        
        # Timeout occurred
        final_pos = self._read_register(0x6064, "Position Actual", signed=True)
        print(f"   Timeout! Final position: {final_pos:,}, Target: {target_position:,}")
        return False