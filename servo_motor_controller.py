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
    - Configurable velocity and acceleration (default 500000 counts/sec and counts/sec²)
    - Absolute home position in encoder ticks
    - Robust motor enabling with multiple fallback methods
    - Status monitoring and diagnostics
    """
    
    def __init__(self, interface="eth0", home_position=0):
        """
        Initialize the servo motor controller
        
        Args:
            interface (str): Network interface name (e.g., "eth0", "enp3s0")
            home_position (int): Absolute home position in encoder ticks (default: 0)
        """
        self.interface = interface
        self.master = None
        self.slave = None
        
        # Motor configuration
        self.encoder_resolution = 262144  # 18-bit encoder counts per revolution
        self.gear_ratio = 1.0            # Gearbox ratio (1.0 = direct drive)
        
        # Motion parameters (default 500000 for both)
        self.current_velocity = 500000      # counts/sec
        self.current_acceleration = 500000  # counts/sec²
        
        # State tracking
        self.is_initialized = False
        self.home_position = home_position   # Absolute home position in encoder ticks
        self.is_moving = False               # Movement status flag
        self.motor_enabled = False           # Motor enable status
        self.last_error = None               # Last error message
        
    
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
            
            # Check current status first
            print("Checking current servo status...")
            status = self._read_status()
            if status:
                print(f"Current status: 0x{status:04X}")
                if status & 0x0004:  # Already enabled
                    print("Motor already enabled!")
                    self.motor_enabled = True
                else:
                    print("Motor not enabled, attempting to enable...")
                    self.motor_enabled = self._try_enable_motor()
            else:
                print("Cannot read status, attempting enable anyway...")
                self.motor_enabled = self._try_enable_motor()
            
            if not self.motor_enabled:
                print("WARNING: Motor may not be enabled. Some functions may not work.")
                print("Try the re_enable() method or check servo drive manually.")
            
            # Display absolute home position
            current_position = self._read_position()
            print(f"Absolute home position set to: {self.home_position:,} counts")
            print(f"Current absolute position: {current_position:,} counts")
            print(f"Relative position from home: {current_position - self.home_position:,} counts")
            
            # Set motion parameters
            self._set_motion_parameters()
            
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

    def _try_enable_motor(self):
        """Try multiple methods to enable the motor"""
        print("Trying different motor enabling methods...")
        
        # Method 1: Check if already enabled
        status = self._read_status()
        if status and (status & 0x0004):
            print("Method 1: Already enabled")
            return True
        
        # Method 2: Try direct operation enabled
        print("Method 2: Direct operation enabled")
        if self._safe_write(0x6040, struct.pack('<H', 0x000F)):
            time.sleep(0.3)
            status = self._read_status()
            if status and (status & 0x0004):
                print("Direct enable successful")
                return True
        
        # Method 3: Standard sequence
        print("Method 3: Standard CiA402 sequence")
        try:
            # Shutdown
            if self._safe_write(0x6040, struct.pack('<H', 0x0006)):
                time.sleep(0.3)
            
            # Switch On  
            if self._safe_write(0x6040, struct.pack('<H', 0x0007)):
                time.sleep(0.3)
            
            # Operation Enabled
            if self._safe_write(0x6040, struct.pack('<H', 0x000F)):
                time.sleep(0.3)
                status = self._read_status()
                if status and (status & 0x0004):
                    print("Standard sequence successful")
                    return True
        except:
            pass
        
        # Method 4: Try with voltage enable first
        print("Method 4: Voltage enable sequence")
        try:
            # Enable voltage
            if self._safe_write(0x6040, struct.pack('<H', 0x0002)):
                time.sleep(0.3)
            
            # Shutdown
            if self._safe_write(0x6040, struct.pack('<H', 0x0006)):
                time.sleep(0.3)
            
            # Switch On
            if self._safe_write(0x6040, struct.pack('<H', 0x0007)):
                time.sleep(0.3)
            
            # Operation Enabled
            if self._safe_write(0x6040, struct.pack('<H', 0x000F)):
                time.sleep(0.3)
                status = self._read_status()
                if status and (status & 0x0004):
                    print("Voltage enable sequence successful")
                    return True
        except:
            pass
        
        print("All enabling methods failed")
        return False

    def _set_motion_parameters(self):
        """Set motion parameters"""
        print(f"Setting motion parameters:")
        print(f"   Velocity: {self.current_velocity:,} counts/sec")
        print(f"   Acceleration: {self.current_acceleration:,} counts/sec²")
        print(f"   Deceleration: {self.current_acceleration:,} counts/sec²")
        
        success = True
        success &= self._safe_write(0x6081, struct.pack('<I', self.current_velocity))
        success &= self._safe_write(0x6083, struct.pack('<I', self.current_acceleration))
        success &= self._safe_write(0x6084, struct.pack('<I', self.current_acceleration))
        
        if not success:
            print("Warning: Some motion parameters failed to set")
        
        time.sleep(0.1)

    def set_motion_parameters(self, velocity=None, acceleration=None):
        """
        Set motion parameters
        
        Args:
            velocity (int): Velocity in counts/sec (None to keep current)
            acceleration (int): Acceleration in counts/sec² (None to keep current)
        """
        if velocity is not None:
            self.current_velocity = velocity
        if acceleration is not None:
            self.current_acceleration = acceleration
        
        self._set_motion_parameters()

    def set_home_position(self, ticks=None):
        """
        Set home position to specific tick value or current position
        
        Args:
            ticks (int): Absolute tick value for home (None = use current position)
        """
        if ticks is None:
            # Set current position as home
            self.home_position = self._read_position()
            print(f"Home position set to current position: {self.home_position:,} counts")
        else:
            # Set specified absolute tick value as home
            self.home_position = int(ticks)
            current = self._read_position()
            print(f"Home position set to: {self.home_position:,} counts")
            print(f"Current position: {current:,} counts")
            print(f"Offset from home: {current - self.home_position:,} counts")

    # ========================================================================================
    # MOTION CONTROL
    # ========================================================================================
    
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
        
        if not self.motor_enabled:
            print("Warning: Motor may not be enabled. Attempting move anyway...")
        
        try:
            self.is_moving = True
            
            # Calculate target position in encoder counts
            target_counts = self._degrees_to_counts(degrees)
            absolute_target = self.home_position + target_counts
            
            print(f"Moving to {degrees}° ({target_counts:+,} counts from home)")
            
            # Show current position
            current_pos = self._read_position()
            current_degrees = self._counts_to_degrees(current_pos - self.home_position)
            print(f"   Current: {current_degrees:.2f}° ({current_pos:,} counts)")
            print(f"   Target:  {degrees}° ({absolute_target:,} counts)")
            
            # Set target position
            if not self._safe_write(0x607A, struct.pack('<i', absolute_target)):
                print("Failed to set target position")
                self.is_moving = False
                return False
            
            time.sleep(0.1)
            
            # Trigger move (set new setpoint bit 4)
            print("   Starting motion...")
            if self._safe_write(0x6040, struct.pack('<H', 0x001F)):  # Set bit 4
                time.sleep(0.1)
                self._safe_write(0x6040, struct.pack('<H', 0x000F))  # Clear bit 4
            else:
                print("Failed to trigger motion")
                self.is_moving = False
                return False
            
            if wait_for_completion:
                success = self._wait_for_motion_complete(absolute_target, timeout)
                self.is_moving = False
                if success:
                    final_pos = self._read_position()
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
        print(f"Moving {degrees:+.1f} degrees (from {current_degrees:.1f} to {target_degrees:.1f})")
        return self.move_to_position(target_degrees, wait_for_completion, timeout)

    def go_home(self, wait_for_completion=True):
        """
        Return motor to home position (0 degrees)
        
        Returns:
            bool: True if successful
        """
        print("Returning to home position...")
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
            current_counts = self._read_position()
            relative_counts = current_counts - self.home_position
            return self._counts_to_degrees(relative_counts)
        except:
            return None

    def show_encoder_info(self):
        """Display encoder ticks and position information"""
        absolute_counts = self._read_position()
        relative_counts = absolute_counts - self.home_position
        degrees = (relative_counts * 360.0) / self.encoder_resolution
        revolutions = degrees / 360.0
        
        print("\n" + "="*60)
        print("ENCODER POSITION")
        print("="*60)
        print(f"Absolute encoder ticks:  {absolute_counts:,} counts")
        print(f"Home position:           {self.home_position:,} counts")
        print(f"Relative encoder ticks:  {relative_counts:,} counts")
        print(f"Position in degrees:     {degrees:.2f}°")
        print(f"Position in revolutions: {revolutions:.3f} rev")
        print(f"Encoder resolution:      {self.encoder_resolution:,} counts/rev")
        print("="*60 + "\n")

    def get_motor_status(self):
        """
        Get detailed motor status information
        
        Returns:
            dict: Motor status with boolean flags, None if error
        """
        if not self.is_initialized:
            return None
        try:
            status_word = self._read_status()
            if status_word is None:
                return None
                
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
            'velocity': self.current_velocity,
            'acceleration': self.current_acceleration,
            'motor_status': self.get_motor_status(),
            'motor_enabled': self.motor_enabled,
            'is_moving': self.is_moving,
            'last_error': self.last_error
        }

    def print_status(self):
        """Print detailed status information"""
        status = self._read_status()
        position = self._read_position()
        degrees = self.get_current_position()
        
        print(f"Position: {position:,} counts ({degrees:.2f} degrees)")
        print(f"Velocity: {self.current_velocity:,} counts/sec")
        print(f"Acceleration: {self.current_acceleration:,} counts/sec²")
        print(f"Motor enabled (cached): {self.motor_enabled}")
        
        if status:
            print(f"Status Word: 0x{status:04X}")
            ready = bool(status & 0x0001)
            switched_on = bool(status & 0x0002)
            enabled = bool(status & 0x0004)
            fault = bool(status & 0x0008)
            target_reached = bool(status & 0x0400)
            
            print(f"Ready: {ready}, Switched On: {switched_on}, Enabled: {enabled}")
            print(f"Fault: {fault}, Target Reached: {target_reached}")
            
            # Update cached status
            self.motor_enabled = enabled
        else:
            print("Could not read status word")

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
        result = self._safe_write(0x6040, struct.pack('<H', 0x0002))
        self.is_moving = False
        self.motor_enabled = False
        self.last_error = None
        return result

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
            self._safe_write(0x6040, struct.pack('<H', 0x0006))  # Shutdown
            time.sleep(0.1)
            self._safe_write(0x6040, struct.pack('<H', 0x0000))  # Disable voltage
            self.is_moving = False
            self.motor_enabled = False
            return True
        except Exception as e:
            error_msg = f"Failed to disable motor: {str(e)}"
            print(f"{error_msg}")
            self.last_error = error_msg
            return False

    def re_enable(self):
        """
        Re-enable motor if it gets disabled
        
        Returns:
            bool: True if successful
        """
        print("Re-enabling motor...")
        self.motor_enabled = self._try_enable_motor()
        return self.motor_enabled

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

    def _safe_write(self, address, data):
        """Write to register with error handling"""
        try:
            self.slave.sdo_write(address, 0, data)
            return True
        except Exception as e:
            print(f"Write failed 0x{address:04X}: {e}")
            return False

    def _read_position(self):
        """Read current position in counts"""
        try:
            data = self.slave.sdo_read(0x6064, 0)  # Position Actual
            return int.from_bytes(data, 'little', signed=True)
        except Exception as e:
            print(f"Error reading position: {e}")
            return self.home_position  # Return home as fallback

    def _read_status(self):
        """Read status word with retry"""
        for retry in range(3):
            try:
                data = self.slave.sdo_read(0x6041, 0)  # Status Word
                return int.from_bytes(data, 'little', signed=False)
            except:
                time.sleep(0.05)
                continue
        return None

    def _wait_for_motion_complete(self, target_position, timeout=30.0, tolerance=1000):
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
        last_print_time = 0
        
        while time.time() - start_time < timeout:
            # Read current position
            current_pos = self._read_position()
            error = abs(target_position - current_pos)
            
            # Show progress every 2 seconds
            current_time = time.time()
            if current_time - last_print_time > 2.0:
                print(f"      Current: {current_pos:,}, Target: {target_position:,}, Error: {error:,}")
                last_print_time = current_time
                
            # Check if within tolerance
            if error < tolerance:
                print("   Target position reached!")
                return True
                
            time.sleep(0.1)
        
        # Timeout occurred
        final_pos = self._read_position()
        print(f"   Timeout! Final position: {final_pos:,}, Target: {target_position:,}")
        return False