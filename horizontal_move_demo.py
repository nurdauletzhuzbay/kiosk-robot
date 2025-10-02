#!/usr/bin/env python3

import time
import signal
import sys
import argparse

# Import servo motor controller (EtherCAT for horizontal movement)
from servo_motor_controller import ServoMotorController

class SimpleRobotMover:
    """Simple robot mover - back and forth between home and 1500"""
    
    def __init__(self, interface='eth0', home_position=0):
        self.interface = interface
        self.home_position = home_position
        self.servo_controller = None
        self.running = False
        
    def connect(self):
        """Connect to the servo motor"""
        try:
            print(f"Connecting to servo on {self.interface}...")
            self.servo_controller = ServoMotorController(self.interface, self.home_position)
            success = self.servo_controller.connect()
            
            if success:
                # Set motion parameters
                self.servo_controller.set_motion_parameters(
                    velocity=500000,
                    acceleration=500000
                )
                print("✓ Servo connected and configured")
                return True
            else:
                print("✗ Failed to connect to servo")
                return False
                
        except Exception as e:
            print(f"✗ Connection error: {e}")
            return False
    
    def move_cycle(self):
        """Execute one cycle: home -> 1500 -> home"""
        try:
            # Move to position 1500
            print("\n→ Moving to position 1500...")
            success = self.servo_controller.move_to_position(1500, wait_for_completion=True, timeout=30.0)
            if not success:
                print("✗ Failed to reach position 1500")
                return False
            
            print("✓ Reached 1500")
            print("  Waiting 5 seconds...")
            time.sleep(5.0)
            
            # Move back to home (0)
            print("\n→ Moving to home position (0)...")
            success = self.servo_controller.move_to_position(0, wait_for_completion=True, timeout=30.0)
            if not success:
                print("✗ Failed to reach home position")
                return False
            
            print("✓ Reached home")
            print("  Waiting 5 seconds...")
            time.sleep(5.0)
            
            return True
            
        except Exception as e:
            print(f"✗ Move cycle error: {e}")
            return False
    
    def run(self, cycles=None):
        """Run the back-and-forth movement
        
        Args:
            cycles: Number of cycles to run (None = infinite)
        """
        self.running = True
        cycle_count = 0
        
        print("\n" + "="*50)
        print("Starting back-and-forth movement")
        print(f"Cycles: {'Infinite (Ctrl+C to stop)' if cycles is None else cycles}")
        print("="*50)
        
        try:
            while self.running:
                cycle_count += 1
                print(f"\n{'='*50}")
                print(f"Cycle {cycle_count}")
                print('='*50)
                
                if not self.move_cycle():
                    print("Movement failed, stopping...")
                    break
                
                if cycles is not None and cycle_count >= cycles:
                    print(f"\nCompleted {cycles} cycles")
                    break
                    
        except KeyboardInterrupt:
            print("\n\nKeyboard interrupt received")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """Clean shutdown"""
        print("\nShutting down...")
        self.running = False
        
        if self.servo_controller:
            try:
                print("Returning to home position...")
                self.servo_controller.move_to_position(0, wait_for_completion=True, timeout=30.0)
                self.servo_controller.close()
                print("✓ Servo disconnected")
            except:
                pass
        
        print("✓ Shutdown complete")


# Global variable for signal handler
robot_mover = None

def signal_handler(signum, frame):
    """Handle Ctrl+C gracefully"""
    print(f"\nReceived signal {signum}")
    global robot_mover
    if robot_mover:
        robot_mover.running = False


def main():
    """Main function"""
    global robot_mover
    
    parser = argparse.ArgumentParser(description='Simple Robot Back-and-Forth Mover')
    parser.add_argument('--interface', '-i', default='eth0',
                       help='EtherCAT network interface (default: eth0)')
    parser.add_argument('--home-position', '-hp', type=int, default=0,
                       help='Absolute home position in encoder ticks (default: 0)')
    parser.add_argument('--cycles', '-c', type=int, default=None,
                       help='Number of cycles to run (default: infinite)')
    
    args = parser.parse_args()
    
    print("\nSimple Robot Mover")
    print("="*50)
    print(f"Interface: {args.interface}")
    print(f"Home Position: {args.home_position} ticks")
    print(f"Target Position: 1500 degrees")
    print(f"Wait Time: 5 seconds at each position")
    print()
    
    # Setup signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Create and run robot mover
    robot_mover = SimpleRobotMover(args.interface, args.home_position)
    
    if robot_mover.connect():
        robot_mover.run(cycles=args.cycles)
    else:
        print("Failed to connect to servo motor")
        return 1
    
    return 0


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)