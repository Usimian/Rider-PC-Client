#!/usr/bin/env python3
# coding=utf-8

"""
Movement Calibration Tester

This tool helps you test and calibrate robot movement commands.
Use this to determine the actual values needed for specific movements.

Usage:
1. Run this script
2. Enter movement commands with specific values
3. Observe actual robot movement
4. Record the actual distance/angle achieved
5. Update movement_calibration.json with correct values
"""

import sys
import time
sys.path.insert(0, '.')

from core.config_manager import ConfigManager
from communication.mqtt_client import MQTTClient
from core.movement_calibration import MovementCalibration

class CalibrationTester:
    def __init__(self):
        print("=" * 60)
        print("ROBOT MOVEMENT CALIBRATION TESTER")
        print("=" * 60)

        # Load configuration
        self.config = ConfigManager()
        broker_host = self.config.get_broker_host()
        broker_port = self.config.get_broker_port()

        print(f"\nConnecting to robot at {broker_host}:{broker_port}...")

        # Connect to MQTT
        self.mqtt_client = MQTTClient(broker_host, broker_port, debug=False)
        self.mqtt_client.connect()
        time.sleep(2)  # Wait for connection

        if not self.mqtt_client.is_connected():
            print("❌ Failed to connect to robot!")
            sys.exit(1)

        print("✅ Connected to robot")

        # Load calibration
        self.calibration = MovementCalibration()

    def test_command(self, x: int, y: int, duration: float = 1.0):
        """Send a movement command for a specific duration"""
        print(f"\n▶ Sending command: x={x}, y={y} for {duration}s")
        print("   (Measure the actual movement now!)")

        # Send movement command
        self.mqtt_client.send_movement_command(x, y)

        # Wait for duration
        time.sleep(duration)

        # Stop
        self.mqtt_client.send_movement_command(0, 0)
        print("⏹ Movement stopped")

    def test_forward(self, speed_value: int, duration: float = 1.0):
        """Test forward movement"""
        print(f"\n🔹 Testing FORWARD movement")
        self.test_command(0, speed_value, duration)

    def test_backward(self, speed_value: int, duration: float = 1.0):
        """Test backward movement"""
        print(f"\n🔹 Testing BACKWARD movement")
        self.test_command(0, -speed_value, duration)

    def test_turn_left(self, turn_value: int, duration: float = 1.0):
        """Test left turn"""
        print(f"\n🔹 Testing LEFT TURN")
        self.test_command(turn_value, 0, duration)

    def test_turn_right(self, turn_value: int, duration: float = 1.0):
        """Test right turn"""
        print(f"\n🔹 Testing RIGHT TURN")
        self.test_command(-turn_value, 0, duration)

    def show_current_calibration(self):
        """Display current calibration values"""
        print("\n" + "=" * 60)
        print("CURRENT CALIBRATION VALUES")
        print("=" * 60)

        points = self.calibration.list_calibration_points()
        for movement_type, point_list in points.items():
            print(f"\n{movement_type.upper()}:")
            for point in point_list:
                print(f"  {point['name']:10s} = {point['value']:4d}  # {point['description']}")

    def interactive_mode(self):
        """Interactive calibration mode"""
        print("\n" + "=" * 60)
        print("INTERACTIVE CALIBRATION MODE")
        print("=" * 60)
        print("\nCommands:")
        print("  f <value> <duration> - Test forward (e.g., 'f 10 2' = value 10 for 2 seconds)")
        print("  b <value> <duration> - Test backward")
        print("  l <value> <duration> - Test left turn")
        print("  r <value> <duration> - Test right turn")
        print("  show                 - Show current calibration")
        print("  update <type> <name> <value> - Update calibration (e.g., 'update forward slow 8')")
        print("  save                 - Save calibration to file")
        print("  quit                 - Exit")
        print()

        while True:
            try:
                cmd = input("calibration> ").strip().lower()

                if not cmd:
                    continue

                parts = cmd.split()

                if cmd == 'quit' or cmd == 'exit':
                    break

                elif cmd == 'show':
                    self.show_current_calibration()

                elif cmd == 'save':
                    self.calibration.save_calibration()

                elif parts[0] == 'f' and len(parts) >= 2:
                    value = int(parts[1])
                    duration = float(parts[2]) if len(parts) > 2 else 1.0
                    self.test_forward(value, duration)

                elif parts[0] == 'b' and len(parts) >= 2:
                    value = int(parts[1])
                    duration = float(parts[2]) if len(parts) > 2 else 1.0
                    self.test_backward(value, duration)

                elif parts[0] == 'l' and len(parts) >= 2:
                    value = int(parts[1])
                    duration = float(parts[2]) if len(parts) > 2 else 1.0
                    self.test_turn_left(value, duration)

                elif parts[0] == 'r' and len(parts) >= 2:
                    value = int(parts[1])
                    duration = float(parts[2]) if len(parts) > 2 else 1.0
                    self.test_turn_right(value, duration)

                elif parts[0] == 'update' and len(parts) >= 4:
                    movement_type = parts[1]
                    name = parts[2]
                    value = int(parts[3])
                    self.calibration.update_calibration_point(movement_type, name, value)

                else:
                    print("❌ Unknown command or invalid syntax")

            except ValueError as e:
                print(f"❌ Invalid value: {e}")
            except KeyboardInterrupt:
                print("\n\n👋 Interrupted by user")
                break
            except Exception as e:
                print(f"❌ Error: {e}")

    def cleanup(self):
        """Clean up and disconnect"""
        print("\n🧹 Cleaning up...")
        if self.mqtt_client:
            # Stop robot
            self.mqtt_client.send_movement_command(0, 0)
            time.sleep(0.5)
            self.mqtt_client.disconnect()
        print("✅ Disconnected")


def main():
    tester = CalibrationTester()
    try:
        tester.show_current_calibration()
        tester.interactive_mode()
    finally:
        tester.cleanup()


if __name__ == "__main__":
    main()
