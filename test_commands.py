#!/usr/bin/env python3
# coding=utf-8

# Test Commands Script - Send test commands to robot via MQTT
# Use this to test robot functions from command line

import json
import time
import paho.mqtt.client as mqtt
import sys

class RobotTester:
    def __init__(self, broker_host='192.168.1.173'):
        self.broker_host = broker_host
        self.client_id = f"robot_tester_{int(time.time())}"
        self.connected = False
        
        # Topic structure (same as PC client)
        self.topics = {
            'status': 'rider/status',
            'battery': 'rider/status/battery', 
            'imu': 'rider/status/imu',
            'control_movement': 'rider/control/movement',
            'control_settings': 'rider/control/settings',
            'control_camera': 'rider/control/camera',
            'control_system': 'rider/control/system',
            'request_battery': 'rider/request/battery'
        }
        
        self.connect_mqtt()
    
    def connect_mqtt(self):
        """Connect to MQTT broker"""
        try:
            self.mqtt_client = mqtt.Client(
                client_id=self.client_id,
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                protocol=mqtt.MQTTv5
            )
            self.mqtt_client.on_connect = self.on_connect
            
            print(f"🔌 Connecting to {self.broker_host}...")
            self.mqtt_client.connect(self.broker_host, 1883, 60)
            self.mqtt_client.loop_start()
            time.sleep(1)  # Wait for connection
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            sys.exit(1)
    
    def on_connect(self, client, userdata, flags, reason_code, properties):
        """Connection callback"""
        if reason_code == 0:
            self.connected = True
            print(f"✅ Connected to robot at {self.broker_host}")
        else:
            print(f"❌ Connection failed, code: {reason_code}")
    
    def send_command(self, topic, command_data):
        """Send command to robot"""
        if not self.connected:
            print("❌ Not connected to robot")
            return False
        
        try:
            payload = json.dumps(command_data)
            result = self.mqtt_client.publish(topic, payload)
            print(f"📤 Sent to {topic}: {payload}")
            return result.rc == 0
        except Exception as e:
            print(f"❌ Failed to send command: {e}")
            return False
    
    def test_movement(self):
        """Test movement commands"""
        print("\n🏃 Testing Movement Commands...")
        
        movements = [
            ("Forward", 0, 50),
            ("Backward", 0, -50),
            ("Left", -50, 0),
            ("Right", 50, 0),
            ("Stop", 0, 0)
        ]
        
        for name, x, y in movements:
            print(f"  {name}: x={x}, y={y}")
            command = {'x': x, 'y': y, 'timestamp': time.time()}
            self.send_command(self.topics['control_movement'], command)
            time.sleep(1)
    
    def test_settings(self):
        """Test settings commands"""
        print("\n⚙️ Testing Settings Commands...")
        
        settings = [
            "toggle_roll_balance",
            "toggle_performance", 
        ]
        
        for setting in settings:
            print(f"  {setting}")
            command = {'action': setting, 'timestamp': time.time()}
            self.send_command(self.topics['control_settings'], command)
            time.sleep(1)
        
        # Test speed change
        print("  change_speed to 1.5")
        command = {'action': 'change_speed', 'value': 1.5, 'timestamp': time.time()}
        self.send_command(self.topics['control_settings'], command)
        time.sleep(1)
    
    def test_camera(self):
        """Test camera commands"""
        print("\n📷 Testing Camera Commands...")
        
        command = {'action': 'toggle_camera', 'timestamp': time.time()}
        self.send_command(self.topics['control_camera'], command)
    
    def test_system(self):
        """Test system commands"""
        print("\n🔋 Testing System Commands...")
        
        # Request battery update
        command = {'action': 'request_battery', 'timestamp': time.time()}
        self.send_command(self.topics['request_battery'], command)
    
    def run_all_tests(self):
        """Run all tests"""
        print("🤖 Robot Function Test Suite")
        print("=" * 50)
        
        if not self.connected:
            print("❌ Not connected to robot. Exiting.")
            return
        
        self.test_movement()
        self.test_settings()
        self.test_camera()
        self.test_system()
        
        print("\n✅ All tests completed!")
        print("💡 Check your PC client GUI and robot to see the effects")
    
    def interactive_mode(self):
        """Interactive testing mode"""
        print("\n🎮 Interactive Mode - Type commands:")
        print("Commands: move, settings, camera, battery, all, quit")
        
        while True:
            try:
                cmd = input("\ntest> ").strip().lower()
                
                if cmd == 'quit' or cmd == 'q':
                    break
                elif cmd == 'move' or cmd == 'm':
                    self.test_movement()
                elif cmd == 'settings' or cmd == 's':
                    self.test_settings()
                elif cmd == 'camera' or cmd == 'c':
                    self.test_camera()
                elif cmd == 'battery' or cmd == 'b':
                    self.test_system()
                elif cmd == 'all':
                    self.run_all_tests()
                else:
                    print("Unknown command. Try: move, settings, camera, battery, all, quit")
                    
            except KeyboardInterrupt:
                break
        
        print("\n👋 Goodbye!")
    
    def cleanup(self):
        """Cleanup connection"""
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()

def main():
    if len(sys.argv) > 1 and sys.argv[1] == '-h':
        print("Usage:")
        print("  python3 test_commands.py [robot_ip]")
        print("  python3 test_commands.py -i [robot_ip]  # Interactive mode")
        return
    
    # Get robot IP
    robot_ip = '192.168.1.173'
    interactive = False
    
    if len(sys.argv) > 1:
        if sys.argv[1] == '-i':
            interactive = True
            if len(sys.argv) > 2:
                robot_ip = sys.argv[2]
        else:
            robot_ip = sys.argv[1]
    
    tester = RobotTester(robot_ip)
    
    try:
        if interactive:
            tester.interactive_mode()
        else:
            tester.run_all_tests()
    finally:
        tester.cleanup()

if __name__ == "__main__":
    main() 