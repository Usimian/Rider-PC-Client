#!/usr/bin/env python3
# coding=utf-8

# Raspberry Pi Robot Server Template
# Implements the MQTT protocol for communication with PC Client
# This is a starting template - customize for your specific hardware

import json
import time
import threading
import paho.mqtt.client as mqtt
from datetime import datetime

class RiderRobotServer:
    def __init__(self, broker_host='localhost', broker_port=1883):
        # MQTT Configuration
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.client_id = f"rider_robot_server_{int(time.time())}"
        
        # Robot State
        self.robot_state = {
            'speed_scale': 1.0,
            'roll_balance_enabled': False,
            'performance_mode_enabled': False,
            'camera_enabled': False,
            'controller_connected': False,
            'height': 85,
            'connection_status': 'connected',
            'battery_level': 100,
            'battery_status': 'normal',
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }
        
        # Topic structure
        self.topics = {
            # Topics to publish (status updates)
            'status': 'rider/status',
            'battery': 'rider/status/battery',
            'imu': 'rider/status/imu',
            
            # Topics to subscribe to (commands from PC)
            'control_movement': 'rider/control/movement',
            'control_settings': 'rider/control/settings',
            'control_camera': 'rider/control/camera',
            'control_system': 'rider/control/system',
            'request_battery': 'rider/request/battery'
        }
        
        # Threading control
        self.running = True
        
        # Initialize MQTT and start publishing threads
        self.connect_mqtt()
        self.start_status_threads()
    
    def connect_mqtt(self):
        """Connect to MQTT broker"""
        try:
            self.mqtt_client = mqtt.Client(
                client_id=self.client_id,
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                protocol=mqtt.MQTTv5
            )
            
            self.mqtt_client.on_connect = self.on_connect
            self.mqtt_client.on_disconnect = self.on_disconnect
            self.mqtt_client.on_message = self.on_message
            
            print(f"🔌 Connecting to MQTT broker at {self.broker_host}:{self.broker_port}")
            self.mqtt_client.connect(self.broker_host, self.broker_port, 60)
            self.mqtt_client.loop_start()
            
        except Exception as e:
            print(f"❌ MQTT connection failed: {e}")
    
    def on_connect(self, client, userdata, flags, reason_code, properties):
        """Callback for MQTT connection"""
        if reason_code == 0:
            print(f"✅ Connected to MQTT broker")
            
            # Subscribe to command topics
            command_topics = [
                self.topics['control_movement'],
                self.topics['control_settings'],
                self.topics['control_camera'],
                self.topics['control_system'],
                self.topics['request_battery']
            ]
            
            for topic in command_topics:
                client.subscribe(topic)
                print(f"📡 Subscribed to {topic}")
        else:
            print(f"❌ MQTT connection failed, reason code: {reason_code}")
    
    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        """Callback for MQTT disconnection"""
        print("❌ Disconnected from MQTT broker")
    
    def on_message(self, client, userdata, msg):
        """Handle incoming MQTT messages (commands from PC client)"""
        try:
            topic = msg.topic
            payload = json.loads(msg.payload.decode())
            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            
            print(f"📩 [{timestamp}] Command received on {topic}")
            print(f"    Data: {payload}")
            
            # Route message to appropriate handler
            if topic == self.topics['control_movement']:
                self.handle_movement_command(payload)
            elif topic == self.topics['control_settings']:
                self.handle_settings_command(payload)
            elif topic == self.topics['control_camera']:
                self.handle_camera_command(payload)
            elif topic == self.topics['control_system']:
                self.handle_system_command(payload)
            elif topic == self.topics['request_battery']:
                self.handle_battery_request(payload)
                
        except Exception as e:
            print(f"❌ Error processing command: {e}")
    
    def handle_movement_command(self, data):
        """Handle movement commands from PC client"""
        x = data.get('x', 0)  # -100 to +100 (left/right)
        y = data.get('y', 0)  # -100 to +100 (backward/forward)
        
        print(f"🏃 Movement command: x={x}, y={y}")
        
        # TODO: Implement your movement logic here
        # Example:
        # - Convert x,y to motor speeds
        # - Apply speed_scale multiplier
        # - Send commands to motor controllers
        # - Handle roll balance if enabled
        
        # Placeholder implementation
        if x == 0 and y == 0:
            print("   ⏹ Stopping robot")
            # self.stop_motors()
        elif y > 0:
            print(f"   ⬆ Moving forward (speed: {y * self.robot_state['speed_scale']})")
            # self.move_forward(y * self.robot_state['speed_scale'])
        elif y < 0:
            print(f"   ⬇ Moving backward (speed: {abs(y) * self.robot_state['speed_scale']})")
            # self.move_backward(abs(y) * self.robot_state['speed_scale'])
        
        if x != 0:
            direction = "right" if x > 0 else "left"
            print(f"   ↔ Turning {direction} (amount: {abs(x)})")
            # self.turn(x)
    
    def handle_settings_command(self, data):
        """Handle settings commands from PC client"""
        action = data.get('action')
        
        print(f"⚙️ Settings command: {action}")
        
        if action == 'toggle_roll_balance':
            self.robot_state['roll_balance_enabled'] = not self.robot_state['roll_balance_enabled']
            status = "enabled" if self.robot_state['roll_balance_enabled'] else "disabled"
            print(f"   🎯 Roll balance {status}")
            # TODO: Enable/disable roll balance compensation
            
        elif action == 'toggle_performance':
            self.robot_state['performance_mode_enabled'] = not self.robot_state['performance_mode_enabled']
            status = "enabled" if self.robot_state['performance_mode_enabled'] else "disabled"
            print(f"   🚀 Performance mode {status}")
            # TODO: Switch between normal/performance modes
            
        elif action == 'change_speed':
            new_speed = data.get('value', 1.0)
            # Validate speed range
            new_speed = max(0.1, min(2.0, new_speed))
            self.robot_state['speed_scale'] = new_speed
            print(f"   🏃 Speed changed to {new_speed}x")
            # TODO: Update movement speed calculations
        
        # Publish updated status
        self.publish_status()
    
    def handle_camera_command(self, data):
        """Handle camera commands from PC client"""
        print(f"📷 Camera command: {data.get('action')}")
        
        self.robot_state['camera_enabled'] = not self.robot_state['camera_enabled']
        status = "enabled" if self.robot_state['camera_enabled'] else "disabled"
        print(f"   📹 Camera {status}")
        
        # TODO: Start/stop camera stream
        # if self.robot_state['camera_enabled']:
        #     self.start_camera()
        # else:
        #     self.stop_camera()
        
        # Publish updated status
        self.publish_status()
    
    def handle_system_command(self, data):
        """Handle system commands from PC client"""
        action = data.get('action')
        
        print(f"🛑 System command: {action}")
        
        if action == 'emergency_stop':
            print("   🚨 EMERGENCY STOP - Stopping all movement")
            # TODO: Implement emergency stop
            # - Stop all motors immediately
            # - Disable all movement
            # - Set robot to safe state
            # self.emergency_stop()
            
        # Publish updated status
        self.publish_status()
    
    def handle_battery_request(self, data):
        """Handle battery status requests from PC client"""
        print("🔋 Battery status requested")
        self.publish_battery_status()
    
    def publish_status(self):
        """Publish robot status to MQTT"""
        status = {
            "timestamp": time.time(),
            "speed_scale": self.robot_state['speed_scale'],
            "roll_balance_enabled": self.robot_state['roll_balance_enabled'],
            "performance_mode_enabled": self.robot_state['performance_mode_enabled'],
            "camera_enabled": self.robot_state['camera_enabled'],
            "controller_connected": self.robot_state['controller_connected'],
            "height": self.robot_state['height'],
            "connection_status": self.robot_state['connection_status']
        }
        
        try:
            self.mqtt_client.publish(self.topics['status'], json.dumps(status))
            print(f"📤 Published status update")
        except Exception as e:
            print(f"❌ Failed to publish status: {e}")
    
    def publish_battery_status(self):
        """Publish battery status to MQTT"""
        # TODO: Read actual battery level from hardware
        # battery_level = self.read_battery_level()
        battery_level = self.robot_state['battery_level']
        
        battery = {
            "timestamp": time.time(),
            "level": battery_level,
            "status": self.robot_state['battery_status'],
            "source": "hardware"
        }
        
        try:
            self.mqtt_client.publish(self.topics['battery'], json.dumps(battery))
            print(f"📤 Published battery status: {battery_level}%")
        except Exception as e:
            print(f"❌ Failed to publish battery status: {e}")
    
    def publish_imu_data(self):
        """Publish IMU data to MQTT"""
        # TODO: Read actual IMU data from sensors
        # roll, pitch, yaw = self.read_imu_data()
        
        imu = {
            "timestamp": time.time(),
            "roll": self.robot_state['roll'],
            "pitch": self.robot_state['pitch'],
            "yaw": self.robot_state['yaw']
        }
        
        try:
            self.mqtt_client.publish(self.topics['imu'], json.dumps(imu))
        except Exception as e:
            print(f"❌ Failed to publish IMU data: {e}")
    
    def start_status_threads(self):
        """Start background threads for status publishing"""
        
        def status_publisher():
            """Publish status every 2 seconds"""
            while self.running:
                try:
                    self.publish_status()
                    time.sleep(2.0)
                except Exception as e:
                    print(f"❌ Status publisher error: {e}")
                    time.sleep(1.0)
        
        def battery_publisher():
            """Publish battery status every 10 seconds"""
            while self.running:
                try:
                    self.publish_battery_status()
                    time.sleep(10.0)
                except Exception as e:
                    print(f"❌ Battery publisher error: {e}")
                    time.sleep(1.0)
        
        def imu_publisher():
            """Publish IMU data every 0.5 seconds"""
            while self.running:
                try:
                    self.publish_imu_data()
                    time.sleep(0.5)
                except Exception as e:
                    print(f"❌ IMU publisher error: {e}")
                    time.sleep(0.1)
        
        # Start publisher threads
        threading.Thread(target=status_publisher, daemon=True).start()
        threading.Thread(target=battery_publisher, daemon=True).start()
        threading.Thread(target=imu_publisher, daemon=True).start()
        
        print("🚀 Status publishing threads started")
    
    def run(self):
        """Main run loop"""
        print("🤖 Rider Robot Server Started")
        print("=" * 50)
        print("Press Ctrl+C to stop")
        
        try:
            while self.running:
                # TODO: Add your main robot logic here
                # - Read sensors
                # - Update controller status
                # - Handle hardware events
                # - Update robot state
                
                time.sleep(0.1)  # 10Hz main loop
                
        except KeyboardInterrupt:
            print("\n🛑 Shutting down...")
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        self.running = False
        if self.mqtt_client:
            self.mqtt_client.loop_stop()
            self.mqtt_client.disconnect()
        print("✅ Cleanup complete")

def main():
    # Default to local MQTT broker, or specify remote broker
    import sys
    broker_host = sys.argv[1] if len(sys.argv) > 1 else 'localhost'
    
    server = RiderRobotServer(broker_host)
    server.run()

if __name__ == "__main__":
    main() 