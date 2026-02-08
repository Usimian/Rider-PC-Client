#!/usr/bin/env python3
# coding=utf-8

# MQTT Monitor - Simple tool to monitor all MQTT traffic from the robot
# This helps debug communication issues

import json
import time
import paho.mqtt.client as mqtt
from datetime import datetime
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.dirname(__file__))
from core.config_manager import ConfigManager

class MQTTMonitor:
    def __init__(self, broker_host=None, broker_port=None):
        # Load from config if not provided
        config = ConfigManager()
        self.broker_host = broker_host or config.get_broker_host()
        self.broker_port = broker_port or config.get_broker_port()
        self.client_id = f"mqtt_monitor_{int(time.time())}"
        
        # MQTT client
        self.mqtt_client = None
        self.connected = False
        
        print(f"MQTT Monitor - Connecting to {broker_host}:{broker_port}")
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
            self.mqtt_client.on_disconnect = self.on_disconnect
            self.mqtt_client.on_message = self.on_message
            
            self.mqtt_client.connect(self.broker_host, self.broker_port, 60)
            self.mqtt_client.loop_forever()
            
        except Exception as e:
            print(f"Failed to connect: {e}")
    
    def on_connect(self, client, userdata, flags, reason_code, properties):
        """Callback for MQTT connection"""
        if reason_code == 0:
            self.connected = True
            print(f"✅ Connected to MQTT broker at {self.broker_host}")
            
            # Subscribe to all rider topics with wildcard
            client.subscribe("rider/#")
            print("📡 Subscribed to rider/# (all rider topics)")
            print("🔍 Monitoring all messages... (Press Ctrl+C to stop)")
            print("-" * 80)
        else:
            print(f"❌ Failed to connect, reason code: {reason_code}")
    
    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        """Callback for MQTT disconnection"""
        self.connected = False
        print("❌ Disconnected from MQTT broker")
    
    def on_message(self, client, userdata, msg):
        """Handle all incoming MQTT messages"""
        try:
            topic = msg.topic
            timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
            
            # Try to parse as JSON
            try:
                payload = json.loads(msg.payload.decode())
                payload_str = json.dumps(payload, indent=2)
                print(f"📩 [{timestamp}] Topic: {topic}")
                print(f"   JSON Payload:")
                for line in payload_str.split('\n'):
                    print(f"     {line}")
            except json.JSONDecodeError:
                # Not JSON, show raw payload
                payload_str = msg.payload.decode()
                print(f"📩 [{timestamp}] Topic: {topic}")
                print(f"   Raw Payload: {payload_str}")
            
            print("-" * 40)
            
        except Exception as e:
            print(f"❌ Error processing message: {e}")
            print(f"   Topic: {msg.topic}")
            print(f"   Raw: {msg.payload}")
    
    def run(self):
        """Run the monitor"""
        try:
            self.mqtt_client.loop_forever()
        except KeyboardInterrupt:
            print("\n🛑 Monitoring stopped by user")
            if self.mqtt_client:
                self.mqtt_client.disconnect()

def main():
    print("🤖 Rider Robot MQTT Monitor")
    print("=" * 50)

    # Allow custom IP from command line, otherwise use rider_config.ini
    broker_ip = sys.argv[1] if len(sys.argv) > 1 else None

    monitor = MQTTMonitor(broker_ip)
    monitor.run()

if __name__ == "__main__":
    main() 