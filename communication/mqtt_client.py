#!/usr/bin/env python3
# coding=utf-8

import json
import time
import paho.mqtt.client as mqtt
from datetime import datetime
from typing import Dict, Any, Callable, Optional

class MQTTClient:
    def __init__(self, broker_host: str, broker_port: int, debug: bool = False):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.debug_mode = debug
        self.client_id = f"rider_pc_client_{int(time.time())}"
        
        # MQTT client
        self.mqtt_client: Optional[mqtt.Client] = None
        self.connected = False
        
        # Topic structure
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
        
        # Callbacks for different message types
        self._message_callbacks: Dict[str, Callable] = {}
        self._connection_callbacks: Dict[str, Callable] = {}
    
    def add_message_callback(self, topic: str, callback: Callable):
        """Add callback for specific topic messages"""
        self._message_callbacks[topic] = callback
    
    def add_connection_callback(self, event: str, callback: Callable):
        """Add callback for connection events (connect/disconnect)"""
        self._connection_callbacks[event] = callback
    
    def debug_print(self, message: str):
        """Print debug message only if debug mode is enabled"""
        if self.debug_mode:
            print(message)
    
    def connect(self) -> bool:
        """Connect to MQTT broker"""
        try:
            # Use MQTT 5.0 protocol with callback API version 2
            self.mqtt_client = mqtt.Client(
                client_id=self.client_id,
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                protocol=mqtt.MQTTv5
            )
            self.mqtt_client.on_connect = self._on_connect
            self.mqtt_client.on_disconnect = self._on_disconnect
            self.mqtt_client.on_message = self._on_message
            
            print(f"Connecting to MQTT broker at {self.broker_host}:{self.broker_port} using MQTT 5.0")
            self.mqtt_client.connect(self.broker_host, self.broker_port, 60)
            self.mqtt_client.loop_start()
            return True
            
        except Exception as e:
            print(f"Failed to connect to MQTT broker: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from MQTT broker - immediate, no waiting"""
        if self.mqtt_client:
            try:
                print("📡 Force disconnecting from MQTT broker...")
                # Stop the loop immediately and don't wait for network
                self.mqtt_client.loop_stop()
                # Force immediate disconnection without waiting
                try:
                    self.mqtt_client.disconnect()
                except:
                    pass  # Don't wait for network response
                # Clear reference immediately
                self.mqtt_client = None
                print("✅ MQTT force disconnected")
            except Exception as e:
                print(f"⚠️ MQTT disconnect error (ignored): {e}")
        self.connected = False

    def graceful_disconnect(self):
        """Gracefully disconnect from MQTT broker with proper cleanup"""
        if not self.connected or not self.mqtt_client:
            self.connected = False
            return
        
        try:
            print("📡 Graceful disconnect initiated...")
            
            # Send stop commands to ensure robot is in safe state
            self.send_emergency_stop()
            self.send_movement_stop()
            
            # Brief pause to allow messages to be sent
            time.sleep(0.2)
            
            print("📡 Disconnecting from MQTT broker...")
            
            # Stop the network loop
            self.mqtt_client.loop_stop()
            
            # Proper MQTT disconnect
            self.mqtt_client.disconnect()
            
            # Clear reference
            self.mqtt_client = None
            self.connected = False
            
            print("✅ MQTT gracefully disconnected")
            
        except Exception as e:
            print(f"⚠️ Graceful disconnect error, falling back to force disconnect: {e}")
            # Fall back to force disconnect if graceful fails
            self.disconnect()

    def send_emergency_stop(self) -> bool:
        """Send emergency stop command during disconnect"""
        try:
            command = {
                'action': 'emergency_stop',
                'timestamp': time.time(),
                'source': 'disconnect_cleanup'
            }
            if self.connected and self.mqtt_client:
                payload = json.dumps(command)
                result = self.mqtt_client.publish(self.topics['control_system'], payload)
                self.debug_print("[CLEANUP] Emergency stop sent during disconnect")
                return result.rc == 0
        except Exception as e:
            self.debug_print(f"[CLEANUP] Failed to send emergency stop: {e}")
        return False

    def send_movement_stop(self) -> bool:
        """Send movement stop command during disconnect"""
        try:
            command = {
                'x': 0,
                'y': 0,
                'timestamp': time.time(),
                'source': 'disconnect_cleanup'
            }
            if self.connected and self.mqtt_client:
                payload = json.dumps(command)
                result = self.mqtt_client.publish(self.topics['control_movement'], payload)
                self.debug_print("[CLEANUP] Movement stop sent during disconnect")
                return result.rc == 0
        except Exception as e:
            self.debug_print(f"[CLEANUP] Failed to send movement stop: {e}")
        return False
    
    def reconnect(self):
        """Reconnect to MQTT broker"""
        self.graceful_disconnect()
        time.sleep(1)
        return self.connect()
    
    def publish_command(self, topic: str, command_data: Dict[str, Any]) -> bool:
        """Publish command with debug logging"""
        if not self.connected:
            self.debug_print(f"[WARN] Cannot send command - not connected to broker")
            return False
            
        try:
            payload = json.dumps(command_data)
            result = self.mqtt_client.publish(topic, payload)
            
            # Debug: Show outgoing message traffic
            if self.debug_mode:
                print(f"[SEND] {datetime.now().strftime('%H:%M:%S.%f')[:-3]} | Topic: {topic}")
                print(f"       Payload: {payload}")
                print(f"       Result: {result.rc} (0=success)")
            
            return result.rc == 0
        except Exception as e:
            print(f"[ERROR] Failed to publish to {topic}: {e}")
            return False
    
    def send_movement_command(self, x: float, y: float) -> bool:
        """Send movement command"""
        command = {
            'x': x,
            'y': y,
            'timestamp': time.time()
        }
        return self.publish_command(self.topics['control_movement'], command)
    
    def send_settings_command(self, action: str, value: Any = None) -> bool:
        """Send settings command"""
        command = {
            'action': action,
            'timestamp': time.time()
        }
        if value is not None:
            command['value'] = value
        return self.publish_command(self.topics['control_settings'], command)
    
    def send_camera_command(self, action: str) -> bool:
        """Send camera command"""
        command = {
            'action': action,
            'timestamp': time.time()
        }
        return self.publish_command(self.topics['control_camera'], command)
    
    def send_system_command(self, action: str) -> bool:
        """Send system command"""
        command = {
            'action': action,
            'timestamp': time.time()
        }
        return self.publish_command(self.topics['control_system'], command)
    
    def _on_connect(self, client, userdata, flags, reason_code, properties):
        """Callback for MQTT connection"""
        if reason_code == 0:
            self.connected = True
            print("Connected to MQTT broker")
            
            # Subscribe to all relevant topics
            if self.debug_mode:
                print("[DEBUG] Available topics:")
                for topic_name, topic in self.topics.items():
                    print(f"  {topic_name}: {topic}")
                
                print("[DEBUG] Subscribing to topics:")
            
            for topic_name, topic in self.topics.items():
                if topic_name.startswith('control'):
                    self.debug_print(f"  Skipping control topic: {topic}")
                    continue  # Don't subscribe to control topics
                client.subscribe(topic)
                self.debug_print(f"  Subscribed to {topic}")
            
            # Notify connection callbacks
            if 'connect' in self._connection_callbacks:
                self._connection_callbacks['connect'](True)
        else:
            print(f"Failed to connect to MQTT broker, reason code {reason_code}")
            # Notify connection callbacks
            if 'connect' in self._connection_callbacks:
                self._connection_callbacks['connect'](False)
    
    def _on_disconnect(self, client, userdata, flags, reason_code, properties):
        """Callback for MQTT disconnection"""
        self.connected = False
        print("Disconnected from MQTT broker")
        # Notify connection callbacks
        if 'disconnect' in self._connection_callbacks:
            self._connection_callbacks['disconnect']()
    
    def _on_message(self, client, userdata, msg):
        """Handle incoming MQTT messages"""
        try:
            topic = msg.topic
            payload_str = msg.payload.decode()
            payload = json.loads(payload_str)
            
            # Debug: Show incoming message traffic
            if self.debug_mode:
                print(f"[RECV] {datetime.now().strftime('%H:%M:%S.%f')[:-3]} | Topic: {topic}")
                print(f"       Payload: {payload_str}")
            
            # Route message to appropriate callback
            if topic in self._message_callbacks:
                self._message_callbacks[topic](payload)
            else:
                self.debug_print(f"[WARN] No callback registered for topic: {topic}")
                
        except Exception as e:
            print(f"[ERROR] Error processing message from topic {msg.topic}: {e}")
            if self.debug_mode:
                print(f"        Raw payload: {msg.payload}")
    
    def is_connected(self) -> bool:
        """Check if MQTT client is connected"""
        return self.connected
    
    def get_topics(self) -> Dict[str, str]:
        """Get available topics"""
        return self.topics.copy() 