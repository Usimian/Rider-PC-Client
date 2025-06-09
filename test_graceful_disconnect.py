#!/usr/bin/env python3
# coding=utf-8

# Test script for graceful disconnect functionality
# This script simulates a brief connection and then tests the graceful disconnect

import sys
import time
from communication.mqtt_client import MQTTClient

def test_graceful_disconnect():
    """Test the graceful disconnect functionality"""
    print("🧪 Testing graceful disconnect functionality...")
    print("=" * 50)
    
    # Create MQTT client with debug enabled
    mqtt_client = MQTTClient("192.168.1.173", 1883, debug=True)
    
    try:
        # Connect
        print("1. Connecting to MQTT broker...")
        if mqtt_client.connect():
            print("✅ Connected successfully")
            
            # Wait a moment for connection to stabilize
            time.sleep(1)
            
            # Send a test movement command
            print("2. Sending test movement command...")
            mqtt_client.send_movement_command(25, 50)
            
            # Wait a moment
            time.sleep(0.5)
            
            # Test graceful disconnect
            print("3. Testing graceful disconnect...")
            mqtt_client.graceful_disconnect()
            
            print("✅ Graceful disconnect test completed")
            
        else:
            print("❌ Failed to connect to MQTT broker")
            print("   Make sure the robot is running and accessible at 192.168.1.173")
            return False
            
    except KeyboardInterrupt:
        print("\n⌨️ Test interrupted by user")
        mqtt_client.graceful_disconnect()
        
    except Exception as e:
        print(f"❌ Test error: {e}")
        # Fallback to force disconnect
        mqtt_client.disconnect()
        return False
    
    print("\n🎉 Test completed successfully!")
    print("The graceful disconnect should have sent:")
    print("  - Emergency stop command")
    print("  - Movement stop command (x=0, y=0)")
    print("  - Proper MQTT disconnect")
    
    return True

if __name__ == "__main__":
    success = test_graceful_disconnect()
    sys.exit(0 if success else 1) 