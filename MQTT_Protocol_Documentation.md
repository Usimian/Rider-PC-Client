# Rider Robot MQTT Protocol Documentation

This document describes the complete MQTT communication protocol between the PC Client and the Raspberry Pi robot server.

## Connection Details
- **Protocol**: MQTT 5.0
- **Default Broker**: 192.168.1.173:1883
- **Client ID Format**: `rider_pc_client_{timestamp}` or `robot_tester_{timestamp}`

## Topic Structure

### 1. Status Topics (Robot → PC Client)
These topics are **published by the robot** and **subscribed to by the PC client**.

#### `rider/status`
**Frequency**: Every ~2 seconds  
**Purpose**: Main robot status updates  
**Message Format**:
```json
{
  "timestamp": 1749239530.312921,
  "speed_scale": 1.0,
  "roll_balance_enabled": true,
  "performance_mode_enabled": false,
  "camera_enabled": false,
  "controller_connected": true,
  "height": 85,
  "connection_status": "connected",
  "cpu_percent": 15.2,
  "cpu_load_1min": 0.85
}
```

**Field Descriptions**:
- `timestamp`: Unix timestamp of when status was generated
- `speed_scale`: Current speed multiplier (0.1 - 2.0)
- `roll_balance_enabled`: Whether roll balance compensation is active
- `performance_mode_enabled`: Whether performance mode is active
- `camera_enabled`: Whether camera is active
- `controller_connected`: Whether gamepad controller is connected
- `height`: Robot height setting (typically 85)
- `connection_status`: "connected" | "disconnected"
- `cpu_percent`: Current CPU usage percentage (0.0 - 100.0)
- `cpu_load_1min`: 1-minute load average (0.0 - 4.0+ typical for 4-core system)

#### `rider/status/battery`
**Frequency**: Every ~10 seconds  
**Purpose**: Battery status updates  
**Message Format**:
```json
{
  "timestamp": 1749239636.9426005,
  "level": 54,
  "status": "normal",
  "source": "hardware"
}
```

**Field Descriptions**:
- `timestamp`: Unix timestamp
- `level`: Battery percentage (0-100)
- `status`: "normal" | "low" | "critical" | "charging"
- `source`: "hardware" | "estimated"

#### `rider/status/imu`
**Frequency**: ~2Hz (every 0.5 seconds)  
**Purpose**: Inertial Measurement Unit data  
**Message Format**:
```json
{
  "timestamp": 1749239641.1890562,
  "roll": 0.0,
  "pitch": 0.0,
  "yaw": 0.0
}
```

**Field Descriptions**:
- `timestamp`: Unix timestamp
- `roll`: Roll angle in degrees (-180 to +180)
- `pitch`: Pitch angle in degrees (-90 to +90)
- `yaw`: Yaw angle in degrees (-180 to +180)

---

## System Performance Monitoring

The robot includes real-time system performance monitoring in the general status messages.

### CPU Metrics Explained

**CPU Percentage (`cpu_percent`)**:
- Shows current CPU usage (0-100%)
- Instantaneous measurement 
- Values > 80% indicate high system load

**Load Average (`cpu_load_1min`)**:
- Average number of processes waiting for CPU over time periods
- Values < 1.0 = system not busy
- Values 1.0-4.0 = moderate load (acceptable for 4-core Raspberry Pi)
- Values > 4.0 = system overloaded

**Typical Values for Raspberry Pi CM4**:
- Normal operation: CPU 5-25%, Load 0.2-0.8
- Active robot control: CPU 20-50%, Load 0.5-1.5
- High load warning: CPU > 80%, Load > 3.0

---

### 2. Control Topics (PC Client → Robot)
These topics are **published by the PC client** and **subscribed to by the robot**.

#### `rider/control/movement`
**Purpose**: Movement commands  
**Triggered by**: Arrow buttons in PC client  
**Message Format**:
```json
{
  "x": 0,
  "y": 50,
  "timestamp": 1749239765.7873564
}
```

**Field Descriptions**:
- `x`: Lateral movement (-100 to +100, negative = left, positive = right)
- `y`: Forward/backward movement (-100 to +100, negative = backward, positive = forward)
- `timestamp`: Unix timestamp when command was sent

**Expected Robot Response**: No direct response, but movement should be reflected in robot behavior.

#### `rider/control/settings`
**Purpose**: Robot settings configuration  
**Triggered by**: Settings buttons and speed slider  
**Message Formats**:

**Toggle Roll Balance**:
```json
{
  "action": "toggle_roll_balance",
  "timestamp": 1749239770.7899299
}
```

**Toggle Performance Mode**:
```json
{
  "action": "toggle_performance",
  "timestamp": 1749239771.790638
}
```

**Change Speed**:
```json
{
  "action": "change_speed",
  "value": 1.5,
  "timestamp": 1749239772.7912588
}
```

**Expected Robot Response**: Updated status should be published to `rider/status` topic reflecting the new settings.

#### `rider/control/camera`
**Purpose**: Camera control  
**Triggered by**: Camera toggle button  
**Message Format**:
```json
{
  "action": "toggle_camera",
  "timestamp": 1749239773.7919118
}
```

**Expected Robot Response**: Updated `camera_enabled` field in `rider/status` topic.

#### `rider/control/system`
**Purpose**: System-level commands  
**Triggered by**: Emergency stop button  
**Message Format**:
```json
{
  "action": "emergency_stop",
  "timestamp": 1749239780.1234567
}
```

**Expected Robot Response**: Robot should immediately stop all movement and publish status update.

#### `rider/request/battery`
**Purpose**: Request immediate battery status update  
**Triggered by**: "Request Battery" button  
**Message Format**:
```json
{
  "action": "request_battery",
  "timestamp": 1749239773.7923646
}
```

**Expected Robot Response**: Immediate publication to `rider/status/battery` topic.

---

## Raspberry Pi Server Implementation Guide

### Required MQTT Subscriptions
Your Pi server should subscribe to these topics:
- `rider/control/movement`
- `rider/control/settings`
- `rider/control/camera`
- `rider/control/system`
- `rider/request/battery`

### Required MQTT Publications
Your Pi server should publish to these topics:
- `rider/status` (every ~2 seconds)
- `rider/status/battery` (every ~10 seconds or when requested)
- `rider/status/imu` (every ~0.5 seconds)

### Message Handling Logic

```python
import os
import psutil

def get_cpu_data():
    """Get current CPU metrics"""
    try:
        cpu_percent = psutil.cpu_percent(interval=None)
        load_avg = os.getloadavg()
        return {
            'cpu_percent': cpu_percent,
            'cpu_load_1min': load_avg[0]
        }
    except:
        return {
            'cpu_percent': 0.0,
            'cpu_load_1min': 0.0
        }

def handle_movement_command(message_data):
    x = message_data.get('x', 0)  # -100 to +100
    y = message_data.get('y', 0)  # -100 to +100
    # Convert to robot movement commands
    # Apply to motors/servos

def handle_settings_command(message_data):
    action = message_data.get('action')
    
    if action == 'toggle_roll_balance':
        # Toggle roll balance compensation
        # Update internal state
        publish_status_update()
    
    elif action == 'toggle_performance':
        # Toggle performance mode
        # Update internal state  
        publish_status_update()
    
    elif action == 'change_speed':
        value = message_data.get('value', 1.0)
        # Update speed scale (0.1 - 2.0)
        # Apply to movement calculations
        publish_status_update()

def handle_camera_command(message_data):
    # Toggle camera on/off
    # Update camera_enabled state
    publish_status_update()

def handle_system_command(message_data):
    action = message_data.get('action')
    if action == 'emergency_stop':
        # Immediately stop all movement
        # Set safe state
        publish_status_update()

def handle_battery_request(message_data):
    # Read current battery level
    # Publish immediate battery status
    publish_battery_status()
```

### Status Publishing Examples

```python
def publish_status():
    # Get current CPU metrics
    cpu_data = get_cpu_data()
    
    status = {
        "timestamp": time.time(),
        "speed_scale": current_speed_scale,
        "roll_balance_enabled": roll_balance_active,
        "performance_mode_enabled": performance_mode_active,
        "camera_enabled": camera_active,
        "controller_connected": controller_connected,
        "height": robot_height,
        "connection_status": "connected",
        "cpu_percent": cpu_data['cpu_percent'],
        "cpu_load_1min": cpu_data['cpu_load_1min']
    }
    mqtt_client.publish("rider/status", json.dumps(status))

def publish_battery_status():
    battery = {
        "timestamp": time.time(),
        "level": get_battery_percentage(),
        "status": get_battery_status(),  # "normal", "low", "critical"
        "source": "hardware"
    }
    mqtt_client.publish("rider/status/battery", json.dumps(battery))

def publish_imu_data():
    imu = {
        "timestamp": time.time(),
        "roll": get_roll_angle(),
        "pitch": get_pitch_angle(),
        "yaw": get_yaw_angle()
    }
    mqtt_client.publish("rider/status/imu", json.dumps(imu))
```

## Testing Your Implementation

Use the provided test tools:

```bash
# Monitor all MQTT traffic
python3 mqtt_monitor.py

# Test all functions automatically  
python3 test_commands.py

# Test interactively
python3 test_commands.py -i
```

## Error Handling

- All messages should include timestamps
- Handle missing fields gracefully with defaults
- Validate ranges for numeric values (speed: 0.1-2.0, movement: -100 to +100)
- Publish status updates whenever settings change
- Maintain connection status and handle disconnections

## Message Timing

- **Status**: Every 2 seconds (not critical timing)
- **IMU**: Every 0.5 seconds (for smooth display)
- **Battery**: Every 10 seconds (or when requested)
- **Commands**: Process immediately when received 