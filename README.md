# Rider Robot PC Client

A desktop monitoring client that mirrors the robot's LCD screen display.

## Quick Setup

### Prerequisites
- Python 3.7+ with tkinter
- Network connection to the robot

### Installation
1. Install dependencies:
   ```bash
   pip install paho-mqtt
   ```

2. Configure robot IP in `rider_config.ini`:
   ```ini
   [mqtt]
   broker_host = 192.168.1.130  # Change to your Pi's IP
   broker_port = 1883
   ```

3. Run the client:
   ```bash
   python pc_client.py
   ```

## Features

**Display (matches robot screen):**
- 🎮 Controller status (upper left)
- ⏰ Current time (center top)
- 🔋 Battery level with progress bar (upper right)
- **SPD:** Speed multiplier
- **BAL:** Roll balance (ON/OFF)
- **FUN:** Performance mode (ON/OFF) 
- **CAM:** Camera status (ON/OFF)
- **Roll/Pitch/Yaw:** Real-time IMU data

**Controls:**
- Movement buttons (↑↓←→⏹)
- Settings toggles
- Speed adjustment slider

## Color Coding
- **Green:** Enabled/Good (battery ≥70%)
- **Yellow:** Warning (battery 40-69%)
- **Red:** Disabled/Low (battery <40%)

## Troubleshooting
1. Check robot IP address
2. Ensure robot's MQTT broker is running
3. Verify network connectivity 