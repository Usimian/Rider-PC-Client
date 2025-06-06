#!/usr/bin/env python3
# coding=utf-8

# Rider Robot PC Client - Standalone Version
# MQTT-based remote control and monitoring client for PC (AMD64)
# This version can be developed and run independently from the Pi
# Marc Wester

import json
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
import paho.mqtt.client as mqtt
from datetime import datetime
import configparser
import os

class RiderPCClient:
    def __init__(self, debug=False):
        # Debug mode flag
        self.debug_mode = debug
        
        # Load configuration
        self.config = self.load_config()
        self.broker_host = self.config.get('mqtt', 'broker_host', fallback='192.168.1.173')
        self.broker_port = self.config.getint('mqtt', 'broker_port', fallback=1883)
        self.client_id = f"rider_pc_client_{int(time.time())}"
        
        # MQTT client
        self.mqtt_client = None
        self.connected = False
        
        # Robot state data
        self.robot_data = {
            'battery_level': 0,
            'speed_scale': 1.0,
            'roll_balance_enabled': False,
            'performance_mode_enabled': False,
            'camera_enabled': False,
            'controller_connected': False,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'height': 85,
            'connection_status': 'disconnected',
            'last_update': None
        }
        
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
        
        # Create GUI
        self.setup_gui()
        
        # Connect to MQTT
        self.connect_mqtt()
        
        # Set up signal handlers for graceful shutdown
        self.setup_signal_handlers()
    
    def load_config(self):
        """Load configuration from file or create default"""
        config = configparser.ConfigParser()
        config_file = 'rider_config.ini'
        
        if os.path.exists(config_file):
            config.read(config_file)
        else:
            # Create default config
            config['mqtt'] = {
                'broker_host': '192.168.1.173',
                'broker_port': '1883'
            }
            config['ui'] = {
                'update_interval': '0.1',
                'theme': 'dark'
            }
            
            with open(config_file, 'w') as f:
                config.write(f)
            if self.debug_mode:
                print(f"Created default config: {config_file}")
        
        return config
    
    def setup_signal_handlers(self):
        """Set up signal handlers for graceful shutdown"""
        import signal
        
        def signal_handler(signum, frame):
            signal_name = signal.Signals(signum).name
            print(f"\n🚨 Received {signal_name} signal - initiating immediate shutdown...")
            self.gui_running = False
            
            # Try very quick cleanup
            try:
                if hasattr(self, 'mqtt_client') and self.mqtt_client:
                    self.mqtt_client.loop_stop()
                    self.mqtt_client = None
                print("📡 MQTT force-stopped")
                
                if hasattr(self, 'root') and self.root:
                    self.root.quit()
                    self.root.destroy()
                print("🖥️ GUI force-destroyed")
                    
            except Exception as e:
                print(f"⚠️ Quick cleanup error (ignored): {e}")
            
            print("✅ Force shutdown complete")
            import sys
            sys.exit(0)
        
        # Handle common termination signals
        signal.signal(signal.SIGINT, signal_handler)   # Ctrl-C
        signal.signal(signal.SIGTERM, signal_handler)  # Termination request
    
    def debug_print(self, message):
        """Print debug message only if debug mode is enabled"""
        if self.debug_mode:
            print(message)
    
    def setup_gui(self):
        """Create the GUI interface matching rider screen layout"""
        self.root = tk.Tk()
        self.root.title(f"Rider Robot PC Client - {self.broker_host}")
        self.root.geometry("1000x800")
        self.root.configure(bg='#0f152e')  # Dark blue background to match rider screen
        
        # Create menu bar
        self.create_menu()
        
        # Connection status bar
        status_bar = tk.Frame(self.root, bg='#1a1a2e', height=30)
        status_bar.pack(fill='x', side='top')
        
        self.connection_status = tk.Label(status_bar, text=f"Connecting to {self.broker_host}...", 
                                        bg='#1a1a2e', fg='yellow', font=('Arial', 10))
        self.connection_status.pack(side='left', padx=10, pady=5)
        
        # Main frame with dark background
        main_frame = tk.Frame(self.root, bg='#0f152e', padx=20, pady=20)
        main_frame.pack(fill='both', expand=True)
        
        # Header frame (matches top of rider screen layout)
        header_frame = tk.Frame(main_frame, bg='#0f152e')
        header_frame.pack(fill='x', pady=(0, 20))
        
        # Controller status (upper left - matches controller icon)
        controller_frame = tk.Frame(header_frame, bg='#0f152e')
        controller_frame.pack(side='left')
        
        tk.Label(controller_frame, text="🎮", font=('Arial', 16), bg='#0f152e', fg='white').pack(side='left')
        self.controller_status_label = tk.Label(controller_frame, text="Controller: OFF", 
                                              font=('Arial', 12, 'bold'), bg='#0f152e', fg='red')
        self.controller_status_label.pack(side='left', padx=(5, 0))
        
        # Current time (center - matches screen layout)
        self.time_label = tk.Label(header_frame, text="--:--", font=('Arial', 16, 'bold'), 
                                 bg='#0f152e', fg='white')
        self.time_label.pack()
        
        # Battery status (upper right - matches battery icon)
        battery_frame = tk.Frame(header_frame, bg='#0f152e')
        battery_frame.pack(side='right')
        
        tk.Label(battery_frame, text="🔋", font=('Arial', 16), bg='#0f152e', fg='white').pack(side='left')
        self.battery_label = tk.Label(battery_frame, text="0%", font=('Arial', 12, 'bold'), 
                                    bg='#0f152e', fg='red')
        self.battery_label.pack(side='left', padx=(5, 0))
        
        self.battery_progress = ttk.Progressbar(battery_frame, length=100, mode='determinate')
        self.battery_progress.pack(side='left', padx=(10, 0))
        
        # Main content frame
        content_frame = tk.Frame(main_frame, bg='#0f152e')
        content_frame.pack(fill='both', expand=True)
        
        # Left side - Robot Status (matches left side of screen)
        status_frame = tk.LabelFrame(content_frame, text="Robot Status", font=('Arial', 12, 'bold'),
                                   bg='#0f152e', fg='white', relief='solid', bd=1)
        status_frame.pack(side='left', fill='both', expand=True, padx=(0, 10))
        
        # Speed (SPD - matches screen layout)
        speed_row = tk.Frame(status_frame, bg='#0f152e')
        speed_row.pack(fill='x', pady=10, padx=20)
        
        tk.Label(speed_row, text="SPD:", font=('Arial', 14, 'bold'), bg='#0f152e', fg='white').pack(side='left')
        self.speed_label = tk.Label(speed_row, text="1.0x", font=('Arial', 14, 'bold'), bg='#0f152e', fg='white')
        self.speed_label.pack(side='left', padx=(20, 0))
        
        # Add speed control
        speed_scale = tk.Scale(status_frame, from_=0.1, to=2.0, resolution=0.1, orient='horizontal',
                             bg='#0f152e', fg='white', highlightbackground='#0f152e',
                             command=self.change_speed, label="Speed Control")
        speed_scale.set(1.0)
        speed_scale.pack(fill='x', padx=20, pady=5)
        
        # Roll Balance (BAL - matches screen layout) 
        balance_row = tk.Frame(status_frame, bg='#0f152e')
        balance_row.pack(fill='x', pady=10, padx=20)
        
        tk.Label(balance_row, text="BAL:", font=('Arial', 14, 'bold'), bg='#0f152e', fg='white').pack(side='left')
        self.roll_balance_label = tk.Label(balance_row, text="OFF", font=('Arial', 14, 'bold'), bg='#0f152e', fg='red')
        self.roll_balance_label.pack(side='left', padx=(20, 0))
        
        # Performance Mode (FUN - matches screen layout)
        performance_row = tk.Frame(status_frame, bg='#0f152e')
        performance_row.pack(fill='x', pady=10, padx=20)
        
        tk.Label(performance_row, text="FUN:", font=('Arial', 14, 'bold'), bg='#0f152e', fg='white').pack(side='left')
        self.performance_label = tk.Label(performance_row, text="OFF", font=('Arial', 14, 'bold'), bg='#0f152e', fg='red')
        self.performance_label.pack(side='left', padx=(20, 0))
        
        # Camera Status
        camera_row = tk.Frame(status_frame, bg='#0f152e')
        camera_row.pack(fill='x', pady=10, padx=20)
        
        tk.Label(camera_row, text="CAM:", font=('Arial', 14, 'bold'), bg='#0f152e', fg='white').pack(side='left')
        self.camera_label = tk.Label(camera_row, text="OFF", font=('Arial', 14, 'bold'), bg='#0f152e', fg='red')
        self.camera_label.pack(side='left', padx=(20, 0))
        
        # Right side - IMU/Odometry Data (matches screen odometry section)
        imu_frame = tk.LabelFrame(content_frame, text="Odometry/IMU Data", font=('Arial', 12, 'bold'),
                                bg='#0f152e', fg='white', relief='solid', bd=1)
        imu_frame.pack(side='right', fill='both', expand=True)
        
        # Roll, Pitch, Yaw display (matches screen odometry format)
        roll_row = tk.Frame(imu_frame, bg='#0f152e')
        roll_row.pack(fill='x', pady=5, padx=20)
        
        tk.Label(roll_row, text="Roll:", font=('Arial', 12, 'bold'), bg='#0f152e', fg='yellow').pack(side='left')
        self.roll_label = tk.Label(roll_row, text="+0.0°", font=('Arial', 12, 'bold'), bg='#0f152e', fg='yellow')
        self.roll_label.pack(side='left', padx=(20, 0))
        
        pitch_row = tk.Frame(imu_frame, bg='#0f152e')
        pitch_row.pack(fill='x', pady=5, padx=20)
        
        tk.Label(pitch_row, text="Pitch:", font=('Arial', 12, 'bold'), bg='#0f152e', fg='yellow').pack(side='left')
        self.pitch_label = tk.Label(pitch_row, text="+0.0°", font=('Arial', 12, 'bold'), bg='#0f152e', fg='yellow')
        self.pitch_label.pack(side='left', padx=(20, 0))
        
        yaw_row = tk.Frame(imu_frame, bg='#0f152e')
        yaw_row.pack(fill='x', pady=5, padx=20)
        
        tk.Label(yaw_row, text="Yaw:", font=('Arial', 12, 'bold'), bg='#0f152e', fg='yellow').pack(side='left')
        self.yaw_label = tk.Label(yaw_row, text="+0.0°", font=('Arial', 12, 'bold'), bg='#0f152e', fg='yellow')
        self.yaw_label.pack(side='left', padx=(20, 0))
        
        # Control frame 
        control_frame = tk.LabelFrame(main_frame, text="Robot Control", font=('Arial', 12, 'bold'),
                                    bg='#0f152e', fg='white', relief='solid', bd=1)
        control_frame.pack(fill='x', pady=(20, 0))
        
        # Control buttons in a compact horizontal layout
        button_frame = tk.Frame(control_frame, bg='#0f152e')
        button_frame.pack(pady=10, padx=20)
        
        # Movement controls
        movement_group = tk.Frame(button_frame, bg='#0f152e')
        movement_group.pack(side='left', padx=(0, 20))
        
        tk.Label(movement_group, text="Movement:", font=('Arial', 10, 'bold'), bg='#0f152e', fg='white').pack()
        
        move_buttons = tk.Frame(movement_group, bg='#0f152e')
        move_buttons.pack()
        
        tk.Button(move_buttons, text="↑", command=lambda: self.send_movement(0, 50), width=3, font=('Arial', 12, 'bold')).grid(row=0, column=1, padx=1, pady=1)
        tk.Button(move_buttons, text="←", command=lambda: self.send_movement(-50, 0), width=3, font=('Arial', 12, 'bold')).grid(row=1, column=0, padx=1, pady=1)
        tk.Button(move_buttons, text="⏹", command=lambda: self.send_movement(0, 0), width=3, font=('Arial', 12, 'bold')).grid(row=1, column=1, padx=1, pady=1)
        tk.Button(move_buttons, text="→", command=lambda: self.send_movement(50, 0), width=3, font=('Arial', 12, 'bold')).grid(row=1, column=2, padx=1, pady=1)
        tk.Button(move_buttons, text="↓", command=lambda: self.send_movement(0, -50), width=3, font=('Arial', 12, 'bold')).grid(row=2, column=1, padx=1, pady=1)
        
        # Settings controls
        settings_group = tk.Frame(button_frame, bg='#0f152e')
        settings_group.pack(side='left', padx=(20, 0))
        
        tk.Label(settings_group, text="Settings:", font=('Arial', 10, 'bold'), bg='#0f152e', fg='white').pack()
        
        tk.Button(settings_group, text="Toggle Roll Balance", command=self.toggle_roll_balance, font=('Arial', 9)).pack(pady=2)
        tk.Button(settings_group, text="Toggle Performance", command=self.toggle_performance, font=('Arial', 9)).pack(pady=2)
        tk.Button(settings_group, text="Toggle Camera", command=self.toggle_camera, font=('Arial', 9)).pack(pady=2)
        
        # System controls
        system_group = tk.Frame(button_frame, bg='#0f152e')
        system_group.pack(side='left', padx=(20, 0))
        
        tk.Label(system_group, text="System:", font=('Arial', 10, 'bold'), bg='#0f152e', fg='white').pack()
        
        tk.Button(system_group, text="Request Battery", command=self.request_battery, font=('Arial', 9)).pack(pady=2)
        tk.Button(system_group, text="Emergency Stop", command=self.emergency_stop, font=('Arial', 9), bg='red', fg='white').pack(pady=2)
        
        # Start GUI update thread
        self.gui_running = True
        self.gui_thread = threading.Thread(target=self.gui_update_loop, daemon=True)
        self.gui_thread.start()
        self.debug_print("🧵 GUI update thread started")
    
    def create_menu(self):
        """Create menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        
        # Connection menu
        connection_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Connection", menu=connection_menu)
        connection_menu.add_command(label="Change Robot IP", command=self.change_robot_ip)
        connection_menu.add_separator()
        connection_menu.add_command(label="Reconnect", command=self.reconnect)
        connection_menu.add_command(label="Disconnect", command=self.disconnect)
        
        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self.show_about)
    
    def change_robot_ip(self):
        """Change robot IP address"""
        new_ip = simpledialog.askstring("Robot IP", "Enter robot IP address:", initialvalue=self.broker_host)
        if new_ip and new_ip != self.broker_host:
            self.broker_host = new_ip
            self.config.set('mqtt', 'broker_host', new_ip)
            with open('rider_config.ini', 'w') as f:
                self.config.write(f)
            self.root.title(f"Rider Robot PC Client - {self.broker_host}")
            self.reconnect()
    
    def show_about(self):
        """Show about dialog"""
        messagebox.showinfo("About", "Rider Robot PC Client v2.0\nDeveloped for dual-environment control")
    
    def gui_update_loop(self):
        """GUI update loop running in separate thread"""
        self.debug_print("🔄 GUI update loop started")
        while self.gui_running:
            try:
                self.update_time()
                self.update_controller_status()
                time.sleep(0.1)  # 10 FPS update rate
            except Exception as e:
                if self.gui_running:  # Only log if we're supposed to be running
                    self.debug_print(f"⚠️ GUI update error: {e}")
                break
        self.debug_print("🔄 GUI update loop stopped")
    
    def update_time(self):
        """Update time display"""
        if not self.gui_running:
            return
        try:
            current_time = datetime.now().strftime("%H:%M")
            self.time_label.config(text=current_time)
        except:
            pass  # GUI might be destroyed
    
    def update_controller_status(self):
        """Update controller connection status display"""
        if not self.gui_running:
            return
        try:
            if self.robot_data['controller_connected']:
                self.controller_status_label.config(text="Controller: ON", fg='green')
            else:
                self.controller_status_label.config(text="Controller: OFF", fg='red')
        except:
            pass  # GUI might be destroyed
    
    def connect_mqtt(self):
        """Connect to MQTT broker"""
        try:
            # Use MQTT 5.0 protocol with callback API version 2
            self.mqtt_client = mqtt.Client(
                client_id=self.client_id,
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                protocol=mqtt.MQTTv5
            )
            self.mqtt_client.on_connect = self.on_connect
            self.mqtt_client.on_disconnect = self.on_disconnect
            self.mqtt_client.on_message = self.on_message
            
            print(f"Connecting to MQTT broker at {self.broker_host}:{self.broker_port} using MQTT 5.0")
            self.mqtt_client.connect(self.broker_host, self.broker_port, 60)
            self.mqtt_client.loop_start()
            
        except Exception as e:
            print(f"Failed to connect to MQTT broker: {e}")
            self.connection_status.config(text=f"Connection failed: {e}", fg='red')
    
    def on_connect(self, client, userdata, flags, reason_code, properties):
        """Callback for MQTT connection"""
        if reason_code == 0:
            self.connected = True
            print("Connected to MQTT broker")
            self.connection_status.config(text=f"Connected to {self.broker_host}", fg='green')
            
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
        else:
            print(f"Failed to connect to MQTT broker, reason code {reason_code}")
            self.connection_status.config(text=f"Connection failed (RC: {reason_code})", fg='red')
    
    def on_disconnect(self, client, userdata, flags, reason_code, properties):
        """Callback for MQTT disconnection"""
        self.connected = False
        self.connection_status.config(text="Disconnected", fg='red')
        print("Disconnected from MQTT broker")
    
    def on_message(self, client, userdata, msg):
        """Handle incoming MQTT messages"""
        try:
            topic = msg.topic
            payload_str = msg.payload.decode()
            payload = json.loads(payload_str)
            
            # Debug: Show incoming message traffic
            if self.debug_mode:
                print(f"[RECV] {datetime.now().strftime('%H:%M:%S.%f')[:-3]} | Topic: {topic}")
                print(f"       Payload: {payload_str}")
            
            if topic == self.topics['status']:
                self.update_robot_status(payload)
            elif topic == self.topics['battery']:
                self.update_battery_status(payload)
            elif topic == self.topics['imu']:
                self.update_imu_data(payload)
                
        except Exception as e:
            print(f"[ERROR] Error processing message from topic {msg.topic}: {e}")
            if self.debug_mode:
                print(f"        Raw payload: {msg.payload}")
    
    def publish_command(self, topic, command_data):
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
    
    def update_robot_status(self, data):
        """Update robot status display"""
        self.debug_print(f"[DEBUG] Updating robot status with data: {data}")
        self.robot_data.update(data)
        
        # Update speed
        speed_val = data.get('speed_scale', 1.0)
        self.speed_label.config(text=f"{speed_val:.1f}x")
        self.debug_print(f"[DEBUG] Updated speed display to: {speed_val}")
        
        # Update roll balance
        roll_balance = data.get('roll_balance_enabled', False)
        if roll_balance:
            self.roll_balance_label.config(text="ON", fg='green')
        else:
            self.roll_balance_label.config(text="OFF", fg='red')
        self.debug_print(f"[DEBUG] Updated roll balance display to: {roll_balance}")
        
        # Update performance mode
        performance = data.get('performance_mode_enabled', False)
        if performance:
            self.performance_label.config(text="ON", fg='green')
        else:
            self.performance_label.config(text="OFF", fg='red')
        self.debug_print(f"[DEBUG] Updated performance display to: {performance}")
        
        # Update camera status
        camera = data.get('camera_enabled', False)
        if camera:
            self.camera_label.config(text="ON", fg='green')
        else:
            self.camera_label.config(text="OFF", fg='red')
        self.debug_print(f"[DEBUG] Updated camera display to: {camera}")
        
        # Update controller status
        controller = data.get('controller_connected', False)
        self.robot_data['controller_connected'] = controller
        self.debug_print(f"[DEBUG] Updated controller status to: {controller}")
    
    def update_battery_status(self, data):
        """Update battery status display"""
        self.debug_print(f"[DEBUG] Updating battery status with: {data}")
        # Robot sends 'level' field, not 'battery_level'
        battery_level = data.get('level', data.get('battery_level', 0))
        self.robot_data['battery_level'] = battery_level
        
        # Update battery label
        self.battery_label.config(text=f"{battery_level}%")
        
        # Update progress bar
        self.battery_progress['value'] = battery_level
        
        # Update colors based on battery level
        if battery_level >= 70:
            color = 'green'
        elif battery_level >= 40:
            color = 'yellow'
        else:
            color = 'red'
        
        self.battery_label.config(fg=color)
        self.debug_print(f"[DEBUG] Updated battery display to: {battery_level}% (color: {color})")
    
    def update_imu_data(self, data):
        """Update IMU/odometry data display"""
        self.debug_print(f"[DEBUG] Updating IMU data with: {data}")
        roll = data.get('roll', 0.0)
        pitch = data.get('pitch', 0.0)
        yaw = data.get('yaw', 0.0)
        
        self.robot_data.update({'roll': roll, 'pitch': pitch, 'yaw': yaw})
        
        # Update labels with proper formatting
        self.roll_label.config(text=f"{roll:+.1f}°")
        self.pitch_label.config(text=f"{pitch:+.1f}°") 
        self.yaw_label.config(text=f"{yaw:+.1f}°")
        self.debug_print(f"[DEBUG] Updated IMU display - Roll: {roll:+.1f}°, Pitch: {pitch:+.1f}°, Yaw: {yaw:+.1f}°")
    
    def send_movement(self, x, y):
        """Send movement command"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Not connected to robot")
            return
        
        command = {
            'x': x,
            'y': y,
            'timestamp': time.time()
        }
        
        self.publish_command(self.topics['control_movement'], command)
    
    def toggle_roll_balance(self):
        """Toggle roll balance setting"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Not connected to robot")
            return
        
        command = {
            'action': 'toggle_roll_balance',
            'timestamp': time.time()
        }
        
        self.publish_command(self.topics['control_settings'], command)
    
    def toggle_performance(self):
        """Toggle performance mode"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Not connected to robot")
            return
        
        command = {
            'action': 'toggle_performance',
            'timestamp': time.time()
        }
        
        self.publish_command(self.topics['control_settings'], command)
    
    def toggle_camera(self):
        """Toggle camera"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Not connected to robot")
            return
        
        command = {
            'action': 'toggle_camera',
            'timestamp': time.time()
        }
        
        self.publish_command(self.topics['control_camera'], command)
    
    def change_speed(self, value):
        """Change speed setting"""
        if not self.connected:
            return
        
        command = {
            'action': 'change_speed',
            'value': float(value),
            'timestamp': time.time()
        }
        
        self.publish_command(self.topics['control_settings'], command)
    
    def request_battery(self):
        """Request battery status update"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Not connected to robot")
            return
        
        command = {
            'action': 'request_battery',
            'timestamp': time.time()
        }
        
        self.publish_command(self.topics['request_battery'], command)
    
    def emergency_stop(self):
        """Emergency stop"""
        if not self.connected:
            messagebox.showwarning("Not Connected", "Not connected to robot")
            return
        
        if messagebox.askyesno("Emergency Stop", "Are you sure you want to emergency stop the robot?"):
            command = {
                'action': 'emergency_stop',
                'timestamp': time.time()
            }
            
            self.publish_command(self.topics['control_system'], command)
    
    def reconnect(self):
        """Reconnect to MQTT broker"""
        self.disconnect()
        time.sleep(1)
        self.connect_mqtt()
    
    def disconnect(self):
        """Disconnect from MQTT broker"""
        if self.mqtt_client:
            try:
                print("📡 Disconnecting from MQTT broker...")
                # Stop the loop immediately and don't wait
                self.mqtt_client.loop_stop()
                # Just mark as disconnected, don't wait for network
                self.mqtt_client = None
                print("✅ MQTT disconnected")
            except Exception as e:
                print(f"⚠️ MQTT disconnect error (ignored): {e}")
        self.connected = False
    
    def run(self):
        """Run the GUI application"""
        def on_window_close():
            """Handle window close event"""
            print("🪟 Window close requested...")
            # Immediately stop all operations
            self.gui_running = False
            
            # Force-stop MQTT immediately
            try:
                if self.mqtt_client:
                    self.mqtt_client.loop_stop()
                    self.mqtt_client = None
                print("📡 MQTT force-stopped")
            except:
                pass
            
            # Destroy GUI immediately  
            try:
                self.root.quit()
                print("✅ Window closed successfully")
            except:
                print("⚠️ Force closing...")
                import sys
                sys.exit(0)
        
        # Set up proper window close handler
        self.root.protocol("WM_DELETE_WINDOW", on_window_close)
        
        try:
            print("🖥️ Starting GUI...")
            self.root.mainloop()
        except KeyboardInterrupt:
            print("\n⌨️ Keyboard interrupt received...")
            self.cleanup()
        except Exception as e:
            print(f"❌ GUI error: {e}")
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        print("🛑 Shutting down PC client...")
        
        # Stop all background operations first
        self.gui_running = False
        
        # Disconnect MQTT with timeout protection
        try:
            self.disconnect()
        except Exception as e:
            print(f"⚠️ MQTT cleanup error (ignored): {e}")
        
        # Give threads a very brief moment to stop
        time.sleep(0.1)
        
        # Force destroy GUI
        try:
            if hasattr(self, 'root') and self.root:
                self.root.quit()
                # Force immediate destruction
                self.root.destroy()
                print("🖥️ GUI destroyed")
        except Exception as e:
            print(f"⚠️ GUI cleanup error (ignored): {e}")
        
        print("✅ PC client cleanup complete")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Rider Robot PC Client - Standalone Version')
    parser.add_argument('-d', '--debug', action='store_true', 
                       help='Enable debug messages showing all MQTT traffic')
    args = parser.parse_args()
    
    print("Rider Robot PC Client - Standalone Version")
    print("==========================================")
    if args.debug:
        print("🐛 Debug mode enabled - showing all MQTT traffic")
    else:
        print("ℹ️ Use -d or --debug flag to see detailed MQTT traffic")
    
    client = None
    try:
        client = RiderPCClient(debug=args.debug)
        client.run()
    except KeyboardInterrupt:
        print("\n⌨️ Keyboard interrupt in main...")
    except Exception as e:
        print(f"❌ Main error: {e}")
    finally:
        if client:
            try:
                # Try cleanup with timeout protection
                import signal
                
                def cleanup_timeout_handler(signum, frame):
                    print("⏰ Cleanup timeout - forcing exit...")
                    import sys
                    sys.exit(0)
                
                # Set 3-second timeout for cleanup
                signal.signal(signal.SIGALRM, cleanup_timeout_handler)
                signal.alarm(3)
                
                client.cleanup()
                
                # Cancel timeout if cleanup succeeded
                signal.alarm(0)
                
            except Exception as e:
                print(f"⚠️ Final cleanup error: {e}")
                print("🚪 Forcing exit...")
                import sys
                sys.exit(0)
        print("👋 Application terminated")

if __name__ == "__main__":
    main() 