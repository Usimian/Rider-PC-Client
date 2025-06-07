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
    
    def draw_battery_icon(self, battery_level):
        """Draw a battery icon that fills based on battery level"""
        if not hasattr(self, 'battery_canvas'):
            return
            
        # Clear the canvas
        self.battery_canvas.delete("all")
        
        # Battery dimensions
        battery_width = 30
        battery_height = 16
        terminal_width = 3
        terminal_height = 8
        
        # Calculate battery body position (centered in canvas)
        x1 = 5
        y1 = 2
        x2 = x1 + battery_width
        y2 = y1 + battery_height
        
        # Draw battery outline (white)
        self.battery_canvas.create_rectangle(x1, y1, x2, y2, outline='white', width=2, fill='#2b2b2b')
        
        # Draw battery terminal (small rectangle on right side)
        terminal_x1 = x2
        terminal_y1 = y1 + (battery_height - terminal_height) // 2
        terminal_x2 = terminal_x1 + terminal_width
        terminal_y2 = terminal_y1 + terminal_height
        self.battery_canvas.create_rectangle(terminal_x1, terminal_y1, terminal_x2, terminal_y2, 
                                           outline='white', width=2, fill='#2b2b2b')
        
        # Draw battery fill based on level
        if battery_level > 0:
            fill_width = max(1, int((battery_width - 4) * battery_level / 100))  # -4 for padding
            fill_x1 = x1 + 2
            fill_y1 = y1 + 2
            fill_x2 = fill_x1 + fill_width
            fill_y2 = y2 - 2
            
            # Color based on battery level
            if battery_level >= 70:
                fill_color = '#4caf50'  # Green
            elif battery_level >= 40:
                fill_color = '#ff9800'  # Orange  
            elif battery_level >= 20:
                fill_color = '#ff5722'  # Red-orange
            else:
                fill_color = '#f44336'  # Red
                
            self.battery_canvas.create_rectangle(fill_x1, fill_y1, fill_x2, fill_y2, 
                                               outline='', fill=fill_color)
    
    def setup_gui(self):
        """Create a clean, organized GUI interface"""
        self.root = tk.Tk()
        self.root.title(f"Rider Robot PC Client - {self.broker_host}")
        self.root.geometry("1200x800")
        self.root.configure(bg='#2b2b2b')  # Modern dark theme
        
        # Configure root window to use grid with proper weights (key for resizing!)
        self.root.grid_rowconfigure(0, weight=0)  # Status bar row (fixed height)
        self.root.grid_rowconfigure(1, weight=1)  # Main content row (expandable)
        self.root.grid_columnconfigure(0, weight=1)
        
        # Force enable resizing immediately after window creation
        self.root.resizable(width=True, height=True)
        self.root.minsize(width=800, height=600)
        self.root.maxsize(width=2000, height=1500)  # Set reasonable maximum size
        
        # Create menu bar
        self.create_menu()
        
        # Connection status bar
        status_bar = tk.Frame(self.root, bg='#404040')
        status_bar.grid(row=0, column=0, sticky="ew")
        
        self.connection_status = tk.Label(status_bar, text=f"Connecting to {self.broker_host}...", 
                                        bg='#404040', fg='#ffd700', font=('Arial', 11, 'bold'))
        self.connection_status.pack(side='left', padx=15, pady=8)
        
        # Current time (right side of status bar)
        self.time_label = tk.Label(status_bar, text="--:--", font=('Arial', 11, 'bold'), 
                                 bg='#404040', fg='white')
        self.time_label.pack(side='right', padx=15, pady=8)
        
        # Main container with padding - use grid to work with root grid configuration
        main_container = tk.Frame(self.root, bg='#2b2b2b')
        main_container.grid(row=1, column=0, sticky="nsew", padx=15, pady=15)
        
        # Configure main container grid weights for proper resizing
        main_container.grid_rowconfigure(1, weight=1)  # Middle row (IMU/Features) expands
        main_container.grid_columnconfigure(0, weight=1)
        
        # Top row - Status cards
        status_row = tk.Frame(main_container, bg='#2b2b2b')
        status_row.grid(row=0, column=0, sticky="ew", pady=(0, 15))
        
        # Battery Status Card
        battery_card = tk.Frame(status_row, bg='#3c3c3c', relief='solid', bd=1)
        battery_card.pack(side='left', padx=(0, 10), pady=5, fill='x', expand=True)
        
        battery_content = tk.Frame(battery_card, bg='#3c3c3c')
        battery_content.pack(pady=15)
        
        # Battery icon and percentage container
        battery_display = tk.Frame(battery_content, bg='#3c3c3c')
        battery_display.pack()
        
        # Create custom battery icon canvas
        self.battery_canvas = tk.Canvas(battery_display, width=40, height=20, bg='#3c3c3c', 
                                       highlightthickness=0, bd=0)
        self.battery_canvas.pack(side='left', padx=(0, 10))
        
        # Battery percentage label
        self.battery_label = tk.Label(battery_display, text="0%", font=('Arial', 16, 'bold'), 
                                    bg='#3c3c3c', fg='red')
        self.battery_label.pack(side='left')
        
        # Draw initial battery outline
        self.draw_battery_icon(0)
        
        # Controller Status Card
        controller_card = tk.Frame(status_row, bg='#3c3c3c', relief='solid', bd=1)
        controller_card.pack(side='left', padx=(0, 10), pady=5, fill='x', expand=True)
        
        # Controller icon - color changes based on status
        self.controller_icon = tk.Label(controller_card, text="🎮", font=('Arial', 30), 
                                       bg='#3c3c3c', fg='#808080')  # Start with grey
        self.controller_icon.pack(pady=15)
        
        # Speed Control Card
        speed_card = tk.Frame(status_row, bg='#3c3c3c', relief='solid', bd=1)
        speed_card.pack(side='left', pady=5, fill='x', expand=True)
        
        tk.Label(speed_card, text="⚡ SPEED", font=('Arial', 10, 'bold'), 
                bg='#3c3c3c', fg='#87ceeb').pack(pady=(8, 2))
        
        self.speed_label = tk.Label(speed_card, text="1.0x", font=('Arial', 16, 'bold'), 
                                   bg='#3c3c3c', fg='white')
        self.speed_label.pack(pady=(0, 5))
        
        speed_scale = tk.Scale(speed_card, from_=0.1, to=2.0, resolution=0.1, orient='horizontal',
                             bg='#3c3c3c', fg='white', highlightbackground='#3c3c3c',
                             command=self.change_speed, length=300, width=15)
        speed_scale.set(1.0)
        speed_scale.pack(pady=(0, 8))
        
        # Middle row - IMU Data and Robot Features
        middle_row = tk.Frame(main_container, bg='#2b2b2b')
        middle_row.grid(row=1, column=0, sticky="nsew", pady=(0, 15))
        
        # Configure middle row for resizing
        middle_row.grid_rowconfigure(0, weight=1)
        middle_row.grid_columnconfigure(0, weight=1)
        middle_row.grid_columnconfigure(1, weight=1)
        
        # IMU Data Panel (Left)
        imu_panel = tk.LabelFrame(middle_row, text="📊 IMU / ORIENTATION DATA", 
                                 font=('Arial', 12, 'bold'), bg='#3c3c3c', fg='#87ceeb',
                                 relief='solid', bd=1)
        imu_panel.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        
        imu_grid = tk.Frame(imu_panel, bg='#3c3c3c')
        imu_grid.pack(padx=20, pady=15, fill='both', expand=True)
        
        # IMU values in a grid
        for i, (label, attr) in enumerate([("Roll", "roll"), ("Pitch", "pitch"), ("Yaw", "yaw")]):
            row = tk.Frame(imu_grid, bg='#3c3c3c')
            row.pack(fill='x', pady=8)
            
            tk.Label(row, text=f"{label}:", font=('Arial', 14, 'bold'), 
                    bg='#3c3c3c', fg='#ffd700', width=6, anchor='w').pack(side='left')
            
            value_label = tk.Label(row, text="+0.0°", font=('Arial', 14, 'bold'), 
                                 bg='#3c3c3c', fg='white')
            value_label.pack(side='left', padx=(10, 0))
            setattr(self, f"{attr}_label", value_label)
        
        # Robot Features Panel (Right)
        features_panel = tk.LabelFrame(middle_row, text="🤖 ROBOT FEATURES", 
                                      font=('Arial', 12, 'bold'), bg='#3c3c3c', fg='#87ceeb',
                                      relief='solid', bd=1)
        features_panel.grid(row=0, column=1, sticky="nsew")
        
        features_grid = tk.Frame(features_panel, bg='#3c3c3c')
        features_grid.pack(padx=20, pady=15, fill='both', expand=True)
        
        # Feature status indicators
        features = [
            ("Roll Balance", "roll_balance", self.toggle_roll_balance),
            ("Performance Mode", "performance", self.toggle_performance), 
            ("Camera", "camera", self.toggle_camera)
        ]
        
        for i, (name, attr, command) in enumerate(features):
            feature_row = tk.Frame(features_grid, bg='#3c3c3c')
            feature_row.pack(fill='x', pady=8)
            
            tk.Label(feature_row, text=f"{name}:", font=('Arial', 12, 'bold'), 
                    bg='#3c3c3c', fg='white', width=15, anchor='w').pack(side='left')
            
            status_label = tk.Label(feature_row, text="OFF", font=('Arial', 12, 'bold'), 
                                   bg='#3c3c3c', fg='red', width=6)
            status_label.pack(side='left', padx=(5, 10))
            setattr(self, f"{attr}_label", status_label)
            
            tk.Button(feature_row, text="Toggle", command=command, 
                     font=('Arial', 9), bg='#555555', fg='white', 
                     activebackground='#666666', width=8).pack(side='right')
        
        # Bottom row - Movement Controls
        control_panel = tk.LabelFrame(main_container, text="🎮 ROBOT CONTROL", 
                                     font=('Arial', 12, 'bold'), bg='#3c3c3c', fg='#87ceeb',
                                     relief='solid', bd=1)
        control_panel.grid(row=2, column=0, sticky="ew")
        
        control_content = tk.Frame(control_panel, bg='#3c3c3c')
        control_content.pack(padx=20, pady=15)
        
        # Movement controls (Left side)
        movement_frame = tk.Frame(control_content, bg='#3c3c3c')
        movement_frame.pack(side='left', padx=(0, 40))
        
        tk.Label(movement_frame, text="Movement Controls", font=('Arial', 11, 'bold'), 
                bg='#3c3c3c', fg='white').pack(pady=(0, 10))
        
        # Arrow button grid
        move_grid = tk.Frame(movement_frame, bg='#3c3c3c')
        move_grid.pack()
        
        move_buttons = [
            ("↑", 0, 50, 0, 1),
            ("←", -50, 0, 1, 0), 
            ("⏹", 0, 0, 1, 1),
            ("→", 50, 0, 1, 2),
            ("↓", 0, -50, 2, 1)
        ]
        
        for text, x, y, row, col in move_buttons:
            btn = tk.Button(move_grid, text=text, command=lambda x=x, y=y: self.send_movement(x, y),
                           width=4, height=2, font=('Arial', 14, 'bold'),
                           bg='#555555', fg='white', activebackground='#666666')
            btn.grid(row=row, column=col, padx=2, pady=2)
        
        # Emergency stop (Right side)
        system_frame = tk.Frame(control_content, bg='#3c3c3c')
        system_frame.pack(side='right')
        
        tk.Button(system_frame, text="🚨 EMERGENCY STOP", command=self.emergency_stop,
                 font=('Arial', 12, 'bold'), bg='#d32f2f', fg='white', 
                 activebackground='#b71c1c', width=18, pady=10).pack()
        
        # Configure ttk styles
        style = ttk.Style()
        
        # Add sizegrip to bottom-right corner for visual resize handle
        self.sizegrip = ttk.Sizegrip(self.root)
        self.sizegrip.grid(row=1, column=0, sticky="se")
        
        # Force update and then enable resizing
        self.root.update_idletasks()
        self.root.resizable(True, True)  # Force resizable again after layout
        
        # Debug: Verify resizable settings
        if self.debug_mode:
            print(f"Final window resizable: {self.root.resizable()}")
            print(f"Final window minsize: {self.root.minsize()}")
            print(f"Final window geometry: {self.root.geometry()}")
        
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
            if not self.connected:
                # MQTT not connected - grey
                self.controller_icon.config(fg='#808080')
                self.debug_print("[DEBUG] Controller icon: GREY (MQTT disconnected)")
            elif self.robot_data['controller_connected']:
                # MQTT connected AND controller responding - green
                self.controller_icon.config(fg='#4caf50')
                self.debug_print("[DEBUG] Controller icon: GREEN (controller connected)")
            else:
                # MQTT connected but no controller response - red
                self.controller_icon.config(fg='#f44336')
                self.debug_print("[DEBUG] Controller icon: RED (controller disconnected)")
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
        """Update robot status display - schedule GUI updates in main thread"""
        self.debug_print(f"[DEBUG] Updating robot status with data: {data}")
        self.robot_data.update(data)
        
        def _update_gui():
            try:
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
            except:
                pass  # GUI might be destroyed
        
        # Schedule GUI update in main thread
        if hasattr(self, 'root'):
            self.root.after(0, _update_gui)
    
    def update_battery_status(self, data):
        """Update battery status display - schedule GUI updates in main thread"""
        self.debug_print(f"[DEBUG] Updating battery status with: {data}")
        # Robot sends 'level' field, not 'battery_level'
        battery_level = data.get('level', data.get('battery_level', 0))
        self.robot_data['battery_level'] = battery_level
        
        def _update_gui():
            try:
                # Update battery percentage label
                self.battery_label.config(text=f"{battery_level}%")
                
                # Update battery icon
                self.draw_battery_icon(battery_level)
                
                # Update percentage label color based on battery level
                if battery_level >= 70:
                    color = '#4caf50'  # Green
                elif battery_level >= 40:
                    color = '#ff9800'  # Orange
                elif battery_level >= 20:
                    color = '#ff5722'  # Red-orange
                else:
                    color = '#f44336'  # Red
                
                self.battery_label.config(fg=color)
                self.debug_print(f"[DEBUG] Updated battery display to: {battery_level}% (color: {color})")
            except:
                pass  # GUI might be destroyed
        
        # Schedule GUI update in main thread
        if hasattr(self, 'root'):
            self.root.after(0, _update_gui)
    
    def update_imu_data(self, data):
        """Update IMU/odometry data display - schedule GUI updates in main thread"""
        self.debug_print(f"[DEBUG] Updating IMU data with: {data}")
        roll = data.get('roll', 0.0)
        pitch = data.get('pitch', 0.0)
        yaw = data.get('yaw', 0.0)
        
        self.robot_data.update({'roll': roll, 'pitch': pitch, 'yaw': yaw})
        
        def _update_gui():
            try:
                # Update labels with proper formatting
                self.roll_label.config(text=f"{roll:+.1f}°")
                self.pitch_label.config(text=f"{pitch:+.1f}°") 
                self.yaw_label.config(text=f"{yaw:+.1f}°")
                self.debug_print(f"[DEBUG] Updated IMU display - Roll: {roll:+.1f}°, Pitch: {pitch:+.1f}°, Yaw: {yaw:+.1f}°")
            except:
                pass  # GUI might be destroyed
        
        # Schedule GUI update in main thread
        if hasattr(self, 'root'):
            self.root.after(0, _update_gui)
    
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
    

    
    def emergency_stop(self):
        """Emergency stop - immediate, no confirmation"""
        if not self.connected:
            # Even if not connected, try to show status - don't block emergency action
            print("⚠️ Emergency stop attempted but not connected to robot")
            return
        
        print("🚨 EMERGENCY STOP ACTIVATED")
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
    
    def run(self):
        """Run the GUI application"""
        def on_window_close():
            """Handle window close event with timeout protection"""
            print("🪟 Window close requested...")
            
            # Set up timeout protection for hanging cleanup
            import threading
            import time
            
            def force_exit():
                time.sleep(2)  # Give 2 seconds for normal cleanup
                print("⏰ Cleanup timeout - forcing exit...")
                import os
                os._exit(0)
            
            # Start timeout thread
            timeout_thread = threading.Thread(target=force_exit, daemon=True)
            timeout_thread.start()
            
            # Immediately stop all operations
            self.gui_running = False
            print("🛑 Stopping GUI operations...")
            
            # Force-stop MQTT with timeout
            try:
                if self.mqtt_client:
                    print("📡 Stopping MQTT...")
                    self.mqtt_client.loop_stop()
                    self.mqtt_client.disconnect()
                    self.mqtt_client = None
                    print("📡 MQTT stopped")
            except Exception as e:
                print(f"⚠️ MQTT cleanup error (ignored): {e}")
            
            # Destroy GUI immediately
            try:
                print("🖥️ Destroying GUI...")
                self.root.quit()
                self.root.destroy()
                print("✅ Window closed successfully")
                import sys
                sys.exit(0)
            except Exception as e:
                print(f"⚠️ Error closing window: {e}")
                import sys
                sys.exit(1)
        
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