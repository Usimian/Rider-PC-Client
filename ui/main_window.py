#!/usr/bin/env python3
# coding=utf-8

import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
from typing import Dict, Callable, Any
from .status_widgets import BatteryWidget, ControllerWidget, SpeedWidget, StatusBar
from .control_panels import IMUPanel, FeaturesPanel, MovementPanel

class MainWindow:
    def __init__(self, broker_host: str, callbacks: Dict[str, Callable], debug: bool = False):
        self.broker_host = broker_host
        self.callbacks = callbacks
        self.debug_mode = debug
        self.setup_main_window()
        self.create_widgets()
        self.create_menu()
    
    def setup_main_window(self):
        """Setup the main window"""
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
    
    def create_widgets(self):
        """Create all GUI widgets"""
        # Status bar at top
        self.status_bar = StatusBar(self.root, self.broker_host)
        self.status_bar.get_widget().grid(row=0, column=0, sticky="ew")
        
        # Main container with padding - use grid to work with root grid configuration
        main_container = tk.Frame(self.root, bg='#2b2b2b')
        main_container.grid(row=1, column=0, sticky="nsew", padx=15, pady=15)
        
        # Configure main container grid weights for proper resizing
        main_container.grid_rowconfigure(1, weight=1)  # Middle row (IMU/Features) expands
        main_container.grid_columnconfigure(0, weight=1)
        
        # Top row - Status cards
        self.create_status_row(main_container)
        
        # Middle row - IMU Data and Robot Features
        self.create_middle_row(main_container)
        
        # Bottom row - Movement Controls
        self.create_bottom_row(main_container)
        
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
    
    def create_status_row(self, parent):
        """Create the top status row with battery, controller, and speed"""
        status_row = tk.Frame(parent, bg='#2b2b2b')
        status_row.grid(row=0, column=0, sticky="ew", pady=(0, 15))
        
        # Battery Status Card
        self.battery_widget = BatteryWidget(status_row)
        self.battery_widget.get_widget().pack(side='left', padx=(0, 10), pady=5, fill='x', expand=True)
        
        # Controller Status Card
        self.controller_widget = ControllerWidget(status_row)
        self.controller_widget.get_widget().pack(side='left', padx=(0, 10), pady=5, fill='x', expand=True)
        
        # Speed Control Card
        speed_callback = self.callbacks.get('change_speed', None)
        self.speed_widget = SpeedWidget(status_row, speed_callback)
        self.speed_widget.get_widget().pack(side='left', pady=5, fill='x', expand=True)
    
    def create_middle_row(self, parent):
        """Create the middle row with IMU and features panels"""
        middle_row = tk.Frame(parent, bg='#2b2b2b')
        middle_row.grid(row=1, column=0, sticky="nsew", pady=(0, 15))
        
        # Configure middle row for resizing
        middle_row.grid_rowconfigure(0, weight=1)
        middle_row.grid_columnconfigure(0, weight=1)
        middle_row.grid_columnconfigure(1, weight=1)
        
        # IMU Data Panel (Left)
        self.imu_panel = IMUPanel(middle_row)
        self.imu_panel.get_widget().grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        
        # Robot Features Panel (Right)
        feature_callbacks = {
            'toggle_roll_balance': self.callbacks.get('toggle_roll_balance', lambda: None),
            'toggle_performance': self.callbacks.get('toggle_performance', lambda: None),
            'toggle_camera': self.callbacks.get('toggle_camera', lambda: None)
        }
        self.features_panel = FeaturesPanel(middle_row, feature_callbacks)
        self.features_panel.get_widget().grid(row=0, column=1, sticky="nsew")
    
    def create_bottom_row(self, parent):
        """Create the bottom row with movement controls"""
        movement_callbacks = {
            'move': self.callbacks.get('send_movement', lambda x, y: None),
            'emergency_stop': self.callbacks.get('emergency_stop', lambda: None)
        }
        self.movement_panel = MovementPanel(parent, movement_callbacks)
        self.movement_panel.get_widget().grid(row=2, column=0, sticky="ew")
    
    def create_menu(self):
        """Create menu bar"""
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)
        
        # Connection menu
        connection_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Connection", menu=connection_menu)
        connection_menu.add_command(label="Change Robot IP", command=self._change_robot_ip)
        connection_menu.add_separator()
        connection_menu.add_command(label="Reconnect", command=self.callbacks.get('reconnect', lambda: None))
        connection_menu.add_command(label="Disconnect", command=self.callbacks.get('disconnect', lambda: None))
        
        # Help menu
        help_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="Help", menu=help_menu)
        help_menu.add_command(label="About", command=self._show_about)
    
    def _change_robot_ip(self):
        """Change robot IP address"""
        new_ip = simpledialog.askstring("Robot IP", "Enter robot IP address:", initialvalue=self.broker_host)
        if new_ip and new_ip != self.broker_host:
            change_ip_callback = self.callbacks.get('change_robot_ip')
            if change_ip_callback:
                change_ip_callback(new_ip)
    
    def _show_about(self):
        """Show about dialog"""
        messagebox.showinfo("About", "Rider Robot PC Client v2.0\nDeveloped for dual-environment control")
    
    def update_title(self, broker_host: str):
        """Update window title with new broker host"""
        self.broker_host = broker_host
        self.root.title(f"Rider Robot PC Client - {broker_host}")
    
    def set_close_callback(self, callback: Callable):
        """Set callback for window close event"""
        self.root.protocol("WM_DELETE_WINDOW", callback)
    
    def mainloop(self):
        """Start the GUI main loop"""
        try:
            print("🖥️ Starting GUI...")
            self.root.mainloop()
        except KeyboardInterrupt:
            print("\n⌨️ Keyboard interrupt received...")
        except Exception as e:
            print(f"❌ GUI error: {e}")
    
    def quit(self):
        """Quit the GUI"""
        try:
            self.root.quit()
        except:
            pass
    
    def destroy(self):
        """Destroy the GUI"""
        try:
            self.root.destroy()
        except:
            pass
    
    def schedule_update(self, callback: Callable):
        """Schedule a callback to run in the main GUI thread"""
        try:
            self.root.after(0, callback)
        except:
            pass  # GUI might be destroyed
    
    # Widget update methods
    def update_connection_status(self, connected: bool, message: str = None):
        """Update connection status display"""
        self.status_bar.update_connection_status(connected, message)
    
    def update_time(self):
        """Update time display"""
        self.status_bar.update_time()
    
    def update_battery(self, data: Dict[str, Any]):
        """Update battery display"""
        self.battery_widget.update_battery(data)
    
    def update_controller_status(self, is_mqtt_connected: bool, is_controller_connected: bool):
        """Update controller status display"""
        self.controller_widget.update_status(is_mqtt_connected, is_controller_connected)
    
    def update_speed(self, speed_value: float):
        """Update speed display"""
        self.speed_widget.update_speed(speed_value)
    
    def update_imu_data(self, data: Dict[str, float]):
        """Update IMU data display"""
        self.imu_panel.update_imu_data(data)
    
    def update_feature_status(self, feature: str, enabled: bool):
        """Update individual feature status"""
        self.features_panel.update_feature_status(feature, enabled)
    
    def update_all_features(self, data: Dict[str, Any]):
        """Update all feature statuses"""
        self.features_panel.update_all_features(data) 