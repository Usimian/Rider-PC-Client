#!/usr/bin/env python3
# coding=utf-8

import time
import threading
from typing import Dict, Callable, Any
from .main_window import MainWindow
from core.robot_state import RobotState

class GUIManager:
    def __init__(self, broker_host: str, robot_state: RobotState, callbacks: Dict[str, Callable], debug: bool = False):
        self.broker_host = broker_host
        self.robot_state = robot_state
        self.callbacks = callbacks
        self.debug_mode = debug
        
        # GUI components
        self.main_window = MainWindow(broker_host, callbacks, debug)
        
        # GUI update thread
        self.gui_running = True
        self.gui_thread = None
        
        # Setup observers for robot state changes
        self._setup_observers()
        
        # Setup GUI update thread
        self._start_gui_thread()
    
    def _setup_observers(self):
        """Setup observers for robot state changes"""
        # Battery updates
        self.robot_state.add_observer('battery', self._on_battery_update)
        
        # IMU updates
        self.robot_state.add_observer('imu', self._on_imu_update)
        
        # Status updates
        self.robot_state.add_observer('status', self._on_status_update)
    
    def _start_gui_thread(self):
        """Start GUI update thread"""
        self.gui_thread = threading.Thread(target=self._gui_update_loop, daemon=True)
        self.gui_thread.start()
        if self.debug_mode:
            print("🧵 GUI update thread started")
    
    def _gui_update_loop(self):
        """GUI update loop running in separate thread"""
        if self.debug_mode:
            print("🔄 GUI update loop started")
        
        while self.gui_running:
            try:
                # Update time display
                self.main_window.schedule_update(self.main_window.update_time)
                
                # Update controller status
                self._update_controller_status()
                
                time.sleep(0.1)  # 10 FPS update rate
            except Exception as e:
                if self.gui_running:  # Only log if we're supposed to be running
                    if self.debug_mode:
                        print(f"⚠️ GUI update error: {e}")
                break
        
        if self.debug_mode:
            print("🔄 GUI update loop stopped")
    
    def _update_controller_status(self):
        """Update controller connection status display"""
        if not self.gui_running:
            return
        
        def _update():
            try:
                mqtt_connected = self.callbacks.get('is_mqtt_connected', lambda: False)()
                controller_connected = self.robot_state.get_controller_connected()  # Uses timeout checking
                self.main_window.update_controller_status(mqtt_connected, controller_connected)
                
                if self.debug_mode:
                    status = "GREEN" if controller_connected else "RED" if mqtt_connected else "GREY"
                    print(f"[DEBUG] Controller icon: {status} (timeout check included)")
            except:
                pass  # GUI might be destroyed
        
        self.main_window.schedule_update(_update)
    
    def _on_battery_update(self, data: Dict[str, Any]):
        """Handle battery status updates"""
        if self.debug_mode:
            print(f"[GUI] Battery update: {data}")
        
        def _update():
            self.main_window.update_battery(data)
        
        self.main_window.schedule_update(_update)
    
    def _on_imu_update(self, data: Dict[str, Any]):
        """Handle IMU data updates"""
        if self.debug_mode:
            print(f"[GUI] IMU update: {data}")
        
        def _update():
            self.main_window.update_imu_data(data)
        
        self.main_window.schedule_update(_update)
    
    def _on_status_update(self, data: Dict[str, Any]):
        """Handle status updates"""
        if self.debug_mode:
            print(f"[GUI] Status update: {data}")
        
        def _update():
            # Update speed if changed
            if 'speed_scale' in data:
                self.main_window.update_speed(data['speed_scale'])
            
            # Update CPU data if any CPU fields are present
            cpu_fields = ['cpu_percent', 'cpu_load_1min']
            if any(field in data for field in cpu_fields):
                cpu_data = self.robot_state.get_cpu_data()
                self.main_window.update_cpu_data(cpu_data)
            
            # Update features
            self.main_window.update_all_features(data)
        
        self.main_window.schedule_update(_update)
    
    def update_connection_status(self, connected: bool, message: str = None):
        """Update connection status"""
        def _update():
            self.main_window.update_connection_status(connected, message)
        
        self.main_window.schedule_update(_update)
    
    def update_broker_host(self, new_host: str):
        """Update broker host"""
        self.broker_host = new_host
        def _update():
            self.main_window.update_title(new_host)
        
        self.main_window.schedule_update(_update)
    
    def update_image_display(self, image_data=None, success=True, error_message=None):
        """Update image display"""
        def _update():
            self.main_window.update_image_display(image_data, success, error_message)
        
        self.main_window.schedule_update(_update)
    
    def set_close_callback(self, callback: Callable):
        """Set callback for window close event"""
        self.main_window.set_close_callback(callback)
    
    def run(self):
        """Run the GUI main loop"""
        self.main_window.mainloop()
    
    def stop(self):
        """Stop the GUI manager"""
        print("🛑 Stopping GUI operations...")
        self.gui_running = False
        
        # Give the GUI thread a moment to stop
        if self.gui_thread and self.gui_thread.is_alive():
            try:
                self.gui_thread.join(timeout=0.2)  # Shorter timeout for force stop
            except:
                pass  # Don't wait too long
        
        # Force quit and destroy GUI immediately
        try:
            self.main_window.quit()
            self.main_window.destroy()
        except:
            pass  # GUI might already be destroyed
        print("🖥️ GUI stopped") 