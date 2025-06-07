#!/usr/bin/env python3
# coding=utf-8

from datetime import datetime, timedelta
from typing import Dict, Any, Callable, List

class RobotState:
    def __init__(self):
        # Robot state data
        self.data = {
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
            'cpu_percent': 0.0,
            'cpu_load_1min': 0.0,
            'cpu_load_5min': 0.0,
            'cpu_load_15min': 0.0,
            'last_update': None
        }
        
        # Controller timeout tracking
        self.controller_timeout_seconds = 5.0  # Consider controller disconnected after 5 seconds
        self.last_controller_update = None
        
        # Observer pattern - callbacks for state changes
        self._observers: Dict[str, List[Callable]] = {
            'battery': [],
            'imu': [],
            'status': [],
            'all': []
        }
    
    def add_observer(self, event_type: str, callback: Callable):
        """Add observer for specific state changes"""
        if event_type in self._observers:
            self._observers[event_type].append(callback)
    
    def remove_observer(self, event_type: str, callback: Callable):
        """Remove observer for specific state changes"""
        if event_type in self._observers and callback in self._observers[event_type]:
            self._observers[event_type].remove(callback)
    
    def _notify_observers(self, event_type: str, data: Dict[str, Any]):
        """Notify all observers of state change"""
        # Notify specific event observers
        for callback in self._observers.get(event_type, []):
            try:
                callback(data)
            except Exception as e:
                print(f"Error in observer callback: {e}")
        
        # Notify global observers
        for callback in self._observers.get('all', []):
            try:
                callback(event_type, data)
            except Exception as e:
                print(f"Error in global observer callback: {e}")
    
    def update_battery(self, data: Dict[str, Any]):
        """Update battery status"""
        # Robot sends 'level' field, not 'battery_level'
        battery_level = data.get('level', data.get('battery_level', 0))
        old_level = self.data['battery_level']
        self.data['battery_level'] = battery_level
        self.data['last_update'] = datetime.now()
        
        # Only notify if value changed
        if old_level != battery_level:
            self._notify_observers('battery', {'level': battery_level})
    
    def update_imu(self, data: Dict[str, Any]):
        """Update IMU/orientation data"""
        old_roll = self.data['roll']
        old_pitch = self.data['pitch']
        old_yaw = self.data['yaw']
        
        self.data['roll'] = data.get('roll', 0.0)
        self.data['pitch'] = data.get('pitch', 0.0)
        self.data['yaw'] = data.get('yaw', 0.0)
        self.data['last_update'] = datetime.now()
        
        # Only notify if values changed significantly (avoid noise)
        if (abs(old_roll - self.data['roll']) > 0.1 or 
            abs(old_pitch - self.data['pitch']) > 0.1 or
            abs(old_yaw - self.data['yaw']) > 0.1):
            self._notify_observers('imu', {
                'roll': self.data['roll'],
                'pitch': self.data['pitch'],
                'yaw': self.data['yaw']
            })
    
    def update_status(self, data: Dict[str, Any]):
        """Update general robot status"""
        changed_fields = {}
        
        # Track which fields actually changed
        for key, value in data.items():
            if key in self.data and self.data[key] != value:
                changed_fields[key] = value
                self.data[key] = value
        
        # Update controller timestamp ONLY if controller is connected (not just when status is present)
        if 'controller_connected' in data and data['controller_connected']:
            self.last_controller_update = datetime.now()
        
        if changed_fields:
            self.data['last_update'] = datetime.now()
            self._notify_observers('status', changed_fields)
    
    def get_current_state(self) -> Dict[str, Any]:
        """Get current complete robot state"""
        return self.data.copy()
    
    def get_battery_level(self) -> int:
        """Get current battery level"""
        return self.data['battery_level']
    
    def get_imu_data(self) -> Dict[str, float]:
        """Get current IMU data"""
        return {
            'roll': self.data['roll'],
            'pitch': self.data['pitch'],
            'yaw': self.data['yaw']
        }
    
    def get_cpu_data(self) -> Dict[str, float]:
        """Get current CPU data"""
        return {
            'cpu_percent': self.data['cpu_percent'],
            'cpu_load_1min': self.data['cpu_load_1min'],
            'cpu_load_5min': self.data['cpu_load_5min'],
            'cpu_load_15min': self.data['cpu_load_15min']
        }
    
    def _check_controller_timeout(self):
        """Check if controller has timed out and update status if needed"""
        if self.last_controller_update is None:
            return  # No controller updates received yet
        
        time_since_update = datetime.now() - self.last_controller_update
        if time_since_update.total_seconds() > self.controller_timeout_seconds:
            # Controller has timed out
            if self.data['controller_connected']:
                # Controller was connected but has now timed out
                self.data['controller_connected'] = False
                self._notify_observers('status', {'controller_connected': False})
    
    def get_controller_connected(self) -> bool:
        """Get current controller connection status with timeout check"""
        self._check_controller_timeout()
        return self.data['controller_connected']
    
    def get_feature_status(self) -> Dict[str, bool]:
        """Get current feature status with controller timeout check"""
        self._check_controller_timeout()
        return {
            'roll_balance_enabled': self.data['roll_balance_enabled'],
            'performance_mode_enabled': self.data['performance_mode_enabled'],
            'camera_enabled': self.data['camera_enabled'],
            'controller_connected': self.data['controller_connected']
        }
    
    def is_connected(self) -> bool:
        """Check if robot is connected"""
        return self.data['connection_status'] == 'connected' 