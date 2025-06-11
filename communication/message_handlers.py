#!/usr/bin/env python3
# coding=utf-8

from typing import Dict, Any
from core.robot_state import RobotState

class MessageHandlers:
    def __init__(self, robot_state: RobotState, debug: bool = False):
        self.robot_state = robot_state
        self.debug_mode = debug
        
        # Image capture callback (will be set by application controller)
        self.image_capture_callback = None
        
        # Topic to handler mapping
        self.handlers = {
            'rider/status': self.handle_status_message,
            'rider/status/battery': self.handle_battery_message,
            'rider/status/imu': self.handle_imu_message,
            'rider/response/image_capture': self.handle_image_capture_response
        }
    
    def debug_print(self, message: str):
        """Print debug message only if debug mode is enabled"""
        if self.debug_mode:
            print(message)
    
    def get_handler(self, topic: str):
        """Get appropriate handler for topic"""
        return self.handlers.get(topic)
    
    def set_image_capture_callback(self, callback):
        """Set callback for image capture responses"""
        self.image_capture_callback = callback
    
    def handle_status_message(self, data: Dict[str, Any]):
        """Handle general status messages including battery level"""
        self.debug_print(f"[HANDLER] Processing status message: {data}")
        self.robot_state.update_status(data)
    
    def handle_battery_message(self, data: Dict[str, Any]):
        """Handle battery status messages"""
        self.debug_print(f"[HANDLER] Processing battery message: {data}")
        self.robot_state.update_battery(data)
    
    def handle_imu_message(self, data: Dict[str, Any]):
        """Handle IMU/orientation messages"""
        self.debug_print(f"[HANDLER] Processing IMU message: {data}")
        self.robot_state.update_imu(data)
    
    def handle_image_capture_response(self, data: Dict[str, Any]):
        """Handle image capture response messages"""
        self.debug_print(f"[HANDLER] Processing image capture response: {data}")
        if self.image_capture_callback:
            self.image_capture_callback(data)
        else:
            self.debug_print("[HANDLER] No image capture callback set") 