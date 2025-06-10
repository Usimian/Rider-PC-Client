#!/usr/bin/env python3
# coding=utf-8

import signal
import time
import sys
from typing import Dict, Any
from tkinter import messagebox

from .config_manager import ConfigManager
from .robot_state import RobotState
from communication.mqtt_client import MQTTClient
from communication.message_handlers import MessageHandlers
from ui.gui_manager import GUIManager

class ApplicationController:
    def __init__(self, debug: bool = False):
        self.debug_mode = debug
        self.cleanup_done = False  # Flag to prevent multiple cleanup calls
        
        # Initialize core components
        self.config_manager = ConfigManager()
        self.robot_state = RobotState()
        
        # Initialize communication
        self.mqtt_client = MQTTClient(
            self.config_manager.get_broker_host(),
            self.config_manager.get_broker_port(),
            debug
        )
        self.message_handlers = MessageHandlers(self.robot_state, debug)
        
        # Initialize GUI
        self.gui_manager = GUIManager(
            self.config_manager.get_broker_host(),
            self.robot_state,
            self._get_gui_callbacks(),
            debug
        )
        
        # Setup MQTT callbacks
        self._setup_mqtt_callbacks()
        
        # Setup signal handlers for graceful shutdown
        self._setup_signal_handlers()
        
        if debug:
            print("🔧 Application controller initialized")
    
    def _get_gui_callbacks(self) -> Dict[str, Any]:
        """Get callbacks for GUI interactions"""
        return {
            'change_speed': self._change_speed,
            'toggle_roll_balance': self._toggle_roll_balance,
            'toggle_performance': self._toggle_performance,
            'toggle_camera': self._toggle_camera,
            'send_movement': self._send_movement,
            'emergency_stop': self._emergency_stop,
            'reset_robot': self._reset_robot,
            'reboot_pi': self._reboot_pi,
            'poweroff_pi': self._poweroff_pi,
            'reconnect': self._reconnect,
            'disconnect': self._disconnect,
            'change_robot_ip': self._change_robot_ip,
            'is_mqtt_connected': self._is_mqtt_connected
        }
    
    def _setup_mqtt_callbacks(self):
        """Setup MQTT message callbacks"""
        # Get topics from MQTT client
        topics = self.mqtt_client.get_topics()
        
        # Register message handlers
        for topic_name, topic in topics.items():
            handler = self.message_handlers.get_handler(topic)
            if handler:
                self.mqtt_client.add_message_callback(topic, handler)
        
        # Register connection callbacks
        self.mqtt_client.add_connection_callback('connect', self._on_mqtt_connect)
        self.mqtt_client.add_connection_callback('disconnect', self._on_mqtt_disconnect)
    
    def _setup_signal_handlers(self):
        """Set up signal handlers for graceful shutdown"""
        def signal_handler(signum, frame):
            signal_name = signal.Signals(signum).name
            print(f"\n🚨 Received {signal_name} signal - initiating immediate shutdown...")
            self._force_shutdown()
        
        # Handle common termination signals
        signal.signal(signal.SIGINT, signal_handler)   # Ctrl-C
        signal.signal(signal.SIGTERM, signal_handler)  # Termination request
    
    def _force_shutdown(self):
        """Force immediate shutdown"""
        try:
            print("✅ Force shutdown initiated")
            
            # Stop GUI
            if hasattr(self, 'gui_manager'):
                self.gui_manager.stop()
            
            # Stop MQTT
            if hasattr(self, 'mqtt_client'):
                self.mqtt_client.graceful_disconnect()
            
            print("✅ Force shutdown complete")
        except Exception as e:
            print(f"⚠️ Shutdown error (ignored): {e}")
        finally:
            sys.exit(0)
    
    # MQTT Event Handlers
    def _on_mqtt_connect(self, success: bool):
        """Handle MQTT connection events"""
        if success:
            self.gui_manager.update_connection_status(True)
        else:
            self.gui_manager.update_connection_status(False, "Connection failed")
    
    def _on_mqtt_disconnect(self):
        """Handle MQTT disconnection events"""
        self.gui_manager.update_connection_status(False)
    
    # GUI Callback Methods
    def _change_speed(self, value: float):
        """Change speed setting"""
        if not self.mqtt_client.is_connected():
            return
        
        self.mqtt_client.send_settings_command('change_speed', value)
        if self.debug_mode:
            print(f"[APP] Speed changed to: {value}")
    
    def _toggle_roll_balance(self):
        """Toggle roll balance setting"""
        if not self.mqtt_client.is_connected():
            messagebox.showwarning("Not Connected", "Not connected to robot")
            return
        
        self.mqtt_client.send_settings_command('toggle_roll_balance')
        if self.debug_mode:
            print("[APP] Roll balance toggled")
    
    def _toggle_performance(self):
        """Toggle performance mode"""
        if not self.mqtt_client.is_connected():
            messagebox.showwarning("Not Connected", "Not connected to robot")
            return
        
        self.mqtt_client.send_settings_command('toggle_performance')
        if self.debug_mode:
            print("[APP] Performance mode toggled")
    
    def _toggle_camera(self):
        """Toggle camera"""
        if not self.mqtt_client.is_connected():
            messagebox.showwarning("Not Connected", "Not connected to robot")
            return
        
        self.mqtt_client.send_camera_command('toggle_camera')
        if self.debug_mode:
            print("[APP] Camera toggled")
    
    def _send_movement(self, x: float, y: float):
        """Send movement command"""
        if not self.mqtt_client.is_connected():
            messagebox.showwarning("Not Connected", "Not connected to robot")
            return
        
        self.mqtt_client.send_movement_command(x, y)
        if self.debug_mode:
            print(f"[APP] Movement command: x={x}, y={y}")
    
    def _emergency_stop(self):
        """Emergency stop - immediate, no confirmation"""
        if not self.mqtt_client.is_connected():
            print("⚠️ Emergency stop attempted but not connected to robot")
            return
        
        print("🚨 EMERGENCY STOP ACTIVATED")
        self.mqtt_client.send_system_command('emergency_stop')
    
    def _reset_robot(self):
        """Reset the robot - requires confirmation"""
        if not self.mqtt_client.is_connected():
            messagebox.showwarning("Not Connected", "Not connected to robot")
            return
        
        # Confirmation dialog
        result = messagebox.askyesno(
            "Reset Robot", 
            "Are you sure you want to reset the robot?\n\nThis will restart the robot software.",
            icon='warning'
        )
        
        if result:
            print("🔄 ROBOT RESET INITIATED")
            self.mqtt_client.send_system_command('reset_robot')
            if self.debug_mode:
                print("[APP] Robot reset command sent")
    
    def _reboot_pi(self):
        """Reboot the Raspberry Pi - requires confirmation"""
        if not self.mqtt_client.is_connected():
            messagebox.showwarning("Not Connected", "Not connected to robot")
            return
        
        # Confirmation dialog
        result = messagebox.askyesno(
            "Reboot Raspberry Pi", 
            "Are you sure you want to reboot the Raspberry Pi?\n\nThis will restart the entire system and you will lose connection.",
            icon='warning'
        )
        
        if result:
            print("🔃 PI REBOOT INITIATED")
            self.mqtt_client.send_system_command('reboot_pi')
            if self.debug_mode:
                print("[APP] Pi reboot command sent")
    
    def _poweroff_pi(self):
        """Power off the Raspberry Pi - requires double confirmation"""
        if not self.mqtt_client.is_connected():
            messagebox.showwarning("Not Connected", "Not connected to robot")
            return
        
        # First confirmation dialog
        result1 = messagebox.askyesno(
            "Power Off Raspberry Pi", 
            "⚠️ WARNING: This will POWER OFF the Raspberry Pi!\n\nYou will need physical access to restart it.\n\nAre you sure you want to continue?",
            icon='warning'
        )
        
        if result1:
            # Second confirmation dialog for extra safety
            result2 = messagebox.askyesno(
                "Final Confirmation", 
                "🚨 FINAL CONFIRMATION 🚨\n\nThis action will completely shut down the robot.\nYou will need to physically restart it.\n\nProceed with power off?",
                icon='error'
            )
            
            if result2:
                print("⚡ PI POWER OFF INITIATED")
                self.mqtt_client.send_system_command('poweroff_pi')
                if self.debug_mode:
                    print("[APP] Pi power off command sent")
    
    def _reconnect(self):
        """Reconnect to MQTT broker"""
        self.mqtt_client.reconnect()
        if self.debug_mode:
            print("[APP] Reconnecting to MQTT broker")
    
    def _disconnect(self):
        """Disconnect from MQTT broker"""
        self.mqtt_client.graceful_disconnect()
        if self.debug_mode:
            print("[APP] Gracefully disconnected from MQTT broker")
    
    def _change_robot_ip(self, new_ip: str):
        """Change robot IP address"""
        if new_ip != self.config_manager.get_broker_host():
            self.config_manager.set_broker_host(new_ip)
            self.gui_manager.update_broker_host(new_ip)
            
            # Update MQTT client with new host
            self.mqtt_client.graceful_disconnect()
            self.mqtt_client.broker_host = new_ip
            self.mqtt_client.connect()
            
            if self.debug_mode:
                print(f"[APP] Robot IP changed to: {new_ip}")
    
    def _is_mqtt_connected(self) -> bool:
        """Check if MQTT client is connected"""
        return self.mqtt_client.is_connected()
    
    def _window_close_handler(self):
        """Handle window close event with timeout protection"""
        print("🪟 Window close requested...")
        
        # Send safety commands before shutdown
        self._send_safety_shutdown_commands()
        
        # Set up timeout protection for hanging cleanup
        import threading
        
        def force_exit():
            time.sleep(3)  # Give 3 seconds for normal cleanup (increased for safety commands)
            print("⏰ Cleanup timeout - forcing exit...")
            import os
            os._exit(0)
        
        # Start timeout thread
        timeout_thread = threading.Thread(target=force_exit, daemon=True)
        timeout_thread.start()
        
        # Perform cleanup
        self.cleanup()

    def _send_safety_shutdown_commands(self):
        """Send safety commands before shutdown to prevent robot corruption"""
        if not hasattr(self, 'mqtt_client') or not self.mqtt_client.is_connected():
            return
        
        try:
            print("🛡️ Sending safety shutdown commands...")
            
            # Send movement stop
            self.mqtt_client.send_movement_command(0, 0)
            
            # Send emergency stop for extra safety
            self.mqtt_client.send_system_command('emergency_stop')
            
            # Brief pause to ensure commands are sent
            time.sleep(0.1)
            
            print("✅ Safety shutdown commands sent")
            
        except Exception as e:
            print(f"⚠️ Error sending safety commands: {e}")
    
    def run(self):
        """Run the application"""
        try:
            print("🚀 Starting Rider Robot PC Client...")
            
            # Connect to MQTT
            self.mqtt_client.connect()
            
            # Setup window close handler
            self.gui_manager.set_close_callback(self._window_close_handler)
            
            # Run GUI (blocking)
            self.gui_manager.run()
            
        except KeyboardInterrupt:
            print("\n⌨️ Keyboard interrupt in main...")
        except Exception as e:
            print(f"❌ Application error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Cleanup resources"""
        # Prevent multiple cleanup calls
        if self.cleanup_done:
            if self.debug_mode:
                print("🛑 Cleanup already done, skipping...")
            return
        
        self.cleanup_done = True
        print("🛑 Shutting down application...")
        
        try:
            # Send safety commands first
            self._send_safety_shutdown_commands()
            
            # Stop GUI
            if hasattr(self, 'gui_manager'):
                self.gui_manager.stop()
            
            # Disconnect MQTT (graceful disconnect includes additional safety commands)
            if hasattr(self, 'mqtt_client'):
                self.mqtt_client.graceful_disconnect()
            
            # Brief pause for cleanup
            time.sleep(0.1)
            
            print("✅ Application cleanup complete")
            
        except Exception as e:
            print(f"⚠️ Cleanup error (ignored): {e}")
        
        print("👋 Application terminated") 