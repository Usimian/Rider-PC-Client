#!/usr/bin/env python3
# coding=utf-8

import signal
import time
import sys
from typing import Dict, Any, List
from tkinter import messagebox

from .config_manager import ConfigManager
from .robot_state import RobotState
from .llm_manager import LLMManager
from communication.mqtt_client import MQTTClient
from communication.message_handlers import MessageHandlers
from ui.gui_manager import GUIManager

class ApplicationController:
    def __init__(self, debug: bool = False):
        self.debug_mode = debug
        self.cleanup_done = False  # Flag to prevent multiple cleanup calls
        self._shutting_down = False  # Flag to prevent multiple signal handling
        
        # Initialize core components
        self.config_manager = ConfigManager()
        self.robot_state = RobotState()
        
        # Initialize LLM (if enabled)
        self.llm_manager = None
        if self.config_manager.is_llm_enabled():
            try:
                self.llm_manager = LLMManager(
                    self.config_manager.get_ollama_url(),
                    debug
                )
                # Set up LLM callbacks
                self.llm_manager.set_response_callback(self._handle_llm_response_chunk)
                self.llm_manager.set_status_callback(self._handle_llm_status)
                if debug:
                    print("✅ LLM Manager initialized successfully")
            except Exception as e:
                if debug:
                    print(f"⚠️ LLM Manager initialization failed: {e}")
                self.llm_manager = None
        
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
        
        # Setup image capture callback
        self.message_handlers.set_image_capture_callback(self._handle_image_capture_response)
        
        # Setup signal handlers for graceful shutdown
        self._setup_signal_handlers()
        
        if debug:
            print("🔧 Application controller initialized")
    
    def _get_gui_callbacks(self) -> Dict[str, Any]:
        """Get callbacks for GUI interactions"""
        callbacks = {
            'change_speed': self._change_speed,
            'change_height': self._change_height,
            'change_body_tilt': self._change_body_tilt,
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
            'is_mqtt_connected': self._is_mqtt_connected,
            'request_image_capture': self._request_image_capture
        }
        
        # Add LLM callbacks if available
        if self.llm_manager:
            callbacks.update({
                'llm_send_image': self._llm_send_image,
                'llm_generate_response': self._llm_generate_response,
                'llm_set_model': self._llm_set_model,
                'llm_clear_conversation': self._llm_clear_conversation,
                'llm_get_models': self._llm_get_models,
                'llm_get_status': self._llm_get_status,
                'llm_test_connection': self._llm_test_connection,
                'llm_apply_settings': self._llm_apply_settings,
                'llm_save_settings': self._llm_save_settings
            })
        
        return callbacks
    
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
            # Prevent multiple signal handling
            if hasattr(self, '_shutting_down') and self._shutting_down:
                print("🔄 Already shutting down, ignoring signal...")
                return
            
            self._shutting_down = True
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
            
            # Stop GUI first - this will exit the tkinter mainloop
            if hasattr(self, 'gui_manager'):
                self.gui_manager.stop()
            
            # Stop MQTT with timeout
            if hasattr(self, 'mqtt_client'):
                self.mqtt_client.disconnect()  # Use fast disconnect instead of graceful
            
            print("✅ Force shutdown complete")
        except Exception as e:
            print(f"⚠️ Shutdown error (ignored): {e}")
        finally:
            # Import os for force exit
            import os
            os._exit(0)  # Force exit without cleanup
    
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

    def _change_height(self, value: int):
        """Change robot height setting"""
        if not self.mqtt_client.is_connected():
            return

        self.mqtt_client.send_settings_command('change_height', value)
        if self.debug_mode:
            print(f"[APP] Height changed to: {value}")

    def _change_body_tilt(self, value: int):
        """Change robot body tilt (left/right)"""
        if not self.mqtt_client.is_connected():
            return

        self.mqtt_client.send_settings_command('change_body_tilt', value)
        if self.debug_mode:
            print(f"[APP] Body tilt changed to: {value}°")

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
    
    def _request_image_capture(self, resolution: str = "high"):
        """Request image capture from robot"""
        if not self.mqtt_client.is_connected():
            messagebox.showwarning("Not Connected", "Not connected to robot")
            return
        
        request_id = self.mqtt_client.send_image_capture_request(resolution)
        if request_id:
            # Store the request ID for tracking if needed
            if self.debug_mode:
                print(f"[APP] Image capture requested: {request_id} ({resolution})")
        else:
            messagebox.showerror("Image Capture Error", "Failed to send image capture request")
    
    def _handle_image_capture_response(self, data: Dict[str, Any]):
        """Handle image capture response from robot"""
        try:
            success = data.get('success', False)
            request_id = data.get('request_id', 'unknown')
            
            if success:
                image_data = data.get('image_data')
                if image_data:
                    # Update GUI with successful image
                    self.gui_manager.update_image_display(image_data, success=True)
                    if self.debug_mode:
                        image_size = data.get('image_size', 'unknown size')
                        resolution = data.get('resolution', 'unknown')
                        print(f"[APP] Image received: {request_id} ({image_size}, {resolution})")
                else:
                    # Success but no image data
                    error_msg = "No image data received"
                    self.gui_manager.update_image_display(None, success=False, error_message=error_msg)
                    if self.debug_mode:
                        print(f"[APP] Image capture successful but no data: {request_id}")
            else:
                # Failed capture
                error_msg = data.get('error', 'Image capture failed')
                self.gui_manager.update_image_display(None, success=False, error_message=error_msg)
                if self.debug_mode:
                    print(f"[APP] Image capture failed: {request_id} - {error_msg}")
                    
        except Exception as e:
            error_msg = f"Error processing image response: {e}"
            self.gui_manager.update_image_display(None, success=False, error_message=error_msg)
            if self.debug_mode:
                print(f"[APP] Image response processing error: {e}")
    
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
        """Handle window close event - immediate and aggressive shutdown"""
        print("🪟 Window close requested - forcing immediate termination...")
        
        # Prevent multiple close events
        if hasattr(self, '_closing') and self._closing:
            return
        self._closing = True
        
        # Set up backup force kill after 1 second
        import threading
        import time
        import signal
        def backup_force_kill():
            time.sleep(1.0)  # Give 1 second max
            print("⏰ Backup force kill activated...")
            # Try multiple ways to kill the process
            try:
                import os
                os.kill(os.getpid(), signal.SIGKILL)  # Force kill signal
            except:
                os._exit(1)  # Fallback to _exit
        
        backup_thread = threading.Thread(target=backup_force_kill, daemon=True)
        backup_thread.start()
        
        # Start immediate shutdown in a separate thread to avoid blocking
        def immediate_shutdown():
            try:
                # Try to send safety commands quickly
                self._send_safety_shutdown_commands()
            except:
                pass  # Don't let safety commands block shutdown
            
            print("🚪 Forcing immediate process termination...")
            import os
            os._exit(0)  # Force immediate termination
        
        # Start shutdown thread and return immediately
        shutdown_thread = threading.Thread(target=immediate_shutdown, daemon=True)
        shutdown_thread.start()
        
        # Also try to destroy the window immediately in current thread
        try:
            self.gui_manager.main_window.root.destroy()
        except:
            pass
    
    def _send_safety_shutdown_commands(self):
        """Send safety commands before shutdown - ultra-fast version"""
        if not hasattr(self, 'mqtt_client') or not self.mqtt_client.is_connected():
            return
        
        try:
            print("🛡️ Sending safety shutdown commands...")
            
            # Send both commands without any error handling to maximize speed
            try:
                self.mqtt_client.send_movement_command(0, 0)
                self.mqtt_client.send_system_command('emergency_stop')
            except:
                pass  # Ignore all errors - speed is critical
            
            print("✅ Safety shutdown commands sent")
            
        except Exception:
            pass  # Ignore all errors to prevent blocking
    
    def run(self):
        """Run the application"""
        try:
            print("🚀 Starting Rider Robot PC Client...")
            
            # Connect to MQTT
            self.mqtt_client.connect()
            
            # Initialize LLM GUI integration
            self._initialize_llm_gui()
            
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
        """Cleanup resources - immediate shutdown version"""
        # Prevent multiple cleanup calls
        if self.cleanup_done:
            if self.debug_mode:
                print("🛑 Cleanup already done, skipping...")
            return
        
        self.cleanup_done = True
        print("🛑 Shutting down application...")
        
        try:
            # Send safety commands first (with minimal delay)
            try:
                self._send_safety_shutdown_commands()
            except Exception as e:
                print(f"⚠️ Safety shutdown commands failed: {e}")
            
            # Cleanup LLM manager
            try:
                if hasattr(self, 'llm_manager') and self.llm_manager:
                    self.llm_manager.cleanup()
            except Exception as e:
                print(f"⚠️ LLM cleanup failed: {e}")
            
            # Stop GUI immediately
            try:
                if hasattr(self, 'gui_manager'):
                    self.gui_manager.stop()
            except Exception as e:
                print(f"⚠️ GUI stop failed: {e}")
            
            # Force disconnect MQTT immediately
            try:
                if hasattr(self, 'mqtt_client'):
                    self.mqtt_client.disconnect()  # Use force disconnect
            except Exception as e:
                print(f"⚠️ MQTT disconnect failed: {e}")
            
            print("✅ Application cleanup complete")
            
        except Exception as e:
            print(f"⚠️ Cleanup error (ignored): {e}")
        
        print("👋 Application terminated")
    
    # LLM Callback Methods
    def _llm_send_image(self, image_data: str):
        """Send image to LLM for analysis"""
        if not self.llm_manager:
            return
        
        self.llm_manager.set_current_image(image_data)
        if self.debug_mode:
            print("🖼️ Image sent to LLM manager")
    
    def _llm_generate_response(self, prompt: str, use_current_image: bool = True):
        """Generate LLM response"""
        if not self.llm_manager:
            if hasattr(self, 'gui_manager'):
                self.gui_manager.add_llm_error("LLM not available")
            return {"success": False, "error": "LLM not available"}
        
        # Set generating status in GUI
        if hasattr(self, 'gui_manager'):
            self.gui_manager.set_llm_generating(True)
        
        # Set up response handling callback
        accumulated_response = ""
        def handle_response():
            nonlocal accumulated_response
            # Wait a bit for the response to complete, then check for results
            import threading
            import time
            
            def check_response():
                nonlocal accumulated_response
                max_wait_time = 30  # Wait up to 30 seconds for response
                check_interval = 2  # Check every 2 seconds
                
                for i in range(max_wait_time // check_interval):
                    time.sleep(check_interval)
                    
                    # Get the latest conversation history
                    history = self.llm_manager.get_conversation_history()
                    
                    # Find the latest assistant response
                    if history and len(history) >= 2:
                        latest_response = history[-1]
                        if latest_response.role == "assistant":
                            accumulated_response = latest_response.content
                            
                            # Update GUI with complete response
                            if hasattr(self, 'gui_manager'):
                                self.gui_manager.add_llm_response(accumulated_response)
                                self.gui_manager.set_llm_generating(False)
                            
                            if self.debug_mode:
                                print(f"🤖 LLM response complete: {len(accumulated_response)} chars")
                            return  # Success - exit early
                    
                    # Check if LLM is still generating
                    if not self.llm_manager.is_busy():
                        # LLM finished but no response found - error
                        if hasattr(self, 'gui_manager'):
                            self.gui_manager.add_llm_error("No response received")
                            self.gui_manager.set_llm_generating(False)
                        return
                
                # Timeout - no response after max wait time
                if hasattr(self, 'gui_manager'):
                    self.gui_manager.add_llm_error("Response generation timed out")
                    self.gui_manager.set_llm_generating(False)
            
            # Start response checking in background
            threading.Thread(target=check_response, daemon=True).start()
        
        # Generate response
        result = self.llm_manager.generate_response(prompt, use_current_image, use_streaming=False)
        
        if result.get("success"):
            # Start response handling
            handle_response()
        else:
            # Handle immediate error
            error_msg = result.get("error", "Unknown error")
            if hasattr(self, 'gui_manager'):
                self.gui_manager.add_llm_error(error_msg)
                self.gui_manager.set_llm_generating(False)
        
        return result
    
    def _llm_set_model(self, model_name: str) -> bool:
        """Set LLM model"""
        if not self.llm_manager:
            return False
        
        return self.llm_manager.set_model(model_name)
    
    def _llm_clear_conversation(self):
        """Clear LLM conversation history"""
        if not self.llm_manager:
            return
        
        self.llm_manager.clear_conversation()
    
    def _llm_get_models(self) -> List[str]:
        """Get available LLM models"""
        if not self.llm_manager:
            return []
        
        return self.llm_manager.get_available_models()
    
    def _llm_get_status(self) -> Dict[str, Any]:
        """Get LLM status"""
        if not self.llm_manager:
            return {"available": False, "error": "LLM not initialized"}
        
        return {
            "available": self.llm_manager.is_server_available(),
            "busy": self.llm_manager.is_busy(),
            "model": self.llm_manager.get_current_model(),
            "settings": self.llm_manager.get_settings()
        }
    
    def _handle_llm_response_chunk(self, chunk: str):
        """Handle streaming LLM response chunk"""
        # Forward to GUI if available
        if hasattr(self, 'gui_manager'):
            self.gui_manager.handle_llm_response_chunk(chunk)
    
    def _handle_llm_status(self, status: str):
        """Handle LLM status updates"""
        if self.debug_mode:
            print(f"🤖 LLM Status: {status}")
        
        # Forward to GUI if available
        if hasattr(self, 'gui_manager'):
            self.gui_manager.handle_llm_status(status)
    
    def _llm_test_connection(self, url: str) -> bool:
        """Test connection to LLM server"""
        if not self.llm_manager:
            return False
        
        try:
            # Create temporary client to test connection
            from communication.ollama_client import OllamaClient
            test_client = OllamaClient(url, self.debug_mode)
            
            result = test_client.is_server_available()
            test_client.close()
            
            return result
            
        except Exception as e:
            if self.debug_mode:
                print(f"❌ Connection test error: {e}")
            return False
    
    def _llm_apply_settings(self, settings: Dict[str, Any]):
        """Apply LLM settings for current session"""
        if not self.llm_manager:
            return
        
        try:
            # Apply settings to LLM manager
            if 'model' in settings:
                self.llm_manager.set_model(settings['model'])
            
            if 'temperature' in settings:
                self.llm_manager.set_temperature(settings['temperature'])
            
            if 'max_tokens' in settings:
                self.llm_manager.set_max_tokens(settings['max_tokens'])
            
            if self.debug_mode:
                print(f"✅ LLM settings applied: {settings}")
                
        except Exception as e:
            if self.debug_mode:
                print(f"❌ Error applying LLM settings: {e}")
    
    def _llm_save_settings(self, settings: Dict[str, Any]) -> bool:
        """Save LLM settings to configuration file"""
        try:
            # Save to config manager
            if 'ollama_url' in settings:
                self.config_manager.set_ollama_url(settings['ollama_url'])
            
            if 'model' in settings:
                self.config_manager.set_llm_default_model(settings['model'])
            
            if 'temperature' in settings:
                self.config_manager.set_llm_temperature(settings['temperature'])
            
            if 'max_tokens' in settings:
                self.config_manager.set_llm_max_tokens(settings['max_tokens'])
            
            if 'enabled' in settings:
                self.config_manager.set_llm_enabled(settings['enabled'])
            
            # Apply current session settings too
            self._llm_apply_settings(settings)
            
            if self.debug_mode:
                print(f"💾 LLM settings saved: {settings}")
            
            return True
            
        except Exception as e:
            if self.debug_mode:
                print(f"❌ Error saving LLM settings: {e}")
            return False
    
    def _initialize_llm_gui(self):
        """Initialize LLM GUI components after GUI manager is created"""
        if not self.llm_manager or not hasattr(self, 'gui_manager'):
            return
        
        try:
            # Update GUI with initial LLM status
            status = self._llm_get_status()
            if status.get("available"):
                models = self._llm_get_models()
                self.gui_manager.update_llm_models(models)
                self.gui_manager.handle_llm_status("Connected")
            else:
                self.gui_manager.handle_llm_status("Disconnected")
            
            if self.debug_mode:
                print("✅ LLM GUI integration initialized")
                
        except Exception as e:
            if self.debug_mode:
                print(f"⚠️ LLM GUI initialization error: {e}") 