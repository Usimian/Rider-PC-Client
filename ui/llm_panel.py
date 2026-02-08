#!/usr/bin/env python3
# coding=utf-8

import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox
from typing import Dict, Callable, List, Optional, Any
import time
import json
import re
from datetime import datetime

class LLMPanel:
    def __init__(self, parent, llm_callbacks: Dict[str, Callable], debug: bool = False):
        self.parent = parent
        self.llm_callbacks = llm_callbacks
        self.debug_mode = debug
        
        # State
        self.current_model = "qwen3-vl:8b"
        self.available_models = []
        self.server_status = "Checking..."
        self.is_generating = False
        
        self.setup_panel()
        
        if debug:
            print("🤖 LLM Panel initialized")
    
    def setup_panel(self):
        """Setup the LLM chat panel"""
        self.panel = tk.LabelFrame(self.parent, text="🧠 AI ASSISTANT", 
                                  font=('Arial', 12, 'bold'), bg='#3c3c3c', fg='#87ceeb',
                                  relief='solid', bd=1)
        
        # Create main container with scrollable chat area and controls
        self._create_header()
        self._create_chat_area()
        self._create_input_area()
        self._create_quick_actions()
        
    def _create_header(self):
        """Create header with model selection and status"""
        header_frame = tk.Frame(self.panel, bg='#3c3c3c')
        header_frame.pack(fill='x', padx=10, pady=(10, 5))
        
        # Model selection
        model_frame = tk.Frame(header_frame, bg='#3c3c3c')
        model_frame.pack(side='left', fill='x', expand=True)
        
        tk.Label(model_frame, text="Model:", font=('Arial', 9), 
                bg='#3c3c3c', fg='white').pack(side='left', padx=(0, 5))
        
        self.model_var = tk.StringVar(value=self.current_model)
        self.model_combo = ttk.Combobox(model_frame, textvariable=self.model_var, 
                                       state="readonly", width=15, font=('Arial', 9))
        self.model_combo.pack(side='left', padx=(0, 10))
        self.model_combo.bind('<<ComboboxSelected>>', self._on_model_change)
        
        # Status indicator
        status_frame = tk.Frame(header_frame, bg='#3c3c3c')
        status_frame.pack(side='right')
        
        self.status_label = tk.Label(status_frame, text="⚡ Checking server...", 
                                    font=('Arial', 9), bg='#3c3c3c', fg='#ffd700')
        self.status_label.pack(side='right')
        
        # Settings button
        settings_btn = tk.Button(status_frame, text="⚙️", font=('Arial', 10), 
                                bg='#555555', fg='white', width=3,
                                activebackground='#666666',
                                command=self._show_settings)
        settings_btn.pack(side='right', padx=(0, 10))
    
    def _create_chat_area(self):
        """Create scrollable chat display area"""
        chat_frame = tk.Frame(self.panel, bg='#2b2b2b', relief='sunken', bd=1)
        chat_frame.pack(fill='both', expand=True, padx=10, pady=5)
        
        # Chat history display
        self.chat_display = scrolledtext.ScrolledText(
            chat_frame, 
            height=12,
            bg='#2b2b2b', 
            fg='white',
            font=('Consolas', 10),
            wrap=tk.WORD,
            state=tk.DISABLED,
            insertbackground='white'
        )
        self.chat_display.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Configure text tags for different message types
        self.chat_display.tag_configure("user", foreground="#87ceeb", font=('Consolas', 10, 'bold'))
        self.chat_display.tag_configure("assistant", foreground="#90ee90", font=('Consolas', 10))
        self.chat_display.tag_configure("system", foreground="#ffd700", font=('Consolas', 9, 'italic'))
        self.chat_display.tag_configure("timestamp", foreground="#888888", font=('Consolas', 8))
        
        # Add welcome message
        self._add_system_message("AI Assistant ready. Capture an image and ask questions!")
    
    def _create_input_area(self):
        """Create text input and send controls"""
        input_frame = tk.Frame(self.panel, bg='#3c3c3c')
        input_frame.pack(fill='x', padx=10, pady=(0, 5))
        
        # Text input
        self.input_text = tk.Text(input_frame, height=2, bg='#4a4a4a', fg='white',
                                 font=('Arial', 10), wrap=tk.WORD,
                                 insertbackground='white')
        self.input_text.pack(fill='x', pady=(0, 5))
        self.input_text.bind('<Return>', self._on_enter_key)
        self.input_text.bind('<Shift-Return>', self._on_shift_enter)
        
        # Control buttons
        button_frame = tk.Frame(input_frame, bg='#3c3c3c')
        button_frame.pack(fill='x')
        
        # Send button
        self.send_btn = tk.Button(button_frame, text="🚀 Send", 
                                 font=('Arial', 10, 'bold'), bg='#4CAF50', fg='white',
                                 activebackground='#45a049', width=10,
                                 command=self._send_message)
        self.send_btn.pack(side='right', padx=(5, 0))
        
        # Clear chat button
        clear_btn = tk.Button(button_frame, text="🧹 Clear", 
                             font=('Arial', 10), bg='#ff6b6b', fg='white',
                             activebackground='#ff5252', width=8,
                             command=self._clear_chat)
        clear_btn.pack(side='right', padx=(5, 0))
        
        # Image status
        self.image_status = tk.Label(button_frame, text="📷 No image", 
                                    font=('Arial', 9), bg='#3c3c3c', fg='#888888')
        self.image_status.pack(side='left')
    
    def _create_quick_actions(self):
        """Create quick action buttons for common robot queries"""
        actions_frame = tk.LabelFrame(self.panel, text="Quick Actions", 
                                     font=('Arial', 10, 'bold'), bg='#3c3c3c', fg='#87ceeb')
        actions_frame.pack(fill='x', padx=10, pady=(0, 10))
        
        # Create two rows of buttons
        row1 = tk.Frame(actions_frame, bg='#3c3c3c')
        row1.pack(fill='x', pady=5)
        
        row2 = tk.Frame(actions_frame, bg='#3c3c3c')
        row2.pack(fill='x', pady=(0, 5))
        
        # Row 1 buttons
        self._create_action_button(row1, "🔍 What do you see?", 
                                  lambda: self._quick_action("analyze"))
        self._create_action_button(row1, "🧭 Navigation help", 
                                  lambda: self._quick_action("navigation"))
        self._create_action_button(row1, "🌍 Describe environment", 
                                  lambda: self._quick_action("environment"))
        
        # Row 2 buttons
        self._create_action_button(row2, "⚠️ Find obstacles", 
                                  lambda: self._quick_action("obstacles"))
        self._create_action_button(row2, "📏 Estimate distances", 
                                  lambda: self._quick_action("distances"))
        self._create_action_button(row2, "💡 Lighting analysis", 
                                  lambda: self._quick_action("lighting"))
    
    def _create_action_button(self, parent, text, command):
        """Create a quick action button"""
        btn = tk.Button(parent, text=text, font=('Arial', 9), 
                       bg='#555555', fg='white', activebackground='#666666',
                       command=command, width=18)
        btn.pack(side='left', padx=2, fill='x', expand=True)
    
    def _on_model_change(self, event=None):
        """Handle model selection change"""
        new_model = self.model_var.get()
        if new_model != self.current_model:
            self.current_model = new_model
            if self.llm_callbacks.get('set_model'):
                success = self.llm_callbacks['set_model'](new_model)
                if success:
                    self._add_system_message(f"Switched to model: {new_model}")
                    if self.debug_mode:
                        print(f"🤖 Model changed to: {new_model}")
                else:
                    self._add_system_message(f"Failed to switch to model: {new_model}")
                    # Revert combo box selection
                    self.model_var.set(self.current_model)
    
    def _on_enter_key(self, event):
        """Handle Enter key in text input"""
        if not event.state & 0x1:  # No Shift key
            self._send_message()
            return 'break'  # Prevent default newline
        return None  # Allow Shift+Enter for newline
    
    def _on_shift_enter(self, event):
        """Handle Shift+Enter for newline"""
        return None  # Allow default behavior
    
    def _send_message(self):
        """Send user message to LLM"""
        message = self.input_text.get("1.0", tk.END).strip()
        if not message:
            return
        
        if self.is_generating:
            messagebox.showwarning("Busy", "AI is currently generating a response. Please wait.")
            return
        
        # Clear input
        self.input_text.delete("1.0", tk.END)
        
        # Add user message to chat
        self._add_user_message(message)
        
        # Send to LLM
        if self.llm_callbacks.get('generate_response'):
            self.llm_callbacks['generate_response'](message, use_current_image=True)
        else:
            self._add_system_message("Error: LLM callback not available")
    
    def _quick_action(self, action_type: str):
        """Handle quick action button press"""
        if self.is_generating:
            messagebox.showwarning("Busy", "AI is currently generating a response. Please wait.")
            return
        
        prompts = {
            "analyze": "What do you see in this image? Describe the scene, objects, and any notable details.",
            "navigation": "Analyze this image from a robot's perspective. Are there any obstacles, hazards, or navigation concerns? What would be safe directions to move?",
            "environment": "Provide a detailed description of this environment. What type of space is this? What objects and features do you see?",
            "obstacles": "Identify any obstacles, barriers, or hazards in this image that a robot should avoid.",
            "distances": "Estimate the distances to various objects in this image. What's close, what's far, and what's in between?",
            "lighting": "Analyze the lighting conditions in this image. Is it bright, dim, natural light, artificial light? Any shadows or glare?"
        }
        
        prompt = prompts.get(action_type, "Analyze this image.")
        
        # Add user message to show what was requested
        action_names = {
            "analyze": "🔍 Image Analysis",
            "navigation": "🧭 Navigation Analysis", 
            "environment": "🌍 Environment Description",
            "obstacles": "⚠️ Obstacle Detection",
            "distances": "📏 Distance Estimation",
            "lighting": "💡 Lighting Analysis"
        }
        
        self._add_user_message(f"{action_names.get(action_type, action_type)}")
        
        # Send to LLM
        if self.llm_callbacks.get('generate_response'):
            self.llm_callbacks['generate_response'](prompt, use_current_image=True)
        else:
            self._add_system_message("Error: LLM callback not available")
    
    def _clear_chat(self):
        """Clear chat history"""
        self.chat_display.config(state=tk.NORMAL)
        self.chat_display.delete("1.0", tk.END)
        self.chat_display.config(state=tk.DISABLED)
        
        # Clear LLM conversation history
        if self.llm_callbacks.get('clear_conversation'):
            self.llm_callbacks['clear_conversation']()
        
        self._add_system_message("Conversation cleared. Ready for new questions!")
    
    def _show_settings(self):
        """Show LLM settings dialog"""
        try:
            from .llm_settings_dialog import LLMSettingsDialog
            
            # Get current settings
            current_settings = {}
            if self.llm_callbacks.get('get_status'):
                status = self.llm_callbacks['get_status']()
                current_settings = status.get('settings', {})
            
            # Prepare settings callbacks
            settings_callbacks = {
                'get_models': self.llm_callbacks.get('get_models', lambda: []),
                'test_connection': self._test_connection_callback,
                'apply_settings': self._apply_settings_callback,
                'save_settings': self._save_settings_callback
            }
            
            # Show settings dialog
            dialog = LLMSettingsDialog(
                self.panel.winfo_toplevel(), 
                current_settings, 
                settings_callbacks, 
                self.debug_mode
            )
            
            result = dialog.show()
            if result:
                self._add_system_message("Settings updated successfully")
                
        except ImportError as e:
            messagebox.showerror("Import Error", f"Settings dialog not available: {e}")
        except Exception as e:
            if self.debug_mode:
                print(f"❌ Settings dialog error: {e}")
            messagebox.showerror("Error", f"Failed to open settings: {e}")
    
    def _test_connection_callback(self, url: str) -> bool:
        """Test connection callback for settings dialog"""
        # This would test connection to specified URL
        if self.llm_callbacks.get('test_connection'):
            return self.llm_callbacks['test_connection'](url)
        else:
            # Simple mock test for now
            return url.startswith('http')
    
    def _apply_settings_callback(self, settings: Dict[str, Any]):
        """Apply settings callback for settings dialog"""
        if self.llm_callbacks.get('apply_settings'):
            self.llm_callbacks['apply_settings'](settings)
        
        # Update local state
        if 'model' in settings:
            self.current_model = settings['model']
            self.model_var.set(self.current_model)
    
    def _save_settings_callback(self, settings: Dict[str, Any]) -> bool:
        """Save settings callback for settings dialog"""
        if self.llm_callbacks.get('save_settings'):
            return self.llm_callbacks['save_settings'](settings)
        return False
    
    def _add_user_message(self, message: str):
        """Add user message to chat display"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        self.chat_display.config(state=tk.NORMAL)
        self.chat_display.insert(tk.END, f"[{timestamp}] ", "timestamp")
        self.chat_display.insert(tk.END, "You: ", "user")
        self.chat_display.insert(tk.END, f"{message}\n\n")
        self.chat_display.config(state=tk.DISABLED)
        self.chat_display.see(tk.END)
    
    def _add_assistant_message(self, message: str):
        """Add assistant message to chat display"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        self.chat_display.config(state=tk.NORMAL)
        self.chat_display.insert(tk.END, f"[{timestamp}] ", "timestamp")
        self.chat_display.insert(tk.END, f"AI ({self.current_model}): ", "assistant")
        self.chat_display.insert(tk.END, f"{message}\n\n")
        self.chat_display.config(state=tk.DISABLED)
        self.chat_display.see(tk.END)
    
    def _add_system_message(self, message: str):
        """Add system message to chat display"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        self.chat_display.config(state=tk.NORMAL)
        self.chat_display.insert(tk.END, f"[{timestamp}] ", "timestamp")
        self.chat_display.insert(tk.END, f"System: {message}\n\n", "system")
        self.chat_display.config(state=tk.DISABLED)
        self.chat_display.see(tk.END)
    
    # Public interface methods
    def update_models(self, models: List[str]):
        """Update available models list"""
        self.available_models = models
        self.model_combo['values'] = models
        
        # Auto-select first vision model if current model not available
        if models and self.current_model not in models:
            # Prefer qwen3-vl models, then llava
            qwen_models = [m for m in models if 'qwen' in m.lower() and 'vl' in m.lower()]
            vision_models = [m for m in models if 'llava' in m.lower()]
            if qwen_models:
                qwen_8b = [m for m in qwen_models if '8b' in m.lower()]
                self.current_model = qwen_8b[0] if qwen_8b else qwen_models[0]
                self.model_var.set(self.current_model)
            elif vision_models:
                self.current_model = vision_models[0]
                self.model_var.set(self.current_model)
    
    def update_server_status(self, status: str, available: bool = True):
        """Update server status display"""
        self.server_status = status
        
        if available:
            self.status_label.config(text=f"✅ {status}", fg='#90ee90')
            self.send_btn.config(state='normal')
        else:
            self.status_label.config(text=f"❌ {status}", fg='#ff6b6b')
            self.send_btn.config(state='disabled')
    
    def update_image_status(self, has_image: bool, image_info: str = ""):
        """Update image availability status"""
        if has_image:
            self.image_status.config(text=f"📷 {image_info}", fg='#90ee90')
        else:
            self.image_status.config(text="📷 No image", fg='#888888')
    
    def set_generating_status(self, is_generating: bool):
        """Update generating status"""
        self.is_generating = is_generating
        
        if is_generating:
            self.send_btn.config(text="⏳ Generating...", state='disabled')
            self.status_label.config(text="🤖 Generating response...", fg='#ffd700')
        else:
            self.send_btn.config(text="🚀 Send", state='normal')
            self.update_server_status(self.server_status, True)
    
    def add_response_chunk(self, chunk: str):
        """Add streaming response chunk (for real-time display)"""
        if not hasattr(self, '_current_response_start'):
            # Start new streaming response
            self._start_streaming_response()
        
        # Add chunk to current response
        self.chat_display.config(state=tk.NORMAL)
        self.chat_display.insert(tk.END, chunk)
        self.chat_display.config(state=tk.DISABLED)
        self.chat_display.see(tk.END)
    
    def _start_streaming_response(self):
        """Start a new streaming response"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        self.chat_display.config(state=tk.NORMAL)
        self.chat_display.insert(tk.END, f"[{timestamp}] ", "timestamp")
        self.chat_display.insert(tk.END, f"AI ({self.current_model}): ", "assistant")
        self.chat_display.config(state=tk.DISABLED)
        
        # Mark streaming start position
        self._current_response_start = self.chat_display.index(tk.END)
    
    def finish_streaming_response(self):
        """Finish the current streaming response"""
        if hasattr(self, '_current_response_start'):
            # Add final newlines
            self.chat_display.config(state=tk.NORMAL)
            self.chat_display.insert(tk.END, "\n\n")
            self.chat_display.config(state=tk.DISABLED)
            self.chat_display.see(tk.END)
            
            # Clear streaming state
            delattr(self, '_current_response_start')
    
    def cancel_streaming_response(self):
        """Cancel current streaming response"""
        if hasattr(self, '_current_response_start'):
            # Add cancellation message
            self.chat_display.config(state=tk.NORMAL)
            self.chat_display.insert(tk.END, " [Response cancelled]\n\n", "system")
            self.chat_display.config(state=tk.DISABLED)
            
            # Clear streaming state
            delattr(self, '_current_response_start')
    
    def add_complete_response(self, response: str):
        """Add complete assistant response and execute any robot commands"""
        self._add_assistant_message(response)
        # Parse and execute any JSON robot commands in the response
        self._parse_and_execute_commands(response)

    def _parse_and_execute_commands(self, response: str):
        """Parse JSON robot commands from LLM response and execute them"""
        # Look for JSON code blocks with robot commands
        json_pattern = r'```json\s*(\{[^`]+\})\s*```'
        matches = re.findall(json_pattern, response, re.DOTALL)

        for match in matches:
            try:
                command = json.loads(match)
                self._execute_robot_command(command)
            except json.JSONDecodeError as e:
                if self.debug_mode:
                    print(f"[LLM] Failed to parse JSON command: {e}")

    def _execute_robot_command(self, command: Dict[str, Any]):
        """Execute a single robot command"""
        action = command.get('action', '').lower()

        print(f"[LLM] Executing robot command: {command}")

        if action == 'move':
            distance = command.get('distance', 0)
            robot_move = self.llm_callbacks.get('robot_move')
            print(f"[LLM] robot_move callback: {robot_move}")
            if robot_move:
                print(f"[LLM] Calling robot_move({distance}, 0)")
                # Send move command with distance (robot expects distance in payload)
                robot_move(distance, 0)  # Second param unused but kept for compatibility
                direction = "forward" if distance > 0 else "backward"
                self._add_system_message(f"🤖 Executing: Move {direction} {abs(distance)}mm")
                if self.debug_mode:
                    print(f"[LLM] Executing move command: distance={distance}mm")
            else:
                print(f"[LLM] ERROR: robot_move callback not found!")
                print(f"[LLM] Available callbacks: {list(self.llm_callbacks.keys())}")

        elif action == 'turn':
            angle = command.get('angle', 0)
            robot_turn = self.llm_callbacks.get('robot_turn')
            print(f"[LLM] robot_turn callback: {robot_turn}")
            if robot_turn:
                print(f"[LLM] Calling robot_turn({angle})")
                robot_turn(angle)
                direction = "right" if angle > 0 else "left"
                self._add_system_message(f"🤖 Executing: Turn {direction} {abs(angle)}°")
                if self.debug_mode:
                    print(f"[LLM] Executing turn command: angle={angle}°")
            else:
                print(f"[LLM] ERROR: robot_turn callback not found!")
                print(f"[LLM] Available callbacks: {list(self.llm_callbacks.keys())}")

        elif action == 'stop':
            robot_stop = self.llm_callbacks.get('robot_stop')
            if robot_stop:
                robot_stop()
                self._add_system_message("🤖 Executing: STOP")
                if self.debug_mode:
                    print("[LLM] Executing stop command")

        else:
            if self.debug_mode:
                print(f"[LLM] Unknown robot command action: {action}")

    def add_error_message(self, error: str):
        """Add error message"""
        self._add_system_message(f"Error: {error}")
    
    def get_widget(self):
        """Return the main widget for packing"""
        return self.panel 