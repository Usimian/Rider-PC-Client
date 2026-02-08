#!/usr/bin/env python3
# coding=utf-8

import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
from typing import Dict, Callable, List, Optional, Any

class LLMSettingsDialog:
    def __init__(self, parent, current_settings: Dict[str, Any], callbacks: Dict[str, Callable], debug: bool = False):
        self.parent = parent
        self.current_settings = current_settings
        self.callbacks = callbacks
        self.debug_mode = debug
        
        # Dialog state
        self.dialog = None
        self.result = None
        self.temp_settings = current_settings.copy()
        
        if debug:
            print("⚙️ LLM Settings Dialog initialized")
    
    def show(self) -> Optional[Dict[str, Any]]:
        """Show the settings dialog and return updated settings"""
        self.result = None
        self._create_dialog()
        
        # Center the dialog
        self._center_dialog()
        
        # Make dialog modal
        self.dialog.transient(self.parent)
        self.dialog.grab_set()
        
        # Wait for dialog to close
        self.parent.wait_window(self.dialog)
        
        return self.result
    
    def _create_dialog(self):
        """Create the settings dialog window"""
        self.dialog = tk.Toplevel(self.parent)
        self.dialog.title("🤖 LLM Settings")
        self.dialog.geometry("500x600")
        self.dialog.configure(bg='#2b2b2b')
        self.dialog.resizable(False, False)
        
        # Create main container
        main_frame = tk.Frame(self.dialog, bg='#2b2b2b')
        main_frame.pack(fill='both', expand=True, padx=20, pady=20)
        
        # Create sections
        self._create_server_section(main_frame)
        self._create_model_section(main_frame)
        self._create_generation_section(main_frame)
        self._create_advanced_section(main_frame)
        self._create_button_section(main_frame)
        
        # Load current values
        self._load_current_settings()
    
    def _create_server_section(self, parent):
        """Create server configuration section"""
        server_frame = tk.LabelFrame(parent, text="🔗 Server Configuration", 
                                    font=('Arial', 11, 'bold'), bg='#3c3c3c', fg='#87ceeb',
                                    relief='solid', bd=1)
        server_frame.pack(fill='x', pady=(0, 15))
        
        # Ollama URL
        url_frame = tk.Frame(server_frame, bg='#3c3c3c')
        url_frame.pack(fill='x', padx=15, pady=10)
        
        tk.Label(url_frame, text="Ollama Server URL:", font=('Arial', 10), 
                bg='#3c3c3c', fg='white').pack(anchor='w')
        
        self.url_var = tk.StringVar()
        url_entry = tk.Entry(url_frame, textvariable=self.url_var, font=('Arial', 10),
                            bg='#4a4a4a', fg='white', insertbackground='white', width=40)
        url_entry.pack(fill='x', pady=(5, 0))
        
        # Test connection button
        test_frame = tk.Frame(server_frame, bg='#3c3c3c')
        test_frame.pack(fill='x', padx=15, pady=(0, 10))
        
        self.test_btn = tk.Button(test_frame, text="🔍 Test Connection", 
                                 font=('Arial', 9), bg='#4CAF50', fg='white',
                                 activebackground='#45a049', command=self._test_connection)
        self.test_btn.pack(side='left')
        
        self.connection_status = tk.Label(test_frame, text="⚪ Not tested", 
                                         font=('Arial', 9), bg='#3c3c3c', fg='#888888')
        self.connection_status.pack(side='right')
    
    def _create_model_section(self, parent):
        """Create model selection section"""
        model_frame = tk.LabelFrame(parent, text="🤖 Model Configuration", 
                                   font=('Arial', 11, 'bold'), bg='#3c3c3c', fg='#87ceeb',
                                   relief='solid', bd=1)
        model_frame.pack(fill='x', pady=(0, 15))
        
        # Available models
        available_frame = tk.Frame(model_frame, bg='#3c3c3c')
        available_frame.pack(fill='x', padx=15, pady=10)
        
        tk.Label(available_frame, text="Available Models:", font=('Arial', 10), 
                bg='#3c3c3c', fg='white').pack(anchor='w')
        
        # Models listbox with scrollbar
        listbox_frame = tk.Frame(available_frame, bg='#3c3c3c')
        listbox_frame.pack(fill='x', pady=(5, 0))
        
        scrollbar = tk.Scrollbar(listbox_frame)
        scrollbar.pack(side='right', fill='y')
        
        self.models_listbox = tk.Listbox(listbox_frame, height=6, font=('Arial', 9),
                                        bg='#4a4a4a', fg='white', selectbackground='#87ceeb',
                                        yscrollcommand=scrollbar.set)
        self.models_listbox.pack(side='left', fill='both', expand=True)
        scrollbar.config(command=self.models_listbox.yview)
        
        # Refresh models button
        refresh_btn = tk.Button(available_frame, text="🔄 Refresh Models", 
                               font=('Arial', 9), bg='#555555', fg='white',
                               activebackground='#666666', command=self._refresh_models)
        refresh_btn.pack(pady=(10, 0))
        
        # Selected model
        selected_frame = tk.Frame(model_frame, bg='#3c3c3c')
        selected_frame.pack(fill='x', padx=15, pady=(0, 10))
        
        tk.Label(selected_frame, text="Selected Model:", font=('Arial', 10), 
                bg='#3c3c3c', fg='white').pack(anchor='w')
        
        self.selected_model_var = tk.StringVar()
        selected_entry = tk.Entry(selected_frame, textvariable=self.selected_model_var, 
                                 font=('Arial', 10), bg='#4a4a4a', fg='white',
                                 insertbackground='white', state='readonly')
        selected_entry.pack(fill='x', pady=(5, 0))
        
        # Bind model selection
        self.models_listbox.bind('<<ListboxSelect>>', self._on_model_select)
    
    def _create_generation_section(self, parent):
        """Create generation parameters section"""
        gen_frame = tk.LabelFrame(parent, text="⚡ Generation Parameters", 
                                 font=('Arial', 11, 'bold'), bg='#3c3c3c', fg='#87ceeb',
                                 relief='solid', bd=1)
        gen_frame.pack(fill='x', pady=(0, 15))
        
        # Temperature
        temp_frame = tk.Frame(gen_frame, bg='#3c3c3c')
        temp_frame.pack(fill='x', padx=15, pady=10)
        
        tk.Label(temp_frame, text="Temperature (Creativity):", font=('Arial', 10), 
                bg='#3c3c3c', fg='white').pack(anchor='w')
        
        temp_control_frame = tk.Frame(temp_frame, bg='#3c3c3c')
        temp_control_frame.pack(fill='x', pady=(5, 0))
        
        self.temp_var = tk.DoubleVar()
        temp_scale = tk.Scale(temp_control_frame, from_=0.0, to=2.0, resolution=0.1,
                             orient='horizontal', variable=self.temp_var, length=300,
                             bg='#4a4a4a', fg='white', highlightbackground='#3c3c3c',
                             troughcolor='#666666', activebackground='#87ceeb')
        temp_scale.pack(side='left', fill='x', expand=True)
        
        self.temp_label = tk.Label(temp_control_frame, text="0.7", font=('Arial', 10), 
                                  bg='#3c3c3c', fg='#87ceeb', width=5)
        self.temp_label.pack(side='right', padx=(10, 0))
        
        temp_scale.bind('<Motion>', self._update_temp_label)
        temp_scale.bind('<ButtonRelease-1>', self._update_temp_label)
        
        # Max tokens
        tokens_frame = tk.Frame(gen_frame, bg='#3c3c3c')
        tokens_frame.pack(fill='x', padx=15, pady=(0, 10))
        
        tk.Label(tokens_frame, text="Max Tokens (Response Length):", font=('Arial', 10), 
                bg='#3c3c3c', fg='white').pack(anchor='w')
        
        tokens_control_frame = tk.Frame(tokens_frame, bg='#3c3c3c')
        tokens_control_frame.pack(fill='x', pady=(5, 0))
        
        self.tokens_var = tk.IntVar()
        tokens_scale = tk.Scale(tokens_control_frame, from_=50, to=2000, resolution=50,
                               orient='horizontal', variable=self.tokens_var, length=300,
                               bg='#4a4a4a', fg='white', highlightbackground='#3c3c3c',
                               troughcolor='#666666', activebackground='#87ceeb')
        tokens_scale.pack(side='left', fill='x', expand=True)
        
        self.tokens_label = tk.Label(tokens_control_frame, text="500", font=('Arial', 10), 
                                    bg='#3c3c3c', fg='#87ceeb', width=5)
        self.tokens_label.pack(side='right', padx=(10, 0))
        
        tokens_scale.bind('<Motion>', self._update_tokens_label)
        tokens_scale.bind('<ButtonRelease-1>', self._update_tokens_label)
    
    def _create_advanced_section(self, parent):
        """Create advanced settings section"""
        adv_frame = tk.LabelFrame(parent, text="🔧 Advanced Settings", 
                                 font=('Arial', 11, 'bold'), bg='#3c3c3c', fg='#87ceeb',
                                 relief='solid', bd=1)
        adv_frame.pack(fill='x', pady=(0, 15))
        
        # Enable/Disable LLM
        enable_frame = tk.Frame(adv_frame, bg='#3c3c3c')
        enable_frame.pack(fill='x', padx=15, pady=10)
        
        self.enabled_var = tk.BooleanVar()
        enable_check = tk.Checkbutton(enable_frame, text="Enable LLM Features", 
                                     variable=self.enabled_var, font=('Arial', 10),
                                     bg='#3c3c3c', fg='white', selectcolor='#4a4a4a',
                                     activebackground='#3c3c3c', activeforeground='white')
        enable_check.pack(anchor='w')
        
        # Debug mode
        debug_frame = tk.Frame(adv_frame, bg='#3c3c3c')
        debug_frame.pack(fill='x', padx=15, pady=(0, 10))
        
        self.debug_var = tk.BooleanVar()
        debug_check = tk.Checkbutton(debug_frame, text="Enable Debug Logging", 
                                    variable=self.debug_var, font=('Arial', 10),
                                    bg='#3c3c3c', fg='white', selectcolor='#4a4a4a',
                                    activebackground='#3c3c3c', activeforeground='white')
        debug_check.pack(anchor='w')
    
    def _create_button_section(self, parent):
        """Create dialog buttons"""
        button_frame = tk.Frame(parent, bg='#2b2b2b')
        button_frame.pack(fill='x', pady=(20, 0))
        
        # Cancel button
        cancel_btn = tk.Button(button_frame, text="❌ Cancel", font=('Arial', 10),
                              bg='#666666', fg='white', activebackground='#777777',
                              width=12, command=self._cancel)
        cancel_btn.pack(side='left')
        
        # Reset button
        reset_btn = tk.Button(button_frame, text="🔄 Reset", font=('Arial', 10),
                             bg='#ff9800', fg='white', activebackground='#f57c00',
                             width=12, command=self._reset_to_defaults)
        reset_btn.pack(side='left', padx=(10, 0))
        
        # Apply button
        apply_btn = tk.Button(button_frame, text="✅ Apply", font=('Arial', 10),
                             bg='#4CAF50', fg='white', activebackground='#45a049',
                             width=12, command=self._apply_settings)
        apply_btn.pack(side='right')
        
        # Save button
        save_btn = tk.Button(button_frame, text="💾 Save", font=('Arial', 10),
                            bg='#2196F3', fg='white', activebackground='#1976D2',
                            width=12, command=self._save_settings)
        save_btn.pack(side='right', padx=(0, 10))
    
    def _load_current_settings(self):
        """Load current settings into dialog"""
        # Server settings
        self.url_var.set(self.current_settings.get('ollama_url', 'http://localhost:11434'))
        
        # Model settings
        self.selected_model_var.set(self.current_settings.get('model', 'qwen3-vl:8b'))
        
        # Generation settings
        self.temp_var.set(self.current_settings.get('temperature', 0.7))
        self.tokens_var.set(self.current_settings.get('max_tokens', 500))
        
        # Advanced settings
        self.enabled_var.set(self.current_settings.get('enabled', True))
        self.debug_var.set(self.current_settings.get('debug', False))
        
        # Update labels
        self._update_temp_label()
        self._update_tokens_label()
        
        # Load available models
        self._refresh_models()
    
    def _refresh_models(self):
        """Refresh available models list"""
        try:
            self.models_listbox.delete(0, tk.END)
            
            if self.callbacks.get('get_models'):
                models = self.callbacks['get_models']()
                for model in models:
                    self.models_listbox.insert(tk.END, model)
                
                # Select current model if in list
                current_model = self.selected_model_var.get()
                if current_model in models:
                    index = models.index(current_model)
                    self.models_listbox.selection_set(index)
                    self.models_listbox.see(index)
                
                if self.debug_mode:
                    print(f"⚙️ Loaded {len(models)} models")
            else:
                self.models_listbox.insert(tk.END, "No models available")
                
        except Exception as e:
            if self.debug_mode:
                print(f"❌ Error refreshing models: {e}")
            messagebox.showerror("Error", f"Failed to refresh models: {e}")
    
    def _test_connection(self):
        """Test connection to ollama server"""
        try:
            self.test_btn.config(state='disabled', text="🔄 Testing...")
            self.connection_status.config(text="⏳ Testing...", fg='#ffd700')
            
            # Update display
            self.dialog.update()
            
            if self.callbacks.get('test_connection'):
                url = self.url_var.get()
                success = self.callbacks['test_connection'](url)
                
                if success:
                    self.connection_status.config(text="✅ Connected", fg='#90ee90')
                    messagebox.showinfo("Success", "Connection successful!")
                else:
                    self.connection_status.config(text="❌ Failed", fg='#ff6b6b')
                    messagebox.showerror("Error", "Connection failed. Check URL and server status.")
            else:
                self.connection_status.config(text="❓ No test available", fg='#888888')
                
        except Exception as e:
            self.connection_status.config(text="❌ Error", fg='#ff6b6b')
            messagebox.showerror("Error", f"Connection test failed: {e}")
        finally:
            self.test_btn.config(state='normal', text="🔍 Test Connection")
    
    def _on_model_select(self, event=None):
        """Handle model selection"""
        selection = self.models_listbox.curselection()
        if selection:
            model = self.models_listbox.get(selection[0])
            self.selected_model_var.set(model)
    
    def _update_temp_label(self, event=None):
        """Update temperature label"""
        value = self.temp_var.get()
        self.temp_label.config(text=f"{value:.1f}")
    
    def _update_tokens_label(self, event=None):
        """Update tokens label"""
        value = self.tokens_var.get()
        self.tokens_label.config(text=str(value))
    
    def _reset_to_defaults(self):
        """Reset settings to defaults"""
        if messagebox.askyesno("Reset Settings", "Reset all settings to default values?"):
            self.url_var.set('http://localhost:11434')
            self.selected_model_var.set('qwen3-vl:8b')
            self.temp_var.set(0.7)
            self.tokens_var.set(500)
            self.enabled_var.set(True)
            self.debug_var.set(False)
            
            self._update_temp_label()
            self._update_tokens_label()
    
    def _apply_settings(self):
        """Apply settings without saving to config"""
        settings = self._get_current_settings()
        self.result = settings
        
        # Apply through callbacks
        if self.callbacks.get('apply_settings'):
            self.callbacks['apply_settings'](settings)
        
        messagebox.showinfo("Applied", "Settings applied for this session only.")
    
    def _save_settings(self):
        """Save settings to configuration"""
        settings = self._get_current_settings()
        self.result = settings
        
        # Save through callbacks
        if self.callbacks.get('save_settings'):
            success = self.callbacks['save_settings'](settings)
            if success:
                messagebox.showinfo("Saved", "Settings saved successfully!\n\nRestart application to apply all changes.")
                self.dialog.destroy()
            else:
                messagebox.showerror("Error", "Failed to save settings.")
        else:
            messagebox.showerror("Error", "Save functionality not available.")
    
    def _cancel(self):
        """Cancel dialog"""
        self.result = None
        self.dialog.destroy()
    
    def _get_current_settings(self) -> Dict[str, Any]:
        """Get current settings from dialog"""
        return {
            'ollama_url': self.url_var.get(),
            'model': self.selected_model_var.get(),
            'temperature': self.temp_var.get(),
            'max_tokens': self.tokens_var.get(),
            'enabled': self.enabled_var.get(),
            'debug': self.debug_var.get()
        }
    
    def _center_dialog(self):
        """Center dialog on parent window"""
        self.dialog.update_idletasks()
        
        # Get parent geometry
        parent_x = self.parent.winfo_rootx()
        parent_y = self.parent.winfo_rooty()
        parent_width = self.parent.winfo_width()
        parent_height = self.parent.winfo_height()
        
        # Get dialog size
        dialog_width = self.dialog.winfo_reqwidth()
        dialog_height = self.dialog.winfo_reqheight()
        
        # Calculate center position
        x = parent_x + (parent_width - dialog_width) // 2
        y = parent_y + (parent_height - dialog_height) // 2
        
        self.dialog.geometry(f"{dialog_width}x{dialog_height}+{x}+{y}") 