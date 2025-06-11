#!/usr/bin/env python3
# coding=utf-8

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from typing import Callable, Optional, Dict, Any
import base64
import io
from PIL import Image, ImageTk
import time

class IMUPanel:
    def __init__(self, parent):
        self.parent = parent
        self.setup_panel()
    
    def setup_panel(self):
        """Setup IMU data panel"""
        self.panel = tk.LabelFrame(self.parent, text="📊 IMU / ORIENTATION DATA", 
                                  font=('Arial', 12, 'bold'), bg='#3c3c3c', fg='#87ceeb',
                                  relief='solid', bd=1, height=200)
        
        # Prevent the LabelFrame from shrinking below its set height
        self.panel.pack_propagate(False)
        
        grid = tk.Frame(self.panel, bg='#3c3c3c')
        grid.pack(padx=20, pady=15, fill='both', expand=False)
        
        # IMU values in a grid
        self.labels = {}
        for i, (label, attr) in enumerate([("Roll", "roll"), ("Pitch", "pitch"), ("Yaw", "yaw")]):
            row = tk.Frame(grid, bg='#3c3c3c')
            row.pack(fill='x', pady=8)
            
            tk.Label(row, text=f"{label}:", font=('Arial', 14, 'bold'), 
                    bg='#3c3c3c', fg='#ffd700', width=6, anchor='w').pack(side='left')
            
            value_label = tk.Label(row, text="+0.0°", font=('Arial', 14, 'bold'), 
                                 bg='#3c3c3c', fg='white')
            value_label.pack(side='left', padx=(10, 0))
            self.labels[attr] = value_label
    
    def update_imu_data(self, data: Dict[str, float]):
        """Update IMU data display"""
        try:
            roll = data.get('roll', 0.0)
            pitch = data.get('pitch', 0.0)
            yaw = data.get('yaw', 0.0)
            
            # Update labels with proper formatting
            self.labels['roll'].config(text=f"{roll:+.1f}°")
            self.labels['pitch'].config(text=f"{pitch:+.1f}°") 
            self.labels['yaw'].config(text=f"{yaw:+.1f}°")
        except:
            pass  # GUI might be destroyed
    
    def get_widget(self):
        """Get the main widget"""
        return self.panel

class FeaturesPanel:
    def __init__(self, parent, callbacks: Dict[str, Callable]):
        self.parent = parent
        self.callbacks = callbacks
        self.setup_panel()
    
    def setup_panel(self):
        """Setup robot features panel"""
        self.panel = tk.LabelFrame(self.parent, text="🤖 ROBOT FEATURES", 
                                  font=('Arial', 12, 'bold'), bg='#3c3c3c', fg='#87ceeb',
                                  relief='solid', bd=1, height=200)
        
        # Prevent the LabelFrame from shrinking below its set height
        self.panel.pack_propagate(False)
        
        grid = tk.Frame(self.panel, bg='#3c3c3c')
        grid.pack(padx=20, pady=15, fill='both', expand=False)
        
        # Feature status indicators
        features = [
            ("Roll Balance", "roll_balance", "toggle_roll_balance"),
            ("Performance Mode", "performance", "toggle_performance"), 
            ("Camera", "camera", "toggle_camera")
        ]
        
        self.status_labels = {}
        for i, (name, attr, callback_key) in enumerate(features):
            feature_row = tk.Frame(grid, bg='#3c3c3c')
            feature_row.pack(fill='x', pady=8)
            
            tk.Label(feature_row, text=f"{name}:", font=('Arial', 12, 'bold'), 
                    bg='#3c3c3c', fg='white', width=15, anchor='w').pack(side='left')
            
            status_label = tk.Label(feature_row, text="OFF", font=('Arial', 12, 'bold'), 
                                   bg='#3c3c3c', fg='red', width=6)
            status_label.pack(side='left', padx=(5, 10))
            self.status_labels[attr] = status_label
            
            callback = self.callbacks.get(callback_key, lambda: None)
            tk.Button(feature_row, text="Toggle", command=callback, 
                     font=('Arial', 9), bg='#555555', fg='white', 
                     activebackground='#666666', width=8).pack(side='right')
    
    def update_feature_status(self, feature: str, enabled: bool):
        """Update individual feature status"""
        if feature in self.status_labels:
            try:
                if enabled:
                    self.status_labels[feature].config(text="ON", fg='green')
                else:
                    self.status_labels[feature].config(text="OFF", fg='red')
            except:
                pass  # GUI might be destroyed
    
    def update_all_features(self, data: Dict[str, Any]):
        """Update all feature statuses"""
        feature_map = {
            'roll_balance_enabled': 'roll_balance',
            'performance_mode_enabled': 'performance',
            'camera_enabled': 'camera'
        }
        
        for data_key, feature_key in feature_map.items():
            if data_key in data:
                self.update_feature_status(feature_key, data[data_key])
    
    def get_widget(self):
        """Get the main widget"""
        return self.panel

class MovementPanel:
    def __init__(self, parent, callbacks: Dict[str, Callable]):
        self.parent = parent
        self.callbacks = callbacks
        self.setup_panel()
    
    def setup_panel(self):
        """Setup movement control panel"""
        self.panel = tk.LabelFrame(self.parent, text="🎮 ROBOT CONTROL", 
                                  font=('Arial', 12, 'bold'), bg='#3c3c3c', fg='#87ceeb',
                                  relief='solid', bd=1)
        
        content = tk.Frame(self.panel, bg='#3c3c3c')
        content.pack(padx=20, pady=15)
        
        # System controls (Left side)
        system_frame = tk.Frame(content, bg='#3c3c3c')
        system_frame.pack(side='left', padx=(0, 40))
        
        tk.Label(system_frame, text="System Controls", font=('Arial', 11, 'bold'), 
                bg='#3c3c3c', fg='white').pack(pady=(0, 10))
        
        emergency_callback = self.callbacks.get('emergency_stop', lambda: None)
        tk.Button(system_frame, text="🚨 EMERGENCY STOP", command=emergency_callback,
                 font=('Arial', 12, 'bold'), bg='#d32f2f', fg='white', 
                 activebackground='#b71c1c', width=18, pady=10).pack(pady=(0, 10))
        
        # Robot reset button
        reset_callback = self.callbacks.get('reset_robot', lambda: None)
        tk.Button(system_frame, text="🔄 RESET ROBOT", command=reset_callback,
                 font=('Arial', 10, 'bold'), bg='#ff9800', fg='white', 
                 activebackground='#f57c00', width=18, pady=8).pack(pady=(0, 5))
        
        # Pi reboot button
        reboot_callback = self.callbacks.get('reboot_pi', lambda: None)
        tk.Button(system_frame, text="🔃 REBOOT PI", command=reboot_callback,
                 font=('Arial', 10, 'bold'), bg='#2196f3', fg='white', 
                 activebackground='#1976d2', width=18, pady=8).pack(pady=(0, 5))
        
        # Pi power off button
        poweroff_callback = self.callbacks.get('poweroff_pi', lambda: None)
        tk.Button(system_frame, text="⚡ POWER OFF PI", command=poweroff_callback,
                 font=('Arial', 10, 'bold'), bg='#9c27b0', fg='white', 
                 activebackground='#7b1fa2', width=18, pady=8).pack()
        
        # Movement controls (Right side)
        movement_frame = tk.Frame(content, bg='#3c3c3c')
        movement_frame.pack(side='right')
        
        tk.Label(movement_frame, text="Movement Controls", font=('Arial', 11, 'bold'), 
                bg='#3c3c3c', fg='white').pack(pady=(0, 10))
        
        # Arrow button grid
        move_grid = tk.Frame(movement_frame, bg='#3c3c3c')
        move_grid.pack()
        
        move_buttons = [
            ("↑", 0, 50, 0, 1),
            ("←", 50, 0, 1, 0), 
            ("⏹", 0, 0, 1, 1),
            ("→", -50, 0, 1, 2),
            ("↓", 0, -50, 2, 1)
        ]
        
        move_callback = self.callbacks.get('move', lambda x, y: None)
        for text, x, y, row, col in move_buttons:
            btn = tk.Button(move_grid, text=text, command=lambda x=x, y=y: move_callback(x, y),
                           width=4, height=2, font=('Arial', 14, 'bold'),
                           bg='#555555', fg='white', activebackground='#666666')
            btn.grid(row=row, column=col, padx=2, pady=2)
    
    def get_widget(self):
        """Get the main widget"""
        return self.panel

class ImageDisplayPanel:
    def __init__(self, parent, image_callback: Callable = None):
        self.parent = parent
        self.image_callback = image_callback  # Callback to request image from robot
        self.current_image = None  # Store current PIL Image
        self.current_image_data = None  # Store current base64 image data
        self.setup_panel()
    
    def setup_panel(self):
        """Setup image display panel"""
        self.panel = tk.LabelFrame(self.parent, text="📷 CAMERA IMAGE", 
                                  font=('Arial', 12, 'bold'), bg='#3c3c3c', fg='#87ceeb',
                                  relief='solid', bd=1)
        
        # Status and controls frame - pack this FIRST to ensure it's always visible
        controls_frame = tk.Frame(self.panel, bg='#3c3c3c')
        controls_frame.pack(side='bottom', fill='x', padx=20, pady=(0, 15))
        
        # Image info/status
        self.status_label = tk.Label(controls_frame, text="📶 Ready to capture image", 
                                    font=('Arial', 10), bg='#3c3c3c', fg='#ffd700')
        self.status_label.pack(side='left')
        
        # Resolution selector
        resolution_frame = tk.Frame(controls_frame, bg='#3c3c3c')
        resolution_frame.pack(side='right', padx=(5, 0))
        
        tk.Label(resolution_frame, text="Resolution:", font=('Arial', 9), 
                bg='#3c3c3c', fg='white').pack(side='left', padx=(0, 5))
        
        self.resolution_var = tk.StringVar(value="high")
        resolution_combo = ttk.Combobox(resolution_frame, textvariable=self.resolution_var, 
                                       values=["high", "low"], state="readonly", width=8)
        resolution_combo.pack(side='left', padx=(0, 10))
        
        # Save screenshot button
        save_btn = tk.Button(controls_frame, text="💾 Save", 
                            font=('Arial', 9), bg='#555555', fg='white', 
                            activebackground='#666666', width=8,
                            command=self._save_image)
        save_btn.pack(side='right')
        
        # Refresh button
        refresh_btn = tk.Button(controls_frame, text="🔄 Refresh", 
                               font=('Arial', 9), bg='#555555', fg='white', 
                               activebackground='#666666', width=10,
                               command=self._refresh_image)
        refresh_btn.pack(side='right', padx=(0, 5))
        
        # Main image display area - pack this AFTER controls to fill remaining space
        self.image_frame = tk.Frame(self.panel, bg='#2b2b2b', relief='sunken', bd=2)
        self.image_frame.pack(side='top', padx=20, pady=15, fill='both', expand=True)
        
        # Image label (will hold the actual image)
        self.image_label = tk.Label(self.image_frame, text="📷\n\nCamera feed will appear here\n\nNo image available", 
                                   font=('Arial', 14), bg='#2b2b2b', fg='#808080',
                                   justify='center')
        self.image_label.pack(fill='both', expand=True, padx=10, pady=10)
        
        # Bind resize event to update image scaling
        self.image_frame.bind('<Configure>', self._on_resize)
    
    def _refresh_image(self):
        """Refresh camera image by requesting new capture"""
        if self.image_callback:
            self.status_label.config(text="🔄 Requesting image capture...")
            resolution = self.resolution_var.get()
            self.image_callback(resolution)
        else:
            self.status_label.config(text="⚠️ No image capture callback available")
    
    def _save_image(self):
        """Save current image to file"""
        if not self.current_image_data:
            messagebox.showwarning("No Image", "No image available to save")
            return
        
        try:
            # Ask user where to save
            filename = filedialog.asksaveasfilename(
                defaultextension=".jpg",
                filetypes=[("JPEG files", "*.jpg"), ("All files", "*.*")],
                title="Save Camera Image"
            )
            
            if filename:
                # Decode base64 and save
                img_bytes = base64.b64decode(self.current_image_data)
                with open(filename, 'wb') as f:
                    f.write(img_bytes)
                self.status_label.config(text=f"💾 Image saved to {filename}")
                # Reset status after 3 seconds
                self.panel.after(3000, lambda: self.status_label.config(text="📶 Image ready"))
        except Exception as e:
            messagebox.showerror("Save Error", f"Failed to save image: {e}")
            self.status_label.config(text="❌ Failed to save image")
    
    def _on_resize(self, event):
        """Handle resize events to rescale image"""
        # Only rescale if we have an image loaded and the event is for the image frame
        if self.current_image and event.widget == self.image_frame:
            # Re-update the image to rescale it to the new size
            self.update_image(self.current_image_data, success=True)
    
    def update_image(self, image_data=None, success=True, error_message=None):
        """Update the displayed image"""
        if not success:
            # Handle error case
            error_msg = error_message or "Unknown error"
            self.image_label.config(
                image="",
                text=f"❌\n\nImage capture failed\n\n{error_msg}",
                compound='center'
            )
            self.status_label.config(text=f"❌ {error_msg}")
            self.current_image = None
            self.current_image_data = None
            return
        
        if image_data is None:
            # No image available
            self.image_label.config(
                image="",
                text="📷\n\nCamera feed will appear here\n\nNo image available",
                compound='center'
            )
            self.status_label.config(text="📶 Ready to capture image")
            self.current_image = None
            self.current_image_data = None
        else:
            try:
                # Decode base64 image data
                img_bytes = base64.b64decode(image_data)
                
                # Load image with PIL
                pil_image = Image.open(io.BytesIO(img_bytes))
                self.current_image = pil_image.copy()  # Store original
                self.current_image_data = image_data  # Store base64 data for saving
                
                # Get available display size dynamically
                self.image_frame.update_idletasks()  # Ensure frame is laid out
                display_width = max(self.image_frame.winfo_width() - 20, 200)  # Account for padding
                display_height = max(self.image_frame.winfo_height() - 20, 150)  # Account for padding
                
                # Calculate scaling to fit display area (maintain aspect ratio)
                img_width, img_height = pil_image.size
                width_ratio = display_width / img_width
                height_ratio = display_height / img_height
                scale_ratio = min(width_ratio, height_ratio)
                
                new_width = int(img_width * scale_ratio)
                new_height = int(img_height * scale_ratio)
                
                # Resize image for display
                display_image = pil_image.resize((new_width, new_height), Image.Resampling.LANCZOS)
                
                # Convert to Tkinter PhotoImage
                photo = ImageTk.PhotoImage(display_image)
                
                # Update the label
                self.image_label.config(
                    image=photo,
                    text="",
                    compound='center'
                )
                self.image_label.image = photo  # Keep a reference to prevent GC
                
                # Calculate image size for status
                size_kb = len(img_bytes) / 1024
                self.status_label.config(text=f"📷 Image: ({size_kb:.1f}KB, {img_width}x{img_height})")
                
            except Exception as e:
                self.image_label.config(
                    image="",
                    text=f"❌\n\nFailed to load image\n\n{str(e)}",
                    compound='center'
                )
                self.status_label.config(text=f"❌ Image load error: {e}")
                self.current_image = None
                self.current_image_data = None
    
    def get_widget(self):
        """Get the main widget"""
        return self.panel 