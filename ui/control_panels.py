#!/usr/bin/env python3
# coding=utf-8

import tkinter as tk
from typing import Callable, Optional, Dict, Any

class IMUPanel:
    def __init__(self, parent):
        self.parent = parent
        self.setup_panel()
    
    def setup_panel(self):
        """Setup IMU data panel"""
        self.panel = tk.LabelFrame(self.parent, text="📊 IMU / ORIENTATION DATA", 
                                  font=('Arial', 12, 'bold'), bg='#3c3c3c', fg='#87ceeb',
                                  relief='solid', bd=1)
        
        grid = tk.Frame(self.panel, bg='#3c3c3c')
        grid.pack(padx=20, pady=15, fill='both', expand=True)
        
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
                                  relief='solid', bd=1)
        
        grid = tk.Frame(self.panel, bg='#3c3c3c')
        grid.pack(padx=20, pady=15, fill='both', expand=True)
        
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
        
        # Movement controls (Left side)
        movement_frame = tk.Frame(content, bg='#3c3c3c')
        movement_frame.pack(side='left', padx=(0, 40))
        
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
        
        # Emergency stop (Right side)
        system_frame = tk.Frame(content, bg='#3c3c3c')
        system_frame.pack(side='right')
        
        emergency_callback = self.callbacks.get('emergency_stop', lambda: None)
        tk.Button(system_frame, text="🚨 EMERGENCY STOP", command=emergency_callback,
                 font=('Arial', 12, 'bold'), bg='#d32f2f', fg='white', 
                 activebackground='#b71c1c', width=18, pady=10).pack()
    
    def get_widget(self):
        """Get the main widget"""
        return self.panel 