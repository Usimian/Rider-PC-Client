#!/usr/bin/env python3
# coding=utf-8

import tkinter as tk
from datetime import datetime
from typing import Callable, Optional, Dict

class BatteryWidget:
    def __init__(self, parent):
        self.parent = parent
        self.setup_widget()
    
    def setup_widget(self):
        """Setup battery status widget"""
        self.card = tk.Frame(self.parent, bg='#3c3c3c', relief='solid', bd=1)
        
        content = tk.Frame(self.card, bg='#3c3c3c')
        content.pack(pady=15)
        
        # Battery icon and percentage container
        display = tk.Frame(content, bg='#3c3c3c')
        display.pack()
        
        # Create custom battery icon canvas
        self.canvas = tk.Canvas(display, width=40, height=20, bg='#3c3c3c', 
                               highlightthickness=0, bd=0)
        self.canvas.pack(side='left', padx=(0, 10))
        
        # Battery percentage label
        self.label = tk.Label(display, text="0%", font=('Arial', 16, 'bold'), 
                             bg='#3c3c3c', fg='red')
        self.label.pack(side='left')
        
        # Draw initial battery outline
        self.draw_battery_icon(0)
    
    def draw_battery_icon(self, battery_level):
        """Draw a battery icon that fills based on battery level"""
        # Clear the canvas
        self.canvas.delete("all")
        
        # Battery dimensions
        battery_width = 30
        battery_height = 16
        terminal_width = 3
        terminal_height = 8
        
        # Calculate battery body position (centered in canvas)
        x1 = 5
        y1 = 2
        x2 = x1 + battery_width
        y2 = y1 + battery_height
        
        # Draw battery outline (white)
        self.canvas.create_rectangle(x1, y1, x2, y2, outline='white', width=2, fill='#2b2b2b')
        
        # Draw battery terminal (small rectangle on right side)
        terminal_x1 = x2
        terminal_y1 = y1 + (battery_height - terminal_height) // 2
        terminal_x2 = terminal_x1 + terminal_width
        terminal_y2 = terminal_y1 + terminal_height
        self.canvas.create_rectangle(terminal_x1, terminal_y1, terminal_x2, terminal_y2, 
                                   outline='white', width=2, fill='#2b2b2b')
        
        # Draw battery fill based on level
        if battery_level > 0:
            fill_width = max(1, int((battery_width - 4) * battery_level / 100))  # -4 for padding
            fill_x1 = x1 + 2
            fill_y1 = y1 + 2
            fill_x2 = fill_x1 + fill_width
            fill_y2 = y2 - 2
            
            # Color based on battery level
            if battery_level >= 70:
                fill_color = '#4caf50'  # Green
            elif battery_level >= 40:
                fill_color = '#ff9800'  # Orange  
            elif battery_level >= 20:
                fill_color = '#ff5722'  # Red-orange
            else:
                fill_color = '#f44336'  # Red
                
            self.canvas.create_rectangle(fill_x1, fill_y1, fill_x2, fill_y2, 
                                       outline='', fill=fill_color)
    
    def update_battery(self, data):
        """Update battery display"""
        battery_level = data.get('level', 0)
        
        # Update battery percentage label
        self.label.config(text=f"{battery_level}%")
        
        # Update battery icon
        self.draw_battery_icon(battery_level)
        
        # Update percentage label color based on battery level
        if battery_level >= 70:
            color = '#4caf50'  # Green
        elif battery_level >= 40:
            color = '#ff9800'  # Orange
        elif battery_level >= 20:
            color = '#ff5722'  # Red-orange
        else:
            color = '#f44336'  # Red
        
        self.label.config(fg=color)
    
    def get_widget(self):
        """Get the main widget"""
        return self.card

class ControllerWidget:
    def __init__(self, parent):
        self.parent = parent
        self.setup_widget()
    
    def setup_widget(self):
        """Setup controller status widget"""
        self.card = tk.Frame(self.parent, bg='#3c3c3c', relief='solid', bd=1)
        
        # Controller icon - color changes based on status
        self.icon = tk.Label(self.card, text="🎮", font=('Arial', 30), 
                           bg='#3c3c3c', fg='#808080')  # Start with grey
        self.icon.pack(pady=15)
    
    def update_status(self, is_mqtt_connected, is_controller_connected):
        """Update controller connection status display"""
        try:
            if not is_mqtt_connected:
                # MQTT not connected - grey
                self.icon.config(fg='#808080')
            elif is_controller_connected:
                # MQTT connected AND controller responding - green
                self.icon.config(fg='#4caf50')
            else:
                # MQTT connected but no controller response - red
                self.icon.config(fg='#f44336')
        except:
            pass  # GUI might be destroyed
    
    def get_widget(self):
        """Get the main widget"""
        return self.card

class SpeedWidget:
    def __init__(self, parent, on_speed_change: Optional[Callable] = None):
        self.parent = parent
        self.on_speed_change = on_speed_change
        self.setup_widget()
    
    def setup_widget(self):
        """Setup speed control widget"""
        self.card = tk.Frame(self.parent, bg='#3c3c3c', relief='solid', bd=1)
        
        tk.Label(self.card, text="⚡ SPEED", font=('Arial', 10, 'bold'), 
                bg='#3c3c3c', fg='#87ceeb').pack(pady=(8, 2))
        
        self.label = tk.Label(self.card, text="1.0x", font=('Arial', 16, 'bold'), 
                             bg='#3c3c3c', fg='white')
        self.label.pack(pady=(0, 5))
        
        self.scale = tk.Scale(self.card, from_=0.1, to=2.0, resolution=0.1, orient='horizontal',
                             bg='#3c3c3c', fg='white', highlightbackground='#3c3c3c',
                             command=self._speed_changed, length=300, width=15)
        self.scale.set(1.0)
        self.scale.pack(pady=(0, 8))
    
    def _speed_changed(self, value):
        """Handle speed scale change"""
        speed_val = float(value)
        self.label.config(text=f"{speed_val:.1f}x")
        if self.on_speed_change:
            self.on_speed_change(speed_val)
    
    def update_speed(self, speed_value):
        """Update speed display"""
        self.label.config(text=f"{speed_value:.1f}x")
        self.scale.set(speed_value)
    
    def get_widget(self):
        """Get the main widget"""
        return self.card

class CPUWidget:
    def __init__(self, parent):
        self.parent = parent
        self.setup_widget()
    
    def setup_widget(self):
        """Setup CPU status widget"""
        self.card = tk.Frame(self.parent, bg='#3c3c3c', relief='solid', bd=1)
        
        # CPU title
        tk.Label(self.card, text="🖥️ CPU", font=('Arial', 10, 'bold'), 
                bg='#3c3c3c', fg='#87ceeb').pack(pady=(8, 2))
        
        # CPU percentage display
        self.cpu_percent_label = tk.Label(self.card, text="0%", font=('Arial', 14, 'bold'), 
                                         bg='#3c3c3c', fg='#4caf50')
        self.cpu_percent_label.pack(pady=(0, 5))
        
        # Load averages
        load_frame = tk.Frame(self.card, bg='#3c3c3c')
        load_frame.pack(pady=(0, 8))
        
        tk.Label(load_frame, text="Load:", font=('Arial', 8), 
                bg='#3c3c3c', fg='#cccccc').pack()
        
        self.load_labels = {}
        for period in ['1m', '5m', '15m']:
            label = tk.Label(load_frame, text=f"{period}: 0.0", font=('Arial', 8), 
                           bg='#3c3c3c', fg='#cccccc')
            label.pack()
            self.load_labels[period] = label
    
    def update_cpu_data(self, data: Dict[str, float]):
        """Update CPU display"""
        try:
            # Update CPU percentage
            cpu_percent = data.get('cpu_percent', 0.0)
            self.cpu_percent_label.config(text=f"{cpu_percent:.1f}%")
            
            # Color based on CPU usage
            if cpu_percent >= 80:
                color = '#f44336'  # Red - high load
            elif cpu_percent >= 60:
                color = '#ff9800'  # Orange - moderate load
            else:
                color = '#4caf50'  # Green - normal load
            
            self.cpu_percent_label.config(fg=color)
            
            # Update load averages
            load_1min = data.get('cpu_load_1min', 0.0)
            load_5min = data.get('cpu_load_5min', 0.0)
            load_15min = data.get('cpu_load_15min', 0.0)
            
            self.load_labels['1m'].config(text=f"1m: {load_1min:.2f}")
            self.load_labels['5m'].config(text=f"5m: {load_5min:.2f}")
            self.load_labels['15m'].config(text=f"15m: {load_15min:.2f}")
            
            # Color load averages based on system load (assuming 4-core Pi)
            for period, value in [('1m', load_1min), ('5m', load_5min), ('15m', load_15min)]:
                if value >= 3.0:
                    load_color = '#f44336'  # Red - overloaded
                elif value >= 1.5:
                    load_color = '#ff9800'  # Orange - high load
                else:
                    load_color = '#4caf50'  # Green - normal load
                
                self.load_labels[period].config(fg=load_color)
        except:
            pass  # GUI might be destroyed
    
    def get_widget(self):
        """Get the main widget"""
        return self.card

class StatusBar:
    def __init__(self, parent, broker_host: str):
        self.parent = parent
        self.broker_host = broker_host
        self.setup_widget()
    
    def setup_widget(self):
        """Setup status bar"""
        self.bar = tk.Frame(self.parent, bg='#404040')
        
        self.connection_status = tk.Label(self.bar, text=f"Connecting to {self.broker_host}...", 
                                        bg='#404040', fg='#ffd700', font=('Arial', 11, 'bold'))
        self.connection_status.pack(side='left', padx=15, pady=8)
        
        # Current time (right side of status bar)
        self.time_label = tk.Label(self.bar, text="--:--", font=('Arial', 11, 'bold'), 
                                 bg='#404040', fg='white')
        self.time_label.pack(side='right', padx=15, pady=8)
    
    def update_connection_status(self, connected: bool, message: str = None):
        """Update connection status"""
        if connected:
            text = message or f"Connected to {self.broker_host}"
            color = 'green'
        else:
            text = message or "Disconnected"
            color = 'red'
        
        self.connection_status.config(text=text, fg=color)
    
    def update_time(self):
        """Update time display"""
        try:
            current_time = datetime.now().strftime("%H:%M")
            self.time_label.config(text=current_time)
        except:
            pass  # GUI might be destroyed
    
    def get_widget(self):
        """Get the main widget"""
        return self.bar 