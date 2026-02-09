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
        
        # CPU percentage row
        cpu_frame = tk.Frame(self.card, bg='#3c3c3c')
        cpu_frame.pack(pady=(10, 5), padx=8, fill='x')
        
        self.cpu_value_label = tk.Label(cpu_frame, text="0%", font=('Arial', 9, 'bold'), 
                                       bg='#3c3c3c', fg='#cccccc', width=6, anchor='e')
        self.cpu_value_label.pack(side='left')
        
        # Make canvas expand to fill most of available space
        self.cpu_canvas = tk.Canvas(cpu_frame, height=24, bg='#2b2b2b', 
                                   highlightthickness=0, bd=0)
        self.cpu_canvas.pack(side='left', padx=(5, 5), fill='x', expand=True)
        
        tk.Label(cpu_frame, text="CPU", font=('Arial', 8), 
                bg='#3c3c3c', fg='#87ceeb').pack(side='left')
        
        # Load row
        load_frame = tk.Frame(self.card, bg='#3c3c3c')
        load_frame.pack(pady=(0, 10), padx=8, fill='x')
        
        self.load_value_label = tk.Label(load_frame, text="0.0", font=('Arial', 9, 'bold'), 
                                        bg='#3c3c3c', fg='#cccccc', width=6, anchor='e')
        self.load_value_label.pack(side='left')
        
        # Make canvas expand to fill most of available space
        self.load_canvas = tk.Canvas(load_frame, height=24, bg='#2b2b2b', 
                                    highlightthickness=0, bd=0)
        self.load_canvas.pack(side='left', padx=(5, 5), fill='x', expand=True)
        
        tk.Label(load_frame, text="Load", font=('Arial', 8), 
                bg='#3c3c3c', fg='#87ceeb').pack(side='left')
        
        # Draw initial empty bars
        self.draw_cpu_bar(0)
        self.draw_load_bar(0)
    
    def draw_cpu_bar(self, cpu_percent):
        """Draw CPU percentage bar"""
        self.cpu_canvas.delete("all")
        
        # Get current canvas width dynamically
        self.cpu_canvas.update_idletasks()
        canvas_width = self.cpu_canvas.winfo_width()
        canvas_height = self.cpu_canvas.winfo_height()
        
        # Use the actual full canvas width without any limitations
        # Remove all width restrictions to use complete available space
        
        # Draw fill based on CPU percentage directly on canvas background
        if cpu_percent > 0:
            # Use complete actual canvas dimensions for fill area
            fill_x1 = 0
            fill_y1 = 0
            fill_width = int(canvas_width * cpu_percent / 100)
            fill_x2 = fill_x1 + fill_width
            fill_y2 = canvas_height
            
            # Ensure minimum width if there's any value
            if fill_width < 1 and cpu_percent > 0:
                fill_width = 1
                fill_x2 = fill_x1 + fill_width
                
            # Color based on CPU usage
            if cpu_percent >= 80:
                fill_color = '#f44336'  # Red - high load
            elif cpu_percent >= 60:
                fill_color = '#ff9800'  # Orange - moderate load
            else:
                fill_color = '#4caf50'  # Green - normal load
            
            # Draw the fill rectangle directly on canvas using full width
            self.cpu_canvas.create_rectangle(fill_x1, fill_y1, fill_x2, fill_y2, 
                                           outline='', fill=fill_color)
    
    def draw_load_bar(self, load_value):
        """Draw load bar (0-4 scale)"""
        self.load_canvas.delete("all")
        
        # Get current canvas width dynamically
        self.load_canvas.update_idletasks()
        canvas_width = self.load_canvas.winfo_width()
        canvas_height = self.load_canvas.winfo_height()
        
        # Use the actual full canvas width without any limitations
        # Remove all width restrictions to use complete available space
        
        # Draw fill based on load (0-4 scale) directly on canvas background
        if load_value > 0:
            # Clamp load value to 0-4 range
            clamped_load = min(max(load_value, 0), 4)
            
            # Use complete actual canvas dimensions for fill area
            fill_x1 = 0
            fill_y1 = 0
            fill_width = int(canvas_width * clamped_load / 4)
            fill_x2 = fill_x1 + fill_width
            fill_y2 = canvas_height
            
            # Ensure minimum width if there's any value
            if fill_width < 1 and load_value > 0:
                fill_width = 1
                fill_x2 = fill_x1 + fill_width
                
            # Color based on load
            if load_value >= 3.0:
                fill_color = '#f44336'  # Red - overloaded
            elif load_value >= 1.5:
                fill_color = '#ff9800'  # Orange - high load
            else:
                fill_color = '#4caf50'  # Green - normal load
            
            # Draw the fill rectangle directly on canvas using full width
            self.load_canvas.create_rectangle(fill_x1, fill_y1, fill_x2, fill_y2, 
                                            outline='', fill=fill_color)
    
    def update_cpu_data(self, data: Dict[str, float]):
        """Update CPU display"""
        try:
            # Update CPU percentage
            cpu_percent = data.get('cpu_percent', 0.0)
            self.cpu_value_label.config(text=f"{cpu_percent:.1f}%")
            self.draw_cpu_bar(cpu_percent)
            
            # Update 1-minute load average
            load_1min = data.get('cpu_load_1min', 0.0)
            self.load_value_label.config(text=f"{load_1min:.2f}")
            self.draw_load_bar(load_1min)
        except:
            pass  # GUI might be destroyed
    
    def get_widget(self):
        """Get the main widget"""
        return self.card


class VoiceStatusWidget:
    """Widget showing voice recognition status"""
    def __init__(self, parent):
        self.parent = parent
        self.setup_widget()

    def setup_widget(self):
        """Setup voice status widget"""
        self.frame = tk.Frame(self.parent, bg='#404040')

        # Voice icon (emoji)
        self.icon_label = tk.Label(self.frame, text="🎤", font=('Arial', 14),
                                   bg='#404040', fg='gray')
        self.icon_label.pack(side='left', padx=(0, 5))

        # Status text
        self.status_label = tk.Label(self.frame, text="Voice: Offline",
                                     font=('Arial', 10), bg='#404040', fg='gray')
        self.status_label.pack(side='left')

        # Partial recognition text (shows what's being heard)
        self.partial_label = tk.Label(self.frame, text="", font=('Arial', 9, 'italic'),
                                      bg='#404040', fg='#aaaaaa')
        self.partial_label.pack(side='left', padx=(10, 0))

    def update_status(self, status: str, partial_text: str = ""):
        """Update voice status
        Args:
            status: 'offline', 'ready', 'listening', 'processing'
            partial_text: Partial recognition text to display
        """
        status_map = {
            'offline': ('🎤', 'Voice: Offline', 'gray'),
            'ready': ('🎤', 'Voice: Ready', '#4caf50'),  # Green
            'listening': ('🔴', 'Voice: Listening', '#ff5722'),  # Red (recording)
            'processing': ('🔄', 'Voice: Processing', '#ff9800')  # Orange
        }

        icon, text, color = status_map.get(status, status_map['offline'])

        self.icon_label.config(text=icon)
        self.status_label.config(text=text, fg=color)

        # Update partial text
        if partial_text:
            display_text = f'"{partial_text}"'
            if len(display_text) > 50:
                display_text = display_text[:47] + '..."'
            self.partial_label.config(text=display_text)
        else:
            self.partial_label.config(text="")

    def get_widget(self):
        """Get the main widget"""
        return self.frame


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

        # Voice status widget (center-left)
        self.voice_widget = VoiceStatusWidget(self.bar)
        self.voice_widget.get_widget().pack(side='left', padx=(30, 15), pady=8)

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

    def update_voice_status(self, status: str, partial_text: str = ""):
        """Update voice recognition status
        Args:
            status: 'offline', 'ready', 'listening', 'processing'
            partial_text: Partial recognition text
        """
        try:
            self.voice_widget.update_status(status, partial_text)
        except:
            pass  # GUI might be destroyed

    def get_widget(self):
        """Get the main widget"""
        return self.bar 