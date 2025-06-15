#!/usr/bin/env python3
# coding=utf-8

"""
UI package for Rider Robot PC Client
Contains all GUI components and widgets including LLM integration
"""

# GUIManager has complex dependencies, import directly when needed
from .main_window import MainWindow
from .status_widgets import BatteryWidget, ControllerWidget, SpeedWidget, CPUWidget, StatusBar
from .control_panels import IMUPanel, FeaturesPanel, MovementPanel, ImageDisplayPanel
from .llm_panel import LLMPanel
from .llm_settings_dialog import LLMSettingsDialog

__all__ = [
    'MainWindow', 
    'BatteryWidget',
    'ControllerWidget',
    'SpeedWidget',
    'CPUWidget',
    'StatusBar',
    'IMUPanel',
    'FeaturesPanel',
    'MovementPanel',
    'ImageDisplayPanel',
    'LLMPanel'
] 