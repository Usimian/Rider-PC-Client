#!/usr/bin/env python3
# coding=utf-8

"""
Core package for Rider Robot PC Client
Contains configuration, state management, and application controller
"""

from .config_manager import ConfigManager
from .robot_state import RobotState
# ApplicationController has complex dependencies, import directly when needed

__all__ = [
    'ConfigManager',
    'RobotState'
] 