#!/usr/bin/env python3
# coding=utf-8

import json
import os
from typing import Dict, Any, Optional, Tuple

class MovementCalibration:
    """Manages movement command calibration for non-linear robot control"""

    def __init__(self, calibration_file='movement_calibration.json'):
        self.calibration_file = calibration_file
        self.calibration_data = {}
        self.load_calibration()

    def load_calibration(self):
        """Load calibration data from file"""
        if os.path.exists(self.calibration_file):
            try:
                with open(self.calibration_file, 'r') as f:
                    self.calibration_data = json.load(f)
                print(f"✓ Movement calibration loaded from {self.calibration_file}")
            except Exception as e:
                print(f"⚠️ Failed to load calibration: {e}")
                self.calibration_data = {}
        else:
            print(f"⚠️ Calibration file not found: {self.calibration_file}")

    def save_calibration(self):
        """Save current calibration data to file"""
        try:
            with open(self.calibration_file, 'w') as f:
                json.dump(self.calibration_data, f, indent=2)
            print(f"✓ Calibration saved to {self.calibration_file}")
            return True
        except Exception as e:
            print(f"⚠️ Failed to save calibration: {e}")
            return False

    def get_value_by_name(self, movement_type: str, name: str) -> Optional[int]:
        """
        Get calibrated value by name (e.g., 'forward', 'slow' -> 6)

        Args:
            movement_type: 'forward', 'backward', 'turn_left', 'turn_right'
            name: Named calibration point (e.g., 'slow', 'normal', '90deg')

        Returns:
            Calibrated value or None if not found
        """
        if movement_type not in self.calibration_data:
            return None

        points = self.calibration_data[movement_type].get('calibration_points', [])
        for point in points:
            # Check both 'speed' and 'angle' keys
            point_name = point.get('speed') or point.get('angle')
            if point_name == name:
                return point.get('value')

        return None

    def get_forward_value(self, speed: str = 'normal') -> int:
        """Get forward movement value by speed name"""
        value = self.get_value_by_name('forward', speed)
        return value if value is not None else 10  # Default fallback

    def get_backward_value(self, speed: str = 'normal') -> int:
        """Get backward movement value by speed name"""
        value = self.get_value_by_name('backward', speed)
        return value if value is not None else -10  # Default fallback

    def get_turn_left_value(self, angle: str = '90deg') -> int:
        """Get left turn value by angle name"""
        value = self.get_value_by_name('turn_left', angle)
        return value if value is not None else 10  # Default fallback

    def get_turn_right_value(self, angle: str = '90deg') -> int:
        """Get right turn value by angle name"""
        value = self.get_value_by_name('turn_right', angle)
        return value if value is not None else -10  # Default fallback

    def get_movement_command(self, action: str, intensity: str = 'normal') -> Tuple[int, int]:
        """
        Get (x, y) command values for a named action

        Args:
            action: 'forward', 'backward', 'turn_left', 'turn_right', 'stop'
            intensity: Speed/angle name (e.g., 'slow', 'normal', '90deg')

        Returns:
            Tuple of (x, y) values for movement command
        """
        if action == 'stop':
            return (0, 0)
        elif action == 'forward':
            return (0, self.get_forward_value(intensity))
        elif action == 'backward':
            return (0, self.get_backward_value(intensity))
        elif action == 'turn_left':
            return (self.get_turn_left_value(intensity), 0)
        elif action == 'turn_right':
            return (self.get_turn_right_value(intensity), 0)
        else:
            return (0, 0)  # Default to stop

    def update_calibration_point(self, movement_type: str, name: str, new_value: int):
        """
        Update a calibration point value

        Args:
            movement_type: 'forward', 'backward', 'turn_left', 'turn_right'
            name: Name of calibration point
            new_value: New value to set
        """
        if movement_type not in self.calibration_data:
            print(f"⚠️ Unknown movement type: {movement_type}")
            return False

        points = self.calibration_data[movement_type].get('calibration_points', [])
        for point in points:
            point_name = point.get('speed') or point.get('angle')
            if point_name == name:
                point['value'] = new_value
                print(f"✓ Updated {movement_type}/{name} to {new_value}")
                return True

        print(f"⚠️ Calibration point not found: {movement_type}/{name}")
        return False

    def list_calibration_points(self) -> Dict[str, Any]:
        """Get all calibration points for display/editing"""
        summary = {}
        for movement_type, data in self.calibration_data.items():
            if movement_type == 'notes':
                continue
            summary[movement_type] = []
            points = data.get('calibration_points', [])
            for point in points:
                name = point.get('speed') or point.get('angle')
                value = point.get('value')
                desc = point.get('description', '')
                summary[movement_type].append({
                    'name': name,
                    'value': value,
                    'description': desc
                })
        return summary


# Example usage
if __name__ == "__main__":
    cal = MovementCalibration()

    print("\n=== Movement Calibration ===\n")

    # Show all calibration points
    points = cal.list_calibration_points()
    for movement_type, point_list in points.items():
        print(f"\n{movement_type.upper()}:")
        for point in point_list:
            print(f"  {point['name']:10s} = {point['value']:3d}  # {point['description']}")

    # Test getting movement commands
    print("\n=== Example Commands ===\n")
    print(f"Forward (slow):  x={cal.get_movement_command('forward', 'slow')[0]:3d}, y={cal.get_movement_command('forward', 'slow')[1]:3d}")
    print(f"Forward (normal): x={cal.get_movement_command('forward', 'normal')[0]:3d}, y={cal.get_movement_command('forward', 'normal')[1]:3d}")
    print(f"Turn left (90°): x={cal.get_movement_command('turn_left', '90deg')[0]:3d}, y={cal.get_movement_command('turn_left', '90deg')[1]:3d}")
    print(f"Turn right (45°): x={cal.get_movement_command('turn_right', '45deg')[0]:3d}, y={cal.get_movement_command('turn_right', '45deg')[1]:3d}")
