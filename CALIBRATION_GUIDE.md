# Robot Movement Calibration Guide

## Overview

Robot movement commands are often non-linear. This calibration system helps you map desired movements (distance, rotation) to actual command values that work for your specific robot.

## Files

- `movement_calibration.json` - Calibration data (edit this after testing)
- `core/movement_calibration.py` - Calibration manager class
- `calibration_tester.py` - Interactive testing tool

## Quick Start

### 1. Run the Calibration Tester

```bash
python3 calibration_tester.py
```

### 2. Test Movement Values

Use the interactive commands to test different values:

```
calibration> f 10 2         # Forward with value 10 for 2 seconds
calibration> b 8 1.5        # Backward with value 8 for 1.5 seconds
calibration> l 12 1         # Left turn with value 12 for 1 second
calibration> r 15 0.8       # Right turn with value 15 for 0.8 seconds
```

### 3. Measure Actual Movement

- For forward/backward: Measure distance traveled in cm/inches
- For turns: Measure actual rotation angle in degrees
- Record these measurements!

### 4. Update Calibration

Based on your measurements, update the calibration:

```
calibration> update forward slow 8      # If value 8 gives good slow speed
calibration> update turn_left 90deg 12  # If value 12 gives ~90° rotation
```

### 5. Save Calibration

```
calibration> save
```

### 6. View Current Values

```
calibration> show
```

## Calibration Strategy

### For Forward/Backward Movement:

1. Start with small values (3-5) and increase until you find:
   - **Crawl**: Minimum value that produces movement
   - **Slow**: Controlled slow speed (good for precision)
   - **Normal**: Comfortable walking speed
   - **Fast**: Maximum safe speed

2. Test with fixed duration (e.g., 2 seconds) and measure distance
3. Repeat multiple times to verify consistency

### For Turning:

1. Test different values to find what produces:
   - **15°**: Small adjustment
   - **45°**: Medium turn
   - **90°**: Quarter turn (right angle)
   - **180°**: Half turn

2. Mark a reference point on the floor to measure rotation
3. Test duration vs. total rotation achieved

## Example Calibration Session

```bash
$ python3 calibration_tester.py

# Test forward movement
calibration> f 5 2
# Robot moves 10cm → Too slow

calibration> f 10 2
# Robot moves 25cm → Good for "normal" speed

calibration> update forward normal 10

# Test 90° turn
calibration> l 10 1
# Robot turns ~60° → Not enough

calibration> l 15 1
# Robot turns ~95° → Close!

calibration> update turn_left 90deg 15

# Save results
calibration> save
```

## Tips

- **Surface matters**: Calibrate on the surface you'll use most
- **Battery level**: Movement may slow as battery drains
- **Load**: Carrying payload affects speed
- **Temperature**: Motors may perform differently when warm/cold
- **Consistency**: Test each value 3-5 times to ensure repeatability

## Using Calibration in Your Code

```python
from core.movement_calibration import MovementCalibration

cal = MovementCalibration()

# Get calibrated values by name
x, y = cal.get_movement_command('forward', 'slow')  # Returns (0, 6)
x, y = cal.get_movement_command('turn_left', '90deg')  # Returns (10, 0)

# Or get specific values
forward_slow = cal.get_forward_value('slow')
turn_90 = cal.get_turn_left_value('90deg')
```

## Advanced: Non-Linear Interpolation

If you discover highly non-linear behavior (e.g., values 1-5 do nothing, then 6 suddenly moves fast), you may need to add more calibration points:

```json
{
  "forward": {
    "calibration_points": [
      {"speed": "dead_zone", "value": 0},
      {"speed": "threshold", "value": 5, "description": "Minimum to start moving"},
      {"speed": "slow", "value": 6, "description": "First usable speed"},
      ...
    ]
  }
}
```

## Troubleshooting

**Robot doesn't move:**
- Value too low (below movement threshold)
- Check MQTT connection
- Verify robot is not in emergency stop mode

**Robot moves too much:**
- Value too high
- Duration too long
- Reduce both and test again

**Inconsistent movement:**
- Battery level varying
- Surface conditions changing
- Robot hardware issues
- Thermal effects on motors

**Rotation not accurate:**
- Try shorter durations with same values
- Test on flat, level surface
- Check if robot has gyroscope stabilization enabled/disabled
