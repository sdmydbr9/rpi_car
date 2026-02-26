# 🎮 Gamepad Controls Guide

## Overview
This document describes all gamepad controls for the Raspberry Pi Car, including the new button mappings for engine shutdown, autopilot control, and emergency braking.

---

## Standard Controls

### Locomotion
- **Left Analog Stick (Y-axis)**: Forward / Reverse Throttle
- **Right Analog Stick (X-axis)**: Left / Right Steering

### Gear Shifting
| Button | Gear | Max Speed |
|--------|------|-----------|
| **A** | Gear 1 | 35% |
| **B** | Gear 2 | 60% |
| **X** | Gear 3 | 80% |
| **Y** | SPORT 🚀 | 100% |

**Note:** When Autopilot is active, the **X** button behavior changes to **Emergency Brake** instead of Gear 3.

### Engine Start/Stop
- **START Button**: Toggle engine on/off (activates/deactivates drive mode)

---

## NEW Features

### 1. Select Button - Turn Off Engine 🔘

**Feature:** Double-click the SELECT button to turn off the engine completely.

**How to Use:**
1. Press **SELECT** button (first click)
2. Press **SELECT** button again within 0.5 seconds (double-click)
3. Engine will shut down (same as setting to STANDBY)

**Button Code:** `BTN_SELECT`

**Test Event Codes:**
```
Event Code: BTN_SELECT      | State: 1  (Press)
Event Code: BTN_SELECT      | State: 0  (Release)
```

**Use Case:** Quick engine shutdown without accessing the menu or using START button.

---

### 2. Autopilot Enablement (LB + RB) 🤖

**Feature:** Press LB and RB bumpers **simultaneously** to toggle autopilot mode on/off.

**How to Use:**
1. Hold down **LB** (Left Bumper) and **RB** (Right Bumper) at the same time
2. Both buttons trigger the autopilot toggle automatically
3. Status will display: `🤖 AUTOPILOT ACTIVE` when enabled
4. Press **LB+RB** again to disable autopilot

**Button Codes:** 
- `BTN_TL` = Left Bumper (LB)
- `BTN_TR` = Right Bumper (RB)

**Autopilot State:**
- ✅ **ENABLED**: Navigation system active, driver can override with steering/throttle
- ❌ **DISABLED**: Normal manual control only

**Use Case:** Engage autonomous driving mode for long-distance navigation or testing.

---

### 3. Emergency Brake (When Autopilot Active) 🛑

**Feature:** While autopilot is running, pressing **X** button activates **Emergency Brake** instead of switching to Gear 3.

**How to Use:**
1. Ensure autopilot is **ENABLED** (see Autopilot Enablement above)
2. Press and hold **X** button to activate emergency braking
3. UI will display: `🛑 EMERGENCY BRAKE ACTIVE`
4. Release **X** button to deactivate emergency brake
5. Car resumes normal autopilot operation

**Button Code:** `BTN_WEST` or `BTN_X`

**Emergency Brake Behavior:**
- Throttle is set to 0% (motors disengage)
- Steering input is still allowed for manual override
- Brakes are released on button release
- Only functional when autopilot is active

**Use Case:** Safety feature to stop the car immediately during autonomous operation.

---

## Control Summary Table

| Control | Type | Action |
|---------|------|--------|
| Left Stick Y | Analog | Throttle (Forward/Reverse) |
| Right Stick X | Analog | Steering (Left/Right) |
| A | Button | Gear 1 (35% speed) |
| B | Button | Gear 2 (60% speed) |
| X | Button | Gear 3 (80% speed) OR Emergency Brake* |
| Y | Button | Sport Mode (100% speed) |
| START | Button | Toggle Engine On/Off |
| SELECT | Button | Double-click to Turn Off Engine |
| LB + RB | Button Combo | Toggle Autopilot Mode |

**\*** X button switches function based on autopilot state:
- Normal Mode: Gear 3
- Autopilot Mode: Emergency Brake

---

## Status Indicators

The telemetry UI displays real-time status:

```
STATUS: 🎮 EvoFox Active
GEAR: [Gear 1] (Max Pwr: 35%)
INPUT: Throttle: 45% | Steering: 0%
MOTORS: Left: 42% | Right: 42%
POWER:  12.5V @ 2.3A

🤖 AUTOPILOT ACTIVE          ← When autopilot is enabled
🛑 EMERGENCY BRAKE ACTIVE    ← When emergency brake is pressed
● ON AIR (LIVE)              ← Engine is running
```

---

## Button Mapping Reference

### Standard Face Buttons
- `BTN_SOUTH` / `BTN_A` = A button
- `BTN_EAST` / `BTN_B` = B button
- `BTN_WEST` / `BTN_X` = X button
- `BTN_NORTH` / `BTN_Y` = Y button

### System Buttons
- `BTN_START` = Start button (engine toggle)
- `BTN_SELECT` = Select button (double-click shutdown)

### Bumpers & Triggers
- `BTN_TL` = Left Trigger / Left Bumper (LB)
- `BTN_TR` = Right Trigger / Right Bumper (RB)

### Analog Sticks
- `ABS_Y` = Left stick vertical axis (0-255, center ~128)
- `ABS_Z` = Right stick horizontal axis (0-255, center ~128)

---

## Safety Features

1. **Emergency Stop** (Keyboard): Press SPACE key in terminal to set throttle to 0%
2. **Engine Cutoff** (Gamepad): SELECT button (double-click) or START button
3. **Both Bumpers Disabled State**: If both LB and RB are held but not pressed together, autopilot won't toggle
4. **Emergency Brake Override**: Always functional when autopilot is active for immediate stopping

---

## Testing

### Test Gamepad Connection
```bash
python gamepad.py
```

### Verify Button Events
```bash
# In terminal, check for button events
cat /dev/input/event*
```

### Expected Test Output for SELECT Button
```
Event Code: BTN_SELECT      | State: 1  (Button pressed)
Event Code: MSC_SCAN        | State: 589835
Event Code: BTN_SELECT      | State: 0  (Button released)
```

### Expected Test Output for LB + RB
```
Event Code: BTN_TL          | State: 1  (LB pressed)
Event Code: BTN_TR          | State: 1  (RB pressed)
→ Autopilot toggled!
```

---

## Troubleshooting

### SELECT Button Not Working
- Verify button code is `BTN_SELECT` in device logs
- Check double-click timeout setting (default: 0.5 seconds)
- Ensure second click occurs within timeout window

### Autopilot Not Toggling
- Confirm both LB and RB codes are received (`BTN_TL` and `BTN_TR`)
- Both buttons must be pressed simultaneously (near same timestamp)
- Try adjusting timing tolerance if needed

### Emergency Brake Not Activating
- Verify autopilot is actually enabled (check UI status)
- Check X button code (`BTN_WEST` or `BTN_X`)
- Ensure X button is held down (not a quick tap)

---

## Configuration Parameters

Edit `gamepad.py` to customize:

```python
# Double-click timeout for SELECT button (seconds)
btn_select_timeout = 0.5

# Joy stick dead zone (how much center drift to ignore)
JOY_DEADZONE = 15

# Gear speeds
GEAR_1_MAX = 35.0
GEAR_2_MAX = 60.0
GEAR_3_MAX = 80.0
SPORT_MAX = 100.0
```

---

## Future Enhancements

- [ ] Adjustable double-click timeout via UI
- [ ] Custom button remapping configuration file
- [ ] Haptic feedback on button state changes
- [ ] Autopilot telemetry logging
- [ ] Emergency brake force calibration

