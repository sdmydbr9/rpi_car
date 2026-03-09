# ADS1115 Quick Setup Guide

## Hardware Wiring

**ADS1115 Breakout Board → Pico W:**

```
ADS1115        Pico W
─────────      ──────
GND     ───────  pin 3 (GND)
VDD     ───────  pin 4 (3V3)
SDA     ───────  pin 6 (GPIO4)
SCL     ───────  pin 7 (GPIO5)
ADDR    ───────  GND (or VDD/SDA/SCL based on desired address)
```

## I2C Address Configuration

The ADDR pin determines the I2C address:

| ADDR Connection | I2C Address | Config Value |
|-----------------|-------------|--------------|
| To GND | 0x48 | `ADS_ADDR = 0x48` |
| To VDD (3V3) | 0x49 | `ADS_ADDR = 0x49` |
| To SDA | 0x4A | `ADS_ADDR = 0x4A` |
| To SCL | 0x4B | `ADS_ADDR = 0x4B` |

**⚠️ Most common:** ADDR pin is usually tied to GND on breakout boards → **address is 0x48**

## Verify Detection

After uploading `pico_sensor_bridge.py` to Pico, check Thonny terminal output:

```
✅ I2C devices found: ['0x68', '0x29', '0x48']
✅ ADS1115 initialized (I2C 0x48)
```

If not found, check your ADDR pin and update the code.

## Using ADC Data on Pi

### Example 1: Battery Voltage Monitoring

```python
from pico_sensor_reader import init_pico_reader, get_battery_voltage

init_pico_reader()

# In your loop:
battery_v = get_battery_voltage(divider_ratio=2.93)  # For 12V max
print(f"Battery: {battery_v:.1f} V")

# Calculate divider ratio:
# ratio = max_battery_voltage / max_adc_voltage
# For 12V battery: 12V / 4.096V ≈ 2.93
```

### Example 2: Current Sense from Shunt

```python
from pico_sensor_reader import get_current_sense

# In your loop:
current_a = get_current_sense(shunt_ohms=0.1)  # 100mΩ shunt resistor
print(f"Current: {current_a:.2f} A")

# Shunt value depends on your resistor:
# I = V / R
# For 0.1Ω shunt: 500 mV = 5 A
```

### Example 3: Raw ADC Reading

```python
from pico_sensor_reader import get_adc_voltage_mv

# Get raw voltage from any channel
ch_a0_mv = get_adc_voltage_mv('A0')  # A0, A1, A2, or A3
print(f"ADC A0: {ch_a0_mv} mV")
```

### Example 4: Direct Packet Access

```python
from pico_sensor_reader import get_sensor_packet

packet = get_sensor_packet()
if packet:
    # Access all 4 ADC channels
    print(f"A0: {packet.adc_a0} mV")
    print(f"A1: {packet.adc_a1} mV")
    print(f"A2: {packet.adc_a2} mV")
    print(f"A3: {packet.adc_a3} mV")
```

## Common Issues

### ADC reads -1.0 mV
- **Cause:** ADS1115 not responding (wrong address or not connected)
- **Fix:** Check I2C wiring and ADDR pin configuration
- **Debug:** Run Pico in Thonny, check startup output for address

### ADC values noisy/unstable
- **Cause:** Analog noise, long cables, or fast state changes
- **Fix:** 
  - Use shielded cables for analog inputs
  - Add 100nF capacitor near analog input pins
  - Filter in software: average last 3-5 readings
  - Keep analog wires away from motor power cables

### Wrong voltage scale
- **Cause:** Divider ratio not configured correctly
- **Fix:** 
  - Measure actual divider resistor values
  - Calculate: `ratio = max_battery_V / 4.096`
  - Update function call: `get_battery_voltage(divider_ratio=YOUR_RATIO)`

## Typical Configuration

For a **12V battery with voltage divider and 100mΩ current shunt:**

```python
# Voltage divider to keep max 12V within ±4.096V range
battery_voltage = get_battery_voltage(divider_ratio=2.93)

# 100mΩ shunt resistor (common for current sensing)
motor_current = get_current_sense(shunt_ohms=0.1)

print(f"Battery: {battery_voltage:.1f} V | Current: {motor_current:.2f} A")
```

## Specifications

| Parameter | Value |
|-----------|-------|
| **Channels** | 4 (single-ended to GND) |
| **Range** | ±4.096 V (we use this range) |
| **Resolution** | 12-bit effective = 0.125 mV/LSB |
| **Sample Rate** | 128 SPS (8ms per channel) |
| **I2C Speed** | 100 kHz (standard, up to 400 kHz) |
| **I2C Addresses** | 0x48, 0x49, 0x4A, 0x4B (via ADDR pin) |

---

**Last Updated:** 20 Feb 2026
