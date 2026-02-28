import time
import board
import busio
import adafruit_vl53l0x

# Create I2C bus
i2c = busio.I2C(board.SCL, board.SDA)

# Create sensor object
tof = adafruit_vl53l0x.VL53L0X(i2c)

print("VL53L0X ToF sensor test")
print("----------------------")

while True:
    distance_mm = tof.range
    print(f"Distance: {distance_mm} mm")
    time.sleep(0.5)
