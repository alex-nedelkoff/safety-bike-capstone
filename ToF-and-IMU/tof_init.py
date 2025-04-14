import time
import board
import busio
import digitalio
from adafruit_vl53l0x import VL53L0X

# Initialize I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Setup XSHUT pins for both sensors
xshut_1 = digitalio.DigitalInOut(board.D17)  # GPIO17 → Sensor 1 XSHUT
xshut_2 = digitalio.DigitalInOut(board.D27)  # GPIO27 → Sensor 2 XSHUT
xshut_1.direction = digitalio.Direction.OUTPUT
xshut_2.direction = digitalio.Direction.OUTPUT

# Step 1: Shutdown both sensors
xshut_1.value = False
xshut_2.value = False
time.sleep(0.1)

# Step 2: Power up first sensor only and assign new address
xshut_1.value = True
time.sleep(0.1)
sensor1 = VL53L0X(i2c)
sensor1.set_address(0x30)  # Assign sensor 1 to 0x30

# Step 3: Power up second sensor and assign different address
xshut_2.value = True
time.sleep(0.1)
sensor2 = VL53L0X(i2c)
sensor2.set_address(0x31)  # Assign sensor 2 to 0x31

print(" Sensors initialized and assigned to 0x30 and 0x31.")