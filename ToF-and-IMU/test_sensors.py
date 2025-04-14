#!/usr/bin/env python3
import time
import smbus2

print("Testing sensors...")

# Initialize I2C
i2c = smbus2.SMBus(1)  # Use I2C bus 1 on Raspberry Pi

# Test MPU6050 first
print("\nTesting MPU6050...")
try:
    # MPU6050 registers
    PWR_MGMT_1 = 0x6B
    ACCEL_XOUT_H = 0x3B
    GYRO_XOUT_H = 0x43
    
    # Wake up MPU6050
    i2c.write_byte_data(0x68, PWR_MGMT_1, 0)
    
    # Read accelerometer data
    accel_data = i2c.read_i2c_block_data(0x68, ACCEL_XOUT_H, 6)
    accel_x = (accel_data[0] << 8) | accel_data[1]
    accel_y = (accel_data[2] << 8) | accel_data[3]
    accel_z = (accel_data[4] << 8) | accel_data[5]
    
    # Read gyroscope data
    gyro_data = i2c.read_i2c_block_data(0x68, GYRO_XOUT_H, 6)
    gyro_x = (gyro_data[0] << 8) | gyro_data[1]
    gyro_y = (gyro_data[2] << 8) | gyro_data[3]
    gyro_z = (gyro_data[4] << 8) | gyro_data[5]
    
    print("MPU6050 initialized successfully")
    print(f"Acceleration: X={accel_x}, Y={accel_y}, Z={accel_z}")
    print(f"Gyro: X={gyro_x}, Y={gyro_y}, Z={gyro_z}")
except Exception as e:
    print(f"MPU6050 error: {e}")

# Test VL53L0X
print("\nTesting VL53L0X...")
try:
    # Try to read the WHO_AM_I register first
    print("Reading WHO_AM_I register...")
    who_am_i = i2c.read_byte_data(0x29, 0xC0)
    print(f"WHO_AM_I register value: 0x{who_am_i:02X}")
    
    # Try to read some basic registers
    print("\nReading basic registers...")
    try:
        # Read SYSRANGE_START register
        sysrange_start = i2c.read_byte_data(0x29, 0x00)
        print(f"SYSRANGE_START: 0x{sysrange_start:02X}")
        
        # Read RESULT_RANGE_STATUS register
        range_status = i2c.read_byte_data(0x29, 0x14)
        print(f"RESULT_RANGE_STATUS: 0x{range_status:02X}")
        
        # Try to start ranging
        print("\nStarting ranging...")
        i2c.write_byte_data(0x29, 0x00, 0x01)  # Start ranging
        time.sleep(0.1)
        
        # Read distance
        distance = i2c.read_word_data(0x29, 0x14)
        print(f"Distance: {distance/1000.0:.2f}m")
        
    except Exception as e:
        print(f"Error reading registers: {e}")
        
except Exception as e:
    print(f"VL53L0X error: {e}")

print("\nTest complete") 