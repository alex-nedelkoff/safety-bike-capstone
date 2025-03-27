import smbus
import time
import math
import sys

class MPU6050:
    def __init__(self, bus_num=1, address=0x68):
        print(f"Initializing MPU6050 on bus {bus_num} at address 0x{address:02X}")
        try:
            self.bus = smbus.SMBus(bus_num)
            print("I2C bus opened successfully")
        except Exception as e:
            print(f"Failed to open I2C bus: {e}")
            sys.exit(1)
            
        self.address = address
        
        # Check if device is responding
        try:
            # Try to read WHO_AM_I register (should return 0x68)
            who_am_i = self.bus.read_byte_data(self.address, 0x75)
            print(f"WHO_AM_I register returned: 0x{who_am_i:02X}")
            if who_am_i != 0x68:
                print(f"Warning: Unexpected WHO_AM_I value. Expected 0x68, got 0x{who_am_i:02X}")
        except Exception as e:
            print(f"Failed to read WHO_AM_I register: {e}")
            print("\nTroubleshooting tips:")
            print("1. Check your wiring:")
            print("   - SDA → GPIO2 (Pin 3)")
            print("   - SCL → GPIO3 (Pin 5)")
            print("   - VCC → 3.3V")
            print("   - GND → Ground")
            print("2. Make sure the MPU6050 is powered (3.3V)")
            print("3. Check for loose connections")
            print("4. Try running 'sudo i2cdetect -y 1' to see if device is detected")
            sys.exit(1)
        
        # Wake up the MPU6050
        try:
            print("Attempting to wake up MPU6050...")
            self.bus.write_byte_data(self.address, 0x6B, 0)
            print("MPU6050 initialized successfully")
        except Exception as e:
            print(f"Failed to wake up MPU6050: {e}")
            sys.exit(1)

    def read_raw_data(self, addr):
        try:
            # Read raw 16-bit value
            high = self.bus.read_byte_data(self.address, addr)
            low = self.bus.read_byte_data(self.address, addr + 1)

            # Combine high and low for 16-bit value
            value = (high << 8) + low

            # Get signed value
            if value > 32768:
                value = value - 65536
            return value
        except Exception as e:
            print(f"Error reading from address 0x{addr:02X}: {e}")
            raise

    def get_data(self):
        try:
            # Read accelerometer data
            acc_x = self.read_raw_data(0x3B) / 16384.0  # Full scale range ±2g
            acc_y = self.read_raw_data(0x3D) / 16384.0
            acc_z = self.read_raw_data(0x3F) / 16384.0

            # Read gyroscope data
            gyro_x = self.read_raw_data(0x43) / 131.0  # Full scale range ±250°/s
            gyro_y = self.read_raw_data(0x45) / 131.0
            gyro_z = self.read_raw_data(0x47) / 131.0

            # Calculate roll and pitch (in degrees)
            roll = math.atan2(acc_y, acc_z) * 180/math.pi
            pitch = math.atan2(-acc_x, math.sqrt(acc_y*acc_y + acc_z*acc_z)) * 180/math.pi

            return {
                'acceleration': {'x': acc_x, 'y': acc_y, 'z': acc_z},
                'gyroscope': {'x': gyro_x, 'y': gyro_y, 'z': gyro_z},
                'angles': {'roll': roll, 'pitch': pitch}
            }
        except Exception as e:
            print(f"Error getting sensor data: {e}")
            raise

def main():
    try:
        mpu = MPU6050()
        print("\nReading data from MPU6050...")
        print("Press Ctrl+C to stop")
        
        while True:
            try:
                data = mpu.get_data()
                
                print("\n" + "="*50)
                print("Acceleration (g):")
                print(f"X: {data['acceleration']['x']:>8.3f}")
                print(f"Y: {data['acceleration']['y']:>8.3f}")
                print(f"Z: {data['acceleration']['z']:>8.3f}")
                
                print("\nGyroscope (°/s):")
                print(f"X: {data['gyroscope']['x']:>8.3f}")
                print(f"Y: {data['gyroscope']['y']:>8.3f}")
                print(f"Z: {data['gyroscope']['z']:>8.3f}")
                
                print("\nOrientation (°):")
                print(f"Roll:  {data['angles']['roll']:>8.3f}")
                print(f"Pitch: {data['angles']['pitch']:>8.3f}")
                
                time.sleep(0.1)  # Read at 10Hz
            except Exception as e:
                print(f"Error in main loop: {e}")
                break

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if 'mpu' in locals():
            try:
                mpu.bus.close()
                print("I2C bus closed")
            except:
                pass

if __name__ == "__main__":
    main() 