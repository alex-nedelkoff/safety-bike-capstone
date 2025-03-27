import time
import sys
import busio
from adafruit_extended_bus import ExtendedI2C
import adafruit_vl53l0x

class TOF_Sensor:
    def __init__(self):
        print("Initializing VL53L0X sensor...")
        
        try:
            # Initialize I2C bus using the system bus number
            i2c = ExtendedI2C(1)  # /dev/i2c-1
            print("I2C initialized")
            
            # Create the VL53L0X object
            self.sensor = adafruit_vl53l0x.VL53L0X(i2c)
            
            # Configure for better accuracy
            # Increase timing budget to 200ms (default is 33ms)
            self.sensor.measurement_timing_budget = 200000
            
            print("VL53L0X initialized successfully")
            
        except Exception as e:
            print(f"Failed to initialize VL53L0X: {e}")
            print("\nTroubleshooting tips:")
            print("1. Check your wiring:")
            print("   - SDA → GPIO2 (Pin 3)")
            print("   - SCL → GPIO3 (Pin 5)")
            print("   - VCC → 3.3V")
            print("   - GND → Ground")
            print("2. Make sure the sensor is powered (3.3V)")
            print("3. Check for loose connections")
            print("4. Try running 'sudo i2cdetect -y 1' to see if device is detected")
            print("5. Ensure the Hailo AI Hat+ isn't conflicting with the I2C bus")
            sys.exit(1)

    def get_distance(self):
        try:
            # Get distance measurement in mm
            return self.sensor.range
        except Exception as e:
            print(f"Error reading distance: {e}")
            raise

    def cleanup(self):
        try:
            self.sensor.stop_ranging()
            print("Sensor stopped")
        except:
            pass

def main():
    try:
        # Initialize sensor
        tof = TOF_Sensor()
        print("\nReading distance measurements...")
        print("Press Ctrl+C to stop")
        
        # Variables for simple moving average
        window_size = 5
        measurements = []
        
        while True:
            try:
                # Get distance measurement
                distance_mm = tof.get_distance()
                
                # Add to moving average window
                measurements.append(distance_mm)
                if len(measurements) > window_size:
                    measurements.pop(0)
                
                # Calculate average
                avg_distance = sum(measurements) / len(measurements)
                
                # Print measurements
                print("\n" + "="*40)
                print(f"Distance: {distance_mm:>4d} mm ({distance_mm/1000:.2f} m)")
                print(f"Average:  {avg_distance:>4.1f} mm ({avg_distance/1000:.2f} m)")
                
                # Wait a bit between measurements
                time.sleep(0.1)  # 10Hz update rate
                
            except Exception as e:
                print(f"Error in main loop: {e}")
                break

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if 'tof' in locals():
            tof.cleanup()

if __name__ == "__main__":
    main() 