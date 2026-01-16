"""Sensor modules for accelerometer and distance sensor."""

import smbus
import time
from abc import ABC, abstractmethod


class Sensor(ABC):
    """Base class for sensors."""
    
    @abstractmethod
    def read(self):
        """Read sensor data and return it."""
        pass


class Accelerometer(Sensor):
    """
    Simple I2C accelerometer (MMA8452).
    
    Provides x, y, z acceleration values.
    """
    
    def __init__(self, i2c_address=0x1D, bus=1):
        """
        Initialize the accelerometer.
        
        Args:
            i2c_address: I2C address of the accelerometer (default: 0x1C for MMA8452)
            bus: I2C bus number (default: 1 for Raspberry Pi)
        """
        self.i2c_address = i2c_address
        self.bus = bus
        
        try:
            self.i2c = smbus.SMBus(bus)
            
            # Step 1: Put sensor in STANDBY mode (clear ACTIVE bit)
            self.i2c.write_byte_data(i2c_address, 0x2A, ctrl_reg1 & 0xFE)  # Clear bit 0
            
            
            # Step 2: Set range to ±8g (register 0x0E, bits 0-1 = 10 for ±8g)
            ctrl_reg2 = self.i2c.read_byte_data(i2c_address, 0x0E)
            self.i2c.write_byte_data(i2c_address, 0x0E, (ctrl_reg2 & 0xFC) | 0x02)  # Set bits 0-1 to 10 for ±8g
            time.sleep(0.1)
            
            # Step 3: Put sensor in ACTIVE mode (set ACTIVE bit)
            ctrl_reg1 = self.i2c.read_byte_data(i2c_address, 0x2A)
            self.i2c.write_byte_data(i2c_address, 0x2A, ctrl_reg1 | 0x01)  # Set bit 0
            time.sleep(0.1)
            
            print("[ACCELEROMETER] Initialized on I2C address 0x{:02X}".format(i2c_address))
        except Exception as e:
            print(f"[ACCELEROMETER] Warning: Could not initialize - {e}")
            print("[ACCELEROMETER] Using simulated mode")
            self.i2c = None
    
    def read(self):
        """
        Read acceleration from MMA8452.
        
        Returns:
            dict with 'x', 'y', 'z' keys containing acceleration values in m/s²
        """
        if self.i2c is None:
            # Simulated data when sensor not available
            return {
                'x': 0.5,
                'y': -0.3,
                'z': 9.8
            }
        
        try:
            # Read accel data from registers 0x01 to 0x06 (6 bytes)
            # Registers: OUT_X_MSB (0x01), OUT_X_LSB (0x02), OUT_Y_MSB (0x03), 
            #            OUT_Y_LSB (0x04), OUT_Z_MSB (0x05), OUT_Z_LSB (0x06)
            accel_data = self.i2c.read_i2c_block_data(self.i2c_address, 0x00, 7)
            
            # MMA8452 data format: 14-bit signed, right-justified in 16-bit register
            # Extract 14-bit values (ignore lower 2 bits)
            x = (accel_data[0] << 8 | accel_data[1]) >> 2
            y = (accel_data[2] << 8 | accel_data[3]) >> 2
            z = (accel_data[4] << 8 | accel_data[5]) >> 2
            
            # Convert to signed 14-bit
            if x > 8191:  # 2^13 - 1
                x -= 16384
            if y > 8191:
                y -= 16384
            if z > 8191:
                z -= 16384
            
            # Convert to m/s² (±8g range: 1g = 9.81 m/s², so 1 unit = 9.81/1024)
            x = (x / 1024.0) * 9.81
            y = (y / 1024.0) * 9.81
            z = (z / 1024.0) * 9.81
            
            return {
                'x': round(x, 2),
                'y': round(y, 2),
                'z': round(z, 2)
            }
        except Exception as e:
            print(f"[ACCELEROMETER] Read error: {e}")
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}



