"""Sensor modules for accelerometer."""

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
            i2c_address: I2C address of the accelerometer (default: 0x1D for MMA8452)
            bus: I2C bus number (default: 1 for Raspberry Pi)
        """
        self.i2c_address = i2c_address
        self.bus = bus
        
        try:
            self.i2c = smbus.SMBus(bus)
            
            # Try to read device ID first to verify communication
            try:
                device_id = self.i2c.read_byte_data(i2c_address, 0x0D)
                print(f"[ACCELEROMETER] Device ID: 0x{device_id:02X}")
            except Exception as e:
                print(f"[ACCELEROMETER] Warning: Could not read device ID - {e}")
            
            # Put sensor in STANDBY mode
            try:
                self.i2c.write_byte_data(i2c_address, 0x2A, 0x00)
                time.sleep(0.1)
                print("[ACCELEROMETER] Set to STANDBY mode")
            except Exception as e:
                print(f"[ACCELEROMETER] Error setting STANDBY: {e}")
            
            # Set range to ±8g (0x02 for ±8g)
            try:
                self.i2c.write_byte_data(i2c_address, 0x0E, 0x02)
                time.sleep(0.1)
                print("[ACCELEROMETER] Set range to ±8g")
            except Exception as e:
                print(f"[ACCELEROMETER] Error setting range: {e}")
            
            # Put sensor in ACTIVE mode
            try:
                self.i2c.write_byte_data(i2c_address, 0x2A, 0x01)
                time.sleep(0.5)
                print("[ACCELEROMETER] Set to ACTIVE mode")
            except Exception as e:
                print(f"[ACCELEROMETER] Error setting ACTIVE: {e}")
            
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
            # Read data back from 0x00 (Status register + 6 acceleration bytes)
            data = self.i2c.read_i2c_block_data(self.i2c_address, 0x00, 7)
            
            # Extract X, Y, Z acceleration values
            # data[0] = Status register
            # data[1:3] = X-Axis (MSB, LSB)
            # data[3:5] = Y-Axis (MSB, LSB)
            # data[5:7] = Z-Axis (MSB, LSB)
            
            # Convert raw values to acceleration
            # Division by 16 converts 14-bit data to 10-bit representation
            x_raw = (data[1] * 256 + data[2]) / 16
            y_raw = (data[3] * 256 + data[4]) / 16
            z_raw = (data[5] * 256 + data[6]) / 16
            
            # Convert to signed values
            if x_raw > 2047:
                x_raw -= 4096
            if y_raw > 2047:
                y_raw -= 4096
            if z_raw > 2047:
                z_raw -= 4096
            
            # Convert to m/s² 
            # For ±8g range: sensitivity is 256 counts/g (after division by 16)
            # So 1 count = 1/256 g = (9.81/256) m/s² = 0.0383 m/s²
            x = (x_raw / 256.0) * 9.81
            y = (y_raw / 256.0) * 9.81
            z = (z_raw / 256.0) * 9.81
            
            return {
                'x': round(x, 2),
                'y': round(y, 2),
                'z': round(z, 2)
            }
        except Exception as e:
            print(f"[ACCELEROMETER] Read error: {e}")
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}



