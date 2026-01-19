import smbus
import time
from abc import ABC, abstractmethod


class Sensor(ABC):
    """Base class for sensors."""

    @abstractmethod
    def read(self):
        """Read sensor data and return it."""
        raise NotImplementedError


class Accelerometer(Sensor):
    """
    Simple I2C accelerometer (MMA8452 / MMA845x family).

    Provides x, y, z acceleration values.
    """

    # Registers
    WHO_AM_I = 0x0D
    CTRL_REG1 = 0x2A
    XYZ_DATA_CFG = 0x0E
    OUT_X_MSB = 0x01  # start of XYZ data

    def __init__(self, i2c_address=0x1D, bus=1, auto_detect=True):
        """
        Initialize the accelerometer.

        Args:
            i2c_address: I2C address (default 0x1D; some boards use 0x1C)
            bus: I2C bus number (default 1 for Raspberry Pi)
            auto_detect: if True, will try 0x1D then 0x1C automatically
        """
        self.bus = bus
        self.i2c_address = i2c_address

        try:
            self.i2c = smbus.SMBus(bus)

            # --- Address auto-detect (0x1D vs 0x1C) ---
            if auto_detect:
                found = None
                for addr in (i2c_address, 0x1D, 0x1C):
                    try:
                        who = self.i2c.read_byte_data(addr, self.WHO_AM_I)
                        # MMA845x WHO_AM_I is commonly 0x2A for MMA8452Q.
                        # But we accept any successful read as "device responds".
                        print(f"[ACCELEROMETER] WHO_AM_I at 0x{addr:02X} = 0x{who:02X}")
                        found = addr
                        break
                    except OSError:
                        continue

                if found is None:
                    raise OSError("No MMA845x device found at 0x1D or 0x1C")
                self.i2c_address = found
            else:
                who = self.i2c.read_byte_data(self.i2c_address, self.WHO_AM_I)
                print(f"[ACCELEROMETER] WHO_AM_I at 0x{self.i2c_address:02X} = 0x{who:02X}")

            # --- Configure device safely (read-modify-write) ---
            self._standby()
            self._set_range_8g()
            self._active()

            print(f"[ACCELEROMETER] Initialized on bus {bus}, address 0x{self.i2c_address:02X}")

        except Exception as e:
            print(f"[ACCELEROMETER] Warning: Could not initialize - {e}")
            print("[ACCELEROMETER] Using simulated mode")
            self.i2c = None

    def _standby(self):
        """Put sensor in STANDBY mode (clear ACTIVE bit only)."""
        try:
            ctrl = self.i2c.read_byte_data(self.i2c_address, self.CTRL_REG1)
            self.i2c.write_byte_data(self.i2c_address, self.CTRL_REG1, ctrl & ~0x01)
            time.sleep(0.05)
            print("[ACCELEROMETER] Set to STANDBY mode")
        except OSError as e:
            raise OSError(f"STANDBY failed (errno={getattr(e,'errno',None)}): {e}") from e

    def _active(self):
        """Put sensor in ACTIVE mode (set ACTIVE bit only)."""
        try:
            ctrl = self.i2c.read_byte_data(self.i2c_address, self.CTRL_REG1)
            self.i2c.write_byte_data(self.i2c_address, self.CTRL_REG1, ctrl | 0x01)
            time.sleep(0.1)
            print("[ACCELEROMETER] Set to ACTIVE mode")
        except OSError as e:
            raise OSError(f"ACTIVE failed (errno={getattr(e,'errno',None)}): {e}") from e

    def _set_range_8g(self):
        """Set accelerometer range to ±8g."""
        try:
            self.i2c.write_byte_data(self.i2c_address, self.XYZ_DATA_CFG, 0x02)
            time.sleep(0.05)
            print("[ACCELEROMETER] Set range to ±8g")
        except OSError as e:
            raise OSError(f"Set range failed (errno={getattr(e,'errno',None)}): {e}") from e

    def read(self):
        """
        Read acceleration.

        Returns:
            dict with 'x', 'y', 'z' keys containing acceleration values in m/s²
        """
        if self.i2c is None:
            return {'x': 0.5, 'y': -0.3, 'z': 9.8}

        try:
            # Read 6 bytes: X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB
            data = self.i2c.read_i2c_block_data(self.i2c_address, self.OUT_X_MSB, 6)

            x_raw = self._convert_14bit(data[0], data[1])
            y_raw = self._convert_14bit(data[2], data[3])
            z_raw = self._convert_14bit(data[4], data[5])

            # For ±8g, sensitivity is 1024 counts/g for 14-bit mode
            # (device outputs 14-bit value; 2^14 = 16384 full-scale -> ±8g)
            # => counts per g = 16384 / 16g = 1024
            x = (x_raw / 1024.0) * 9.81
            y = (y_raw / 1024.0) * 9.81
            z = (z_raw / 1024.0) * 9.81

            return {'x': round(x, 2), 'y': round(y, 2), 'z': round(z, 2)}

        except OSError as e:
            # Errno 121 usually = wrong address/wiring/device not responding
            print(f"[ACCELEROMETER] Read OSError (errno={getattr(e,'errno',None)}): {e}")
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}
        except Exception as e:
            print(f"[ACCELEROMETER] Read error: {e}")
            return {'x': 0.0, 'y': 0.0, 'z': 0.0}

    @staticmethod
    def _convert_14bit(msb, lsb):
        """Convert MMA845x 14-bit left-justified value to signed int."""
        raw = (msb << 8) | lsb
        raw >>= 2  # bottom 2 bits are not part of the 14-bit value
        if raw > 8191:
            raw -= 16384
        return raw


class ToFSensor(Sensor):
    """
    Time-of-flight distance sensor using Adafruit CircuitPython drivers.

    Supports VL53L0X via adafruit-circuitpython-vl53l0x.
    """

    def __init__(self, i2c_address=0x29):
        """Initialize the ToF sensor."""
        self._device = None
        self.i2c_address = i2c_address

        try:
            import board
            import busio
            import adafruit_vl53l0x

            i2c = busio.I2C(board.SCL, board.SDA)
            self._device = adafruit_vl53l0x.VL53L0X(i2c)
            if self.i2c_address != 0x29:
                if hasattr(self._device, "set_address"):
                    self._device.set_address(self.i2c_address)
                else:
                    print("[TOF] Warning: driver does not support set_address")
            print(f"[TOF] Initialized VL53L0X on I2C (0x{self.i2c_address:02X})")
        except Exception as e:
            print(f"[TOF] Warning: Could not initialize - {e}")
            print("[TOF] Using simulated mode")
            self._device = None

    def read(self):
        """
        Read distance in millimeters.

        Returns:
            dict with 'distance_mm' key
        """
        if self._device is None:
            return {"distance_mm": 0.0}

        try:
            distance = self._device.range
            return {"distance_mm": float(distance)}
        except Exception as e:
            print(f"[TOF] Read error: {e}")
            return {"distance_mm": 0.0}
