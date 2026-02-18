import smbus
import time
import threading
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

    WHO_AM_I = 0x0D
    CTRL_REG1 = 0x2A
    XYZ_DATA_CFG = 0x0E
    OUT_X_MSB = 0x01  # start of XYZ data

    def __init__(self, i2c_address=0x1D, bus=1, auto_detect=True):
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

            self._standby()
            self._set_range_8g()
            self._active()

            print(f"[ACCELEROMETER] Initialized on bus {bus}, address 0x{self.i2c_address:02X}")

        except Exception as e:
            print(f"[ACCELEROMETER] Warning: Could not initialize - {e}")
            print("[ACCELEROMETER] Using simulated mode")
            self.i2c = None

    def _standby(self):
        try:
            ctrl = self.i2c.read_byte_data(self.i2c_address, self.CTRL_REG1)
            self.i2c.write_byte_data(self.i2c_address, self.CTRL_REG1, ctrl & ~0x01)
            time.sleep(0.05)
            print("[ACCELEROMETER] Set to STANDBY mode")
        except OSError as e:
            raise OSError(f"STANDBY failed (errno={getattr(e,'errno',None)}): {e}") from e

    def _active(self):
        try:
            ctrl = self.i2c.read_byte_data(self.i2c_address, self.CTRL_REG1)
            self.i2c.write_byte_data(self.i2c_address, self.CTRL_REG1, ctrl | 0x01)
            time.sleep(0.1)
            print("[ACCELEROMETER] Set to ACTIVE mode")
        except OSError as e:
            raise OSError(f"ACTIVE failed (errno={getattr(e,'errno',None)}): {e}") from e

    def _set_range_8g(self):
        try:
            self.i2c.write_byte_data(self.i2c_address, self.XYZ_DATA_CFG, 0x02)
            time.sleep(0.05)
            print("[ACCELEROMETER] Set range to ±8g")
        except OSError as e:
            raise OSError(f"Set range failed (errno={getattr(e,'errno',None)}): {e}") from e

    def read(self):
        if self.i2c is None:
            return {"x": 0.5, "y": -0.3, "z": 9.8}

        try:
            data = self.i2c.read_i2c_block_data(self.i2c_address, self.OUT_X_MSB, 6)

            x_raw = self._convert_14bit(data[0], data[1])
            y_raw = self._convert_14bit(data[2], data[3])
            z_raw = self._convert_14bit(data[4], data[5])

            x = (x_raw / 1024.0) * 9.81
            y = (y_raw / 1024.0) * 9.81
            z = (z_raw / 1024.0) * 9.81

            return {"x": round(x, 2), "y": round(y, 2), "z": round(z, 2)}

        except OSError as e:
            print(f"[ACCELEROMETER] Read OSError (errno={getattr(e,'errno',None)}): {e}")
            return {"x": 0.0, "y": 0.0, "z": 0.0}
        except Exception as e:
            print(f"[ACCELEROMETER] Read error: {e}")
            return {"x": 0.0, "y": 0.0, "z": 0.0}

    @staticmethod
    def _convert_14bit(msb, lsb):
        raw = (msb << 8) | lsb
        raw >>= 2
        if raw > 8191:
            raw -= 16384
        return raw


class HallSensor:
    """
    Hall effect sensor for counting rotations.
    Your signal is 0->1 when magnet present, so we use GPIO.RISING.
    """

    def __init__(self, pin, pull_up=True, name="HALL_SENSOR"):
        self.pin = int(pin)
        self.pull_up = bool(pull_up)
        self.name = name

        self.GPIO = None
        self._count = 0
        self._lock = threading.Lock()

        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO

            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)

            # ---- KEY FIX for "failed to add edge detection" ----
            # Clear any stale event detection + reset this pin before re-adding.
            try:
                GPIO.remove_event_detect(self.pin)
            except Exception:
                pass
            try:
                GPIO.cleanup(self.pin)  # only this pin
            except Exception:
                pass
            # ----------------------------------------------------

            pull = GPIO.PUD_UP if self.pull_up else GPIO.PUD_DOWN
            GPIO.setup(self.pin, GPIO.IN, pull_up_down=pull)

            GPIO.add_event_detect(
                self.pin,
                GPIO.RISING,
                callback=self._on_detected,
            )

            print(f"[{self.name}] Initialized on GPIO {self.pin} (edge=RISING)")

        except ImportError:
            print(f"[{self.name}] Warning: RPi.GPIO not available, using simulated mode")
        except Exception as e:
            print(f"[{self.name}] Warning : Could not Initialize - {type(e).__name__}: {e}")

    def _on_detected(self, _channel=None):
        with self._lock:
            self._count += 1

    def get_count(self):
        with self._lock:
            return self._count

    def reset_count(self):
        with self._lock:
            self._count = 0

    def cleanup(self):
        if self.GPIO is None:
            return
        try:
            self.GPIO.remove_event_detect(self.pin)
        except Exception:
            pass
        try:
            self.GPIO.cleanup(self.pin)
        except Exception:
            pass


class ToFSensor(Sensor):
    """
    Time-of-flight distance sensor using Adafruit CircuitPython drivers.
    Supports VL53L0X via adafruit-circuitpython-vl53l0x.
    """

    def __init__(self, i2c_address=0x29):
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
        if self._device is None:
            return {"distance_mm": 0.0}

        try:
            distance = self._device.range
            return {"distance_mm": float(distance)}
        except Exception as e:
            print(f"[TOF] Read error: {e}")
            return {"distance_mm": 0.0}
