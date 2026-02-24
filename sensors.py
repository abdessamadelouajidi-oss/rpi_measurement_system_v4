import smbus
import time
import threading
from abc import ABC, abstractmethod


class Sensor(ABC):
    @abstractmethod
    def read(self):
        raise NotImplementedError


class Accelerometer(Sensor):
    WHO_AM_I = 0x0D
    CTRL_REG1 = 0x2A
    XYZ_DATA_CFG = 0x0E
    OUT_X_MSB = 0x01

    def __init__(self, i2c_address=0x1D, bus=1, auto_detect=True):
        self.bus = bus
        self.i2c_address = i2c_address

        try:
            self.i2c = smbus.SMBus(bus)

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
        ctrl = self.i2c.read_byte_data(self.i2c_address, self.CTRL_REG1)
        self.i2c.write_byte_data(self.i2c_address, self.CTRL_REG1, ctrl & ~0x01)
        time.sleep(0.05)

    def _active(self):
        ctrl = self.i2c.read_byte_data(self.i2c_address, self.CTRL_REG1)
        self.i2c.write_byte_data(self.i2c_address, self.CTRL_REG1, ctrl | 0x01)
        time.sleep(0.1)

    def _set_range_8g(self):
        self.i2c.write_byte_data(self.i2c_address, self.XYZ_DATA_CFG, 0x02)
        time.sleep(0.05)

    def read(self):
        if self.i2c is None:
            return {"x": 999, "y": 999, "z": 999}
        
        try:
             """
            data = self.i2c.read_i2c_block_data(self.i2c_address, self.OUT_X_MSB, 6)

            x_raw = self._convert_14bit(data[0], data[1])
            y_raw = self._convert_14bit(data[2], data[3])
            z_raw = self._convert_14bit(data[4], data[5])

            x = (x_raw / 1024.0) * 9.81
            y = (y_raw / 1024.0) * 9.81
            z = (z_raw / 1024.0) * 9.81
             """
             x=y=z=0.0
            return {"x": round(x, 2), "y": round(y, 2), "z": round(z, 2)}
        except Exception:
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
    Threaded polling hall sensor.
    Counts ONLY ONE per "interaction":
      - counts on HIGH -> LOW
      - then locks until signal returns to HIGH and stays stable for a few samples
    """

    def __init__(
        self,
        pin,
        pull_up=True,
        name="HALL_SENSOR",
        poll_hz=800,
        stable_samples=5,   # how many consecutive samples must be HIGH before re-arming
    ):
        self.pin = int(pin)
        self.pull_up = bool(pull_up)
        self.name = name
        self.poll_hz = int(poll_hz)
        self.stable_samples = max(1, int(stable_samples))

        self.GPIO = None
        self._count = 0
        self._lock = threading.Lock()

        self._stop = threading.Event()
        self._thread = None

        try:
            import RPi.GPIO as GPIO
            self.GPIO = GPIO

            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)

            try:
                GPIO.cleanup(self.pin)
            except Exception:
                pass

            pull = GPIO.PUD_UP if self.pull_up else GPIO.PUD_DOWN
            GPIO.setup(self.pin, GPIO.IN, pull_up_down=pull)

            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

            print(
                f"[{self.name}] Polling GPIO {self.pin} at ~{self.poll_hz} Hz "
                f"(one-count-per-interaction, stable_samples={self.stable_samples})"
            )

        except ImportError:
            print(f"[{self.name}] Warning: RPi.GPIO not available, using simulated mode")
        except Exception as e:
            print(f"[{self.name}] Warning: Could not initialize - {type(e).__name__}: {e}")

    def _run(self):
        period = .0 / self.poll_hz if self.poll_hz > 0 else 0.001

        # Armed means "ready to count the next HIGH->LOW"
        armed = True
        high_streak = 0

        last = self.GPIO.input(self.pin)

        while not self._stop.is_set():
            cur = self.GPIO.input(self.pin)

            if armed:
                # Count one on HIGH -> LOW transition
                if last == 1 and cur == 0:
                    with self._lock:
                        self._count += 1
                    armed = False
                    high_streak = 0
            else:
                # Not armed: wait until it returns HIGH and stays HIGH for N samples
                if cur == 1:
                    high_streak += 1
                    if high_streak >= self.stable_samples:
                        armed = True
                        high_streak = 0
                else:
                    high_streak = 0

            last = cur
            time.sleep(period)

    def  get_count(self):
        with self._lock:
            return self._count

    def reset_count(self):
        with self._lock:
            self._count = 0

    def cleanup(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

        if self.GPIO is None:
            return
        try:
            self.GPIO.cleanup(self.pin)
        except Exception:
            pass


class ToFSensor(Sensor):
    def __init__(self, i2c_address=0x29):
        self._device = None
        self.i2c_address = i2c_address

        try:
            import board
            import busio
            import adafruit_vl53l0x

            i2c = busio.I2C(board.SCL, board.SDA)
            self._device = adafruit_vl53l0x.VL53L0X(i2c)

            print(f"[TOF] Initialized VL53L0X on I2C (0x{self.i2c_address:02X})")

        except Exception as e:
            print(f"[TOF] Warning: Could not initialize - {e}")
            print("[TOF] Using simulated mode")
            self._device = None

    def read(self):
        if self._device is None:
            return {"distance_mm": 0.0}
        try:
            return {"distance_mm": float(self._device.range)}
        except Exception:
            return {"distance_mm": 0.0}
