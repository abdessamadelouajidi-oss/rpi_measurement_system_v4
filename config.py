"""Configuration settings for the measurement system."""

# Sensor configuration
ACCELEROMETER_I2C_ADDRESS = 0x1C  # Default MMA8452 I2C address
TOF_ENABLED = True  # VL53L0X time-of-flight sensor
TOF_I2C_ADDRESS = 0x29  # Default VL53L0X address

# Hall sensor (spin counter)
HALL_ENABLED = True  # Set False to disable spin counting
HALL_SENSOR_PIN = 22  # GPIO pin for Hall sensor input
HALL_PULL_UP = True  # Use pull-up resistor if sensor is open-collector
HALL_EDGE = "FALLING"  # "FALLING", "RISING", or "BOTH"
HALL_DEBOUNCE_MS = 10  # GPIO debounce time in milliseconds
HALL_MIN_INTERVAL_S = 0.0  # Optional extra software debounce in seconds

# Measurement settings
READING_INTERVAL = 1.0  # Read vibration every 1.0 second while measuring

# CSV output
CSV_OUTPUT_PATH = "measurements.csv"  # Saved after shutdown

# USB copy settings
USB_COPY_ANY = True  # Copy to all mounted USB drives found under /media or /run/media
USB_CHECK_INTERVAL = 1.0  # Seconds between USB mount checks
