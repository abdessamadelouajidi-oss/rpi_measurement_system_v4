# Raspberry Pi Vibration Measurement System v4

A service-friendly, console-only vibration measurement system for Raspberry Pi. This version runs continuously (no buttons or LEDs) until the service is stopped.

## Features

- Continuous measurement at a fixed interval
- Accelerometer (I2C/MMA8452) vibration readings
- Optional VL53L0X ToF distance readings
- Optional Hall sensor spin counter (logged per sample)
- CSV export to `measurements.csv`
- Optional USB auto-copy when a drive is inserted
- Graceful shutdown on SIGTERM or Ctrl+C

## Project Structure

```
rpi_measurement_system_v4/
|- main.py              # Main application entry point
|- sensors.py           # Accelerometer, ToF, Hall sensor classes
|- config.py            # Configuration settings (pins, intervals)
|- state_machine.py     # Unused in v4 (left for reference)
|- buttons.py           # Unused in v4 (left for reference)
|- leds.py              # Unused in v4 (left for reference)
|- requirements.txt     # Python dependencies
`- README.md            # This file
```

## Hardware Configuration

### I2C Sensors
- Accelerometer (MMA8452): I2C address 0x1C
- ToF (VL53L0X): I2C address 0x29 (optional)

### Hall Sensor (Spin Counter)
- GPIO pin configured in `config.py` (default GPIO 22)
- Use pull-up if your Hall sensor is open-collector
- One magnet per revolution for 1 pulse per spin

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Update settings in `config.py` as needed (pins, intervals, optional sensors).

3. Run directly or via your service:
   ```bash
   python main.py
   ```

## Usage

- The program starts measuring immediately and runs until the service is stopped.
- To stop manually, press Ctrl+C; for systemd, stop the service normally.
- Data is saved to `measurements.csv` on shutdown and during USB copy events.

## Output Example

```
============================================================
Raspberry Pi Vibration Measurement System
============================================================

[ACCELEROMETER] Initialized on bus 1, address 0x1C
[TOF] Initialized VL53L0X on I2C (0x29)
[HALL_SENSOR] Initialized on GPIO 22 (edge=FALLING)

System started. Running continuously.
Stop the service or press Ctrl+C to stop and save.
------------------------------------------------------------

[2026-02-18 14:32:45] Vibration - X=+0.12m/s^2 Y=-0.08m/s^2 Z=+9.81m/s^2
[2026-02-18 14:32:46] Vibration - X=+0.11m/s^2 Y=-0.09m/s^2 Z=+9.80m/s^2
```

## Notes

- All pins and options are configurable in `config.py`
- The Hall sensor count is logged per row as `spin_count`
- If you see double counts, consider a stronger pull-up resistor or cleaner wiring.
- Press Ctrl+C to safely stop when running manually

## Dependencies

- `RPi.GPIO` - Raspberry Pi GPIO control
- `smbus-cffi` - I2C communication for sensors
- `adafruit-blinka` and `adafruit-circuitpython-vl53l0x` for ToF sensor support
