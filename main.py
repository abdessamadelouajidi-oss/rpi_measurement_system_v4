"""Main application loop for the Raspberry Pi vibration measurement system."""

import time
import sys
import csv
import os
import shutil
from state_machine import StateMachine
from sensors import Accelerometer, ToFSensor
from buttons import BeginButton, PowerButton
from leds import IdleLED, MeasuringLED, CopyLED
from config import (
    READING_INTERVAL,
    BEGIN_BUTTON_PIN,
    POWER_BUTTON_PIN,
    ACCELEROMETER_I2C_ADDRESS,
    TOF_ENABLED,
    TOF_I2C_ADDRESS,
    IDLE_LED_PIN,
    MEASURING_LED_PIN,
    MEASURING_LED_BLINK_INTERVAL,
    CSV_OUTPUT_PATH,
    USB_COPY_LED_PIN,
    USB_COPY_LED_BLINK_INTERVAL,
    USB_COPY_ANY,
    USB_CHECK_INTERVAL,
)


class MeasurementSystem:
    """Main vibration measurement system coordinator."""
    
    def __init__(self):
        """Initialize the measurement system."""
        print("=" * 60)
        print("Raspberry Pi Vibration Measurement System")
        print("=" * 60)
        print()
        
        # Initialize state machine
        self.state_machine = StateMachine()
        
        # Initialize sensor
        print("Initializing accelerometer...")
        self.accelerometer = Accelerometer(i2c_address=ACCELEROMETER_I2C_ADDRESS)
        print()

        self.tof = None
        if TOF_ENABLED:
            print("Initializing VL53L0X ToF sensor...")
            self.tof = ToFSensor(i2c_address=TOF_I2C_ADDRESS)
            print()
        
        # Initialize buttons
        print("Initializing buttons...")
        self.begin_button = BeginButton(pin=BEGIN_BUTTON_PIN)
        self.begin_button.set_callback(self.on_begin_button_pressed)
        
        self.power_button = PowerButton(pin=POWER_BUTTON_PIN)
        self.power_button.set_shutdown_callback(self.on_shutdown)
        print()
        
        # Initialize LEDs
        print("Initializing LEDs...")
        self.idle_led = IdleLED(pin=IDLE_LED_PIN)
        self.measuring_led = MeasuringLED(
            pin=MEASURING_LED_PIN,
            blink_interval=MEASURING_LED_BLINK_INTERVAL
        )
        self.idle_led.turn_on()  # Start with IDLE LED on
        self.usb_copy_led = CopyLED(
            pin=USB_COPY_LED_PIN,
            blink_interval=USB_COPY_LED_BLINK_INTERVAL
        )
        self.usb_copy_led.set_idle()
        print()
        
        self.running = True
        self.last_reading_time = 0
        self.readings = []
        self.csv_output_path = CSV_OUTPUT_PATH
        self.usb_copy_any = USB_COPY_ANY
        self.usb_seen_mounts = set()
        self.last_usb_check_time = 0
    
    def on_begin_button_pressed(self):
        """Handle BEGIN button press - toggle measurement state."""
        self.state_machine.toggle_measurement()
        
        # Update LED states
        if self.state_machine.is_measuring():
            self.idle_led.turn_off()
            # Don't turn on measuring LED here - it will blink in the main loop
        else:
            self.measuring_led.turn_off()
            self.idle_led.turn_on()
    
    def on_shutdown(self):
        """Handle POWER button hold - stop measuring and return to IDLE."""
        if self.state_machine.is_measuring():
            self.state_machine.stop_measurement()
        self.measuring_led.turn_off()
        self.idle_led.turn_on()
        self.save_readings_to_csv()
        print("\n[POWER] Measurement stopped. Returned to IDLE.")
    
    def read_vibration(self):
        """Read accelerometer and print vibration data."""
        try:
            accel_data = self.accelerometer.read()
            tof_data = self.tof.read() if self.tof else {"distance_mm": None}
            
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            self.readings.append(
                {
                    "timestamp": timestamp,
                    "x": accel_data["x"],
                    "y": accel_data["y"],
                    "z": accel_data["z"],
                    "distance_mm": tof_data["distance_mm"],
                }
            )
            distance_text = (
                f"{tof_data['distance_mm']:.1f}mm"
                if tof_data["distance_mm"] is not None
                else "N/A"
            )
            print(
                f"[{timestamp}] Vibration - "
                f"X={accel_data['x']:+.2f}m/s² "
                f"Y={accel_data['y']:+.2f}m/s² "
                f"Z={accel_data['z']:+.2f}m/s²"
            )
            if self.tof:
                print(f"[{timestamp}] Distance - D={distance_text}")
        except Exception as e:
            print(f"[ERROR] Failed to read accelerometer: {e}")

    def _is_removable_mount(self, device, fstype, mount_point):
        if not device.startswith(("/dev/sd", "/dev/mmcblk")):
            return False
        if fstype not in {"vfat", "exfat", "ext4"}:
            return False
        return mount_point.startswith("/media") or mount_point.startswith("/run/media")

    def _scan_usb_mounts(self):
        """Return all mounted USB paths under /media and /run/media."""
        if not self.usb_copy_any:
            return []

        mounts = set()
        try:
            with open("/proc/mounts", "r") as mounts_file:
                for line in mounts_file:
                    parts = line.split()
                    if len(parts) < 3:
                        continue
                    device, mount_point, fstype = parts[0], parts[1], parts[2]
                    if self._is_removable_mount(device, fstype, mount_point):
                        mounts.add(mount_point)
        except Exception as e:
            print(f"[USB] Failed to scan mounts: {e}")
        return sorted(mounts)

    def _build_usb_csv_path(self, mount_path):
        """Build a timestamped CSV path on the USB drive."""
        base = os.path.splitext(os.path.basename(self.csv_output_path))[0]
        stamp = time.strftime("%Y%m%d_%H%M%S")
        return os.path.join(mount_path, f"{base}_{stamp}.csv")

    def _copy_csv_to_mounts(self, mount_paths):
        """Copy the latest CSV to one or more USB mount paths."""
        if not self.readings:
            print("[USB] No readings to copy yet.")
            return

        self.save_readings_to_csv()

        success = False
        for mount_path in mount_paths:
            try:
                usb_csv_path = self._build_usb_csv_path(mount_path)
                shutil.copy2(self.csv_output_path, usb_csv_path)
                print(f"[USB] Copied CSV to {usb_csv_path}")
                success = True
            except Exception as e:
                print(f"[USB] Copy failed for {mount_path}: {e}")

        if not success:
            print("[USB] No copies succeeded.")

    def _check_usb_copy(self):
        """Detect USB insertion/removal and copy CSV when inserted."""
        mounts = set(self._scan_usb_mounts())
        new_mounts = mounts - self.usb_seen_mounts

        if new_mounts:
            self._copy_csv_to_mounts(sorted(new_mounts))

        self.usb_seen_mounts = mounts
        if mounts:
            self.usb_copy_led.set_copied()
        else:
            self.usb_copy_led.set_idle()
    
    def run(self):
        """Main application loop."""
        print("System ready. Press BEGIN button to start measuring.")
        print("Hold POWER button for 2+ seconds to stop and save.")
        print("-" * 60)
        print()
        
        try:
            while self.running:
                # Check button states
                self.begin_button.check_press()
                self.power_button.check_hold()
                
                # Update LEDs based on state
                if self.state_machine.is_measuring():
                    self.measuring_led.update()  # Blink the measuring LED
                self.usb_copy_led.update()
                
                # Read accelerometer if measuring
                current_time = time.time()
                if (
                    self.state_machine.is_measuring() and
                    current_time - self.last_reading_time >= READING_INTERVAL
                ):
                    self.read_vibration()
                    self.last_reading_time = current_time

                if current_time - self.last_usb_check_time >= USB_CHECK_INTERVAL:
                    self.last_usb_check_time = current_time
                    self._check_usb_copy()
                
                # Small sleep to avoid busy-waiting
                time.sleep(0.05)
        
        except KeyboardInterrupt:
            print("\n\nKeyboard interrupt received.")
            self.on_shutdown()
        
        finally:
            self.cleanup()

    def save_readings_to_csv(self):
        """Save collected readings to a CSV file after shutdown."""
        if not self.readings:
            print("[CSV] No readings to save.")
            return

        try:
            with open(self.csv_output_path, "w", newline="") as csv_file:
                writer = csv.DictWriter(
                    csv_file,
                    fieldnames=["timestamp", "x", "y", "z", "distance_mm"],
                )
                writer.writeheader()
                writer.writerows(self.readings)
            print(f"[CSV] Saved {len(self.readings)} readings to {self.csv_output_path}")
        except Exception as e:
            print(f"[CSV] Failed to write CSV file: {e}")
    
    def cleanup(self):
        """Clean up GPIO and other resources."""
        print("Cleaning up...")
        try:
            import RPi.GPIO as GPIO
            GPIO.cleanup()
            print("GPIO cleanup complete.")
        except Exception:
            pass

        self.save_readings_to_csv()
        
        print("=" * 60)
        print("System shutdown complete.")
        print("=" * 60)


def main():
    """Entry point."""
    system = MeasurementSystem()
    system.run()


if __name__ == "__main__":
    main()
