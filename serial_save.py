# log_velocity.py

import serial
import datetime
import csv

# --- Config ---
COM_PORT = 'COM12'        # Replace with your port
BAUD_RATE = 115200       # Match with STM32 config
LOG_FILE = 'velocity_log.csv'

# --- Serial Setup ---
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)

# --- Open CSV File ---
with open(LOG_FILE, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['timestamp', 'velocity'])  # Header

    print(f"Logging to {LOG_FILE}. Press Ctrl+C to stop.\n")

    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()

            if line.startswith("velocity:"):
                try:
                    value_str = line.split(":")[1].strip()
                    velocity = float(value_str)
                    timestamp = datetime.datetime.now().isoformat()

                    writer.writerow([timestamp, velocity])
                    print(f"{timestamp} | {velocity:.4f}")

                except ValueError:
                    continue

    except KeyboardInterrupt:
        print("\nLogging stopped by user.")

    finally:
        ser.close()