#!/usr/bin/env python3
import serial
import time
import json
from datetime import datetime
import os

# === CONFIG ===
SERIAL_PORT = '/dev/ttyACM0'   # change if needed
BAUD = 9600
JSON_LOG_FILE = 'sensor_data.json'  # appended lines (one JSON object per line)
CSV_LOG_FILE  = 'sensor_data.csv'   # optional csv
# ==============

def ensure_csv_header(path):
    if not os.path.exists(path):
        with open(path, 'w') as f:
            f.write('timestamp,PH,TDS,TURB,TEMP\n')

def append_to_csv(path, ts, data):
    with open(path, 'a') as f:
        f.write(f'{ts},{data.get("PH","")},{data.get("TDS","")},{data.get("TURB","")},{data.get("TEMP","")}\n')

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
    except Exception as e:
        print("Error opening serial port:", e)
        return

    time.sleep(2)  # wait for Arduino reset
    print("Connected to Arduino on", SERIAL_PORT)

    ensure_csv_header(CSV_LOG_FILE)

    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue
            # try parse JSON
            try:
                data = json.loads(line)
                # store in variables
                ph_value = float(data.get("PH", 0.0))
                tds_value = float(data.get("TDS", 0.0))
                turb_value = float(data.get("TURB", 0.0))
                temp_value = float(data.get("TEMP", 0.0))

                # timestamp
                ts = datetime.now().isoformat()

                # print pretty
                print(ts, json.dumps(data, indent=None))

                # append to JSON file (one object per line)
                with open(JSON_LOG_FILE, 'a') as fjson:
                    fjson.write(json.dumps({"timestamp": ts, **data}) + '\n')

                # append to CSV
                append_to_csv(CSV_LOG_FILE, ts, data)

                # Do other processing here (database, MQTT, HTTP post, etc.)

            except json.JSONDecodeError:
                print("Invalid JSON received:", line)
            except Exception as e:
                print("Error processing line:", e)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
