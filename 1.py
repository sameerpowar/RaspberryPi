#!/usr/bin/env python3
import serial
import time
import json
from datetime import datetime
import os
import glob

# === CONFIG ===
SERIAL_PORT = '/dev/ttyACM0'       # Arduino USB port
BAUD = 9600
JSON_LOG_FILE = 'sensor_data.json'
CSV_LOG_FILE  = 'sensor_data.csv'
# ==============

# ==========================================================
#   DS18B20 SETUP
# ==========================================================
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = '/sys/bus/w1/devices/'
device_folder = glob.glob(base_dir + '28*')[0]     # first DS18B20
device_file = device_folder + '/w1_slave'

def read_temp_raw():
    with open(device_file, 'r') as f:
        return f.readlines()

def read_ds18b20():
    """Reads DS18B20 temperature (°C and °F)."""
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != 'YES':
        time.sleep(0.2)
        lines = read_temp_raw()

    equals_pos = lines[1].find('t=')
    if equals_pos != -1:
        temp_string = lines[1][equals_pos+2:]
        temp_c = float(temp_string) / 1000.0
        temp_f = temp_c * 9.0 / 5.0 + 32.0
        return temp_c, temp_f
    return None, None

# ==========================================================
#   CSV HEADER HELPER
# ==========================================================
def ensure_csv_header(path):
    if not os.path.exists(path):
        with open(path, 'w') as f:
            f.write("timestamp,PH,TDS,TURB,DS18B20_C,DS18B20_F\n")

def append_to_csv(path, ts, data):
    with open(path, 'a') as f:
        f.write(
            f'{ts},{data.get("PH","")},{data.get("TDS","")},'
            f'{data.get("TURB","")},{data.get("DS18B20_C","")},'
            f'{data.get("DS18B20_F","")}\n'
        )

# ==========================================================
#   MAIN PROGRAM
# ==========================================================
def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
    except Exception as e:
        print("Error opening serial port:", e)
        return

    time.sleep(2)
    print("Connected to Arduino on", SERIAL_PORT)

    ensure_csv_header(CSV_LOG_FILE)

    try:
        while True:

            # ---- Read DS18B20 ----
            ds_c, ds_f = read_ds18b20()

            # ---- Read Arduino JSON Line ----
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            try:
                arduino = json.loads(line)

                # timestamp
                ts = datetime.now().isoformat()

                # Merge sensors (REMOVED Arduino TEMP)
                full_data = {
                    "timestamp": ts,
                    "PH": arduino.get("PH"),
                    "TDS": arduino.get("TDS"),
                    "TURB": arduino.get("TURB"),
                    "C": ds_c,
                    "F": ds_f
                }

                # ---- Print to terminal ----
                print(json.dumps(full_data, indent=2))

                # ---- Log JSON ----
                with open(JSON_LOG_FILE, 'a') as fjson:
                    fjson.write(json.dumps(full_data) + '\n')

                # ---- Log CSV ----
                append_to_csv(CSV_LOG_FILE, ts, full_data)

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
