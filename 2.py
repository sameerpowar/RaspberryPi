#!/usr/bin/env python3
import serial
import time
import json
from datetime import datetime
import os
import glob
import RPi.GPIO as GPIO
import threading

# ==========================================================
#   CONFIG
# ==========================================================
SERIAL_PORT = "/dev/ttyACM0"
BAUD = 9600
JSON_LOG_FILE = "sensor_data.json"
CSV_LOG_FILE = "sensor_data.csv"

# ==========================================================
#   GPIO SETUP
# ==========================================================
BUTTON_PIN = 17
MOTOR_PIN = 27
SOLENOID_PIN = 22

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(MOTOR_PIN, GPIO.OUT)
GPIO.setup(SOLENOID_PIN, GPIO.OUT)

GPIO.output(MOTOR_PIN, GPIO.LOW)
GPIO.output(SOLENOID_PIN, GPIO.LOW)

pump_running = False

# ==========================================================
#   DS18B20 TEMPERATURE SENSOR
# ==========================================================
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')

base_dir = "/sys/bus/w1/devices/"
device_folder = glob.glob(base_dir + "28*")[0]
device_file = device_folder + "/w1_slave"

def read_temp_raw():
    with open(device_file, 'r') as f:
        return f.readlines()

def read_ds18b20():
    lines = read_temp_raw()
    while lines[0].strip()[-3:] != "YES":
        time.sleep(0.2)
        lines = read_temp_raw()

    pos = lines[1].find("t=")
    if pos != -1:
        temp_c = float(lines[1][pos+2:]) / 1000.0
        temp_f = temp_c * 9/5 + 32
        return temp_c, temp_f
    return None, None

# ==========================================================
#   CSV HEADER CHECK
# ==========================================================
def ensure_csv_header(path):
    if not os.path.exists(path):
        with open(path, "w") as f:
            f.write("timestamp,pH,TDS,Turbidity,Temp_C,Temp_F,pump_running\n")

# ==========================================================
#   SENSOR FORMULAS (MATCHING YOUR ARDUINO)
# ==========================================================

def adc_to_voltage(adc_raw):
    return (adc_raw * 5.0) / 1024.0   # Arduino reference uses 1024 steps

# ----------- pH CALCULATION (Arduino formula) -----------
# Arduino formula:
# volt = avgADC * 5.0 / 1024 / 6
# pH = -5.70 * volt + calibration_value
calibration_value = 10

def calc_ph_arduino(raw_adc):
    volt = (raw_adc * 5.0) / 1024.0
    volt = volt / 6.0
    pH = (-5.70 * volt) + calibration_value
    return pH

# ----------- TDS CALCULATION (Keyestudio official) -----------
def calc_tds_arduino(raw_adc, tempC):
    v = adc_to_voltage(raw_adc)

    compensation_coef = 1 + 0.02 * (tempC - 25)
    v_comp = v / compensation_coef

    tds = (133.42 * v_comp**3 - 255.86 * v_comp**2 + 857.39 * v_comp) * 0.5
    return tds

# ----------- TURBIDITY CALCULATION -----------
def calc_turbidity(raw_adc):
    v = adc_to_voltage(raw_adc)
    ntu = max(0, (v - 4.2) * -1120)
    return ntu

# ==========================================================
#   PUMP THREAD
# ==========================================================
def pump_thread(duration=60):
    global pump_running

    if pump_running:
        return

    pump_running = True
    GPIO.output(MOTOR_PIN, GPIO.HIGH)
    GPIO.output(SOLENOID_PIN, GPIO.HIGH)

    time.sleep(6)

    GPIO.output(MOTOR_PIN, GPIO.LOW)
    GPIO.output(SOLENOID_PIN, GPIO.LOW)

    pump_running = False

# ==========================================================
#   MAIN LOOP
# ==========================================================
def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
        print("Connected to Arduino")
        time.sleep(2)
    except Exception as e:
        print("Error:", e)
        return

    ensure_csv_header(CSV_LOG_FILE)

    while True:
        try:
            # ------- BUTTON -------
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                if not pump_running:
                    threading.Thread(target=pump_thread, args=(60,), daemon=True).start()
                time.sleep(0.3)

            # ------- DS18B20 -------
            tempC, tempF = read_ds18b20()

            # ------- SERIAL FROM ARDUINO -------
            line = ser.readline().decode().strip()
            if not line:
                continue

            parts = line.split(",")
            if len(parts) != 3:
                continue

            rawPH = int(parts[0])
            rawTDS = int(parts[1])
            rawTURB = int(parts[2])

            # ------- CALCULATIONS (MATCH ARDUINO) -------
            pH = calc_ph_arduino(rawPH)
            tds = calc_tds_arduino(rawTDS, tempC)
            turb = calc_turbidity(rawTURB)

            ts = datetime.now().isoformat()

            data = {
                "timestamp": ts,
                "pH": round(pH, 2),
                "TDS": round(tds, 2),
                "Turbidity": round(turb, 2),
                "Temp_C": tempC,
                "Temp_F": tempF,
                "pump_running": pump_running
            }

            print(json.dumps(data, indent=2))

            with open(JSON_LOG_FILE, "a") as f:
                f.write(json.dumps(data) + "\n")

            with open(CSV_LOG_FILE, "a") as f:
                f.write(f"{ts},{pH},{tds},{turb},{tempC},{tempF},{pump_running}\n")

        except Exception as e:
            print("Error:", e)

# ==========================================================
#   CLEANUP
# ==========================================================
try:
    main()
finally:
    GPIO.cleanup()
