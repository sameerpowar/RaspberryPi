#!/usr/bin/env python3
import os
import serial
import time
import json
import RPi.GPIO as GPIO
import threading
import ds18b20 as TempSensor
from datetime import datetime

#   CONFIG
SERIAL_PORT = "/dev/ttyACM0"
BAUD = 9600
#json Log file
JSON_LOG_FILE = "sensor_data.json"

#   GPIO SETUP
PUSH_BTN_PIN = 17
MOTOR_PIN = 27
SOLENOID_PIN = 22
TEMPSENSOR_PIN = 4


#initialisation
GPIO.setmode(GPIO.BCM)
GPIO.setup(PUSH_BTN_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(MOTOR_PIN, GPIO.OUT)
GPIO.setup(SOLENOID_PIN, GPIO.OUT)
GPIO.output(MOTOR_PIN, GPIO.LOW)
GPIO.output(SOLENOID_PIN, GPIO.LOW)
pump_running = False

#Program starts here
def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=1)
        print("Connected to Arduino")
        time.sleep(2)
    except Exception as e:
        print("Error:", e)
        return

    while True:
        try:
            #if push button prssed and motor is not running then turn on the motor
            if GPIO.input(PUSH_BTN_PIN) == GPIO.LOW:
                if not pump_running:
                    threading.Thread(target=pump_thread, args=(60,), daemon=True).start()
                time.sleep(0.3)

            #read temp in celcius and feranide
            tempC, tempF = TempSensor.read_ds18b20()

            #todo: assign values here respectively  
            rawPH = 1
            rawTDS = 2
            rawTURB = 3

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

        except Exception as e:
            print("Error:", e)

# ==========================================================
#   SENSOR FORMULAS (MATCHING YOUR ARDUINO)
# ==========================================================

def adc_to_voltage(adc_raw):
    return (adc_raw * 5.0) / 1024.0   
# Arduino reference uses 1024 steps

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
#   CLEANUP
# ==========================================================
try:
    main()
finally:
    GPIO.cleanup()
