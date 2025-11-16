import RPi.GPIO as GPIO
import time

# GPIO pins to use (for 7 bits)
GPIO_PINS = [25, 24, 23, 22, 21, 30, 14]  # Example GPIO pins

# Set up GPIO
GPIO.setmode(GPIO.BCM)  # Use BCM pin numbering
GPIO.setup(GPIO_PINS, GPIO.OUT)  # Set all pins as output

def output_to_gpio(value):
    # Convert the value to a 7-bit binary and output to the GPIO pins
    for i in range(7):
        GPIO.output(GPIO_PINS[i], (value >> i) & 1)

try:
    for value in range(0x00, 0x80):  # Loop from 0x00 to 0x7F
        output_to_gpio(value)
        print(f"Outputting: {value:02X}")  # Print the value in hexadecimal
        time.sleep(0.5)  # Delay for visibility (adjust as needed)

except KeyboardInterrupt:
    print("Exiting program")

finally:
    GPIO.cleanup()  # Reset GPIO settings
