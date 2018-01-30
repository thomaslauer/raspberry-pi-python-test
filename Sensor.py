import RPi.GPIO as GPIO
import Adafruit_BNO055 as BNO055

bno = BNO055.BNO055(serial_port="/dev/ttyS0")