import RPi.GPIO as GPIO
import time

from Motor import Encoder

GPIO.setmode(GPIO.BCM)

GPIO.setup(17, GPIO.IN)
GPIO.setup(18, GPIO.IN)

a = Encoder(17, 18)

while True:
    time.sleep(0.01)
    print(a.position)
