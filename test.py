import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

GPIO.setup(17, GPIO.IN)
GPIO.setup(18, GPIO.IN)

while True:
    val17 = GPIO.input(17)
    val18 = GPIO.input(18)

    print(str(val17) + ' ' + str(val18))
    time.sleep(0.001)