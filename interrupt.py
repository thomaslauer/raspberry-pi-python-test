import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)

class Encoder:
    positionKey = [[0,0], [1,0], [1,1], [0,1]]

    def __init__(self, chanA, chanB):
        self.chanA = chanA
        self.chanB = chanB

        GPIO.setup(self.chanA, GPIO.IN)
        GPIO.setup(self.chanB, GPIO.IN)

        GPIO.add_event_detect(self.chanA, GPIO.BOTH, callback=self.gpio_callback)
        GPIO.add_event_detect(self.chanB, GPIO.BOTH, callback=self.gpio_callback)

        self.currentPosition = self.findPosition(GPIO.input(self.chanA), GPIO.input(self.chanB))
        self.pos = 0
    
    def gpio_callback(self, gpio_id):
        channelA = GPIO.input(self.chanA)
        channelB = GPIO.input(self.chanB)

        self.lastPosition = self.currentPosition
        self.currentPosition = self.findPosition(channelA, channelB)

        if((self.lastPosition + 1) % 4 == self.currentPosition):
            self.pos = self.pos + 1
        else:
            self.pos = self.pos - 1
    
    def findPosition(self, channelA, channelB):
        currentList = [channelA, channelB]
        currentPosition = -1
        for i in range(4):
            if self.positionKey[i] == currentList:
                currentPosition = i

        return currentPosition


GPIO.setup(17, GPIO.IN)
GPIO.setup(18, GPIO.IN)

a = Encoder(17, 18)

while True:
    time.sleep(0.01)