import RPi.GPIO as GPIO

class Encoder:
    positionKey = [[0,0], [1,0], [1,1], [0,1]]

    def __init__(self, chanA, chanB):
        self.chanA = chanA
        self.chanB = chanB

        # Set two channels as inputs
        GPIO.setup(self.chanA, GPIO.IN)
        GPIO.setup(self.chanB, GPIO.IN)

        # Register callback with both channels
        GPIO.add_event_detect(self.chanA, GPIO.BOTH, callback=self.gpio_callback)
        GPIO.add_event_detect(self.chanB, GPIO.BOTH, callback=self.gpio_callback)

        # Set the current stage to the current position of the motor
        self.currentStage = self.findPosition(GPIO.input(self.chanA), GPIO.input(self.chanB))

        # Default to a position of 0
        self.pos = 0
    
    def gpio_callback(self, gpio_id):
        channelA = GPIO.input(self.chanA)
        channelB = GPIO.input(self.chanB)

        self.lastPosition = self.currentStage
        self.currentStage = self.findPosition(channelA, channelB)

        if((self.lastPosition + 1) % 4 == self.currentStage):
            self.pos = self.pos + 1
        else:
            self.pos = self.pos - 1
    
    def findPosition(self, channelA, channelB):
        currentList = [channelA, channelB]
        stage = -1
        for i in range(4):
            if self.positionKey[i] == currentList:
                stage = i

        return stage
