import math
import matplotlib.pyplot as plt

g = 9.8
t = 0.01

class Robot:

    def __init__(self, cogLength, mass):
        """"
        need to add starting positions (velocity, angle)
        maybe make a way to do list of commands to execute at
        a certain time, etc
        """
        
        self.cogLength = cogLength
        self.mass = mass
        self.angle = math.pi/2
        self.angularVelocity = 0
        self.angularAcceleration = 0
    
    def updatePhysics(self):
        """
        Calculates the change in physics. Uses the equation (g/l)sin(theta)
        to find the angluar acceleration of the inverted pendulum, then performs
        a lazy integration
        """
        self.angularAcceleration = (g/self.cogLength) * math.sin(self.angle)
        self.angularVelocity += self.angularAcceleration * t
        self.angle += self.angularVelocity * t
    

def main():
    robot = Robot(0.5, 1)

    # define the time in seconds of the simulation
    time = 5

    for i in range(int(time / t)):
        robot.updatePhysics()
        print("Angle: {:2.4f}  Velocity: {:2.4f}  Acceleration: {:2.4f}"
            .format(robot.angle * 180 / math.pi, robot.angularVelocity, robot.angularAcceleration))

main()