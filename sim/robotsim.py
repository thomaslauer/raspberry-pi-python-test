import math

g = 9.8
t = 0.01

debug = True

class PID:
    """
    Runs the PID controller algorithm
    """

    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.target = 0
        self.lastVal = 0
        self.integralError = 0
        
        self.numCycles = 0
    
    def calculatePID(self, val):

        error = val - self.target

        self.integralError += error * t

        proportional = error * self.kp * t
        integral = self.integralError * self.ki
        derivative = self.kd * (val - self.lastVal) / t

        if(self.numCycles == 0):
            # ignore d output, we don't have any slope to calculate
            derivative = 0

        if(debug):
            print("p: {:2.4f} i: {:2.4f} d: {:2.4}", proportional, integral, derivative)

        self.lastVal = val
        self.numCycles += 1
        return proportional + integral + derivative

    def resetPID(self):
        self.numCycles = 0
        self.lastVal = 0
        self.integralError = 0

class Robot:

    def __init__(self, cogLength, mass, controller):
        """"
        need to add starting positions (velocity, angle)
        maybe make a way to do list of commands to execute at
        a certain time, etc
        """
        self.cogLength = cogLength
        self.mass = mass

        self.controller = controller

        self.angle = 0.001
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
    
    def updateController(self):
        """
        Puts the current angle through the PID algorithm to update the motors
        
        TODO figure out what the PID should be controlling (acceleration or velocity)
        and add that to the controls.
        """
        pidVal = self.controller.calculatePID(self.angle)
        return pidVal
    

def main():
    robot = Robot(0.5, 1, PID(100, 10, 1))

    # define the time in seconds of the simulation
    time = 5

    for i in range(int(time / t)):
        robot.updatePhysics()
        print("Angle: {:2.4f}  Velocity: {:2.4f}  Acceleration: {:2.4f}"
            .format(robot.angle * 180 / math.pi, robot.angularVelocity, robot.angularAcceleration))
        
        print("Controller output: {:2.8f}".format(robot.updateController()))

main()