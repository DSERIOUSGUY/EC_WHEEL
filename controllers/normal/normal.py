from math import atan, pi, cos

from chair import Wheelchair
from trackpad import TrackpadInput
from sensor import SensorInput
from cruise import CruiseControlInput

class ECWheel(Wheelchair):
    def __init__(self):
        super().__init__()

    def setupInputs(self):
        timestep = self.__class__.TIME_STEP

        mouse = self.getMouse()
        mouse.enable(timestep)

        keyboard = self.getKeyboard() #using the Keyboard to sim UI buttons
        keyboard.enable(timestep)

        sensors = tuple( self.getDevice(sensor) for sensor in  [
            "FrontLeft",
            "FrontRight",
            "Front",
            "Right",
            "Back",
            "Left",
            "LedgeDetector"
        ] )
        for sensor in sensors:
            sensor.enable(timestep)

        self.inputs = {
            "trackpad": TrackpadInput(mouse),
            "cruise": CruiseControlInput(keyboard),
            "sensor": SensorInput(sensors)
        }

    def getDirection(self, theta):
        if theta >= -pi/4 and theta < pi/4:
            return "forwards"
        elif theta >= pi/4 and theta < 3*pi/4:
            return "right"
        elif theta >= -3*pi/4 and theta < -pi/4:
            return "left"
        else:
            return "backwards"

    def move(self):
        # get the target velocity from the trackpad and the maximum allowed speed from the sensors
        self.setTarget(self.inputs["trackpad"].getTarget())
        self.setLimits(self.inputs["sensor"].getLimits(self.target))

        lwheel, rwheel = self.wheels

        # translate the target into actual motor velocities
        deadRadius = 0.1
        r, theta = self.target # target in terms of both distance from center and angle from north
        direction = self.getDirection(theta)

        self.inputs["cruise"].checkActive()
        cruising = self.inputs["cruise"].active

        if cruising and r < deadRadius:
            self.setTarget(self.inputs["cruise"].getTarget())
            r, theta = self.target
            direction = self.getDirection(theta)
        elif cruising and r >= deadRadius:
            print("handing user control")
            self.inputs["cruise"].setActive(False)

        if r < deadRadius: # Center of trackpad
            lwheel.setVelocity(0)
            rwheel.setVelocity(0)
        else:
            if direction == "forwards":
                lwheel.setVelocity(1)
                rwheel.setVelocity(1)
                self.inputs["cruise"].setMemory(self.target)
            elif direction == "right":
                lwheel.setVelocity(1)
                rwheel.setVelocity(-1/2)
            elif direction == "left":
                lwheel.setVelocity(-1/2)
                rwheel.setVelocity(1)
            elif direction == "backwards":
                lwheel.setVelocity(-1)
                rwheel.setVelocity(-1)

        lwheel.update()
        rwheel.update()

controller = ECWheel()
controller.go()
