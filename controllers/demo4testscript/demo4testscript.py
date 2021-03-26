from controller import Supervisor
import numpy as np
from math import pi

from parts.monitor import Monitor
from parts.trackpad import TrackpadInput
from parts.wheel import Wheel

class Wheelchair(Supervisor):
    """
    The main class for the wheelchair controller. Holds the motor control logic, as well as
    the setup for the controller

    Attributes
    ----------
    wheels  -- the `Wheel` objects holding the main wheels
    sensors -- the webots sensor objects on the chair
    inputs  -- a dictionary of input controllers to use
    target  -- the target velocity. Of the form (scale, theta), where scale is a value between [0, 1]
               indicating the desired proportion of max speed and theta is an angle (in radians)
               indicating the desired bearing of travel [-pi, pi] where positive values are clockwise
               from straight ahead

    """
    TIME_STEP = 64
    WHEEL_VEL = 5

    def __init__(self):
        super().__init__()

        self.target = (0, 0)

        self.wheels = tuple( Wheel(self.getDevice(wheel), Wheelchair.WHEEL_VEL, 1.0, 0.0) for wheel in ["leftWheel", "rightWheel"] )

        self.inputs = { "trackpad": TrackpadInput(self.getMouse(), Wheelchair.TIME_STEP), }

        self.monitor = Monitor(8, 4, 64, self.getSelected(), self.getFromDef)

    def setTarget(self, targetVelocity):
        #do some processing/error detection
        self.target = targetVelocity

    def setLimits(self, limits):
        if limits is None:
            raise Exception("uh oh")

        #also so some processing
        for limit, wheel in zip(limits, self.wheels):
            wheel.setLimit(limit)

    def move(self):
        self.setTarget(self.inputs["trackpad"].getTarget())
        #self.setLimits(self.inputs["sensor"].getLimits(self.target))

        # translate the target into actual motor velocities
        deadRadius = 0.1
        r, theta = self.target

        lwheel, rwheel = self.wheels

        if r < deadRadius:
            lwheel.setVelocity(0)
            rwheel.setVelocity(0)
        elif theta >= -pi/4 and theta < pi/4:
            lwheel.setVelocity(1)
            rwheel.setVelocity(1)
        elif theta >= pi/4 and theta < 3*pi/4:
            lwheel.setVelocity(1)
            rwheel.setVelocity(-1)
        elif theta >= -3*pi/4 and theta < -pi/4:
            lwheel.setVelocity(-1)
            rwheel.setVelocity(1)
        else:
            lwheel.setVelocity(-1)
            rwheel.setVelocity(-1)

        lwheel.update()
        rwheel.update()

    def go(self):
        while self.step(Wheelchair.TIME_STEP) != -1:
            self.move()
            self.monitor.update_view()

controller = Wheelchair()
controller.go()
