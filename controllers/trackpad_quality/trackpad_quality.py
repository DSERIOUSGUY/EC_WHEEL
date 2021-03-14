from controller import Supervisor
from numpy import inf, array
from math import atan, pi
from interface import Interface

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
    class Wheel:
        """
        Container class for a wheel

        Attributes
        ----------
        wheel   -- the webots wheel object
        maxVel  -- the maximum angular velocity of the wheel (in radians)
        limit   -- the proportion of the maximum velocity that is currently allowed to spin at
                   ranges between [0, 1]
        current -- the current proportion of the maximum that the wheel is spinning at
                   ranges between [-1, 1]
        """
        def __init__(self, wheel, maxVel, limit, current=0.0):
            self.wheel = wheel
            self.maxVel = maxVel
            self.limit = limit

            self.setVelocity(current)

            self.wheel.setPosition(float("inf"))
            self.wheel.setVelocity(self.current)

        def setVelocity(self, scale):
            if scale >= 0:
                self.current = min(scale, self.limit)
            else:
                self.current = max(scale, -self.limit)

        def setLimit(self, limit):
            self.limit = limit

        def update(self):
            self.wheel.setVelocity(self.current)

    TIME_STEP = 64
    WHEEL_VEL = 2*pi

    def __init__(self):
        super().__init__()

        self.target = (0, 0)

        self.vels = []
        self.wheels = tuple( Wheelchair.Wheel(self.getDevice(wheel), Wheelchair.WHEEL_VEL, 1.0, 0.0) for wheel in ["leftWheel", "rightWheel"] )

        self.inputs = { "trackpad": TrackpadInput(self.getMouse(), Wheelchair.TIME_STEP) }

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


class TrackpadInput:
    def __init__(self, mouse, timestep):
        self.mouse = mouse
        self.mouse.enable(timestep)

        self.interface_size = 500
        self.interface = Interface(self.interface_size)


    def getTarget(self):
        mousestate = self.mouse.getState()

        # original u, v are [0, 1] from top-left to bottom-right
        # transform to x, y: [-1, 1] from bottom-left to top-right
        x = 2 * (mousestate.u - 0.5)
        y = 2 * (0.5 - mousestate.v)

        # converting to polar coordinates (capping radius at 1)
        r = min(1, x**2 + y**2)

        # bearing from north, +ve is right
        if y == 0:
            if x == 0:
                theta = 0
            elif x > 0:
                theta = pi/2
            else:
                theta = -pi/2
        else:
            theta = atan(x/y)

            if y < 0:
                theta += pi if x >= 0 else -pi

        self.interface.show_input(mousestate.u*self.interface_size, mousestate.v*self.interface_size)
        return (r, theta)

controller = Wheelchair()
controller.go()
