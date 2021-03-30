from controller import Robot, Supervisor

class Wheel:
    """
    Container class for a wheel

    Attributes
    ----------
    wheel   -- the webots wheel object
    maxVel  -- the maximum angular velocity of the wheel (in radians)
    limit   -- the proportion of the maximum velocity that is
               currently allowed to spin at ranges between [0, 1]
    current -- the current proportion of the maximum that the wheel is
               spinning at ranges between [-1, 1]
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

class Wheelchair(Robot):
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

        self.wheels = tuple( Wheel(self.getDevice(wheel), self.__class__.WHEEL_VEL, 1.0, 0.0) for wheel in ["leftWheel", "rightWheel"] )

        self.setupInputs()

    def setupInputs(self):
        pass

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
        pass

    def go(self):
        while self.step(self.__class__.TIME_STEP) != -1:
            self.move()

class WheelchairSupervisor(Supervisor):
    """
    The main class for the wheelchair controller, when supervisor functionality is needed. Holds the motor control logic, as well as
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

        self.wheels = tuple( Wheel(self.getDevice(wheel), self.__class__.WHEEL_VEL, 1.0, 0.0) for wheel in ["leftWheel", "rightWheel"] )

        self.setupInputs()

    def setupInputs(self):
        pass

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
        pass

    def go(self):
        while self.step(self.__class__.TIME_STEP) != -1:
            self.move()
