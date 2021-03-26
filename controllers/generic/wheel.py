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
