from math import atan, pi

class TrackpadInput:
    def __init__(self, mouse, timestep):
        self.mouse = mouse
        self.mouse.enable(timestep)

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

        return (r, theta)
