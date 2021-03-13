from controller import Robot
from numpy import inf, zeros
from math import atan, pi

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
            # choose the scale closest to 0 between the trackpad input and sensor input
            if scale >= 0:
                self.current = min(scale, self.limit) * self.maxVel
            else:
                self.current = max(scale, -self.limit) * self.maxVel

        def setLimit(self, limit):
            self.limit = limit

        def update(self):
            self.wheel.setVelocity(self.current)

    TIME_STEP = 64
    WHEEL_VEL = 1.5

    # -------------------------------------- END OF WHEEL ----------------------------------------------------

    def __init__(self):
        super().__init__()
        global speedMemory

        self.target = (0, 0)

        self.wheels = tuple( Wheelchair.Wheel(self.getDevice(wheel), Wheelchair.WHEEL_VEL, 1.0, 0.0) for wheel in ["leftWheel", "rightWheel"] )
        self.sensors = tuple( self.getDevice(sensor) for sensor in  [
            "Sharp's IR sensor GP2D120 FrontLeft",
            "Sharp's IR sensor GP2D120 FrontRight",
            "Sharp's IR sensor GP2D120 Front",
            "Sharp's IR sensor GP2D120 Right",
            "Sharp's IR sensor GP2D120 Back",
            "Sharp's IR sensor GP2D120 Left",
            "Sharp's IR sensor GP2D120 LedgeDetector"   
            ] )

        self.inputs = { "trackpad": TrackpadInput(self.getMouse(), Wheelchair.TIME_STEP),
                        "sensor": SensorInput(self.sensors, Wheelchair.TIME_STEP) }

    def setTarget(self, targetVelocity):
        # do some processing/error detection
        self.target = targetVelocity

    def setLimits(self, limits):
        if limits is None:
            raise Exception("uh oh")

        #also so some processing
        
        for limit, wheel in zip(limits, self.wheels):
            wheel.setLimit(limit)

    def move(self):
        global speedMemory
        global cruiseControlFlag
        global lastClickState
        global lastClickStates

        # get the target velocity from the trackpad and the maximum allowed speed from the sensors
        self.setTarget(self.inputs["trackpad"].getTarget())
        self.setLimits(self.inputs["sensor"].getLimits(self.target))

        # translate the target into actual motor velocities
        deadRadius = 0.1
        r, theta, leftClick = self.target # target in terms of both distance from center and angle from north

        lwheel_mem, rwheel_mem = speedMemory

        lwheel, rwheel = self.wheels

        #print(lwheel.limit)
        #print(rwheel.limit)

        #print(leftClick)


        if not lastClickState == cruiseControlFlag:
            if leftClick:
                if cruiseControlFlag:
                    cruiseControlFlag = False
                else:
                    cruiseControlFlag = True

        print(cruiseControlFlag)
        lastClickState = leftClick

        


        if cruiseControlFlag and r < deadRadius:
            lwheel.setVelocity(lwheel_mem)
            rwheel.setVelocity(rwheel_mem)

        else:

            if r < deadRadius: # Center of trackpad
                lwheel.setVelocity(0)
                rwheel.setVelocity(0)
            elif theta >= -pi/4 and theta < pi/4: # Going Forwards
                cruiseControlFlag = False

                lwheel.setVelocity(1)
                rwheel.setVelocity(1)

                speedMemory = (lwheel.current, rwheel.current)

                lwheel.update()
                rwheel.update()

                return

            elif theta >= pi/4 and theta < 3*pi/4: # Going Right
                cruiseControlFlag = False
                lwheel.setVelocity(0)
                rwheel.setVelocity(1)
            elif theta >= -3*pi/4 and theta < -pi/4: # Going Left
                cruiseControlFlag = False
                lwheel.setVelocity(1)
                rwheel.setVelocity(0)
            else: # Going Backwards
                cruiseControlFlag = False
                lwheel.setVelocity(-1)
                rwheel.setVelocity(-1)


        lwheel.update()
        rwheel.update()

    def go(self):
        while self.step(Wheelchair.TIME_STEP) != -1:
            self.move()

#--------------------------------------- END OF WHEELCHAIR -------------------------------------------------------------


class TrackpadInput:

    def __init__(self, mouse, timestep):
        self.mouse = mouse
        self.mouse.enable(timestep)

    def getTarget(self):
        mousestate = self.mouse.getState()

        """""
        Approaches = ->
            for top right 
            u -> 1
            v -> 0

            for top left 
            u -> 0
            v -> 0

            for right 
            u -> 1
            v -> 1/2

            for left 
            u -> 0
            v -> 1/2
        """
        # original u, v are [0, 1] from top-left to bottom-right
        # transform to x, y: [-1, 1] from bottom-left to top-right
        x = 2 * (mousestate.u - 0.5)
        y = 2 * (0.5 - mousestate.v)
        leftClick = mousestate.left

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

        return (r, theta, leftClick)

class SensorInput:

    """
    Main class for sensor reading and obstacle avoidance

    Attributes
    ----------
    sensors    -- the webots sensor objects on the chair
    SENSOR_MAX -- furthest distance for an obstacle to be detected
    SENSOR_MIN -- closest distance for an obstacle to be detected by forward sensors
    queue      -- 5 element queue to store smoothed values of last 5 sensor readings
    
    """

    # define the best range for the sensors in terms of noise
    SENSOR_MAX = 1.3
    SENSOR_MIN = 0.7
    RANGE = SENSOR_MAX - SENSOR_MIN

    def __init__(self, sensors, timestep):
        self.sensors = sensors
        for sensor in self.sensors:
            sensor.enable(timestep)
        
        self.queue = [] # queue for noise reduction purposes


    # Conversion of sensor signal to useable distance measurements
    @staticmethod
    def voltageToMetersFormula(x):
        return 1.784*(x**(-0.4215)) - 1.11

    # Returns the ratio of max speed that the wheelchair should be limited to
    def getLimits(self, target):
        sensorData = self.getSensorData()
        r , theta, leftClick = target

        deadRadius = 0.1

        # Apply a moving average over the last 5 readings to the sensor readings to lessen any spikes in reading
        if len(self.queue) < 5:
            self.queue.append(sensorData)
        else:
            averages = zeros(len(self.queue[0]))
            for i in range(len(averages)):
                for j in range(len(self.queue)):
                    averages[i] += self.queue[j][i]
                averages[i] = averages[i] / len(self.queue[j])
                sensorData[i] = (sensorData[i] * 0.75) + (averages[i] * 0.25)

        # If the central forward facing sensor gets a reading closer than the front of the wheelchair, ignore it
        if sensorData[1] < 0.45:
            sensorData[1] = 1.6
        
        # Take the closest reading from the forward facing sensors as the reading to use
        mindist = min(sensorData[0], (sensorData[1] + 0.05), sensorData[2])

        # If the user inputs in the trackpad deadzone, disregard any readings
        if r < deadRadius:
            print("Center")
            return (1.0, 1.0)

        # If the user wants to move forward
        elif theta >= -pi/4 and theta < pi/4:
            print("Forwards")
            # If there is a ledge in front of the wheelchair, stop movement
            if self.detectLedge(sensorData[6]):
                return (0.0,0.0)

            # If the forward reading is within detection range
            if (mindist < SensorInput.SENSOR_MAX and mindist > SensorInput.SENSOR_MIN):
                # Scale the limit relative to the distance to the object
                return( ((mindist - SensorInput.SENSOR_MIN)/SensorInput.RANGE),
                        ((mindist - SensorInput.SENSOR_MIN)/SensorInput.RANGE))

            # If there is an object closer than the minimum distance, stop movement
            elif (mindist < SensorInput.SENSOR_MIN):
                return (0.0, 0.0)
            
            # Otherwise allow free control
            else:
                return (1.0, 1.0)

        # If the user is turning right
        elif theta >= pi/4 and theta < 3*pi/4:
            # If there is an obstacle that would block the turn, stop movement
            if (sensorData[3] < SensorInput.SENSOR_MIN - 0.25): # offset the minimum to allow turning that wouldn't block
                return (0.0, 0.0)
            else:
                return (1.0, 1.0)

        # If the user is turning left
        elif theta >= -3*pi/4 and theta < -pi/4:
            print("Left")
            # If there is an obstacle that would block the turn, stop movement
            if (sensorData[6] < SensorInput.SENSOR_MIN - 0.25):
                return (0.0, 0.0)
            else:
                return (1.0, 1.0)

        # If the user is reversing
        else:
            # Essentially same as forward just in the other direction
            if (sensorData[4] < SensorInput.SENSOR_MAX and sensorData[4] > SensorInput.SENSOR_MIN):
                if self.detectLedge(sensorData[6]):
                    return (0.0,0.0)
                return( ((sensorData[4] - SensorInput.SENSOR_MIN)/SensorInput.RANGE),
                ((sensorData[4] - SensorInput.SENSOR_MIN)/SensorInput.RANGE))
            elif (sensorData[4] < SensorInput.SENSOR_MIN):
                return (0.0, 0.0)
            else:
                if self.detectLedge(sensorData[6]):
                    return (0.0,0.0)
                return (1.0, 1.0)

        # Update the noise reduction queue
        self.pop_queue(sensorData)
    
    # Gets sensor readings and converts to usable distance readings for all sensors
    def getSensorData(self):
        return [
            SensorInput.voltageToMetersFormula(sensor.getValue())
            for sensor in self.sensors
        ]
    
    # Returns True if a ledge has been detected, False otherwise
    def detectLedge(self, ledgeSensor):
        if not ledgeSensor < 1.00:
            return True
        return False

    def pop_queue(self, sensorData):
        self.queue.pop(0)
        self.queue.append(sensorData)

speedMemory = (1.0,1.0)

cruiseControlFlag = False
lastClickState = False

lastClickStates = []


controller = Wheelchair()
controller.go()
