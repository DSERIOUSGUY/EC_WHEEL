from numpy import zeros
from math import pi

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

    def __init__(self, sensors):
        self.sensors = sensors
        self.queue = [] # queue for noise reduction purposes


    # Conversion of sensor signal to useable distance measurements
    @staticmethod
    def voltageToMetersFormula(x):
        return 1.784*(x**(-0.4215)) - 1.11

    # Returns the ratio of max speed that the wheelchair should be limited to
    def getLimits(self, target):
        sensorData = self.getSensorData()
        r, theta = target

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
