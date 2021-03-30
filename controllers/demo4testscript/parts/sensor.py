class SensorInput:
    SENSOR_MAX = 5.5
    SENSOR_MIN = 1.0
    RANGE = SENSOR_MAX - SENSOR_MIN

    def voltageToMetersFormula(x):
        return 20.24*(x**(-4.76))+0.6632

    def __init__(self, sensors, timestep):
        self.sensors = sensors
        for sensor in self.sensors:
            sensor.enable(timestep)

    def getLimits(self, target):
        sensorData = self.getSensorData()
        r, theta = target

        if theta < -pi/4 or theta >= pi/4:
            #nothing forwards
            return (1.0, 1.0)

        #this is the moving forward part
        if (sensorData[0] <  SensorInput.SENSOR_MAX * 0.85 and sensorData[0] > 1.5):
            return ((sensorData[0] - SensorInput.SENSOR_MIN)/SensorInput.RANGE,
                    (sensorData[0] - SensorInput.SENSOR_MIN)/SensorInput.RANGE)
        elif (sensorData[0] < 1.5):
            return (0.0, 0.0)
        elif (sensorData[1] < SensorInput.SENSOR_MAX * 0.85 and sensorData[1] > 1.5):
            return ((sensorData[1] - SensorInput.SENSOR_MIN)/SensorInput.RANGE,
                    (sensorData[1] - SensorInput.SENSOR_MIN)/SensorInput.RANGE)
        elif (sensorData[1] < 1.5):
            return (0.0, 0.0)
        else:
            return (1.0, 1.0)

    def getSensorData(self):
        return [
            SensorInput.voltageToMetersFormula(sensor.getValue())
            for sensor in self.sensors
        ]
