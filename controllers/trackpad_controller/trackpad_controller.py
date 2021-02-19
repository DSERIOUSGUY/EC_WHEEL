# 2021 ECWHeel©. All rights reserved. 

from controller import Robot

#The range of sensor when it is most reliable due to the inherent noise
DISTANCE_UPPER_BOUND = 1.3
DISTANCE_LOWER_BOUND = 0.7
RANGE = DISTANCE_UPPER_BOUND - DISTANCE_LOWER_BOUND

TIME_STEP = 64
robot = Robot()

wheels = []
wheels.append(robot.getDevice("leftWheel"))
wheels.append(robot.getDevice("rightWheel"))

#Sensors numbering starts from 0 for the one located on the left facing forward,
#Each following number corresponds to the next sensor clockwise.
sensors = []
sensors.append(robot.getDevice("Sharp's IR sensor GP2D120 FrontLeft"))
sensors.append(robot.getDevice("Sharp's IR sensor GP2D120 FrontRight"))
sensors.append(robot.getDevice("Sharp's IR sensor GP2D120 Right"))
sensors.append(robot.getDevice("Sharp's IR sensor GP2D120 Back"))
sensors.append(robot.getDevice("Sharp's IR sensor GP2D120 Left"))

for i in sensors:
    i.enable(TIME_STEP)

wheels[0].setPosition(float('inf'))
wheels[0].setVelocity(0.0)

wheels[1].setPosition(float('inf'))
wheels[1].setVelocity(0.0)

leftspeed = 3.0
rightspeed = 3.0

mouse = robot.getMouse()

mouse.enable(TIME_STEP)

def trackpad(sensorData):
    mousestate = mouse.getState()
   
    x = mousestate.u 
    y = -mousestate.v
    leftspeed = 0.0
    rightspeed = 0.0

    if y >= -x and y >= x - 1:
        #Go forward
        #Checking if sensors have detected obstacles and adjusting the max forward speed
        leftspeed = 2.0
        rightspeed = 2.0
        #Checking the Left sensor facing forwards for if there is an obstacle in the range
        if (sensorData[0] < DISTANCE_UPPER_BOUND and sensorData[0] > DISTANCE_LOWER_BOUND):
            leftspeed = 2.0 * ((sensorData[0] - DISTANCE_LOWER_BOUND)/RANGE)
            rightspeed = 2.0 * ((sensorData[0] - DISTANCE_LOWER_BOUND)/RANGE)
        #If the sensor value is lower then the lower bound,
        #Wheelchair cannot move forward, only any other direction 
        elif (sensorData[0] < DISTANCE_LOWER_BOUND):
            leftspeed = 0.0
            rightspeed = 0.0
        #Checking the Right sensor facing forward for if there is an obstacle in the range
        elif (sensorData[1] < DISTANCE_UPPER_BOUND and sensorData[1] > DISTANCE_LOWER_BOUND):
            leftspeed = 2.0 * ((sensorData[1] - DISTANCE_LOWER_BOUND)/RANGE)
            rightspeed = 2.0 * ((sensorData[1] - DISTANCE_LOWER_BOUND)/RANGE)
        elif (sensorData[1] < DISTANCE_LOWER_BOUND):
            leftspeed = 0.0
            rightspeed = 0.0
        print("forward")
     
    if y < -x and y < x - 1:
        #Go back
        leftspeed = -2.0
        rightspeed = -2.0
        print("back")        
                
    if y < -x and y > x - 1:
        #turn left
        leftspeed = 2.0
        rightspeed = -2.0
        print("left")    
        
                
    if y > -x and y < x - 1:
        #turn right
        leftspeed = -2.0
        rightspeed = 2.0
        print("right")
            
    
    if (y+0.5)**2 + (x-0.5)**2 < 0.01:
        leftspeed = 0 
        rightspeed = 0
        print("stop")
        
    speed = [leftspeed, rightspeed] 
    return speed

#Getting the raw values from the sensors (voltage),
#And transforming them into meters.    
def getSensorData():

    def voltageToMetersFormula(x):
        return 1.784*(x**(-0.4215)) - 1.11

    sensorDataInMeters = []
    for sensor in sensors:
        sensorDataInMeters.append(voltageToMetersFormula(sensor.getValue()))

    return sensorDataInMeters

def setActuators(speeds):
    wheels[0].setVelocity(speeds[0])
    wheels[1].setVelocity(speeds[1])

while robot.step(TIME_STEP) != -1:
    leftspeed = 0
    rightspeed = 0  
    sensorData = getSensorData()
    speeds = trackpad(sensorData)

    #for data in sensorDataInMeters:
    #    print(str(data))
    
    setActuators(speeds)