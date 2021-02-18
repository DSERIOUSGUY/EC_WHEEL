from controller import Robot
from numpy import inf

SENSOR_MAX = 5.5
SENSOR_MIN = 1.0
RANGE = SENSOR_MAX - SENSOR_MIN

def voltageToMetersFormula(x):
    return 20.24*(x**(-4.76))+0.6632

TIME_STEP = 64
robot = Robot()

wheels = []
wheels.append(robot.getDevice("leftWheel"))
wheels.append(robot.getDevice("rightWheel"))

sensors = []
sensors.append(robot.getDevice("Sharp's IR sensor GP2Y0A710K0F FrontLeft"))
sensors.append(robot.getDevice("Sharp's IR sensor GP2Y0A710K0F FrontRight"))
sensors.append(robot.getDevice("Sharp's IR sensor GP2Y0A710K0F Right"))
sensors.append(robot.getDevice("Sharp's IR sensor GP2Y0A710K0F Back"))
sensors.append(robot.getDevice("Sharp's IR sensor GP2Y0A710K0F Left"))

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
        leftspeed = 2.0
        rightspeed = 2.0
        print(str(sensorData[0]))
        print(str(sensorData[1]))
        #Checking if sensors have detected obstacles and adjusting the max forward speed
        if (sensorData[0] <  SENSOR_MAX * 0.85 and sensorData[0] > 1.5):
            leftspeed = 2.0 * ((sensorData[0] - SENSOR_MIN)/RANGE)
            rightspeed = 2.0 * ((sensorData[0] - SENSOR_MIN)/RANGE) 
        elif (sensorData[0] < 1.5):
            leftspeed = 0.0
            rightspeed = 0.0
        elif (sensorData[1] < SENSOR_MAX * 0.85 and sensorData[1] > 1.5):
            leftspeed = 2.0 * ((sensorData[1] - SENSOR_MIN)/RANGE)
            rightspeed = 2.0 * ((sensorData[1] - SENSOR_MIN)/RANGE)
        elif (sensorData[1] < 1.5):
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
    
def getSensorData():
    sensorDataInMeters = []
    for j in sensors:
        sensorDataInMeters.append(voltageToMetersFormula(j.getValue()))
    return sensorDataInMeters

while robot.step(TIME_STEP) != -1:
    leftspeed = 0
    rightspeed = 0  
    sensorDataInMeters = getSensorData()
    speeds = trackpad(sensorDataInMeters)
    #for data in sensorDataInMeters:
        #print(str(data))
    print("FrontLeft " + str(sensorDataInMeters[0]))
    print("FrontRight " + str(sensorDataInMeters[1]))
    
    wheels[0].setVelocity(speeds[0])
    wheels[1].setVelocity(speeds[1])