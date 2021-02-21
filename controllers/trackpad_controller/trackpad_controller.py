# 2021 ECWHeel©. All rights reserved. 

from controller import Robot
import numpy as np
import matplotlib.pyplot as plt
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

#Measuring real distance to from the sensors with no noise for testing
distanceFromFrontLeft = robot.getDevice("real distance from Front Left")
distanceFromFrontLeft.enable(TIME_STEP)
distanceFromFrontRight = robot.getDevice("real distance from Front Right")
distanceFromFrontRight.enable(TIME_STEP)


#Queue for averaging sensor data
queue = []
counter = 0

#Data Collection Variables
testingArray1 = np.zeros((150, 2))
testingArray2 = np.zeros((150, 2))
testingArray3 = np.zeros((150, 1))
testingArray4 = np.zeros((150, 1))
count = 0

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
    y = 2
    x = 1
    #                          ---REIMPLEMENT THIS LATER---
    #mousestate = mouse.getState() 
   
    #x = mousestate.u 
    #y = -mousestate.v
    
    #                                --- END --- 
    
    leftspeed = 0.0
    rightspeed = 0.0

    if y >= -x and y >= x - 1:
        #Go forward
        #Checking if sensors have detected obstacles and adjusting the max forward speed
        mindist = min(sensorData[0], sensorData[1])
        #Checking the Left sensor facing forwards for if there is an obstacle in the range
        if (mindist < DISTANCE_UPPER_BOUND and mindist > DISTANCE_LOWER_BOUND):
            leftspeed = 2.0 * ((mindist - DISTANCE_LOWER_BOUND)/RANGE)
            rightspeed = 2.0 * ((mindist - DISTANCE_LOWER_BOUND)/RANGE)
        #If the sensor value is lower then the lower bound,
        #Wheelchair cannot move forward, only any other direction 
        elif (mindist <= DISTANCE_LOWER_BOUND):
            print('YOYOYOYOYO1')
            leftspeed = 0.0
            rightspeed = 0.0
        else:
            leftspeed = 2.0
            rightspeed = 2.0
            
        """
        #Checking the Right sensor facing forward for if there is an obstacle in the range
        if (sensorData[1] < DISTANCE_UPPER_BOUND and sensorData[1] > DISTANCE_LOWER_BOUND):
            leftspeed = 2.0 * ((sensorData[1] - DISTANCE_LOWER_BOUND)/RANGE)
            rightspeed = 2.0 * ((sensorData[1] - DISTANCE_LOWER_BOUND)/RANGE)
        elif (sensorData[1] < DISTANCE_LOWER_BOUND):
            print('YOYOYOYOYO2')
            leftspeed = 0.0
            rightspeed = 0.0
        print("forward")
        """
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
    counter += 1
    print(counter)
    sensorData = getSensorData()
    if len(queue) < 5: 
        leftspeed = 0
        rightspeed = 0  

        queue.append(sensorData)
        speeds = trackpad(sensorData)
        setActuators(speeds)
    
    else:
        averages = np.zeros(len(queue[0]))
        for i in range(len(averages)):
            for j in range(len(queue)):
                averages[i] += queue[j][i]
            averages[i] = averages[i] / len(queue)
            sensorData[i] = (sensorData[i] * 0.5) + (averages[i] * 0.5)
            
        speeds = trackpad(sensorData)
        setActuators(speeds)
        queue.pop(0)
        queue.append(sensorData)
    

    #print("Left Wheel Velocity: " + str(wheels[0].getVelocity()))
    #print("Rigth Wheel Velocity: " + str(wheels[1].getVelocity()))
    #print("Left Whell Acceleration: " + str(wheels[0].getAcceleration()))
    #print("Right Wheel Acceleration: " + str(wheels[1].getAcceleration()))

    #### Data Collection for first 300 ticks and Plot making ####
    
    #print("Left Sensor Value: " + str(sensorData[0]))
    #print("Actual Front Left Sensor Distance: " + str(distanceFromFrontLeft.getValue()))

    #print("Right Sensor Value: " + str(sensorData[1]))
    #print("Actual Front Right Sensor Distance: " + str(distanceFromFrontRight.getValue()))

    arrange = np.arange(0, 150, dtype=np.float32)
    j = 1
    while (j < 150):
        arrange[j] = arrange[j]/64
        j += 1

    if (count < 150):
        if (sensorData[0] < 1.3):
            testingArray1[count][0] = sensorData[0]
        if (distanceFromFrontLeft.getValue() < 5):
            testingArray1[count][1] = distanceFromFrontLeft.getValue()
        if (sensorData[1] < 1.3):
            testingArray2[count][0] = sensorData[1]
        if (distanceFromFrontRight.getValue() < 5):
            testingArray2[count][1] = distanceFromFrontRight.getValue() 
        if (count == 0):
            testingArray3[count] = 2.0
            testingArray4[count] = 2.0
        else:
            testingArray3[count] = wheels[0].getVelocity()
            testingArray4[count] = wheels[1].getVelocity()

        count += 1

    if (count == 150):

        testingArray1[ testingArray1==0 ] = np.nan
        testingArray2[ testingArray2==0 ] = np.nan

        fig1 = plt.figure()
        ax = fig1.add_subplot(111)

        ax.set(title="Actual Distance vs FrontLeft Sensor Distance", xlabel="x", ylabel="y")
        ax.plot(arrange, testingArray1[:,0],   label='Data from FrontLeft sensor')
        ax.plot(arrange, testingArray1[:,1],   label='Real-world measurements')

        fig, ax = plt.subplots(constrained_layout=True)
        
        line1 = ax.plot(arrange, testingArray1[:,0], label='Measured Distance')
        line1 = ax.plot(arrange, testingArray1[:,1], label='Actual Distance', linestyle='dashed')
        ax.set(title="Wheel Rotation Speed vs Sensor Measured Distance (Left)", xlabel="Time (s)")

        line3 = ax.plot(arrange, testingArray3, label='Velocity')
        ax.set_ylabel('Distance (m)', color = 'black')
        
        secax = ax.secondary_yaxis('right')
        secax.set_ylabel('Wheel revolution speed (Rad/s)')
        secax.set_color('green')
        #ax[1].set(title="Left Wheel Velocity", xlabel="Time Step", ylabel="Rad/s")
        ax.legend()
        plt.savefig("Figure_1.png")

        fig, ax = plt.subplots(constrained_layout=True)
        
        line1 = ax.plot(arrange, testingArray2[:,0], label='Measured Distance')
        line1 = ax.plot(arrange, testingArray2[:,1], label='Actual Distance', linestyle='dashed')
        ax.set(title="Wheel Rotation Speed vs Sensor Measured Distance (Right)", xlabel="Time (s)")

        line3 = ax.plot(arrange, testingArray4, label='Velocity')
        ax.set_ylabel('Distance (m)', color = 'black')
        
        secax = ax.secondary_yaxis('right')
        secax.set_ylabel('Wheel revolution speed (Rad/s)')
        secax.set_color('green')
        #ax[1].set(title="Left Wheel Velocity", xlabel="Time Step", ylabel="Rad/s")
        ax.legend()
        plt.savefig("Figure_2.png")
        plt.show()
        
    
    
    