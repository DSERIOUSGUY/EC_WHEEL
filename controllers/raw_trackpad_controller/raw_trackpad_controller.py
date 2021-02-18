from controller import Robot
​
TIME_STEP = 64
robot = Robot()
​
wheels = []
wheels.append(robot.getDevice("leftWheel"))
wheels.append(robot.getDevice("rightWheel"))
​
wheels[0].setPosition(float('inf'))
wheels[0].setVelocity(0.0)
​
wheels[1].setPosition(float('inf'))
wheels[1].setVelocity(0.0)
​
leftspeed = 3.0
rightspeed = 3.0
​
mouse = robot.getMouse()
​
mouse.enable(TIME_STEP)
​
while robot.step(TIME_STEP) != -1:
    mousestate = mouse.getState()
   
    x = mousestate.u 
    y = -mousestate.v
    
    
   
    if y >= -x and y >= x - 1:
        #Go forward
        leftspeed = 2.0
        rightspeed = 2.0
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
            
    
    wheels[0].setVelocity(leftspeed)
    wheels[1].setVelocity(rightspeed)