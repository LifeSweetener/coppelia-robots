#python
include random

## GLOBALS:
right, counter_ = True, 0                                       ## for properly robots turns
robot, leftJointHandle, rightJointHandle, bumperSensorHandle, backwardModeUntilTime, changeOrientationTime, graph, X, Y = 0, 0, 0, 0, 0, 0, 0, 0, 0

##===========================================================
## Initialize a robot and his position graphics:
def init():
    global robot, leftJointHandle, rightJointHandle, bumperSensorHandle
    robot = sim.getObject('.')
    leftJointHandle = sim.getObject("./leftJoint_")
    rightJointHandle = sim.getObject("./rightJoint_")
    bumperSensorHandle = sim.getObject("./bumperForceSensor_")
    
    ## Graph initialization:
    global graph, X, Y
    graph = sim.getObject('/Graph', {'index': 0})
    X = sim.addGraphStream(graph, 'X coord', 'm')
    Y = sim.addGraphStream(graph, 'Y coord', 'm')
    sim.addGraphCurve(graph, 'XY Robot\'s Position', 2, [X,Y], [0,0], '')

##===========================================================
## Robot's basic behaviour:
def actuation(away):
    counter_N = 7                                                ## turn left or turn right for N cycle steps
    
    global backwardModeUntilTime, changeOrientationTime
    
    currentTime=sim.getSimulationTime()
    result = sim.readForceSensor(bumperSensorHandle)
    if result[0] > 0 and result[1][2] < -5:                      ## if the robot drive into a wall...
        backwardModeUntilTime = currentTime + 0.1 + random()/10  ## then set 0.1-0.2 seconds moving backwards
        position = sim.getObjectPosition(bumperSensorHandle, -1)
        sim.setStringSignal('STRIKE', str(position[0]) + ' ' + str(position[1]))
    
    if (currentTime < backwardModeUntilTime):
        global right, counter_                                   ## sets turns left and right of the robot in certain manner
        counter_ = counter_ + 1
        if (random() < 0.5 and counter_ == 0) or (counter_ > 0 and right == True):      ## turn right...
            sim.setJointTargetVelocity(leftJointHandle, -1)
            sim.setJointTargetVelocity(rightJointHandle, -10)
            if (counter_ == counter_N):
                right = False    ## next time - turn left!
                counter_ = 0
        elif (random() >= 0.5 and counter_ == 0) or (counter_ > 0 and right == False):  ## turn left...
            sim.setJointTargetVelocity(leftJointHandle, -10)
            sim.setJointTargetVelocity(rightJointHandle, -1)
            if (counter_ == counter_N):
                right = True     ## next time - turn right!
                counter_ = 0
    else:
        sim.setJointTargetVelocity(leftJointHandle, 10)
        sim.setJointTargetVelocity(rightJointHandle, 10)
        if away:                                                                                        ## if there is a robot near this robot...
            if (currentTime >= changeOrientationTime):                                                  ## and if 150 milliseconds passed since the last 180 degrees rotation then...
                matrix = sim.getObjectMatrix(robot, -1)                                                 ## get current robot's orientation
                matrix = sim.rotateAroundAxis(matrix, [0,0,1], sim.getObjectPosition(robot, -1), 180)   ## rotate 180 degrees
                sim.setObjectMatrix(robot, -1, matrix)                                                  ## set new robot's orientation (after 180 degrees rotation)
                changeOrientationTime = currentTime + 1.0                                               ## next time rotate only after 150 milliseconds!
    
    ## Graph modifying:
    pos = sim.getObjectPosition(robot, -1)
    sim.setGraphStreamValue(graph, X, pos[0])
    sim.setGraphStreamValue(graph, Y, pos[1])

##===========================================================
## Robot starts to turn around when another robot is nearby:
def interaction():
    N = 4                       ## robots amount
    away = False                ## does current robot has to try to go away?
    for i in range(N):
        rob = sim.getObject('/dr12', {'index': i})
        if robot != rob:
            myPos = sim.getObjectPosition(robot, -1)
            robPos = sim.getObjectPosition(rob, -1)
            if (math.fabs(myPos[0] - robPos[0]) < 0.3) and (math.fabs(myPos[1] - robPos[1]) < 0.3):  ## if another robot is nearby...
                away = True     ## set "away" behaviour of this robot
                break
    
    actuation(away)             ## call behaviour function

def sysCall_thread():
    ## Python thread initialization:
    sim.setThreadAutomaticSwitch(True)
    init()
    
    ## Python thread main loop:
    while sim.getThreadExistRequest() == False:
        sim.handleExtCalls()
        interaction()
