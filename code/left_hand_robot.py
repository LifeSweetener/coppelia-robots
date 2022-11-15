#python
include random

## GLOBALS:
robot, leftJointHandle, rightJointHandle, changeOrientationTime, graph, X, Y = 0, 0, 0, 0, 0, 0, 0
proximitySensors = 0                        ## datchiki
baseVelocity = 5                            ## skorost
blockTime, Left90Time, flag = 0, 0, False   ## to do turns properly
N = 4                                       ## robots amount

## GO AHEAD:
def goAhead():
    sim.setJointTargetVelocity(leftJointHandle, baseVelocity)
    sim.setJointTargetVelocity(rightJointHandle, baseVelocity)

## GO LEFT:
def turnLeft(rad):
    matrix = sim.getObjectMatrix(robot, -1)                                                     ## get current robot's orientation
    matrix = sim.rotateAroundAxis(matrix, [0,0,5], sim.getObjectPosition(robot, -1), rad)       ## rotate {grad} degrees
    sim.setObjectMatrix(robot, -1, matrix)                                                      ## set new robot's orientation (after {grad} degrees rotation)
    print('Left-Bander2: left ' + str(int(rad*180/math.pi)))

## GO RIGHT:
def turnRight(rad):
    matrix = sim.getObjectMatrix(robot, -1)                                                     ## get current robot's orientation
    matrix = sim.rotateAroundAxis(matrix, [0,0,-5], sim.getObjectPosition(robot, -1), rad)      ## rotate {grad} degrees
    sim.setObjectMatrix(robot, -1, matrix)                                                      ## set new robot's orientation (after {grad} degrees rotation)
    print('Left-Bander2: right ' + str(int(rad*180/math.pi)))

## TURN 180 GRAD:
def turnAround():
    matrix = sim.getObjectMatrix(robot, -1)                                                     ## get current robot's orientation
    matrix = sim.rotateAroundAxis(matrix, [0,0,5], sim.getObjectPosition(robot, -1), math.pi)   ## rotate 180 degrees
    sim.setObjectMatrix(robot, -1, matrix)                                                      ## set new robot's orientation (after 180 degrees rotation)
    one = 1
    if random() < 0.5:
        one = -1
    sim.setObjectPosition(robot, -1, [sim.getObjectPosition(robot, -1)[0]+0.01*one,sim.getObjectPosition(robot, -1)[1]+0.01*one,sim.getObjectPosition(robot, -1)[2]])
    print('Left-Bander2: around 180')

## IS DETECTED OBJECT A ROBOT BUT NOT WALL:
def isObjectARobot(objectHandle):
    for i in range(N):
        if objectHandle == sim.getObject('/dr12', {'index': i}):
            return True
    
    return False

## to correctly draw detected points on map!
def orientation(Rxyz):
    PI = math.pi
    if (Rxyz > -PI/4) and (Rxyz <= PI/4):
        return 'RIGHT'
    elif (Rxyz > PI/4) and (Rxyz <= 3*PI/4):
        return 'TOP'
    elif ((Rxyz > 3*PI/4) and (Rxyz <= 5*PI/4)) or ((Rxyz < -3*PI/4) and (Rxyz >= -5*PI/4)):
        return 'LEFT'
    elif (Rxyz < -PI/4) and (Rxyz >= -3*PI/4):
        return 'BOTTOM'
    

##===========================================================
## Initialize a robot and his position graphics:
def init():
    global robot, leftJointHandle, rightJointHandle, proximitySensors
    robot = sim.getObject('.')
    leftJointHandle = sim.getObject("./leftJoint_")
    rightJointHandle = sim.getObject("./rightJoint_")
    
    proximitySensors = dict()
    proximitySensors.update(front=sim.getObject('./Proximity_sensor', {'index': 0}))
    proximitySensors.update(left=sim.getObject('./Proximity_sensor', {'index': 1}))
    proximitySensors.update(right=sim.getObject('./Proximity_sensor', {'index': 2}))
    
    goAhead()
    
    ## Graph initialization:
    global graph, X, Y
    graph = sim.getObject('/Graph', {'index': 2})
    X = sim.addGraphStream(graph, 'X coord', 'm')
    Y = sim.addGraphStream(graph, 'Y coord', 'm')
    sim.addGraphCurve(graph, 'XY Robot\'s Position', 2, [X,Y], [0,0], '')

##===========================================================
## Robot's basic behaviour:
def actuation():
    global blockTime, Left90Time, flag
    if (sim.getSimulationTime() < blockTime):
        return
    
    if (sim.getSimulationTime() > Left90Time):
        flag = False
    
    detectedObjectHandle, detectedPoint, distance, result = sim.handleProximitySensor(proximitySensors['front'])[-2::-1]
    detectedObjectHandleLeft, detectedPointLeft, distanceLeft, resultLeft = sim.handleProximitySensor(proximitySensors['left'])[-2::-1]
    detectedObjectHandleRight, detectedPointRight, distanceRight, resultRight = sim.handleProximitySensor(proximitySensors['right'])[-2::-1]
    
    for i in range(N):
        if i != 2:
            if detectedObjectHandleRight == sim.getObject('/dr12', {'index': i}):
                resultRight = -1
            if detectedObjectHandle == sim.getObject('/dr12', {'index': i}):
                result = -1
            if detectedObjectHandleLeft == sim.getObject('/dr12', {'index': i}):
                resultLeft = -1
    
    DISTMAX = 0.2
    DISTMIN = 0.05
    PI = math.pi
    if (distanceLeft > DISTMAX) and (not flag):
        turnLeft(PI/20)
        flag = True
        Left90Time = sim.getSimulationTime() + 1.9
    elif (distanceLeft < DISTMIN) and (distanceLeft != 0) and (not flag):
        turnRight(PI/20)
        flag = True
        Left90Time = sim.getSimulationTime() + 1.9
    elif (distanceLeft == 0) and (not flag):
        turnLeft(PI/2)
        flag = True
        Left90Time = sim.getSimulationTime() + 2.5
    
    if (resultRight == 1) and (resultLeft == 1) and (result == 1):
        turnAround()
    elif (resultRight == 0) and (resultLeft == 1) and (result == 1):
        turnRight(PI/2)
    
    ## Draw detected points on map:
    Rxyz = sim.getObjectOrientation(robot, -1)                                 ## rotations floats about axes Z, Y and X respectively
    Rz = list(sim.alphaBetaGammaToYawPitchRoll(Rxyz[0], Rxyz[1], Rxyz[2]))[0]  ## current robot rotation about !Z axe!
    
    if (result):
        if not isObjectARobot(detectedObjectHandle):
            #position = sim.getObjectPosition(detectedObjectHandle, -1) -> ALTERNATIVE!
            if (orientation(Rz) == 'TOP'):
                position = [sim.getObjectPosition(robot, -1)[0], sim.getObjectPosition(robot, -1)[1] + distance]
            elif (orientation(Rz) == 'BOTTOM'):
                position = [sim.getObjectPosition(robot, -1)[0], sim.getObjectPosition(robot, -1)[1] - distance]
            elif (orientation(Rz) == 'LEFT'):
                position = [sim.getObjectPosition(robot, -1)[0] + distance, sim.getObjectPosition(robot, -1)[1]]
            elif (orientation(Rz) == 'RIGHT'):
                position = [sim.getObjectPosition(robot, -1)[0] - distance, sim.getObjectPosition(robot, -1)[1]]
            
            sim.setStringSignal('FEEL', str(position[0]) + ' ' + str(position[1]))
    if (resultLeft):
        if not isObjectARobot(detectedObjectHandleLeft):
            #positionLeft = sim.getObjectPosition(detectedObjectHandleLeft, -1) -> ALTERNATIVE!
            if (orientation(Rz) == 'TOP'):
                positionLeft = [sim.getObjectPosition(robot, -1)[0] - distanceLeft, sim.getObjectPosition(robot, -1)[1]]
            elif (orientation(Rz) == 'BOTTOM'):
                positionLeft = [sim.getObjectPosition(robot, -1)[0] + distanceLeft, sim.getObjectPosition(robot, -1)[1] - distanceLeft]
            elif (orientation(Rz) == 'LEFT'):
                positionLeft = [sim.getObjectPosition(robot, -1)[0], sim.getObjectPosition(robot, -1)[1] - distanceLeft]
            elif (orientation(Rz) == 'RIGHT'):
                positionLeft = [sim.getObjectPosition(robot, -1)[0], sim.getObjectPosition(robot, -1)[1] + distanceLeft]
            
            sim.setStringSignal('FEEL', str(positionLeft[0]) + ' ' + str(positionLeft[1]))
    if (resultRight):
        if not isObjectARobot(detectedObjectHandleRight):
            #positionRight = sim.getObjectPosition(detectedObjectHandleRight, -1) -> ALTERNATIVE!
            if (orientation(Rz) == 'TOP'):
                positionRight = [sim.getObjectPosition(robot, -1)[0] + distanceRight, sim.getObjectPosition(robot, -1)[1]]
            elif (orientation(Rz) == 'BOTTOM'):
                positionRight = [sim.getObjectPosition(robot, -1)[0] - distanceRight, sim.getObjectPosition(robot, -1)[1]]
            elif (orientation(Rz) == 'LEFT'):
                positionRight = [sim.getObjectPosition(robot, -1)[0], sim.getObjectPosition(robot, -1)[1] + distanceRight]
            elif (orientation(Rz) == 'RIGHT'):
                positionRight = [sim.getObjectPosition(robot, -1)[0], sim.getObjectPosition(robot, -1)[1] - distanceRight]
            
            sim.setStringSignal('FEEL', str(positionRight[0]) + ' ' + str(positionRight[1]))
    
    ## Graph modifying:
    pos = sim.getObjectPosition(robot, -1)
    sim.setGraphStreamValue(graph, X, pos[0])
    sim.setGraphStreamValue(graph, Y, pos[1])

def sysCall_thread():
    ## Python thread initialization:
    sim.setThreadAutomaticSwitch(True)
    init()
    
    ## Python thread main loop:
    while not sim.getThreadExistRequest():
        actuation()
