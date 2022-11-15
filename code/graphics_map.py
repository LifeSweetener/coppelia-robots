#python
include graphics

##GLOBALS:
window = 0

def showGraphics():
    global window
    window = GraphWin('Graphics Window', 800, 600)
    window.setCoords(5, 3, 15, 10)

def sysCall_thread():
    sim.setThreadAutomaticSwitch(True)
    showGraphics()
    
    temp = ''
    tempFeel = ''
    while True:
        ## ROBOTS STRIKES (BUMPER CONTACTS):
        valueString = sim.getStringSignal('STRIKE')
        if (valueString != temp) and (valueString):
            strikePositionString = valueString
            strikePosition = strikePositionString.split(' ')
            strikePosition = [float(x) for x in strikePosition]
            
            dotToDraw = Circle(Point(strikePosition[0], strikePosition[1]), 0.02)
            dotToDraw.setFill('black')
            try:
                dotToDraw.draw(window)
            except Exception as ex:
                print('Error occured: ' + str(type(ex)) + '\n\n' + str(ex))
            temp = strikePositionString
        
        # ROBOTS PROXIMITY SENSORS:
        valueString = sim.getStringSignal('FEEL')
        if (valueString != tempFeel) and (valueString):
            feelPositionString = valueString
            feelPosition = feelPositionString.split(' ')
            feelPosition = [float(x) for x in feelPosition]
            
            dotToDraw = Circle(Point(feelPosition[0], feelPosition[1]), 0.02)
            dotToDraw.setFill('red')
            try:
                dotToDraw.draw(window)
            except Exception as ex:
                print('Error occured: ' + str(type(ex)) + '\n\n' + str(ex))
            tempFeel = feelPositionString
    
    window.close()
