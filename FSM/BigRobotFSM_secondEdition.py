import packetBuilder
import robotInitialization
import BigRobotTrajectories
import time

stateNumber = 0

class BaseState(object):    
    def run(self):
        self.Execute()
        while(True):            
            new_state = self.check_state()
            if new_state is not None:
                return new_state
    
    def check_state(self):
	#TODO remove next string if everything OK
        #for method in type(self).__dict__:
	for method in dir(self):
            if not method.startswith('check_'):
                continue
            newState = getattr(self, method)()
            if newState is not None:
                return newState
    
    def check_for_enemy(self):
	if robot.CheckForEnemy():
		robot.StopRobot()
		robot.WaitForEnemyDesapearing()
		return trajectoryPoints[stateNumber]
    
    def check_game_time(self):
	currentTime = time.time()
	if (startTime - currentTime >= 88):
		robot.StopRobot()

class PickCubes(BaseState):
    def __init__(self, level, cubesNumber):
        self.level = level
        self.cubesNumber = cubesNumber
        
    def Execute(self):
        print "We want to take ", self.cubesNumber, " cubes from ", self.level, "level"
    
    def check_is_cubes_were_taken(self):
        print "Cubes were taken"
        global stateNumber
        stateNumber = stateNumber + 1
        return trajectoryPoints[stateNumber]

class PickTower(BaseState):
    def __init__(self, level):
        self.level = level        
        
    def Execute(self):
        print "We want to take tower from ", self.level, "level"
    
    def check_is_cubes_were_taken(self):
        print "Tower was taken"
        global stateNumber
        stateNumber = stateNumber + 1
        return trajectoryPoints[stateNumber]
                
class MoveToPointState(BaseState):
    def __init__(self, x, y, alpha):                
        self.x = x
        self.y = y
        self.alpha = alpha
        
    def Execute(self):
        print "Go to point ", self.x, self.y, self.alpha      
        print robot.GoToPoint(self.x, self.y, self.alpha)    
  
    def check_coordinates(self):        
        currentCoordinates = robot.GetCoordinates()                 
        if abs(currentCoordinates[0] - self.x) <= 0.02  and abs(currentCoordinates[1] - self.y) <=0.02 and abs(currentCoordinates[2] - self.alpha) <= 0.087:
            print "Point ", self.x, self.y, self.alpha, " was reached"
            global stateNumber
            if stateNumber + 1 is len(trajectoryPoints): 
                return 0
            stateNumber = stateNumber + 1
            return trajectoryPoints[stateNumber]
            #return MoveToPointState(self.robot, trajectoryPoints[stateNumber][0], trajectoryPoints[stateNumber][1], trajectoryPoints[stateNumber][2])             

class BigRobot(object):

    def __init__(self):        
        self.robot = robotInitialization.BigRobot()        

    def GoToPoint(self, x, y, alpha):
        currentCoordinates = self.robot.GetCurrentCoordinates()
        desiredRelativeCoordinates = [x - currentCoordinates[0], y - currentCoordinates[1], alpha - currentCoordinates[2]]
        self.robot.RelativeMovement(desiredRelativeCoordinates)
    
    def StopRobot(self):
	self.robot.StopRobot()
	
    def CheckForEnemy(self):
	return self.robot.CheckForEnemy()
    
    def WaitForEnemyDesapearing():
	while(self.robot.CheckForEnemy()):
		time.sleep(1)
	self.robot.ActivateRobotAfterStopping()			
            
    def GetCoordinates(self):
        return self.robot.GetCurrentCoordinates()
    
	#def PickTower(self, activeSide, floor):
		
	#def CheckTower(self, activeSide):
		
	#def MoveTrack(self, positionNumber):
		
	#def PickCubes(self, floor):	
		
	#def ThrowTower(self, activeSide):
		
	#def ThrowCubes(self, cubesNumber):
		
	#def TurnOnGrab(self):
		
	#def TurnOffGrab(self):

#### BEGINNING OF THE MAIN PROGRAM ####

robot = BigRobot()

trajectoryPoints

trajectoryPoints_left = MoveToPointState(0, 0, 0), \
PickCubes(2, 2), \
PickTower(1), \
MoveToPointState(-0.1, -0.1, -1.57), \
MoveToPointState(0, 0, 0)

trajectoryPoints_right = MoveToPointState(0, 0, 0), \
PickCubes(2, 2), \
PickTower(1), \
MoveToPointState(0.1, 0.1, 1.57), \
MoveToPointState(0, 0, 0)

if GetPlayingFieldSide is 0:
	trajectoryPoints = trajectoryPoints_left
else:
	trajectoryPoints = trajectoryPoints_right

while (robot.CheckForTheBegginigOfGame())
	time.sleep(1)

startTime = time.time()

currentState = trajectoryPoints[stateNumber]
while currentState is not 0:    
    currentState = currentState.run() 





