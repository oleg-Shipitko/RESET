import packetBuilder
import robotInitialization
import BigRobotTrajectories
import time
import lidar

#GLOBAL VARIABLES:
#TODO: Calculate number of cubes inside
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

class CaptureCubes(BaseState):
    def __init__(self, level, cubesNumber = 2, tryNumber = 1):
	self.tryNumber = tryNumber		
        self.level = level
        self.cubesNumber = cubesNumber
	self.capturedCubes = None
        
    def Execute(self):
	self.capturedCubes = robot.TakeCubes()
        print "We want to take ", self.cubesNumber, " cubes from ", self.level, "level"
       
    def check_cubes_were_taken(self):
	if self.capturedCubes is cubesNumber:
		print "Cubes were taken"
		ThrowCubes(self.level) 
	if self.capturedCubes is 1:
		print "Not all cubes were taken"
		return ThrowCubes(self.level, True)  
	if self.capturedCubes is 0:
		return CaptureCubes(self.level, 2, 2)
    
class ThrowCubes(BaseState):
	def __init__(self, level, needRetryPreviousAction = False):
		self.level = level
		self.needRetryPreviousAction = needRetryPreviousAction
	
	def Execute(self):
		print "throw Cubes"
	
	def check_cubes_were_throwed():
wwwqe
		
		if self.needRetryPreviousAction is True:
			print "Second try to take cubes"
			return CaptureCubes(self.level, 2, 2)
		else:
			print "Cubes were throwed. Next action"
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
	initialCoordinates = [0.151, 0.920, 0]
        self.robot = robotInitialization.BigRobot(initialCoordinates)	  

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
        return lidar.localisation()
    
    def TakeCubes(self):
    	print "PickUpCubesManipulator"
	print "OpenCubesManipulator"
	print "PutDownCubesManipulator"
	self.capturedCubes = "CloseCubesManipupuliator"
	return self.capturedCubes
    
    def ThrowCubes(self):
	print "PickUpCubesManipulator"
	print "OpenCubesManipulator"   
        
    
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

trajectoryPoints_left = \
MoveToPointState(0.395, 1.332, 1.046), \
MoveToPointState(0.499, 1.932, 1.046), \
MoveToPointState(1.337, 1.932, 1.046), \
MoveToPointState(0.499, 1.932, 1.046), \
MoveToPointState(1.337, 1.932, 1.046), \
MoveToPointState(0.493, 1.754, 1.046), \
MoveToPointState(0.415, 1.317, 1.046), \
MoveToPointState(0.151, 0.92, 1.046), \
MoveToPointState(0.323, 0.07, -1.046), \
MoveToPointState(0.547, 0.246, -1.046), \
MoveToPointState(0.596, 0.110, -1.046), \
MoveToPointState(0.490, 0.674, -1.046), \
MoveToPointState(0.462, 1.332, -1.046), \


trajectoryPoints_right = MoveToPointState(0, 0, 0), \
PickCubes(2, 2), \
PickTower(1), \
MoveToPointState(0.1, 0.1, 1.57), \
MoveToPointState(0, 0, 0)

'''if GetPlayingFieldSide is 0:
	trajectoryPoints = trajectoryPoints_left
else:
	trajectoryPoints = trajectoryPoints_right'''

trajectoryPoints = trajectoryPoints_left

while (robot.CheckForTheBegginigOfGame())
	time.sleep(1)

startTime = time.time()

'''currentState = trajectoryPoints[stateNumber]
while currentState is not 0:    
    currentState = currentState.run() '''





