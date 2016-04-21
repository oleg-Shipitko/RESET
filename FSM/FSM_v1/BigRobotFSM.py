import packetBuilder
import robotInitialization
import BigRobotTrajectories

stateNumber = 0
trajectoryPoints = BigRobotTrajectories.Trajectories

class BaseState(object):    
    def run(self):
        self.Execute()
        while(True):            
            new_state = self.check_state()
            if new_state is not None:
                return new_state
    
    def check_state(self):
        for method in type(self).__dict__:
            if not method.startswith('check_'):
                continue
            newState = getattr(self, method)()
            if newState is not None:
                return newState
                
class MoveToPointState(BaseState):
    def __init__(self, robot, x, y, alpha):
        self.robot = robot
        self.x = x
        self.y = y
        self.alpha = alpha
        
    def Execute(self):
        print "Go to point ", self.x, self.y, self.alpha      
        print self.robot.GoToPoint(self.x, self.y, self.alpha)
    
    def check_coordinates(self):        
        currentCoordinates = self.robot.GetCoordinates()                 
        if abs(currentCoordinates[0] - self.x) <= 0.02  and abs(currentCoordinates[1] - self.y) <=0.02 and abs(currentCoordinates[2] - self.alpha) <= 0.087:
            print "Point ", self.x, self.y, self.alpha, " was reached"
            global stateNumber
            if stateNumber + 1 is len(trajectoryPoints): 
                return 0
            stateNumber = stateNumber + 1
            return MoveToPointState(self.robot, trajectoryPoints[stateNumber][0], trajectoryPoints[stateNumber][1], trajectoryPoints[stateNumber][2])             

class RobotActions(object):

    def __init__(self):        
        self.robot = robotInitialization.BigRobot()

    def GoToPoint(self, x, y, alpha):
        currentCoordinates = self.robot.GetCurrentCoordinates()
        desiredRelativeCoordinates = [x - currentCoordinates[0], y - currentCoordinates[1], alpha - currentCoordinates[2]]
        self.robot.RelativeMovement(desiredRelativeCoordinates)
            
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
 
robot = RobotActions()
currentState = MoveToPointState(robot, trajectoryPoints[stateNumber][0], trajectoryPoints[stateNumber][1], trajectoryPoints[stateNumber][2])
while currentState is not 0:    
    currentState = currentState.run()




