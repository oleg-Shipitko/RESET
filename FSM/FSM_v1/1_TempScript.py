import packetBuilder
import robotInitialization
import BigRobotTrajectories
import time
import lidar3
import traceback
import multiprocessing
import math

#TODO: Calculate number of cubes inside
stateNumber = 0

def get_angles_diff(a1, a2):
    target = (a1 + 180) % 360 - 180
    cur = (a2 + 180) % 360 - 180
    return (target - cur + 180) % 360 - 180

def GoToNextAction():
     global stateNumber
     if stateNumber + 1 is len(trajectoryPoints): 
        return 0
     stateNumber = stateNumber + 1
     return trajectoryPoints[stateNumber]
	
class BaseState(object):    
    def run(self):
        #print('base state start')
        self.Execute()
        while(True):
            #print('base state iter')
            new_state = self.check_state()
            #print('new state', new_state)
            if new_state is not None:
                return new_state
    
    def check_state(self):
        for method in dir(self):
                if not method.startswith('check_') or method == 'check_state':
                    continue
                #print('===', method)
                newState = getattr(self, method)()
                #print('new_state', newState)
                if newState is not None:
                    return newState
    
    def check_for_enemy(self):
        is_enemy = robot.CheckForEnemy()
        #print('-------', is_enemy)
        if is_enemy:
            robot.StopRobot()
            robot.WaitForEnemyDesapearing()
            return trajectoryPoints[stateNumber]
    
    def check_game_time(self):
        currentTime = time.time()
        if (startTime - currentTime >= 88):
            robot.StopRobot()
		
    def execute(self):
        raise NotImplementedError

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
        if self.capturedCubes is 1 and tryNumber is 1:
            print "Not all cubes were taken. 1-st try."
            return ThrowCubes(self.level, self.cubesNumber, True)
        if self.capturedCubes is 1 and tryNumber is 2:
            print "Cubes were not been. 2-nd try."
            return CaptureCubes(self.level, self.cubesNumber, True)
			
    def check_cubes_were_not_taken(self):
        if self.capturedCubes is 0 and tryNumber is 1:
            print "Cubes were not taken. 1-st try."
            return CaptureCubes(self.level, True)
        if self.capturedCubes is 0 and tryNumber is 2:
            print "Cubes were not taken. 2-nd try."
            return GoToNextAction()
    
class ThrowCubes(BaseState):
    def __init__(self, level, needRetryPreviousAction = False):
        self.level = level
        self.needRetryPreviousAction = needRetryPreviousAction
	
    def Execute(self):
        print "throw Cubes"
	
    def check_cubes_were_throwed():		
        if self.needRetryPreviousAction is True:
			print "Second try to take cubes"
			return CaptureCubes(self.level, 2, 2)
        else:
			print "Cubes were throwed. Next action"
			return GoToNextAction()

class ReleaseStickState(object):
	def Execute(self):
		robot.ReleaseStick()
	
	def check_stick_was_released(self):
		time.sleep(3)
		return GoToNextAction()
		
class RaiseStickState(object):
	def Execute(self):
		robot.RaiseStick()
	
	def check_stick_was_raised(self):
		time.sleep(2)
		return GoToNextAction()

class UnloadAllCubesState(object):
	def Execute(self):
		print "Use sucker"
		print "Unload all cubes state"
	
	def check_cube_was_sucked():
		print "Cube was sucked"
		return GoToNextAction()

 
class PutDownCubesManipulatorState(BaseState):
    def Execute(self):
        print "Cubes manipulator was puted down"
    
    def check_time_wait(self):
        time.sleep(2)
        return GoToNextAction()
                
class MoveToPointState(BaseState):
    def __init__(self, x, y, alpha):                
        self.x = x
        self.y = y
        self.alpha = alpha
        
    def Execute(self):
        print "Go to point ", self.x, self.y, self.alpha      
        robot.GoToPoint(self.x, self.y, self.alpha)    
  
    def check_coordinates(self):        
        currentCoordinates = robot.GetRelative()
        #robot.SetCorrectCoordinates(currentCoordinates)
        #print 'Displ: ', [abs(currentCoordinates[0] - self.x), abs(currentCoordinates[1] - self.y),abs(get_angles_diff(math.degrees(self.alpha),math.degrees(currentCoordinates[2])))]
        if abs(currentCoordinates[0] - self.x) <= 0.02  and abs(currentCoordinates[1] - self.y) <=0.02 and abs(get_angles_diff(math.degrees(self.alpha),math.degrees(currentCoordinates[2]))) <= 4:
            #currentCoordinates = robot.GetCoordinates()
            #robot.SetCorrectCoordinates(currentCoordinates)
            time.sleep(2)
            return GoToNextAction()
        #if abs(currentCoordinates[0] - self.x) <= 0.02  and abs(currentCoordinates[1] - self.y) <=0.02: 
        #    #robot.SetCorrectCoordinates(currentCoordinates)
        #    print "Stopped"
        #    return RotateRobotState()
        #print 'Not reached'
            #return MoveToPointState(self.robot, trajectoryPoints[stateNumber][0], trajectoryPoints[stateNumber][1], trajectoryPoints[stateNumber][2])             
        #robot.SetCorrectCoordinates(currentCoordinates)
        
class RotateRobotState(BaseState):
    def __init__(self, alpha):
        self.alpha = alpha
        
    def Execute(self):
        currentCoordinates = robot.GetCoordinates()
        robot.GoToPoint(currentCoordinates[0], currentCoordinates[1], self.alpha)
        
    def check_angle(self):
        currentCoordinates = robot.GetCoordinates()
        if abs(get_angles_diff(math.degrees(self.alpha),math.degrees(currentCoordinates[2]))) <= 2:
            return GoToNextAction()
        
class UnloadCubes(object):
    def __init__(self):
        self.startTime = time.time()
        
    def Execute(self):
        robot.SwitchOnVibrationTrack()
        robot.SwitchOnTrack()
        
    def check_all_cubes_were_unloaded(self):
        if robot.IsCubeInRobot() is not True:
            robot.SwitchOnVibrationTrack()
            robot.SwitchOnTrack()
            return GoToNextAction()
    
    def check_time_for_operation(self):
        if time.time() - self.startTime > 10:
            robot.SwitchOnVibrationTrack()
            robot.SwitchOnTrack()
            return GoToNextAction()
        

class BigRobot(object):
	
    def __init__(self, lock):        
        initialCoordinates = [0.149, 0.734, 0]
        self.robot = robotInitialization.BigRobot(initialCoordinates,lock)	  

    def GoToPoint(self, x, y, alpha):
        currentCoordinates = self.GetRelative()
        #print 'Curent coordinates ', currentCoordinates
        #desiredRelativeCoordinates = [x - currentCoordinates[0], y - currentCoordinates[1], alpha - currentCoordinates[2]]
        #self.robot.RelativeMovement(desiredRelativeCoordinates, currentCoordinates)
        #self.robot.SetCoordinates(currentCoordinates)
        self.robot.globMov(x,y,alpha)
        
    def StopRobot(self):
        self.robot.StopRobot()
	
    def CheckForEnemy(self):
        return self.robot.CheckForEnemy()
    
    def WaitForEnemyDesapearing():
        while(self.robot.CheckForEnemy()):
            time.sleep(1)
        self.robot.ActivateRobotAfterStopping()			
            
    def GetCoordinates(self):
        time.sleep(0.05)
        #lock_val.acquire()
        #print 'control in main'
        coords = shared[:]
        #lock_val.release()
        return [(coords[0]-4.0)/1000, (coords[1]-3.0)/1000, coords[2]]
        
    def GetRelative(self):
        return self.robot.GetCurrentCoordinates()
        
    def SetCorrectCoordinates(self, coordinates):
        self.robot.SetCoordCont(coordinates)
        
    def TakeCubes(self):
    	print "PickUpCubesManipulator"
        print "OpenCubesManipulator"
        print "PutDownCubesManipulator"
        self.capturedCubes = "CloseCubesManipupuliator"
        return self.capturedCubes
    
    def ThrowCubes(self):
        print "PickUpCubesManipulator"
        print "OpenCubesManipulator"
    
    def ReleaseStick(self):
        print "Stick was released"
    
    def RaiseStick(self):   
        print "Stick was raised"
    
    def UnloadCubesState(self):
        #While we have cubes unload cubes
        print "cubes were unloaded"
        
    def SwitchOnVibrationTrack(self):
        print "Vibration track was switched on"
    
    def SwitchOffVibrationTrack(self):
        print "Vibration track was switced off"
    
    def SwitchOnTrack(self):
        print "Track was switched on"
    
    def SwitchOfTrack(self):
        print "Track was switched off"
    
    def IsCubeInRobot(self):
        print "Check is cube in robot"
        return True
    
    #def PickTower(self, activeSide, floor):
		
	#def CheckTower(self, activeSide):
		
	#def MoveTrack(self, positionNumber):
#### BEGINNING OF THE MAIN PROGRAM ####


trajectory_right = [
MoveToPointState(0.149, 0.734, 0),
MoveToPointState(0.403, 0.942, 0),
MoveToPointState(1.089, 0.942, 1.57),
MoveToPointState(0.403, 0.932, 0),
MoveToPointState(0.567, 0.410, 0),
MoveToPointState(0.945, 0.478, 0),
MoveToPointState(0.945, 0.478, 1.57),
MoveToPointState(0.945, 0.312, 1.57),
MoveToPointState(1.003, 0.302, 1.57),
MoveToPointState(0.945, 0.312, 1.57),
MoveToPointState(0.945, 0.312, 0),
MoveToPointState(0.945, 0.312, -1.57),
MoveToPointState(0.945, 0.478, -1.57),
MoveToPointState(0.567, 0.410, -1.57),
MoveToPointState(0.567, 0.410, 0),
MoveToPointState(0.789, 1.004, 0),
MoveToPointState(0.789, 1.004, 3.14),
MoveToPointState(0.984, 1.011, 3.14)]

trajectory_left = [
MoveToPointState(2.886, 0.788, 3.14),
MoveToPointState(2.6, 0.985, 3.14),
MoveToPointState(1.915, 0.985, 3.14),
MoveToPointState(2.600, 0.985, 3.14),
MoveToPointState(2.588, 0.193, 3.14),
MoveToPointState(2.389, 0.183, 3.14),
MoveToPointState(2.311, 0.449, 3.14),
MoveToPointState(2.311, 0.449, -1.57),
MoveToPointState(1.991, 0.527, -1.57),
MoveToPointState(1.991, 0.308, -1.57),
MoveToPointState(1.991, 0.527, -1.57),
MoveToPointState(2.311, 0.449, -1.57),
MoveToPointState(2.600, 0.985, 3.14),
MoveToPointState(2.600, 0.985, 0),
MoveToPointState(2.088, 0.92, 0)]

'''if GetPlayingFieldSide is 0:
	trajectoryPoints = trajectoryPoints_left
else:
	trajectoryPoints = trajectoryPoints_right'''

trajectoryPoints = trajectory_right
'''try:
    robot = BigRobot()
    lidar_robot = lidar2.Robot(True)
    lidar_robot.set(151.0, 920.0, 0.0)
    particle = [lidar2.Robot(True) for i in xrange(100)]
    while (robot.CheckForTheBegginigOfGame())
        time.sleep(1)

    startTime = time.time()
    currentState = trajectoryPoints[stateNumber]
    while currentState is not 0:    
        raw_input("Press enter...")
        currentState = currentState.run() 
except:
    traceback.print_exc()
    robot.robot.s.shutdown(2)			
    robot.robot.s.close()'''

lock = multiprocessing.Lock()
lock_val = multiprocessing.Lock()
shared = multiprocessing.Array('d', [0.0, 0.0, 0.0], lock = False)	
robot = BigRobot(lock)
computerPort = robot.robot.computerPort
commands = robot.robot.commands
l = multiprocessing.Process(target=lidar3.localisation, args =(lock, lock_val, shared, computerPort, commands))
l.start()
startTime = time.time()
currentState = trajectoryPoints[stateNumber]
while currentState is not 0:    
    raw_input("Press enter...")
    currentState = currentState.run()
