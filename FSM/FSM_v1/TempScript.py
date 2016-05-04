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
startTime = 0
checkTime = 0

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
        is_enemy = False
        #is_enemy = robot.CheckForEnemy()
        if is_enemy:
            robot.StopRobot()
            robot.WaitForEnemyDesapearing()
            return trajectoryPoints[stateNumber]
    
    def check_game_time(self):
        currentTime = time.time()
        print "curTime: ", currentTime
        print "strTime", startTime
        print "diference: ", currentTime - startTime
        if (currentTime - startTime >= 85 and checkTime == 1):
            print "stop work"
            robot.GoToPoint(0.8655, 0.915, -3.14)
            time.sleep(1)
            robot.StopRobot()
            
		
    def execute(self):
        raise NotImplementedError
        
class OpenCubesManipulatorState(BaseState):
    def __init__(self,angle):
        self.angle = angle
    
    def Execute(self):
        self.capturedCubes = robot.OpenCubeManipulator(self.angle) 
       
    def check_cubes_were_taken(self):
       return GoToNextAction() 

class CaptureCubesState(BaseState):
    def __init__(self, angle):
        self.angle = angle
        
    def Execute(self):
        self.capturedCubes = robot.TakeCubes(self.angle) 
       
    def check_cubes_were_taken(self):
       time.sleep(1)
       return GoToNextAction() 

class ReleaseStickState(BaseState):
	def Execute(self):
		robot.ReleaseStick()
	
	def check_stick_was_released(self):		
		return GoToNextAction()
		
class RaiseStickState(BaseState):
	def Execute(self):
		robot.RaiseStick()
	
	def check_stick_was_raised(self):		
		return GoToNextAction()

class SwitchOnVibrationTableState(BaseState):
	def Execute(self):
		robot.robot.SwitchOnVibrateTable()
	
	def check_stick_was_released(self):		
		return GoToNextAction()
		
class SwitchOffVibrationTableState(BaseState):
	def Execute(self):
		robot.robot.SwitchOffVibrationTable()
	
	def check_stick_was_raised(self):		
		return GoToNextAction()

class GetCurrentTimeState(BaseState):
	def Execute(self):
		global startTime
		startTime = time.time()
		global checkTime
		checkTime = 1
		print 'set strat time :',startTime 
	
	def check_time(self):		
		return GoToNextAction()

class SwitchOffBeltsState(BaseState):
	def Execute(self):
		robot.robot.SwitchOffBelts()
	
	def check_stick_was_released(self):		
		return GoToNextAction()
		
class SwitchOnBeltsState(BaseState):
	def Execute(self):
		robot.robot.SwitchOnBelts()
	
	def check_stick_was_raised(self):		
		return GoToNextAction()

class UnloadAllCubesState(BaseState):
	def Execute(self):
		print "Use sucker"
		print "Unload all cubes state"
	
	def check_cube_was_sucked():
		print "Cube was sucked"
		return 0

class WaitUntillTheEndOfTheGame(BaseState):
    def Execute(self):
     print "Wait End"

 
class PutDownCubesManipulatorState(BaseState):
    def Execute(self):
        print "Cubes manipulator was puted down"
    
    def check_time_wait(self):
        time.sleep(2)
        return GoToNextAction()
        
class CorrectCoordinatesState_1(BaseState):
    def __init__(self, x, y, alpha):
        self.x = x
        self.y = y
        self.alpha = alpha
        
    def Execute(self):
        robot.GoToPoint(self.x, self.y, self.alpha)
    
    def check_coordinates(self):        
        currentCoordinates = robot.GetRelative()        
        if abs(currentCoordinates[0] - self.x) <= 0.005  and abs(currentCoordinates[1] - self.y) <=0.005 and abs(get_angles_diff(math.degrees(self.alpha),math.degrees(currentCoordinates[2]))) <= 1:
            coordinates = (0.12, currentCoordinates[1], -1.57)
            robot.robot.SetCoordinates(coordinates)
            return GoToNextAction()
            
class CorrectCoordinatesState_2(BaseState):
    def __init__(self, x, y, alpha):
        self.x = x
        self.y = y
        self.alpha = alpha
        
    def Execute(self):
        robot.GoToPoint(self.x, self.y, self.alpha)
    
    def check_coordinates(self):
        currentCoordinates = robot.GetRelative()
        if abs(currentCoordinates[0] - self.x) <= 0.005  and abs(currentCoordinates[1] - self.y) <=0.005 and abs(get_angles_diff(math.degrees(self.alpha),math.degrees(currentCoordinates[2]))) <= 1:
            coordinates = (currentCoordinates[0], 0.12, -1.57)
            robot.robot.SetCoordinates(coordinates)
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
        if abs(currentCoordinates[0] - self.x) <= 0.005  and abs(currentCoordinates[1] - self.y) <=0.005 and abs(get_angles_diff(math.degrees(self.alpha),math.degrees(currentCoordinates[2]))) <= 1:
            print 'reached end point', currentCoordinates            
            return GoToNextAction()
  
    def check_coordinates(self):        
        currentCoordinates = robot.GetRelative()
        #robot.SetCorrectCoordinates(currentCoordinates)
        #print 'Displ: ', [abs(currentCoordinates[0] - self.x), abs(currentCoordinates[1] - self.y),abs(get_angles_diff(math.degrees(self.alpha),math.degrees(currentCoordinates[2])))]
        if abs(currentCoordinates[0] - self.x) <= 0.005  and abs(currentCoordinates[1] - self.y) <=0.005 and abs(get_angles_diff(math.degrees(self.alpha),math.degrees(currentCoordinates[2]))) <= 1:
            #time.sleep(2)
            #ScurrentCoordinates = robot.GetCoordinates()
            print 'erached end point', currentCoordinates
            #robot.SetCorrectCoordinates(currentCoordinates)
            return GoToNextAction()
        #if abs(currentCoordinates[0] - self.x) <= 0.02  and abs(currentCoordinates[1] - self.y) <=0.02: 
        #    #robot.SetCorrectCoordinates(currentCoordinates)
        #    print "Stopped"
        #    return RotateRobotState()
        #print 'Not reached'
            #return MoveToPointState(self.robot, trajectoryPoints[stateNumber][0], trajectoryPoints[stateNumber][1], trajectoryPoints[stateNumber][2])             
        #robot.SetCorrectCoordinates(currentCoordinates)
        
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
        initialCoordinates = [0.1525, 0.72, 0]
        self.robot = robotInitialization.BigRobot(initialCoordinates,lock)	  

    def GoToPoint(self, x, y, alpha): 
        currentCoordinates = self.GetRelative()
        #print 'Curent coordinates ', currentCoordinates
        #desiredRelativeCoordinates = [x - currentCoordinates[0], y - currentCoordinates[1], alpha - currentCoordinates[2]]
        #self.robot.RelativeMovement(desiredRelativeCoordinates, currentCoordinates)
        #self.robot.SetCoordinates(currentCoordinates)
        self.robot.globMov(x,y,alpha)
        
    def StopRobot(self):
        self.robot.SwitchOffVibrationTable()
        self.robot.SwitchOffBelts()
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
        return [(coords[0]-47.0)/1000, (coords[1]+20.0)/1000, coords[2]]
            
    def GetRelative(self):
        return self.robot.GetCurrentCoordinates()
        
    def SetCorrectCoordinates(self, coordinates):
        self.robot.SetCoordCont(coordinates)
    
    def OpenCubeManipulator(self, angle):
        self.robot.OpenCubeCollector()
        time.sleep(1)
        self.robot.SetManipulatorAngle(angle)
        time.sleep(1)
                
    def TakeCubes(self, manipulatorAngle):
        self.robot.OpenCubeCollector()
        time.sleep(2)
        self.robot.SetManipulatorAngle(manipulatorAngle)
        time.sleep(3)
        self.robot.CloseCubeCollector()
        time.sleep(2)
        self.robot.SetManipulatorAngle(290)
        time.sleep(1)
        self.robot.SetManipulatorAngle(280)
        self.robot.OpenCubeCollector()
        
    def ThrowCubes(self):
        print "PickUpCubesManipulator"
        print "OpenCubesManipulator"
    
    def ReleaseStick(self):
        self.robot.ReleaseCubeMoovers()
        time.sleep(1)
    
    def RaiseStick(self):   
        self.robot.RaiseCubeMoovers()
        time.sleep(1)
    
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
MoveToPointState(0.4244, 0.72, 0),
MoveToPointState(0.4244, 0.935, 0), # start to move cubes
GetCurrentTimeState(),
MoveToPointState(1.161, 0.935, 0),
MoveToPointState(0.4244, 0.935, 0),
CorrectCoordinatesState_1(-0.03, 0.935, -1.57),
#MoveToPointState(0.6025, 0.4375, 0),
MoveToPointState(0.6025, 0.4375, -1.57),
MoveToPointState(1.05, 0.4375, -1.57),
CorrectCoordinatesState_2(1.085, -0.03, -1.57),#near cube
MoveToPointState(1.08, 0.25, -1.57),
MoveToPointState(0.94, 0.24, -1.57),
OpenCubesManipulatorState(180),
MoveToPointState(1.08, 0.25, -1.57),
OpenCubesManipulatorState(275),
MoveToPointState(0.97, 0.4, -1.57),
MoveToPointState(0.97, 0.4, -1.8),
ReleaseStickState(),
MoveToPointState(0.97, 0.22, -1.8),
MoveToPointState(1.04, 0.23, -1.57),
MoveToPointState(0.97, 0.26, -1.57),
RaiseStickState(),
CaptureCubesState(165),
SwitchOnVibrationTableState(),
MoveToPointState(0.97, 0.27, -1.57),
CaptureCubesState(125),
SwitchOffVibrationTableState(),
MoveToPointState(0.962, 0.4375, -1.57),
MoveToPointState(0.6025, 0.4375, -1.57),

MoveToPointState(0.65, 0.11, -1.57), # clse the doors
MoveToPointState(0.65, 0.18, -1.57),
MoveToPointState(0.35, 0.18, -1.57),
MoveToPointState(0.35, 0.11, -1.57),

MoveToPointState(0.6025, 0.4375, -3.14),#
MoveToPointState(0.6025, 0.915, -3.14),
MoveToPointState(0.9655, 0.915, -3.14),
SwitchOnVibrationTableState(),
SwitchOnBeltsState(),
WaitUntillTheEndOfTheGame()]

trajectory_left = [
MoveToPointState(2.847, 0.72, 0),
MoveToPointState(2.6, 0.915, 0),
MoveToPointState(1.855, 0.915, 0),
MoveToPointState(2.326, 0.915, 0),
MoveToPointState(2.326, 0.915, -1.57),
MoveToPointState(2.326, 0.352, -1.57),
#CaptureCubesState(130),
MoveToPointState(2.175, 0.352, -1.57),
#CaptureCubesState(270),
MoveToPointState(2.013, 0.352, -1.57),
MoveToPointState(2.013, 0.312, -1.57),
MoveToPointState(2.013, 0.362, -1.57),
MoveToPointState(2.625, 0.951, -1.57),
MoveToPointState(2.625, 0.951, -3.14),
MoveToPointState(2.043, 0.251, -3.14)]

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
#l = multiprocessing.Process(target=lidar3.localisation, args =(lock, lock_val, shared, computerPort, commands))
#l.start()
time.sleep(2)
startTime = time.time()
currentState = trajectoryPoints[stateNumber]
while currentState is not 0:    
    #raw_input("Press enter...")
    currentState = currentState.run()
