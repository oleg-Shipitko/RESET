import serialWrapper 
import packetBuilder
import packetParser
import sys
import time
from serial.tools import list_ports
import socket
import multiprocessing

VID = 1155
PID = 22336
SNR = '3677346C3034'

class BigRobot(object):

    def __init__(self, initialCoordinates, lock):
        portNumber = self.GetPortNumber()	
        self.computerPort = serialWrapper.SerialWrapper(portNumber)
        self.commands = packetBuilder.CommandsList()
        self.lock = lock
        self.InitializeRobot(initialCoordinates)        
        #self.s = self.InitilizeHokuyo('192.168.0.10', 10940, 8192)

    def InitializeRobot(self, initialCoordinates):
		
		"""Initialize PID, Trajectory, Kinematics"""
		# Build packet for sending to robot	command
		packet = packetBuilder.BuildPacket(self.commands.switchOnPid)
			
		# send packet to port. sendRequest method will wait answer from robot. In case 
		# if you don't need answer possible to use 'sendData' method (just send data, 
		# without waiting for answer)
		startT = time.time()	
		with self.lock:
			recievedPacket = self.computerPort.sendRequest(packet.bytearray)
		endT = time.time()
		print 'Recieved ', (endT - startT)
		if recievedPacket.reply == 'Ok':
			print 'PID controller On'		
		else:
			raise Exception('switchOnPid failed')
		
		packet = packetBuilder.BuildPacket(self.commands.switchOnKinematicCalculation)
		startT = time.time()
		with self.lock:
			recievedPacket = self.computerPort.sendRequest(packet.bytearray)
		endT = time.time()
		print 'Recieved ', (endT - startT)
		if recievedPacket.reply == 'Ok':
			print 'Kinematics ON'
		else:
			raise Exception('switchOnKinematicCalculation failed')
					
		packet = packetBuilder.BuildPacket(self.commands.switchOnTrajectoryRegulator)
		startT = time.time()	
		with self.lock:
			recievedPacket = self.computerPort.sendRequest(packet.bytearray)
		endT = time.time()
		print 'Recieved ', (endT - startT)
		if recievedPacket.reply == 'Ok':
			print 'Trajectory regulator ON'
		else:
			raise Exception('switchOnTrajectoryRegulator failed')
		
		self.SetCoordinates(initialCoordinates)	
        
    def GetPortNumber(self):
        for port in list_ports.comports():
            print port.serial_number, port.pid, port.vid
            if (port.serial_number == SNR) and (port.pid == PID) and (port.vid == VID):	
                return ('/dev/'+ port.name)
    
    def SetCoordinates(self, coordinates):
        print 'set this: ', coordinates
        packet = packetBuilder.BuildPacket(self.commands.setCoordinates, coordinates)
        with self.lock:
			recievedPacket = self.computerPort.sendRequest(packet.bytearray)
    
    def SetCoordCont(self,coordinates):
        print 'Set coord: ', coordinates
        packet = packetBuilder.BuildPacket(self.commands.setCorectCoordinates, coordinates)
        with self.lock:
			recievedPacket = self.computerPort.sendRequest(packet.bytearray)
    
    def GetPlayingFieldSide(self):
        return 0
    
    def CheckForTheBegginigOfGame(self):
        return True

    def GetCurrentCoordinates(self):
        packet = packetBuilder.BuildPacket(self.commands.getCurentCoordinates)
        with self.lock:	
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
        return recievedPacket.GetReply()
    
    def StopRobot(self):
        packet = packetBuilder.BuildPacket(self.commands.switchOffTrajectoryRegulator)
        recievedPacket = self.computerPort.sendRequest(packet.bytearray)
        movementSpeed = [0, 0, 0]	
        packet = packetBuilder.BuildPacket(self.commands.setMovementSpeed, movementSpeed)
        with self.lock:
			recievedPacket = self.computerPort.sendRequest(packet.bytearray)
    
    def CheckForEnemy(self):
        packet_1 = packetBuilder.BuildPacket(self.commands.getADCPinState, 2)
        packet_2 = packetBuilder.BuildPacket(self.commands.getADCPinState, 3)
        packet_3 = packetBuilder.BuildPacket(self.commands.getADCPinState, 4)
        packet_4 = packetBuilder.BuildPacket(self.commands.getADCPinState, 5)
        with self.lock:
            right = self.computerPort.sendRequest(packet_1.bytearray).reply * 0.0822 * 2.54
            back = self.computerPort.sendRequest(packet_2.bytearray).reply * 0.0822 * 2.54
            left = self.computerPort.sendRequest(packet_3.bytearray).reply * 0.0822 * 2.54
            front = self.computerPort.sendRequest(packet_4.bytearray).reply * 0.0822 * 2.54
        return 	front, left, back, right
        
    def RelativeMovement(self, coordinates,currentCoordinates):
        dsplcmnt = [coordinates[0], coordinates[1], coordinates[2], 1]	
        oldCoord = currentCoordinates
        newCoord = [oldCoord[0] + dsplcmnt[0], oldCoord[1] + dsplcmnt[1], oldCoord[2] + dsplcmnt[2], 1]	
        packet = packetBuilder.BuildPacket(self.commands.addPointToStack, newCoord)
        with self.lock:	
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
			
    def globMov(self, x, y, fi):
        coordinates = [x, y, fi, 1]	
        packet = packetBuilder.BuildPacket(self.commands.addPointToStack, coordinates)
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)		
	
    def ActivateRobotAfterStopping(self):
        packet = packetBuilder.BuildPacket(self.commands.switchOnKinematicCalculation)
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
    
    def ReleaseCubeMoovers(self):
        packet = packetBuilder.BuildPacket(self.commands.releaseCubeMoovers)
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
	
    def RaiseCubeMoovers(self):
        packet = packetBuilder.BuildPacket(self.commands.raiseCubeMoovers   )
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
        
    def SetManipulatorAngle(self, angle):
        packet = packetBuilder.BuildPacket(self.commands.setManipulatorAngle, angle)
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
    
    def CloseCubeCollector(self):
        packet = packetBuilder.BuildPacket(self.commands.closeCubeCollector)
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
            
    def OpenCubeCollector(self):
        packet = packetBuilder.BuildPacket(self.commands.openCubeCollector)
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
            
    def SwitchOnVibrationTable(self):
        packet = packetBuilder.BuildPacket(self.commands.switchOnVibrationTable)
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
            
    def SwitchOnBelts(self):
        packet = packetBuilder.BuildPacket(self.commands.switchOnBelts)
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
            
    def SwitchOffBelts(self):
        packet = packetBuilder.BuildPacket(self.commands.switchOffBelts)
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
            
    def StartGame(self):
        packet = packetBuilder.BuildPacket(self.commands.startGame)
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
            
    def SwitchOnVibrateTable(self):
        packet = packetBuilder.BuildPacket(self.commands.switchOnVibrationTable)
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
    
    def SwitchOffVibrationTable(self):
        packet = packetBuilder.BuildPacket(self.commands.switchOffVibrationTable)
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
            
    def SwitchOffBelts(self):
        packet = packetBuilder.BuildPacket(self.commands.switchOffBelts)
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
            
    def SwitchOnBelts(self):
        packet = packetBuilder.BuildPacket(self.commands.switchOnBelts)
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
    
        
    def translation(self, ruler, robot_global):
		"""Translates points from origin coord system, to left point coord
		system coordnate system. New coord sys is paralel with X and Y axis, 
		it's just translated and Y is inverted"""
		robot_local = (robot_global[0] + ruler[0], robot_global[1] + 
		ruler[1])
		
		return robot_local

    def rotation(self, robot_local, angle):
		"""Rotates robot points from left point sys to landmark coord sys"""
		rot_mat = numpy.array([[math.cos(angle), -math.sin(angle)], 
			[math.sin(angle), math.cos(angle)]])
		robot = numpy.asarray(robot_local)
		product = numpy.dot(rot_mat, robot)
		
		return product