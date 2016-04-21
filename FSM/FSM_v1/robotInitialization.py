import serialWrapper 
import packetBuilder
import packetParser
import sys
import time
from serial.tools import list_ports
import socket

VID = '0483'
PID = '5740'
SNR = '336234893534'

class BigRobot(object):

    def __init__(self, initialCoordinates, lock):
        portNumber = self.GetPortNumber()	
        #self.computerPort = serialWrapper.SerialWrapper("/dev/ttyACM2")
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
        """Find all ports, and returns one with defined STM values"""	
        for port in list_ports.comports():
		print port[2]
		if port[2] == "USB VID:PID=0483:5740 SER=336234893534 LOCATION=1-3.3.2":
			return port[0]            
    
    def SetCoordinates(self, coordinates):
        packet = packetBuilder.BuildPacket(self.commands.setCoordinates, coordinates)
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
        packet = packetBuilder.BuildPacket(self.commands.switchOffKinematicCalculation)
        recievedPacket = self.computerPort.sendRequest(packet.bytearray)
        movementSpeed = [0, 0, 0]	
        packet = packetBuilder.BuildPacket(self.commands.setMovementSpeed, movementSpeed)
        with self.lock:
			recievedPacket = self.computerPort.sendRequest(packet.bytearray)
    
    def CheckForEnemy(self):
        '''packet = packetBuilder.BuildPacket(self.commands.getADCPinState, 1)
        with self.lock:
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
        value = recievedPacket * 0.0822 * 2.54;
	
        if value < 5:
            return true'''    
        return False
		
    def RelativeMovement(self, coordinates):
        dsplcmnt = [coordinates[0], coordinates[1], coordinates[2], 1]
        startT = time.time()	
        oldCoord = self.GetCurrentCoordinates()
        newCoord = [oldCoord[0] + dsplcmnt[0], oldCoord[1] + dsplcmnt[1], oldCoord[2] + dsplcmnt[2], 1]
        endT = time.time()		
        packet = packetBuilder.BuildPacket(self.commands.addPointToStack, newCoord)
        with self.lock:	
            recievedPacket = self.computerPort.sendRequest(packet.bytearray)
			
    def globMov(self, x, y, fi):
        coordinates = [x, y, fi, 1]	
        packet = packetBuilder.BuildPacket(commands.addPointToStack, coordinates)
        with lock:
            recievedPacket = computerPort.sendRequest(packet.bytearray)		
	
    def ActivateRobotAfterStopping(self):
        packet = packetBuilder.BuildPacket(self.commands.switchOnKinematicCalculation)
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