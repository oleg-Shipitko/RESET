#!/usr/bin/env python2
import serialWrapper 
import packetBuilder
import packetParser
from serial.tools import list_ports
import sys
import time
import numpy
import cv2
import get_position
import math 

# STM32 USB microcontroller ID
VID = '0483'
PID = '5740'
SNR = '338434693534'

def initPTC():
	"""Initialize PID, Trajectory, Kinematics"""
	# Build packet for sending to robot	
	packet = packetBuilder.BuildPacket(commands.switchOnPid)
		
	# send packet to port. sendRequest method will wait answer from robot. In case 
	# if you don't need answer possible to use 'sendData' method (just send data, 
	# without waiting for answer)
	startT = time.time()	
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	endT = time.time()
	print 'Recieved ', (endT - startT)
	if recievedPacket.reply == 'Ok':
		print 'PID controller On'		
	else:
		raise Exception('switchOnPid failed')
	
	packet = packetBuilder.BuildPacket(commands.switchOnTrajectoryRegulator)
	startT = time.time()	
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	endT = time.time()
	print 'Recieved ', (endT - startT)
	if recievedPacket.reply == 'Ok':
		print 'Trajectory regulator ON'
	else:
		raise Exception('switchOnTrajectoryRegulator failed')
	
	packet = packetBuilder.BuildPacket(commands.switchOnKinematicCalculation)
	startT = time.time()
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	endT = time.time()
	print 'Recieved ', (endT - startT)
	if recievedPacket.reply == 'Ok':
		print 'Kinematics ON'
	else:
		raise Exception('switchOnKinematicCalculation failed')
		
def getCoord():
	"""Return robot current coordinates"""		
	packet = packetBuilder.BuildPacket(commands.getCurentCoordinates)
	startT = time.time()	
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	endT = time.time()
	print 'GetCoord time: ', (endT - startT)	
	print 'Current robot coordinates: ', recievedPacket.reply
	return recievedPacket.reply
	
def relMov(x, y, fi):
	dsplcmnt = [x, y, fi, 1]
	startT = time.time()	
	oldCoord = getCoord()
	newCoord = [oldCoord[0] + dsplcmnt[0], oldCoord[1] + dsplcmnt[1], oldCoord[2]
				 + dsplcmnt[2], 1]
	endT = time.time()
	#print 'Rel Mov ', (endT - startT)	
	#print 'Displacement: ', dsplcmnt	
	#print 'Old Coord: ', oldCoord	
	#print 'New Coord: ', newCoord	
	packet = packetBuilder.BuildPacket(commands.addPointToStack, newCoord)
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	
def localMov(x, y, fi):
	dsplcmnt = [x, y]
	oldCoord = getCoord()
	newCoordRotated = rotation(dsplcmnt, oldCoord[2])
	newCoordTranslated = translation(newCoordRotated, oldCoord)
	globalCoord = [newCoordTranslated[0], newCoordTranslated[1], oldCoord[2]+ fi, 1]
	print globalCoord
	packet = packetBuilder.BuildPacket(commands.addPointToStack, globalCoord)
	recievedPacket = computerPort.sendRequest(packet.bytearray)

def portNumber():
	"""Find all ports, and returns one with defined STM values"""	
	for port in list_ports.comports():		
		if port[2] == 'USB VID:PID=%s:%s SNR=%s' %(VID,PID,SNR):	
			return port[0]
		return port[0]

def globMov(x, y, fi):	
	coordinates = [x, y, fi, 1]
	print 'Movement command: ', coordinates
	
	packet = packetBuilder.BuildPacket(commands.addPointToStack, coordinates)
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	
	if recievedPacket.reply != 'Ok':
		raise Exception('add PointToStack failed')
		
def getCurrentCoordinates():
	packet = packetBuilder.BuildPacket(commands.getCurentCoordinates)
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	currentCoordinates = recievedPacket.GetReply()
	return currentCoordinates
	

def translation(ruler, robot_global):
	"""Translates points from origin coord system, to left point coord
	system coordnate system. New coord sys is paralel with X and Y axis, 
	it's just translated and Y is inverted"""
	robot_local = (robot_global[0] + ruler[0], robot_global[1] + 
	ruler[1])
	return robot_local

def rotation(robot_local, angle):
	"""Rotates robot points from left point sys to landmark coord sys"""
	rot_mat = numpy.array([[math.cos(angle), -math.sin(angle)], 
		[math.sin(angle), math.cos(angle)]])
	print 'Rot_mat: ', rot_mat
	robot = numpy.asarray(robot_local)
	product = numpy.dot(rot_mat, robot)
	return product
	
			
# COM port initialization 
computerPort = serialWrapper.SerialWrapper("/dev/ttyACM2")

# we will choose commands which we want to send from this list
commands = packetBuilder.CommandsList()

# Initialize PID, Trajectory and Kinematics

initPTC()
objectPosition = get_position.GetObjectPosition()

seashelWasNotDetected = True
#cap = cv2.VideoCapture(1)

while (seashelWasNotDetected):
	time.sleep(5)
	#currentCoordinates = getCurrentCoordinates()
	#print 'Current coordinates: ' + str(currentCoordinates[0]) + ', '+ str(currentCoordinates[1]) + ', '+ str(currentCoordinates[2])
	seashellCoordinates = objectPosition.get_position()
		
	if (seashellCoordinates is None):
			
			print 'Seashell was not found. Turning robot around...'	
			#currentCoordinates[2] = currentCoordinates[2] + (3.14 / 8)
			#packet = packetBuilder.BuildPacket(commands.addPointToStack, currentCoordinates)
			#recievedPacket = computerPort.sendRequest(packet.bytearray)			
	else:
			print 'Seashell was found.'			
			addCoordinates = [-seashellCoordinates[0]/100.0, -seashellCoordinates[1]/100.0 - 0.05, 0, 1]
			theta = -math.atan(1.0*addCoordinates[0] / addCoordinates[1])
			print 'Theta: ' + str(theta)
			print addCoordinates[0]
			print addCoordinates[1]
			print addCoordinates[2]
			
			#addCoordinates = [0, 0, 0, 1]
			
			localMov(addCoordinates[0], addCoordinates[1], theta)
			#packet = packetBuilder.BuildPacket(commands.addPointToStack, addCoordinates)
			#recievedPacket = computerPort.sendRequest(packet.bytearray)
			
			#seashellCoordinates = objectPosition.get_position()
			#print 'Seashell was found. 2nd step: move to the seashell...'					
			
			#print seashellCoordinates[1]
			#print seashellCoordinates[0]
			#print seashellCoordinates[0]
			#print seashellCoordinates[1]
			#print seashellCoordinates[2]
			#theta = atan(seashellCoordinates[1] / seashellCoordinates[0])
			
			#addCoordinates = [seashellCoordinates[0] / 100, seashellCoordinates[1] / 100 , theta, 1]
			#currentCoordinates[0] = currentCoordinates[0] / 1000 + seashellCoordinates[0]
			#currentCoordinates[1] = currentCoordinates[1] / 1000 + seashellCoordinates[1] - 0.08
			#packet = packetBuilder.BuildPacket(commands.addPointToStack, addCoordinates)
			#recievedPacket = computerPort.sendRequest(packet.bytearray)					
			seashelWasNotDetected = False

	
	
	

	


