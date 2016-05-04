from serial.tools import list_ports
import numpy as np
import serialWrapper 
import packetBuilder
import packetParser
import sys
import time
import socket
import math
import random
import traceback
import multiprocessing
import lidar3_debug
import lidarGui
from ctypes import Structure, c_double
import matplotlib.pyplot as plt

# STM32 USB microcontroller ID
#VID = 1155
#PID = 22336
#SNR = '336234893534'

VID = 1155
PID = 22336
SNR = '3677346C3034'



####################
#      CONTROL     #
####################

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

def portNumber():
	"""Find all ports, and returns one with defined STM values"""	
	for port in list_ports.comports():
		print port 
		print port.serial_number, port.pid, port.vid
		if (port.serial_number == SNR) and (port.pid == PID) and (port.vid == VID):	
			return port.name

def globMov():
	print '\nInput coordinates and speed type'	
	# Get user input for movement
	x = float(raw_input('X: '))
	y = float(raw_input('Y: '))
	fi = float(raw_input('angle: '))
	speed = int(raw_input('speed type (0 = normal, 1 = stop, 2 = stand): '))
	coordinates = [x, y, fi, speed]
	#print 'Movement command: ', coordinates
	
	packet = packetBuilder.BuildPacket(commands.addPointToStack, coordinates)
	with lock:
		recievedPacket = computerPort.sendRequest(packet.bytearray)
		if recievedPacket.reply != 'Ok':
			raise Exception('add PointToStack failed')


def relMov(x = False, y = False, fi = False, speed = False):
	"""Move robot relative to its current coord"""
	print '\nInput robot displacement and speed type'
	if x == False:	
		x = float(raw_input('X: '))
	if y == False:	
		y = float(raw_input('Y: '))
	if fi == False:		
		fi = float(raw_input('angle: '))
	if speed == False:	
		speed = int(raw_input('speed type (0 = normal, 1 = stop, 2 = stand): '))
	dsplcmnt = [x, y, fi, speed]
	#startT = time.time()	
	oldCoord = getCoord()
	newCoord = [oldCoord[0] + dsplcmnt[0], oldCoord[1] + dsplcmnt[1], oldCoord[2]
				 + dsplcmnt[2], speed]
	#endT = time.time()
	#print 'Rel Mov ', (endT - startT)	
	#print 'Displacement: ', dsplcmnt	
	#print 'Old Coord: ', oldCoord	
	#print 'New Coord: ', newCoord	
	packet = packetBuilder.BuildPacket(commands.addPointToStack, newCoord)
	with lock:
		recievedPacket = computerPort.sendRequest(packet.bytearray)

def setStart(x,y,fi):
	coordinates = [x, y, fi]

	packet = packetBuilder.BuildPacket(commands.setCoordinates, coordinates)
	with lock:
		recievedPacket = computerPort.sendRequest(packet.bytearray)
		if recievedPacket.reply != 'Ok':
			raise Exception('setCoordinates failed')

	
	
def setCoord():
	print '\nSet current robot coordinates'	
	x = float(raw_input('X: '))
	y = float(raw_input('Y: '))
	fi = float(raw_input('angle: '))	
	coordinates = [x, y, fi]

	packet = packetBuilder.BuildPacket(commands.setCorectCoordinates, coordinates)
	with lock:
		recievedPacket = computerPort.sendRequest(packet.bytearray)
		if recievedPacket.reply != 'Ok':
			raise Exception('setCoordinates failed')

def setCCoord(x,y,fi):
	sharedcor.value = 1
	coordinates = [x, y, fi]
	while sharedcor.value == 0:
		continue
	packet = packetBuilder.BuildPacket(commands.setCorectCoordinates, coordinates)
	with lock:
		recievedPacket = computerPort.sendRequest(packet.bytearray)
		if recievedPacket.reply != 'Ok':
			raise Exception('setCoordinates failed')

def getCoord():
	"""Return robot current coordinates"""		
	packet = packetBuilder.BuildPacket(commands.getCurentCoordinates)
	#startT = time.time()	
	with lock:
		recievedPacket = computerPort.sendRequest(packet.bytearray)
	#endT = time.time()
	#print 'GetCoord time: ', (endT - startT)	
	#print 'Current robot coordinates: ', recievedPacket.reply
		return recievedPacket.reply

	#packet = packetBuilder.BuildPacket(commands.switchOnKinematicCalculation)
	#recievedPacket = computerPort.sendRequest(packet.bytearray)

def correction():
	#robot = getCoord()
	#print robot
	lidar = shared[:]
	print 'lidar  correction: ', lidar
	#diff = [lidar[0]/1000-robot[0],lidar[1]/1000-robot[1],lidar[2]-robot[2]]
	setCCoord(lidar[0]/1000, lidar[1]/1000, lidar[2])

    
def getLidar():
	return shared[:]
    
def detectStop():
	old = getCoord()
	time.sleep(0.5)
	new = getCoord()
	suma = sum([new[0]-old[0], new[1]-old[1],new[2]-old[2]])
	if suma < 0.001:
		return True
	return False

class Pose(Structure):
    _fields_ = [('x', c_double), ('y', c_double), ('fi', c_double)]
    
def weights():
	""" setup an XY plot canvas """
	global plidar, wlidar
	pipx = [i.x for i in plidar]
	pipy = [i.y for i in plidar]
	pipfi =[math.degrees(i.fi) for i in plidar]
	#we = [(wlidar[i],wlidar[i],wlidar[i]) for i in xrange(100)]
	#print we
	#print pip
	plt.ion()
	n, bins, patches = plt.hist((pipx,pipy,pipfi), bins = 50, weights = (wlidar,wlidar,wlidar), 
								rwidth = 0.9, color = ('b','r','g'))
	plt.grid(True)
	plt.draw()
	raw_input("<Hit Enter To Close>")
	plt.close()
################	
##   START    ##	
################	
	
port = '/dev/'+portNumber()
if port:
	print 'STM32 found on port %s' %port
else:
	print 'No STM32 found. Aborting'
	sys.exit()

# COM port initialization 
computerPort = serialWrapper.SerialWrapper(port)

# we will choose commands which we want to send from this list
commands = packetBuilder.CommandsList()

# Initialize PID, Trajectory and Kinematics
initPTC()
iteration = 0

lock = multiprocessing.Lock()
shared = multiprocessing.Array('d', [0.0, 0.0, 0.0])
sharedcor = multiprocessing.Value('i', 0)
wlidar = multiprocessing.Array('d', 200)
plidar = multiprocessing.Array(Pose, [(0.0, 0.0, 0.0) for i in xrange(200)])
l = multiprocessing.Process(target=lidar3_debug.localisation, args =(lock,shared,computerPort,commands,plidar,wlidar,sharedcor))
l.start()
g = multiprocessing.Process(target=lidarGui.begin, args =(shared,))
g.start()
setStart(0.152,0.72,0.0)
#figure, lines = init_xy_plot()
comm_list = {1: globMov, 2: relMov, 3: setCoord, 4: getCoord, 5: getLidar, 
				6: correction, 7: weights}
while True:
	try:
		iteration += 1	
		print '\nList of available commands: \n1 Global Movement\n2 Relative Movement'\
			'\n3 Set Coordinates\n4 Get Coordinates\n5 Get Lidar\n6 Correction'\
            '\n7 Histogram'
		command = int(raw_input('Command number: '))
		print comm_list[command]()
		print 'Command ended'
		#print shared[:]
	except:
		print 'Traceback line in main: '
		traceback.print_exc()
		sys.exit()
	#Communication test	
	#getCoord()
	#print 'Iteration: ', iteration

#relMov(0.1, 0.1, 1.571, 2)
#print 'new'
#relMov(-0.1, -0.1, -1.571, 2)
#print 'new'
#relMov(0.1, 0.1, 1.571, 2)
#print 'new'
#relMov(-0.1, -0.1, -1.571, 2)
#print 'new'
#relMov(0.1, 0.1, 1.571, 2)
#print 'new'
#relMov(-0.1, -0.1, -1.571, 2)
#print 'stop'