import serialWrapper 
import packetBuilder
import packetParser
from serial.tools import list_ports
import sys

# STM32 USB microcontroller ID
VID = '0483'
PID = '5740'
SNR = '338434693534'

def initPTC():
	"""Initialize PID, Trajectory, Kinematics"""
	# build packet for sending to robot. 'switchOffPid' doesn't require parameters
	packet = packetBuilder.BuildPacket(commands.switchOnPid)
	# send packet to port. sendRequest method will wait answer from robot. In case 
	# if you don't need answer possible to use 'sendData' method (just send data, 
	# without waiting for answer)
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	# If don't get right response, raise exception
	#if recievedPacket.reply == 'Ok':
	#	print 'PID controller ON'		
	#else:
	#	raise Exception('switchOnPid failed')

	packet = packetBuilder.BuildPacket(commands.switchOnTrajectoryRegulator)
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	#if recievedPacket.reply == 'Ok':
	#	print 'Trajectory regulator ON'
	#else:
	#	raise Exception('switchOnTrajectoryRegulator failed')

	packet = packetBuilder.BuildPacket(commands.switchOnKinematicCalculation)
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	#if recievedPacket.reply == 'Ok':
	#	print 'Kinematics ON'
	#else:
	#	raise Exception('switchOnKinematicCalculation failed')

def port_number():
	"""Find all ports, and returns one with defined STM values"""	
	for port in list_ports.comports():
		if port[2] == 'USB VID:PID=%s:%s SNR=%s' %(VID,PID,SNR):	
			return port[0]

def movement():
	print '\nInput coordinates and speed type'	
	# Get user input for movement
	x = float(raw_input('X: '))
	y = float(raw_input('Y: '))
	fi = float(raw_input('angle: '))
	speed = int(raw_input('speed type (1 = normal, 2 = stop, 3 = stand): '))
	coordinates = [x, y, fi, speed]
	print 'Movement command: ', coordinates

	# build packet for sending to robot. 'setCoordinates' command require 
	# 3 parameters (x , y, angle(rad)). 
	packet = packetBuilder.BuildPacket(commands.addPointToStack, coordinates)
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	
	#if recievedPacket.reply != 'Ok':
	#	raise Exception('add PointToStack failed')
	
	packet = packetBuilder.BuildPacket(commands.getCurentCoordinates)
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	#print type(recievedPacket.string)
	#print type(recievedPacket.byteArray)
	#print type(recievedPacket.reply)
	#for i in recievedPacket.reply:
	#	print unichr(i)
	#print '\nCurrent coordinates: ', recievedPacket.reply
	
def setCoord():
	print '\nSet current robot coordinates'	
	x = float(raw_input('X: '))
	y = float(raw_input('Y: '))
	fi = float(raw_input('angle: '))	
	coordinates = [x, y, fi]

	packet = packetBuilder.BuildPacket(commands.setCoordinates, coordinates)
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	#if recievedPacket.reply != 'Ok':
	#	raise Exception('setCoordinates failed')

port = port_number()
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

while True:
	print '\nList of available commands: \n1 Movement\n2 Get Coordinates',\
			'\n3 Set Coordinates' 	
	command = raw_input('Command number: ')
	if command == '1':
		movement()
	elif command == '2':
		pass
	elif command == '3':
		setCoord()	
	




	
	
	

	


