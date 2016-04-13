import serialWrapper 
import packetBuilder
import packetParser
from serial.tools import list_ports
import sys
import time

# STM32 USB microcontroller ID
VID = 1155
PID = 22336
SNR = '336234893534'

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
	print 'Movement command: ', coordinates
	
	packet = packetBuilder.BuildPacket(commands.addPointToStack, coordinates)
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	
	if recievedPacket.reply != 'Ok':
		raise Exception('add PointToStack failed')
	print 'Wait 5 seconds timeout (robot is moving)....'
	time.sleep(5)
	print 'Get current coodrdinates:'
	packet = packetBuilder.BuildPacket(commands.getCurentCoordinates)
	recievedPacket = computerPort.sendRequest(packet.bytearray)

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
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	
def setCoord():
	print '\nSet current robot coordinates'	
	x = float(raw_input('X: '))
	y = float(raw_input('Y: '))
	fi = float(raw_input('angle: '))	
	coordinates = [x, y, fi]

	packet = packetBuilder.BuildPacket(commands.setCoordinates, coordinates)
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	if recievedPacket.reply != 'Ok':
		raise Exception('setCoordinates failed')

def getCoord():
	"""Return robot current coordinates"""		
	packet = packetBuilder.BuildPacket(commands.getCurentCoordinates)
	startT = time.time()	
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	endT = time.time()
	print 'GetCoord time: ', (endT - startT)	
	print 'Current robot coordinates: ', recievedPacket.reply
	return recievedPacket.reply

def stopMotors():
	coord = getCoord()
	print coord
	print type(coord[0])
	coord.append(0)
	print coord
	#coord = [3.0, -2.0, 0.0, 1]
	packet = packetBuilder.BuildPacket(commands.stopAllMotors, coord)
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	#print 'hi'	
	time.sleep(5)
	coord = [5.0, 0.0, 0.0, 1]
	packet = packetBuilder.BuildPacket(commands.stopAllMotors, coord)
	recievedPacket = computerPort.sendRequest(packet.bytearray)	
	
	return recievedPacket.reply

	#packet = packetBuilder.BuildPacket(commands.switchOnKinematicCalculation)
	#recievedPacket = computerPort.sendRequest(packet.bytearray)
	
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
comm_list = {1: globMov, 2: relMov, 3: setCoord, 4: getCoord, 5: stopMotors}
while True:
	iteration += 1	
	print '\nList of available commands: \n1 Global Movement\n2 Relative Movement'\
		'\n3 Set Coordinates\n4 Get Coordinates\n5 Stop Movement' 	
	command = int(raw_input('Command number: '))
	comm_list[command]()

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

	
	
	

	


