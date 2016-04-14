import prob_motion_model as pmm
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
import ttest

# STM32 USB microcontroller ID
VID = 1155
PID = 22336
SNR = '336234893534'

# Hokuyo socket parameters
TCP_IP = '192.168.0.10'
TCP_PORT = 10940
BUFFER_SIZE = 8192 #4096
# Number of particles
N = 100
# Dimensions of the playing field  
WORLD_X = 3000
WORLD_Y = 2000
# Beacon location: 1(left middle), 2(right lower), 3(right upper)
# BEACONS = [(-62,1000),(3062,-62),(3062,2062)]
BEACONS = [(2994,0),(2994,1996),(0,962)]


class Robot(object):
	def __init__(self):
		self.x = random.random() * WORLD_X
		self.y = random.random() * WORLD_Y
		self.orientation = random.random() * 2.0 * math.pi
		# Dummy sensor noise
		self.sense_noise = 50
		self.sense_angle_noise = math.radians(5)

	def set(self, new_x, new_y, new_orientation):
		if new_x < 0 or new_x > WORLD_X:
			pass
		else:
			self.x = int(new_x)
		if new_y < 0 or new_y > WORLD_Y:
			pass
		else:
			self.y = int(new_y)
		self.orientation = new_orientation % (2 * math.pi)
		
	def move(self, rel_motion):
		new_x, new_y, new_orientation = pmm.prob(self.pose(), rel_motion)	
		new_robot = Robot()
		new_robot.set(new_x, new_y, new_orientation)
		return new_robot

	def r_motion(self):
		"""Return robot current coordinates"""	
		packet = packetBuilder.BuildPacket(commands.getCurentCoordinates)	
		recievedPacket = computerPort.sendRequest(packet.bytearray)
		old = recievedPacket.reply
		time.sleep(0.005)
		packet = packetBuilder.BuildPacket(commands.getCurentCoordinates)	
		recievedPacket = computerPort.sendRequest(packet.bytearray)
		new = recievedPacket.reply
		return [new[0]-old[0], new[1]-old[1],new[2]-old[2]]
		
	def pose(self):
		return self.x, self.y, self.orientation

	def weight(self, measurement):
		beacons_sorted = sort_beacons(self.orientation, BEACONS, self.x, self.y)
		#beacons_sorted = BEACONS
		#print 'SORTED BEACONS: ', beacons_sorted		
		prob = 1.0
		#print 'measurement in weights: ', measurement
		if len(beacons_sorted) != len(measurement):
			#print 'beacons not the same length' 
			return 0.0		
		for i in xrange(len(beacons_sorted)):
			dist = math.sqrt((self.x - beacons_sorted[i][0]) ** 2 + (self.y - beacons_sorted[i][1]) ** 2)
			# Angle between current orientation and global beacons, measure counterclockwise
			fi = angle(angle_conv(beacons_sorted[i][1] - self.y, beacons_sorted[i][0] - self.x) - self.orientation)
			#print 'calculated distance: ', dist, measurement[i][1]
			#print 'calculated angle: ', fi, angle2(measurement[i][0])
			try:
				prob_trans = self.gaussian_trans(dist, self.sense_noise, measurement[i][1])
				#print 'prob_trans: ', prob_trans
				prob_rot = self.gaussian_rot(fi, self.sense_angle_noise, angle2(measurement[i][0]))
				#print 'prob_rot: ', prob_rot/10
				#prob *= ((self.gaussian_trans(dist, self.sense_noise, measurement[i][1])) * \
			 	#	self.gaussian_rot(fi, self.sense_angle_noise, angle2(measurement[i][0])))
				prob *= (prob_trans*(prob_rot/10))
			except IndexError:
				print 'INDEX ERROR'				
				prob *= 1
		#print 'Probability:................................. ', prob
		return prob

	def gaussian_trans(self, mu, sigma, x):
		try:
			return math.exp(- ((mu - (x+29)) ** 2) / (sigma ** 2) / 2.0) / \
				(sigma * math.sqrt(2.0 * math.pi))
		except OverflowError: 
			return 0.0		

	def gaussian_rot(self, mu, sigma, x):
		try:
			return math.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / \
				(sigma * math.sqrt(2.0 * math.pi))
		except OverflowError: 
			return 0.0	

	def __str__(self):
		return 'Particle pose: x = %i mm, y = %i mm, theta = %.2f deg' \
			%(self.x, self.y, np.degrees(self.orientation))

def angle_conv(y, x):
	angle = math.atan2(y, x)
	if angle<0:
		angle += (2*math.pi)
	return angle

def angle(measurement):
	if measurement < 0:
		measurement += (2*math.pi)	
	return measurement

def angle2(measurement):
	if measurement < 3*math.pi/4:
		measurement += (5*math.pi/4)
		return measurement
	else:
		measurement -= (3*math.pi/4)
		return measurement

# Finds angle of lidar start arm from global x axis (takes self.orientation)
def angle3(orientation):
	start = orientation - 3*math.pi/4
	if start < 0:
		start += (2*math.pi)
	return start

# Finds angle between angle3 and beacon (takes angle3 of orientation, and angle_conv of beacon point)
def angle4(arm, beacon):
	angle = beacon - arm
	if angle < 0:
		angle += (2*math.pi)
	return angle

def sort_beacons(orientation,BEACONS,x,y):
	arm = angle3(orientation)
	beacons = [angle_conv(BEACONS[i][1] - y, BEACONS[i][0] - x) for i in xrange(3)]
	order =[angle4(arm,beacon) for beacon in beacons]
	beacons_sort = [BEACONS for (order,BEACONS) in sorted(zip(order,BEACONS)) if order <= 3*math.pi/2]
	return beacons_sort

def init_lidar():
	global s
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	print 'test'	
	s.connect((TCP_IP, TCP_PORT))
	time.sleep(0.1)
	s.send('BM\r')
	data = s.recv(BUFFER_SIZE)
	time.sleep(0.1)	
	for i in xrange(3):
		s.send('GE0000108000\r')
		data = s.recv(BUFFER_SIZE)
		time.sleep(0.1)
	print 'Ready'

def  lidar_scan():
	global myrobot
	global p
	global s
	try:	
		#print 'ITERATION:.......', iteration
		# Move robot; noise is in prob function
		rel_motion = myrobot.r_motion()
		print rel_motion
		myrobot = myrobot.move(rel_motion)
		#print 'Robot after movement: ', myrobot
			
		# Move particles
			
		p2 = [p[i].move(rel_motion) for i in xrange(N)]
		p = p2

		s.send('GE0000108000\r')
		data_lidar = s.recv(BUFFER_SIZE)
		# Lidar sense - returns distance to 3 beacons
		lidar = ttest.update_di(data_lidar) 
		# Calculate the weights 			
		w =[p[i].weight(lidar) for i in xrange(N)]
		w = np.asarray(w)
		w /= w.sum()
		try:
		# Probability random pick - use np.random alg
			p3 = np.random.choice(p, N, p = w)
			p = list(p3)
			mean_val = [(p[i].x, p[i].y, p[i].orientation) for i in xrange(len(p))]

		# Set myrobot to particle with max w
			center = np.mean(mean_val, axis = 0)
			myrobot.x, myrobot.y, myrobot.orientation = center[0], center[1], center[2]
		except:
			print 'error with choice'
			pass			
		print myrobot
	except:			
		traceback.print_exc()
		s.send('QT\r')
		s.shutdown(2)			
		s.close()

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
	switch = True
	while switch:
		old = getCoord()
		old2 = [round(i,5) for i in old]
		time.sleep(0.05)
		new = getCoord()
		new2 = [round(i,5) for i in new]
		if new2 == old2:
			switch = False
			print 'stoped'
			print lidar_scan()
			break
		lidar_scan()
	return new2	
	
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
	#startT = time.time()	
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	#endT = time.time()
	#print 'GetCoord time: ', (endT - startT)	
	#print 'Current robot coordinates: ', recievedPacket.reply
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

# Connect to lidar 
init_lidar()
# Set Robot randomly 
myrobot = Robot()
print 'First: ', myrobot
# Set N random particles
p = [Robot() for i in xrange(N)]

# Initialize PID, Trajectory and Kinematics
initPTC()
iteration = 0
comm_list = {1: globMov, 2: relMov, 3: setCoord, 4: getCoord, 5: stopMotors, 6: lidar_scan}
while True:
	try:
		iteration += 1	
		print '\nList of available commands: \n1 Global Movement\n2 Relative Movement'\
			'\n3 Set Coordinates\n4 Get Coordinates\n5 Stop Movement\n6 Lidar Scan' 	
		command = int(raw_input('Command number: '))
		print comm_list[command]()
		print 'Command ended'
	except:
		traceback.print_exc()
		s.send('QT\r')
		s.shutdown(2)			
		s.close()
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

	
	
	

	


