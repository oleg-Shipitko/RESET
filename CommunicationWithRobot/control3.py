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
from collections import deque
# STM32 USB microcontroller ID
#VID = 1155
#PID = 22336
#SNR = '336234893534'

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
BEACONS = [(-56,1000),(3062,-56),(3055,2014)]


class Robot(object):

	def __init__(self, first):
		"""Initialize robot/particle with random position"""
		if first:
			self.x = random.gauss(514.0, 5) 
			self.y = random.gauss(217.0, 5) 
			self.orientation = 0.0

	def set(self, x_new, y_new, orientation_new):
		"""Set particle position on the field"""
		if 0 <= x_new <= WORLD_X:
			self.x = x_new
		else:
			self.x = random.gauss(1587, 5) 
		if 0 <= y_new <= WORLD_Y:
			self.y = y_new
		else:
			self.y = random.gauss(349, 5)
		self.orientation = orientation_new % (2 * math.pi)
		
	def move(self, delta):
		"""Move particle by creating new one and setting position"""
		# From robot I get relative position. And I can do new relative minus 
		# old relative to get displacement dx, dy, dtheta
		#if delta[2] == 0:
		x_new = self.x + delta[0] + random.gauss(0, 3)
		y_new = self.y + delta[1] + random.gauss(0, 3)
		orientation_new = self.orientation + delta[2] + random.gauss(0, 0.05)	
		new_robot = Robot(False)
		new_robot.set(x_new, y_new, orientation_new)
		return new_robot

	def pose(self):
		return self.x, self.y, self.orientation

	def weight(self, x_rob, y_rob, BEACONS):			
		temp_beac = [(beacon[0] - self.x, beacon[1] - self.y) for beacon in BEACONS]
		beacons = [(math.cos(self.orientation)*beac[0] + math.sin(self.orientation)*beac[1],
				-math.sin(self.orientation)*beac[0] + math.cos(self.orientation)*beac[1])
				for beac in temp_beac]
		beacon = [0, 0, 0]
		num_point = [0, 0, 0]
		for j in xrange(len(x_rob)):
			l1 = abs(math.sqrt((beacons[0][0] - x_rob[j])**2 + 
					(beacons[0][1] - y_rob[j])**2) - 40)
			l2 = abs(math.sqrt((beacons[1][0] - x_rob[j])**2 + 
					(beacons[1][1] - y_rob[j])**2) - 40)
			l3 = abs(math.sqrt((beacons[2][0] - x_rob[j])**2 + 
					(beacons[2][1] - y_rob[j])**2) - 40)
			lmin = l1
			num = 0
			if l2 < lmin:
				lmin = l2
				num = 1
			if l3 < lmin:
				lmin = l3
				num = 2
			beacon[num] += lmin
			num_point[num] += 1
		median =[(beacon[i]/num_point[i]) for i in xrange(3) if num_point[i] != 0]
		try:
			return 1.0/sum(median)
		except ZeroDivisionError:
			return 0
			
	def __str__(self):
		return 'Particle pose: x = %.2f mm, y = %.2f mm, theta = %.2f deg' \
			%(self.x, self.y, np.degrees(self.orientation))
def init_lidar():
	global myrobot
	global p
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


def localisation():
	#relative_motion = [0,0,0]
	#last_position
	global myrobot
	global p
	global s
	global old
	while 1:
		try:
			rel_motion, old = relative_motion()
			print 'rel motion', rel_motion
			if abs(rel_motion[0]) < 0.001 and abs(rel_motion[1]) < 0.001 and abs(rel_motion[2]) < 0.000001:
				print 'Stopped moving'
				print myrobot
				break 
			start = time.time()		
			p2 = [p[i].move(rel_motion) for i in xrange(N)]
			p = p2
			s.send('GE0000108000\r')
			data_lidar = s.recv(BUFFER_SIZE)
			angle, distance = lidar_scan(data_lidar, myrobot.pose()) 
			x_rob, y_rob = p_trans(angle, distance)			
			w =[p[i].weight(x_rob, y_rob, BEACONS) for i in xrange(N)]
			w = np.asarray(w)
			w /= w.sum()
			mean_orientation = mean_angl(p, w)
			try:
				mean_val = [(p[i].x*w[i], p[i].y*w[i]) for i in xrange(N)]
				p3 = np.random.choice(p, N, p = w)
				p = list(p3)
				center = np.sum(mean_val, axis = 0)
				myrobot.set(center[0], center[1], mean_orientation)
			except:
				pass
	
			#print myrobot
			end = time.time()
			#print start - end
		except:			
			traceback.print_exc()
			s.shutdown(2)			
			s.close()

def  lidar_scan2():
	global myrobot
	global p
	global s
	global old
	iteration = int(raw_input('How many time to scan: '))
	try:	
		for numb in xrange(iteration):
			rel_motion, old = relative_motion()
			print rel_motion
			start = time.time()		
			p2 = [p[i].move(rel_motion) for i in xrange(N)]
			p = p2
			s.send('GE0000108000\r')
			data_lidar = s.recv(BUFFER_SIZE)
			angle, distance = lidar_scan(data_lidar, myrobot.pose()) 
			x_rob, y_rob = p_trans(angle, distance)			
			w =[p[i].weight(x_rob, y_rob, BEACONS) for i in xrange(N)]
			w = np.asarray(w)
			w /= w.sum()
			mean_orientation = mean_angl(p, w)
			try:
				mean_val = [(p[i].x*w[i], p[i].y*w[i]) for i in xrange(N)]
				p3 = np.random.choice(p, N, p = w)
				p = list(p3)
				center = np.sum(mean_val, axis = 0)
				myrobot.set(center[0], center[1], mean_orientation)
			except:
				pass
					
			print myrobot
			end = time.time()
			print start - end
	except:			
		print 'Traceback line in lidar: ' 
		traceback.print_exc()
		s.shutdown(2)			
		s.close()

# Calculate robot orientation
def mean_angl(p, w):
	x = 0 
	y = 0
	for i in xrange(100):
		x += w[i]*math.cos(p[i].orientation)
		y += w[i]*math.sin(p[i].orientation)
	angle = math.atan2(y,x)
	if angle < 0:
		return angle + (2*math.pi)
	return angle

# Extract data form lidar scan
def lidar_scan(answer, pose):		
	answer = answer.split('\n')		
	dist = answer[2:-2]
	dist2 = [item[:-1] for item in dist]
	dist4 = ''.join(dist2)			
	step = 0 
	idxh = 3 			
	polar_graph = deque()
	append_pg = polar_graph.append
	angle = deque()	
	append_a = angle.append
	lend = len(dist4)
	while idxh <= lend:
		point = dist_val(dist4[idxh-3:idxh])
		if dist_val(dist4[idxh:idxh+3]) > 1100 and point < 4000:	
			append_pg(point)
			append_a(step)
		idxh += 6		
		step += 0.004363323129985824
	return angle, polar_graph

# Transforms lidar measurement to xy in robot coord sys
def p_trans(agl, pit):
	x_rob = [pit[i] * math.cos(angle5(agl[i])) for i in xrange(len(agl))]
	y_rob = [pit[i] * math.sin(angle5(agl[i])) for i in xrange(len(agl))]
	return x_rob, y_rob

# Transforms lidar point angle in robot coord sys
def angle5(angle):
	if angle >= math.pi/4:
		return angle - math.pi/4
	else:
		return angle + 7*math.pi/4

# Calculate odometry elative motion
def relative_motion():
	"""Return robot current coordinates"""
	global old
	time.sleep(0.08)
	print 'old', old
	packet = packetBuilder.BuildPacket(commands.getCurentCoordinates)	
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	new = recievedPacket.reply
	print 'new', new
	return [(new[0]-old[0])*1000, (new[1]-old[1])*1000, new[2]-old[2]], new

# Calculate dist and angle from raw lidar data
def dist_val(value):	
	try:	
		return ((ord(value[0])-48)<<12)|((ord(value[1])-48)<<6)|(ord(value[2])-48)
	except IndexError:	
		return 0

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
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	
	if recievedPacket.reply != 'Ok':
		raise Exception('add PointToStack failed')
	localisation()

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
	#old2 = [round(i,6) for i in oldCoord]
	#old = oldCoord
	localisation()
	
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
myrobot = Robot(True)
myrobot.x, myrobot.y, myrobot.orientation = 514.0, 217.0, 0.0
print 'First: ', myrobot
# Set N random particles
p = [Robot(True) for i in xrange(N)]
old = [0.0, 0.0, 0.0]
# Initialize PID, Trajectory and Kinematics
initPTC()
iteration = 0
print 'start lidar scan'
lidar_scan2()
#figure, lines = init_xy_plot()
comm_list = {1: globMov, 2: relMov, 3: setCoord, 4: getCoord, 5: stopMotors, 6: lidar_scan2}
while True:
	try:
		iteration += 1	
		print '\nList of available commands: \n1 Global Movement\n2 Relative Movement'\
			'\n3 Set Coordinates\n4 Get Coordinates\n5 Stop Movement\n6 Lidar Scan' 	
		command = int(raw_input('Command number: '))
		if command == 1 or command == 2 or command == 6:
			print comm_list[command]()
		else:
			print comm_list[command]()
		print 'Command ended'
	except:
		print 'Traceback line in main: '
		traceback.print_exc()
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

	
	
	

	


