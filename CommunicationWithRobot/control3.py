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
import matplotlib.pyplot as plt
 
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
BEACONS = [(-56,1000),(3062,-56),(3055,2014)]
#BEACONS = [(2994,0),(2994,1996),(0,962)]


class Robot(object):
	def __init__(self):
		self.x = random.gauss(1587, 50) 
		self.y = random.gauss(349, 50) 
		self.orientation = random.gauss(math.pi/2, 0.1)
		# Dummy sensor noise
		self.sense_noise = 50
		self.sense_angle_noise = math.radians(5)

	def set(self, new_x, new_y, new_orientation):
		if new_x < 0 or new_x > WORLD_X:
			pass
		else:
			self.x = new_x
		if new_y < 0 or new_y > WORLD_Y:
			pass
		else:
			self.y = new_y
		self.orientation = new_orientation % (2 * math.pi)
		
	def move(self, rel_motion):
		new_x, new_y, new_orientation = pmm.prob(self.pose(), rel_motion)	
		new_robot = Robot()
		new_robot.set(new_x, new_y, new_orientation)
		return new_robot

	def r_motion(self):
		"""Return robot current coordinates"""	
		time.sleep(0.03)
		packet = packetBuilder.BuildPacket(commands.getCurentCoordinates)	
		recievedPacket = computerPort.sendRequest(packet.bytearray)
		old = recievedPacket.reply
		#print 'old', old
		time.sleep(0.03)
		packet = packetBuilder.BuildPacket(commands.getCurentCoordinates)	
		recievedPacket = computerPort.sendRequest(packet.bytearray)
		new = recievedPacket.reply
		#print 'new', new
		#print 'difference: ', [new[0]-old[0], new[1]-old[1],new[2]-old[2]]
		return [(new[0]-old[0])*1000, (new[1]-old[1])*1000,new[2]-old[2]]
		
	def pose(self):
		return self.x, self.y, self.orientation

	def weight(self, x_rob, y_rob):		
		beacons = ttest.beacon_transform1(BEACONS, self.pose())
		#bx = [beacons[0][0],beacons[1][0],beacons[2][0]]
		#by = [beacons[0][1],beacons[1][1],beacons[2][1]]
		#update_xy_plot(bx + x_rob, by+y_rob)	
		beacon = [0, 0, 0]
		num_point = [0, 0, 0]
		for j in xrange(len(x_rob)):
			l1 = abs(math.sqrt((beacons[0][0] - x_rob[j])**2 + (beacons[0][1] - y_rob[j])**2) - 40)
			l2 = abs(math.sqrt((beacons[1][0] - x_rob[j])**2 + (beacons[1][1] - y_rob[j])**2) - 40)
			l3 = abs(math.sqrt((beacons[2][0] - x_rob[j])**2 + (beacons[2][1] - y_rob[j])**2) - 40)
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
		return 1.0/sum(median)
	
	def __str__(self):
		return 'Particle pose: x = %i mm, y = %i mm, theta = %.2f deg' \
			%(self.x, self.y, np.degrees(self.orientation))

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
	switch = True
	
	try:	
		while switch:
			rel_motion = myrobot.r_motion()
			#print 'REL MOTION: ', rel_motion
			if rel_motion[0] < 0.000001 and rel_motion[1] < 0.000001 and rel_motion[2] < 0.00000001:
				print myrobot
				break    
			p2 = [p[i].move(rel_motion) for i in xrange(N)]
			p = p2

			s.send('GE0000108000\r')
			data_lidar = s.recv(BUFFER_SIZE)
			# Lidar sense - returns distance to 3 beacons
			angle, distance = ttest.update_di2(data_lidar, myrobot.pose())
			# Calculate the weights 			
			x_rob, y_rob = ttest.p_trans(angle, distance) 			
			w =[p[i].weight2(x_rob, y_rob) for i in xrange(N)]
			w = np.asarray(w)
			w /= w.sum()
			mean_orientation = mean_angl(p, w)
			try:
				# Probability random pick - use np.random alg
				mean_val = [(p[i].x*w[i], p[i].y*w[i], p[i].orientation*w[i]) for i in xrange(len(p))]
				p3 = np.random.choice(p, N, p = w)
				p = list(p3)
	
				center = np.sum(mean_val, axis = 0)
				myrobot.x, myrobot.y, myrobot.orientation = center[0], center[1], mean_orientation
			except:
				print 'error with choice'
				pass			
			print myrobot
	except:			
		print 'Traceback line in lidar: ' 
		traceback.print_exc()
		s.shutdown(2)			
		s.close()

def  lidar_scan2():
	global myrobot
	global p
	global s
	iteration = int(raw_input('How many time to scan: '))
	try:	
		for numb in xrange(iteration):
			rel_motion2 = myrobot.r_motion()
			rel_motion = [rel_motion2[0]*1000, rel_motion2[1]*1000, rel_motion2[2]]
			# Move particles	
			p2 = [p[i].move(rel_motion) for i in xrange(N)]
			p = p2

			s.send('GE0000108000\r')
			data_lidar = s.recv(BUFFER_SIZE)
			# Lidar sense - returns distance to 3 beacons
			angle, distance = ttest.update_di2(data_lidar, myrobot.pose())
			# Calculate the weights
			x_rob, y_rob = ttest.p_trans(angle, distance) 			
			w =[p[i].weight2(x_rob, y_rob) for i in xrange(N)]
			w = np.asarray(w)
			w /= w.sum()
			mean_orientation = mean_angl(p, w)
			try:
			# Probability random pick - use np.random alg
				mean_val = [(p[i].x*w[i], p[i].y*w[i], p[i].orientation*w[i]) for i in xrange(len(p))]
				max_w = np.max(w)
				p3 = np.random.choice(p, N, p = w)
				p = list(p3)
	
				# Set myrobot to particle with max w
				center = np.sum(mean_val, axis = 0)
				myrobot.x, myrobot.y, myrobot.orientation = center[0], center[1], mean_orientation
			except:
				print 'error with choice'
				pass			
			print myrobot
	except:			
		print 'Traceback line in lidar: ' 
		traceback.print_exc()
		s.shutdown(2)			
		s.close()

def mean_angl(particles, w):
	x, y = 0, 0
	for i, p in enumerate(particles):
		x += w[i]*math.cos(p.orientation)
		y += w[i]*math.sin(p.orientation)
	angle = math.atan2(y,x)
	if angle < 0:
		return angle + (2*math.pi)
	return angle

def init_xy_plot():
	""" setup an XY plot canvas """
	plt.ion()
	figure = plt.figure(figsize=(6, 4),
						dpi=200,
						facecolor="w",
						edgecolor="k")
	ax = figure.add_subplot(111)
	lines, = ax.plot([],[],linestyle="none",
						marker=".",
						markersize=1,
						markerfacecolor="blue")
	ax.set_xlim(-200, 3000)
	ax.set_ylim(-200, 2000)
	ax.grid()
	return figure, lines

def update_xy_plot(x, y):
	""" re-draw the XY plot with new current_frame """
	lines.set_xdata(x)
	lines.set_ydata(y)
	figure.canvas.draw()

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
	#print 'Movement command: ', coordinates
	
	packet = packetBuilder.BuildPacket(commands.addPointToStack, coordinates)
	recievedPacket = computerPort.sendRequest(packet.bytearray)
	
	if recievedPacket.reply != 'Ok':
		raise Exception('add PointToStack failed')
	lidar_scan()

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
	lidar_scan()
	
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
myrobot.x, myrobot.y, myrobot.orientation = 1587.0, 349.0, math.pi/2
print 'First: ', myrobot
# Set N random particles
p = [Robot() for i in xrange(N)]

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

	
	
	

	


