import numpy as np
import time
import socket
import math
import random
import traceback
import serialWrapper
import packetBuilder
import packetParser
from collections import deque
from serial.tools import list_ports


##############################
## Hokuyo socket parameters ##
## Should go into main prog ##
##############################
TCP_IP = '192.168.0.10'
TCP_PORT = 10940
BUFFER_SIZE = 8192 #4096

VID = 1155
PID = 22336
SNR = '336234893534'

# Initialize socket connection
# HAS TO GO INTO MAIN PROG
#s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)	
#s.connect((TCP_IP, TCP_PORT))
#time.sleep(0.1)
#s.send('BM\r')
#data = s.recv(BUFFER_SIZE)
#time.sleep(0.1)	
#for i in xrange(3):
#	s.send('GE0000108000\r')
#	data = s.recv(BUFFER_SIZE)
#	time.sleep(0.1)
# Initialize robot and particles
# HAS TO GO INTO MAIN PROG
#myrobot = Robot(True)
#myrobot.x, myrobot.y, myrobot.orientation = 1587.0, 349.0, 0
#p = [Robot(True) for i in xrange(N)]

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
			self.x = random.gauss(155.3, 5) 
			self.y = random.gauss(755.5, 5) 
			self.orientation = random.gauss(0.0, 0.1)

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
		x_new = self.x + delta[0] + random.gauss(0, 2)
		y_new = self.y + delta[1] + random.gauss(0, 2)
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

###############################################################################

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
def relative_motion(old, computerPort, commands, lock):
		"""Return robot current coordinates"""	
		#print 'old', old
		time.sleep(0.05)
		packet = packetBuilder.BuildPacket(commands.getCurentCoordinates)	
		with lock:
			recievedPacket = computerPort.sendRequest(packet.bytearray)
		new = recievedPacket.reply	
		#print 'new', new
		return [(new[0]-old[0])*1000, (new[1]-old[1])*1000, new[2]-old[2]], new

# Calculate dist and angle from raw lidar data
def dist_val(value):	
	try:	
		return ((ord(value[0])-48)<<12)|((ord(value[1])-48)<<6)|(ord(value[2])-48)
	except IndexError:	
		return 0	

def connect_stm():
	for port in list_ports.comports():
		if (port.serial_number == SNR) and (port.pid == PID) and (port.vid == VID):	
			port = '/dev/' + port.name
		else:
			print 'No STM32 found. Aborting'
	computerPort = serialWrapper.SerialWrapper(port)
	commands = packetBuilder.CommandsList()	
	return computerPort, commands
	
# Localisation
def localisation(lock, shared, computerPort, commands):
	lock = lock
	shared = shared
	computerPort = computerPort 
	commands = commands
	#computerPort, commands = connect_stm()
	
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)	
	s.connect((TCP_IP, TCP_PORT))
	#print '2'
	time.sleep(0.1)
	s.send('BM\r')
	data = s.recv(BUFFER_SIZE)
	time.sleep(0.1)	
	for i in xrange(3):
		s.send('GE0000108000\r')
		data = s.recv(BUFFER_SIZE)
		time.sleep(0.1)
	
	myrobot = Robot(True)
	myrobot.x, myrobot.y, myrobot.orientation = 155.3, 755.5, 0.0
	p = [Robot(True) for i in xrange(N)]
	old = [0.0, 0.0, 0.0]
	try:
		while 1:
			rel_motion, old2 = relative_motion(old, computerPort, commands, lock)
			#if abs(rel_motion[0]) < 0.001 and abs(rel_motion[1]) < 0.001 and abs(rel_motion[2]) < 0.000001:
			#	print 'Stopped moving'
			#	return myrobot.x, myrobot.y, myrobot.orientation
			#start = time.time()		
			p2 = [p[i].move(rel_motion) for i in xrange(N)]
			p = p2
			old = old2
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
            
			with shared.get_lock():
				shared[0] = myrobot.x
				shared[1] = myrobot.y
				shared[2] = myrobot.orientation
			#end = time.time()
			#print start - end
	except:			
		traceback.print_exc()
		s.shutdown(2)			
		s.close()
#try:
#	myrobot = Robot(True)
#	myrobot.x, myrobot.y, myrobot.orientation = 524.0, 225.0, 0
#	p = [Robot(True) for i in xrange(N)]
#	localisation(myrobot, p, s)
#except:			
#	traceback.print_exc()
#	s.shutdown(2)			
#	s.close()