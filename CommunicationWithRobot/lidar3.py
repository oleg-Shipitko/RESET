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
BEACONS = [(-56,1000),(3056,-56),(3056,2056)] # left starting possition (0,0 in corner near beach huts)
# BEACONS = [(-56,-55),(-56,2056),(3055,1000)] # right starting possition (0,0 in corner near beach huts)

class Robot(object):

	def __init__(self, first):
		"""Initialize robot/particle with random position"""
		if first:
			self.x = random.gauss(120.0, 5) 
			self.y = random.gauss(980.0, 5)  
			self.orientation = random.gauss(0.0, 0.1)

	def set(self, x_new, y_new, orientation_new):
		"""Set particle position on the field"""
		if 0 <= x_new <= WORLD_X:
			self.x = x_new
		else:
			self.x = random.gauss(151.0, 5) 
		if 0 <= y_new <= WORLD_Y:
			self.y = y_new
		else:
			self.y = random.gauss(756.5, 5)
		self.orientation = orientation_new % (2 * math.pi)
		
	def move(self, delta):
		"""Move particle by creating new one and setting position"""
		# From robot I get relative position. And I can do new relative minus 
		# old relative to get displacement dx, dy, dtheta
		#if delta[2] == 0:
		x_new = self.x + delta[0] + random.gauss(0, 1)
		y_new = self.y + delta[1] + random.gauss(0, 1)
		orientation_new = self.orientation + delta[2] + random.gauss(0, 0.03)	
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
		#time.sleep(0.05)
		packet = packetBuilder.BuildPacket(commands.getCurentCoordinates)	
		with lock:
			#print 'lidar has lock'
			recievedPacket = computerPort.sendRequest(packet.bytearray)
		new = recievedPacket.reply	
		#print 'new=========', new
		return [(new[0]-old[0])*1000, (new[1]-old[1])*1000, new[2]-old[2]], new

# Calculate dist and angle from raw lidar data
def dist_val(value):	
	try:	
		return ((ord(value[0])-48)<<12)|((ord(value[1])-48)<<6)|(ord(value[2])-48)
	except IndexError:	
		return 0	

def my_sum(val):
	f = lambda x,y: (x[0]+y[0],x[1]+y[1])
	return reduce(f, val)
	
# Localisation
def localisation(lock, shared, computerPort, commands):
	lock = lock
	shared = shared
	computerPort = computerPort 
	commands = commands
	#computerPort, commands = connect_stm()
	n_trash = N/2
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
	myrobot.x, myrobot.y, myrobot.orientation = 120.0, 980.0, 0.0
	print myrobot
	p = [Robot(True) for i in xrange(N)]
	old = [0.12,0.98,0.0]
	w_re = [1.0/N for i in xrange(N)]
	w_prev = [1.0 for i in xrange(N)]
	try:
		while 1:
			#start = time.time()
			rel_motion, old2 = relative_motion(old, computerPort, commands, lock)
			#print 'rel motion: ', rel_motion
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
			w_next =[p[i].weight(x_rob, y_rob, BEACONS) for i in xrange(N)]

			w = [w_prev[i]*w_next[i] for i in xrange(N)]
			w_sum = sum(w)
			#print 'sum w: ', w_sum
            
			try:
				w_norm = [i/w_sum for i in w]
				#print 'w_norm: ', w_norm
			except ZeroDivisionError:
				print 'zero weights in lidar'
				w_norm = [0.0 for i in xrange(N)]
                
			mean_val = [(p[i].x*w_norm[i], p[i].y*w_norm[i]) for i in xrange(N)]
			#print 'mean_val: ', mean_val
			mean_orientation = mean_angl(p, w_norm)
			center = np.sum(mean_val, axis = 0)
			#center = my_sum(mean_val)
			myrobot.set(center[0], center[1], mean_orientation)
			
			shared[0] = myrobot.x
			shared[1] = myrobot.y
			shared[2] = myrobot.orientation
			
			n_eff = 1.0/(sum([math.pow(i, 2) for i in w_norm]))
			#print 'effective sample size: ', n_eff
			if n_eff < n_trash and sum(rel_motion) > 0:
				try:
					#print 'doing resempling' 
					p3 = np.random.choice(p, N, p = w_norm)
					p = p3
					w_prev = w_re
					#end = time.time()
					#print start-end
					continue
				except:
					print 'error with choice'
					continue

			w_prev = w_next
			#end = time.time()
			#print start-end
			#packet2 = packetBuilder.BuildPacket(commands.setCorectCoordinates, [myrobot.x/1000, myrobot.y/1000, myrobot.orientation])
			#with lock:
			#print 'lidar has lock'
			#	recievedPacket = computerPort.sendRequest(packet2.bytearray)
			#lock_val.acquire()
				#print 'control in lidar'
			#print myrobot

			#lock_val.release()
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