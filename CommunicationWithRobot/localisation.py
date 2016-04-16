# Monte Carlo Localisation

import prob_motion_model as pmm
#import hokuyo
import random
import numpy as np
import matplotlib.pyplot as plt
import time
import socket
import copy
import ttest
import math
import traceback

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
# Dummy list before I implement odometry reading and conversion to global coord
rel_motion = [0, 0, 0]
# Dummy sensor noise
#sense_noise = 1.5

MARKER_POSITIONS = [(514,217),(1587,349),(2597,248),
					(514,951),(1593,903),(2602,918),
					(514,1677),(1613,1677),(2618,1699)]
	
class Robot(object):
	def __init__(self):
		self.x = random.gauss(1587, 5) 
		self.y = random.gauss(349, 5) 
		self.orientation = 0.0
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
		#print 'rel motion: ', rel_motion, self.pose()
		new_x, new_y, new_orientation = pmm.prob(self.pose(), rel_motion)	
		new_robot = Robot()
		new_robot.set(new_x, new_y, new_orientation)
		return new_robot

	#def r_motion(self):
	#	"""Return robot current coordinates"""	
	#	packet = packetBuilder.BuildPacket(commands.getCurentCoordinates)	
	#	recievedPacket = computerPort.sendRequest(packet.bytearray)
	#	old = recievedPacket.reply
	#	time.sleep(0.005)
	#	packet = packetBuilder.BuildPacket(commands.getCurentCoordinates)	
	#	recievedPacket = computerPort.sendRequest(packet.bytearray)
	#	new = recievedPacket.reply
	#	return [new[0]-old[0], new[1]-old[1],new[2]-old[2]]
		
	def pose(self):
		return self.x, self.y, self.orientation
	
	
	def weight2(self, x_rob, y_rob):		
		beacons = ttest.beacon_transform1(BEACONS, self.pose())
		#bx = [beacons[0][0],beacons[1][0],beacons[2][0]]
		#by = [beacons[0][1],beacons[1][1],beacons[2][1]]
		#update_xy_plot(bx + x_rob, by+y_rob)	
		#print self.pose()
		#print 'trans beac: ', beacons
		beacon = [0, 0, 0]
		num_point = [0, 0, 0]
		#median = 0.0
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
		#for j in xrange(3):
		#	if num_point[j] != 0:
		#		median += (beacon[j]/num_point[j]) 	
		#return 1.0/median

		median =[(beacon[i]/num_point[i]) for i in xrange(3) if num_point[i] != 0]
		return 1.0/sum(median)
	
	def __str__(self):
		return 'Particle pose: x = %.2f mm, y = %.2f mm, theta = %.2f deg' \
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
	ax.set_xlim(-2000, 3500)
	ax.set_ylim(-2000, 2500)
	ax.grid()
	return figure, lines

def update_xy_plot(x, y):
	""" re-draw the XY plot with new current_frame """
	lines.set_xdata(x)
	lines.set_ydata(y)
	figure.canvas.draw()

def init_polar_plot():
	""" setup a polar plot canvas """

	plt.ion()
	figure = plt.figure(figsize=(6, 6), 
						dpi=160, 
						facecolor="w", 
						edgecolor="k")
	ax = figure.add_subplot(111, polar=True)
	lines, = ax.plot([],[], 
					linestyle="none", 
					marker=".", 
					markersize=3, 
					markerfacecolor="blue")
	ax.set_rmax(4000)
	ax.set_theta_direction(1) #set to clockwise
	ax.set_theta_offset(-np.pi/4) #offset by 90 degree so that 0 degree is at 12 o'clock
	#ax.grid()
	return figure, lines

def update_polar_plot(angle, dist):
	""" re-draw the polar plot with new current_frame """

	lines2.set_xdata(angle)
	lines2.set_ydata(dist)
	figure2.canvas.draw()

if __name__ == '__main__':
	# Initialize socket connection
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
	# Initialize the plot
	#figure2, lines2 = init_polar_plot()
	figure, lines = init_xy_plot()
	
	# Set Robot randomly 
	myrobot = Robot()
	myrobot.x, myrobot.y, myrobot.orientation = 1587.0, 349.0, 0
	#print 'First: ', myrobot
	# Set N random particles
	p = [Robot() for i in xrange(N)]
	#for i in p:
	#	print 'First particles: ', i
	update_xy_plot([p[i].x for i in xrange(N)], [p[i].y for i in xrange(N)])
	#time.sleep(5)
	iteration = 1
	while True:
		try:	
	#for iteration in xrange(30):
			#print 'ITERATION:.......', iteration
		# Move robot; noise is in prob function
			#rel_motion = myrobot.r_motion()
			#print rel_motion
			#myrobot = myrobot.move(rel_motion)
		#print 'Robot after movement: ', myrobot
			
		# Move particles
			
			p2 = [p[i].move(rel_motion) for i in xrange(N)]
			p = p2
		#for i in p:
		#	print 'Particels after movement: ', i
			s.send('GE0000108000\r')
			data_lidar = s.recv(BUFFER_SIZE)
		# Lidar sense - returns distance to 3 beacons
			combined, langle, lgraph, bec_x, bec_y = ttest.update_di2(data_lidar, myrobot.pose()) 
			#print 'After lidar'			
			#try:
			#	update_polar_plot(langle, lgraph)
			#except:
			#	print 'not same length'
		# Calculate the weights 
			#print 'lidar data: ', lidar
			x_rob, y_rob = ttest.p_trans(combined)			
			w =[p[i].weight2(x_rob, y_rob) for i in xrange(N)]
			w = np.asarray(w)
			#print 'just weights: ', w
			w /= w.sum()
			#print 'normalized weights: ', w
			#print 'sum of weights: ', np.sum(w)
			mean_orientation = mean_angl(p, w)
			try:
		# Probability random pick - use np.random alg
				mean_val = [(p[i].x*w[i], p[i].y*w[i], p[i].orientation*w[i]) for i in xrange(len(p))]
				max_w = np.max(w)
				p3 = np.random.choice(p, N, p = w)
				p = list(p3)
				#mean_val = [(p[i].x*w[i], p[i].y*w[i], p[i].orientation*w[i]) for i in xrange(len(p))]
		#print 'list particles after random: ', p

		# Set myrobot to particle with max w
				#index2 = np.nonzero(w == w.max())[0][0]
				#myrobot = copy.deepcopy(p[index2])
				center = np.sum(mean_val, axis = 0)
				myrobot.x, myrobot.y, myrobot.orientation = center[0], center[1], mean_orientation
				pass
			except:
				print 'error with choice'
				pass
		# for i in p:
		# 	print 'Final particles: ', i
			update_xy_plot([p[i].x for i in xrange(N)]+bec_x, [p[i].y for i in xrange(N)]+bec_y)	
			#update_xy_plot([p[i].x for i in xrange(N)], [p[i].y for i in xrange(N)])		
			print myrobot
			#iteration += 1
			#time.sleep(0.05)
		except:			
			traceback.print_exc()
			s.shutdown(2)			
			s.close()

	