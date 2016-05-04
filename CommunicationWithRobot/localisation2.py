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
from serial.tools import list_ports
import serialWrapper 
import packetBuilder
import packetParser
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

VID = 1155
PID = 22336
SNR = '336234893534'

	

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
		#beacons_sorted = sort_beacons(self.orientation, BEACONS, self.x, self.y)
		beacons_sorted = BEACONS
		x_glob, y_glob = ttest.point_transform(measurement, self.pose())
		#print 'SORTED BEACONS: ', beacons_sorted		
		prob = 1.0
		#print 'measurement in weights: ', measurement
		if len(beacons_sorted) != len(measurement):
			#print 'beacons not the same length' 
			return 0.0		
		for i in xrange(len(beacons_sorted)):
			try:
				dist = math.sqrt((self.x - x_glob[i]) ** 2 + (self.y - y_glob[i] ** 2))
			except ValueError:
				dist = 0.0
			# Angle between current orientation and global beacons, measure counterclockwise
			#fi = angle(angle_conv(beacons_sorted[i][1] - self.y, beacons_sorted[i][0] - self.x) - self.orientation)
			#print 'calculated distance: ', dist, measurement[i][1]
			#print 'calculated angle: ', fi, angle2(measurement[i][0])
			try:
				prob_trans = self.gaussian_trans(dist, self.sense_noise, measurement[i][1])
				#print 'prob_trans: ', prob_trans
				#prob_rot = self.gaussian_rot(fi, self.sense_angle_noise, angle2(measurement[i][0]))
				#print 'prob_rot: ', prob_rot/10
				#prob *= ((self.gaussian_trans(dist, self.sense_noise, measurement[i][1])) * \
			 	#	self.gaussian_rot(fi, self.sense_angle_noise, angle2(measurement[i][0])))
				prob *= (prob_trans)#*(prob_rot/10))
			except IndexError:
				print 'INDEX ERROR'				
				prob *= 1
		#print 'Probability:................................. ', prob
		return prob

	def gaussian_trans(self, mu, sigma, x):
		# calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
		#print 'Gaussian mu: ', mu
		#print 'Gaussian sigma: ', sigma
		#print 'Gaussian x: ', x
		# try: 
		# 	print 'Gaussian calculation: ', math.exp(- ((mu - (x+29)) ** 2) / (sigma ** 2) / 2.0) / \
		# 		math.sqrt(2.0 * math.pi * (sigma ** 2))
		# except OverflowError: 
		# 	print 'Gaussian calculation rouding: ', 0
		try:
			return math.exp(- ((mu - (x+40)) ** 2) / (sigma ** 2) / 2.0) / \
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
	ax.set_xlim(0, 3000)
	ax.set_ylim(0, 2000)
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

def portNumber():
	"""Find all ports, and returns one with defined STM values"""	
	for port in list_ports.comports():
		if (port.serial_number == SNR) and (port.pid == PID) and (port.vid == VID):	
			return port.name

if __name__ == '__main__':
	port = '/dev/'+portNumber()
	computerPort = serialWrapper.SerialWrapper(port)
	commands = packetBuilder.CommandsList()
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
	#figure, lines = init_xy_plot()
	
	# Set Robot randomly 
	myrobot = Robot()
	myrobot.x, myrobot.y, myrobot.orientation = 1587.0, 349.0, math.pi/2
	print 'First: ', myrobot
	# Set N random particles
	p = [Robot() for i in xrange(N)]
	#for i in p:
	#	print 'First particles: ', i
	#update_xy_plot([p[i].x for i in xrange(N)], [p[i].y for i in xrange(N)])
	#time.sleep(5)
	#rel_motion = []
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
			lidar, langle, lgraph = ttest.update_di(data_lidar) 
			#print 'After lidar'			
			#try:
			#	update_polar_plot(langle, lgraph)
			#except:
			#	print 'not same length'
		# Calculate the weights 
			#print 'lidar data: ', lidar			
			w =[p[i].weight(lidar) for i in xrange(N)]
			w = np.asarray(w)
			w /= w.sum()
			#print 'just weights: ', w
			#print 'sum of weights: ', np.sum(w)
			try:
		# Probability random pick - use np.random alg
				p3 = np.random.choice(p, N, p = w)
				p = list(p3)
				mean_val = [(p[i].x, p[i].y, p[i].orientation) for i in xrange(len(p))]
		#print 'list particles after random: ', p

		# Set myrobot to particle with max w
				#index2 = np.nonzero(w == w.max())[0][0]
				#myrobot = copy.deepcopy(p[index2])
				center = np.mean(mean_val, axis = 0)
				myrobot.x, myrobot.y, myrobot.orientation = center[0], center[1], center[2]
			except:
				print 'error with choice'
				pass
		# for i in p:
		# 	print 'Final particles: ', i
			#update_xy_plot([p[i].x for i in xrange(N)], [p[i].y for i in xrange(N)])			
			print myrobot
			time.sleep(0.05)
		except:			
			traceback.print_exc()
			s.send('QT\r')
			s.shutdown(2)			
			s.close()

	 
