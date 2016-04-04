# Monte Carlo Localisation

# Initialize robot position - random
# Sense (lidar)
# Initialize particles - random uniform 
# Move robot (in our case will be 0,0,0)
# Sense (lidar)
# Move every particle
# Calculate weight for every particle (probability)
# Resample (probability random pick)
# Set the one with biggest probability as real position

# Change kinematic model to give only one particle with noise
# But have many particles

import prob_motion_model as pmm
import hokuyo
import random
import numpy as np
import matplotlib.pyplot as plt
import time
import socket
import copy

# Hokuyo socket parameters
TCP_IP = '192.168.0.10'
TCP_PORT = 10940
BUFFER_SIZE = 8192 #4096
# Number of particles
N = 300
# Dimensions of the playing field  
WORLD_X = 3000
WORLD_Y = 2000
# Beacon location: 1(left middle), 2(right lower), 3(right upper)
BEACONS = [(-62,1000),(3062,-62),(3062,2062)]
# Dummy list before I implement odometry reading and conversion to global coord
rel_motion = [0, 0, 0]
# Dummpy sensor noise
sense_noise = 3.0


class Robot(object):
	def __init__(self):
		self.x = random.random() * WORLD_X
		self.y = random.random() * WORLD_Y
		self.orientation = random.random() * 2.0 * np.pi
		# Dummpy sensor noise
		self.sense_noise = 3.0

	def set(self, new_x, new_y, new_orientation):
		if new_x < 0 or new_x > WORLD_X:
			raise ValueError, 'X coordinate out of bound'
		if new_y < 0 or new_y > WORLD_Y:
			raise ValueError, 'Y coordinate out of bound'
		if new_orientation < 0 or new_orientation >= 2 * np.pi:
			raise ValueError, 'Orientation must be in [0..2pi]'
		self.x = int(new_x)
		self.y = int(new_y)
		self.orientation = float(new_orientation)

	def r_motion():
		# Take odometry reading and convert to global coord
		pass
		
	def pose(self):
		return self.x, self.y, self.orientation

	def weight(self, measurement):
		prob = 1.0
		for i in xrange(len(BEACONS)):
			dist = sqrt((self.x - BEACONS[i][0]) ** 2 + (self.y - BEACONS[i][1]) ** 2)
			prob *= self.Gaussian(dist, self.sense_noise, measurement[i])
		return prob

	def Gaussian(self, mu, sigma, x):
		
		# calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
		return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))		

	def __str__(self):
		return 'Particle pose: x = %i mm, y = %i mm, theta = %.2f deg' \
			%(self.x, self.y, np.degrees(self.orientation))

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
						markersize=3,
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

if __name__ == '__main__':
	# Initialize socket connection
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))

	# Initialize the plot
	figure, lines = init_xy_plot()
	
	# Set Robot randomly 
	myrobot = Robot()
	# Set N random particles
	p = [Robot() for i in xrange(N)]

	update_xy_plot([p[i].x for i in xrange(N)], [p[i].y for i in xrange(N)])

	time.sleep(5)
	# Move robot; noise is in prob function
	myrobot.set(pmm.prob(myrobot.pose()), rel_motion)
	# Lidar sense - returns distance to 3 beacons
	lidar = hokuyo()

	# Move particles
	p2 = [p[i].set(pmm.prob(p[i].pose(), rel_motion)) for i in xrange(N)]
	# for i in xrange(N):
	# 	p2.append(p[i].set(pmm.prob(p[i].pose(), rel_motion)))
	# p = p2

	# Calculate the weights 
	w =[p[i].weight(lidar) for i in xrange(N)]
	# for i in xrange(N):
	# 	w.append(p[i].weight(*arguments))
	w = np.asarray(w)
	
	# Probability random pick - use np.random alg
	p3 = np.random.choice(p, gfbdf, p = w )

	# Set myrobot to particle with max w
	myrobot = copy.deepcopy(p[w.index(max(w))])