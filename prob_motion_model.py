
"""
Sampling algorithm for Probabilistic Odometry Holonomic Motion Model
Ref:	Probabilistic Robotics, ch 5.4, pp 136
		http://ais.informatik.uni-freiburg.de/teaching/ss11/robotics/slides/06-motion-models.pdf
"""

import math
import random
import matplotlib.pyplot as plt

x, y, tetha = 0, 0, 0 # t-1 position in global coord sys
x_bar, y_bar, tetha_bar = 2, 3, 0 # t-1 odometry position in local coord sys
x_bar_prime, y_bar_prime, tetha_bar_prime = 3, 3.5, math.pi/3 # t odometry position in local coord sys
alpha1, alpha2, alpha3 = 0.1,0.1,0.05 # robot error parameters 
x_plot = []
y_plot = []
tetha_plot = []
tetha_dummy = []

def prob3(x, y, tetha, x_bar, y_bar, tetha_bar, x_bar_prime, y_bar_prime, tetha_bar_prime):
	"""Calculates the final robor position in global coord sys"""
	delta_x = x_bar_prime - x_bar
	delta_y = y_bar_prime - y_bar
	delta_rot = tetha_bar_prime - tetha_bar

	edelta_x = delta_x - sample(alpha1*math.fabs(delta_x) + alpha2*math.fabs(delta_y)/3 + alpha3*math.fabs(delta_rot)/3)
	edelta_y = delta_y - sample(alpha1*math.fabs(delta_x)/3 + alpha2*math.fabs(delta_y) + alpha3*math.fabs(delta_rot)/3)
	edelta_rot = delta_rot - sample(alpha1*math.fabs(delta_x)/3 + alpha2*math.fabs(delta_y)/3 + alpha3*math.fabs(delta_rot))

	x_prime = x + edelta_x 
	y_prime = y + edelta_y 
	tetha_prime = tetha + edelta_rot 
	return x_prime, y_prime, tetha_prime

def prob2(x, y, tetha, x_bar, y_bar, tetha_bar, x_bar_prime, y_bar_prime, tetha_bar_prime):
	"""Calculates the final robor position in global coord sys"""
	delta_trans = math.sqrt(math.pow(x_bar - x_bar_prime, 2) + math.pow(y_bar - y_bar_prime, 2))
	delta_rot = tetha_bar_prime - tetha_bar
	delta_angle = math.atan2(y_bar_prime - y_bar, x_bar_prime - x_bar)

	edelta_trans = delta_trans - sample(alpha1*math.fabs(delta_trans) + alpha3*math.fabs(delta_rot)/3)
	edelta_rot = delta_rot - sample(alpha1*math.fabs(delta_trans)/3 + alpha3*math.fabs(delta_rot))
	edelta_angle = delta_angle - sample(alpha1*math.fabs(delta_trans)/4) # dividing by 4 gives same 1D result as prob

	x_prime = x + edelta_trans*math.cos(edelta_angle) 
	y_prime = y + edelta_trans*math.sin(edelta_angle)
	tetha_prime = tetha + edelta_rot 
	return x_prime, y_prime, tetha_prime

def prob(pose, delta):
	# From robot I get relative position. And I can do new relative minus 
	# old relative to get displacement dx, dy, dtheta
	if delta[2] == 0:
		x_prime = pose[0] + delta[0] + trans_noise()
		y_prime = pose[1] + delta[1] + trans_noise()
		theta_prime = pose[2] + theta_noise()
	# else:
	# 	x_prime = pose[0] + delta[0] + noise
	# 	y_prime = pose[1] + delta[1] + noise
	# 	theta_prime = pose[2] + delta[2] + noise + drift # pp 3 

	return x_prime, y_prime, theta_prime

def trans_noise(): 
	return random.gauss(0, 10)

def theta_noise():
	return random.gauss(0, 0.1)

def sample(b):
	"""Draws a sample from the normal distribution"""
	#b = math.sqrt(b2)
	r = 0
	for i in xrange(12):
		r += random.uniform(-b, b)
	return 0.5*r

if __name__ == '__main__':

	for i in range(500):
		"""Construct probable poositions""" 
		x_temp, y_temp, tetha_temp = prob2(x, y, tetha, x_bar, y_bar, tetha_bar, x_bar_prime, y_bar_prime, tetha_bar_prime)
		x_plot.append(x_temp)
		y_plot.append(y_temp)
		tetha_plot.append(tetha_temp)
		tetha_dummy.append(0)

	#plt.plot(tetha_plot, tetha_dummy, 'ro')
	plt.plot(x_plot, y_plot, 'ro')
	plt.axis([0, 5, 0, 5])
	plt.show()

