"""There are 4 distinctive coordinate systems in:

Origin - in the image origin (upper left corner), with horizontal X to right, 
	and vertical Y to down
Left (ruler) point - in ruler's left point, with horizontal X to right, and 
	vertical Y to up
Landmark - in ruler's left point, with X in direction of ruler's right point 
	(usualy not horizontal) and Y normal to X axis pointing up
Robot - in robot center, X in direction of robot's movement,
	Y normal to it upwards

There is a mixture of numpy arrays and lists throughout the code, which can 
create some confusion"""

import numpy as np
import cv2
import glob
import math

# class robotClass(object):
# 	"""Robot object. Takes an image as input, returns coordinates and 
# 	orientation in landmark coord system"""

# 	def __init__(self, sample, end=None):
# 		"""Gets all relevant contours from the picture"""
# 		self.pap = cv2.imread(sample)
# 		self.cap = self.pap.copy()
		
# 		# Convert BGR to HSV
# 		self.hsv = cv2.cvtColor(self.cap, cv2.COLOR_BGR2HSV)

# 		# define range of blue color in HSV
# 		lower_blue = np.array([100,50,50])
# 		upper_blue = np.array([120,255,255])

# 		# Threshold the HSV image to get only blue colors
# 		self.mask = cv2.inRange(self.hsv, lower_blue, upper_blue)

# 		# Bitwise-AND mask and original image
# 		self.res = cv2.bitwise_and(self.cap,self.cap, mask= self.mask)

# 		#cv2.imwrite('bluehope.jpg',self.res)

# 		# Make a copy of res
# 		self.im = self.res.copy()

# 		# Convert picture to grayscale and get contours
# 		self.imgray = cv2.cvtColor(self.im,cv2.COLOR_BGR2GRAY)
# 		ret,self.thresh = cv2.threshold(self.imgray,50,255,0)
# 		self.image, self.contours, self.hierarchy = cv2.findContours(
# 			self.thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

# 		# Take only contours with area > 2000 and sort them by area ascending
# 		self.contours2 = [self.contours[i] for i in range(len(self.contours)) 
# 			if cv2.contourArea(self.contours[i]) > 2000]
# 		self.contours2.sort(key=lambda contours2: cv2.contourArea(contours2))

# 		# Makes a list of contours' center coordinates in origin ciird sys
# 		self.coordinates = self.get_coordinates(self.contours2)
# 		print "coordinates sorted by area: ", self.coordinates

		# Extract ruler coordinates in origin coord sys, and mm/px ratio
		#self.ruler, self.ratio = self.get_ruler(self.coordinates)
		# print 'Ruler in origin sys: ', self.ruler
		# print 'Lndmrk in origin sys, ratio: ', self.ruler[0][0], \
		# 	self.ruler[0][1], self.ratio
		
		# # Writes point coordinates on picture
		# for i in range(len(self.ruler)):
		# 	cv2.putText(self.im, str((('%.3f' % self.ruler[i][0], '%.3f' % 
		# 		self.ruler[i][1]))), (int(self.ruler[i][0])-200, 
		# 		int(self.ruler[i][1])+120), cv2.FONT_HERSHEY_SIMPLEX, 1, 
		# 		(255,255,255), 3)

		# Extract robot circles origin coordinates, by area ascending + center
		# self.robot_global = self.get_robot(self.coordinates)
		# print 'Robot in origin sys: ',self.robot_global
		# # Write robot circle coordinates
		# for i in range(len(self.robot_global)):
		# 	if i == 2:
		# 		cv2.putText(self.im, str((('%.3f' % self.robot_global[i][0], 
		# 			'%.3f' % self.robot_global[i][1]))), 
		# 			(int(self.robot_global[i][0])-70, 
		# 			int(self.robot_global[i][1])+120), cv2.FONT_HERSHEY_SIMPLEX,
		# 			 1, (255,255,255), 3)
		# 	else:
		# 		cv2.putText(self.im, str((('%.3f' % self.robot_global[i][0], 
		# 			'%.3f' % self.robot_global[i][1]))), 
		# 			(int(self.robot_global[i][0])-200, 
		# 			int(self.robot_global[i][1])+120), cv2.FONT_HERSHEY_SIMPLEX,
		# 			 1, (255,255,255), 3)
		
		# Robot coordinates in left pont coordinate system 
		# self.robot_local = self.translation(self.ruler, self.robot_global)
		# print 'Robot coordinates in left pont system: ', self.robot_local
		# # Wites robot coordinates in left point system
		# for i in range(len(self.robot_local)):
		# 	if i == 2:
		# 		cv2.putText(self.im, str((('%.3f' % self.robot_local[i][0], 
		# 			'%.3f' % self.robot_local[i][1]))), 
		# 			(int(self.robot_global[i][0])-70, 
		# 			int(self.robot_global[i][1])+160), cv2.FONT_HERSHEY_SIMPLEX,
		# 			 1, (255,255,255), 3)
		# 	else:
		# 		cv2.putText(self.im, str((('%.3f' % self.robot_local[i][0], 
		# 			'%.3f' % self.robot_local[i][1]))), 
		# 			(int(self.robot_global[i][0])-200, 
		# 			int(self.robot_global[i][1])+160), cv2.FONT_HERSHEY_SIMPLEX,
		# 			 1, (255,255,255), 3)

		# Coord and slope of the landmarks, in left point syst, from origin sys
		# self.landmark_coord = self.translation(self.ruler, self.ruler)
		# self.landmark_slope = math.atan2(self.landmark_coord[1][1],
		# 	self.landmark_coord[1][0])
		# print 'Landmark coord in left point sys: ', self.landmark_coord
		# print 'Landmark angle, rad in left point sys: ', self.landmark_slope
		# # Writes coord and slope (in degrees) of landmarks in left point sys
		# for i in range(len(self.landmark_coord)):
		# 	cv2.putText(self.im, str((('%.3f' % self.landmark_coord[i][0], 
		# 		'%.3f' % self.landmark_coord[i][1], '%.3f' % math.degrees(
		# 		self.landmark_slope)))), (int(self.ruler[i][0])-200, 
		# 		int(self.ruler[i][1])+160), cv2.FONT_HERSHEY_SIMPLEX, 1, 
		# 		(255,255,255), 3)


		# Rotates robot from left point sys, to landmark sys and converts to mm
		# self.robot_landmark = self.rotation(self.robot_local, 
		# 	-self.landmark_slope)
		# print 'Robot coord in landmark sys: ', self.robot_landmark
		# self.robot_landmark = np.asarray(self.robot_landmark)*self.ratio
		# self.robot_landmark = map(tuple, self.robot_landmark)
		# print 'Robot coord in landmark sys, mm: ', self.robot_landmark

		# # Robot's slope in relation to landmark coord syst
		# self.robot_slope = math.atan2(self.robot_landmark[2][1] - 
		# 	self.robot_landmark[3][1], self.robot_landmark[2][0] - 
		# 	self.robot_landmark[3][0])
		# print 'Robot angle in lndmrk sys, rad: ', self.robot_slope
		# print 'Robot angle in lndmrk sys, deg: ', math.degrees(self.robot_slope)

		# # Writes robot circle position and angle in landmark system, in mm
		# for i in range(len(self.robot_landmark)):
		# 	if i == 2:
		# 		if end == 2:
		# 			cv2.putText(self.im, str((('%.3f' % self.robot_landmark[i][0], 
		# 				'%.3f' % self.robot_landmark[i][1], '%.3f' % 
		# 				math.degrees(self.robot_slope)))), 
		# 				(int(self.robot_global[i][0])-140, 
		# 				int(self.robot_global[i][1])+200), 
		# 				cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)
		# 		else:
		# 			cv2.putText(self.im, str((('%.3f' % self.robot_landmark[i][0], 
		# 				'%.3f' % self.robot_landmark[i][1], '%.3f' % 
		# 				math.degrees(self.robot_slope)))), 
		# 				(int(self.robot_global[i][0])-70, 
		# 				int(self.robot_global[i][1])+200),
		# 				cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)
		# 	else:
		# 		cv2.putText(self.im, str((('%.3f' % self.robot_landmark[i][0], 
		# 			'%.3f' % self.robot_landmark[i][1]))), 
		# 			(int(self.robot_global[i][0])-200, 
		# 			int(self.robot_global[i][1])+200), 
		# 			cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)

	# def get_coordinates(self, contours2):
	# 	"""Takes contours2 and gives coord of their centers, area ascending"""
	# 	coordinates = []
	# 	for i in range(len(contours2)):
	# 		M = cv2.moments(contours2[i])
	# 		cx = M['m10']/M['m00']
	# 		cy = M['m01']/M['m00']
	# 		coordinates.append((cx, cy)) 
	# 		cv2.circle(self.im,(int(cx),int(cy)), 1, (0,0,255), 3)
	# 	return coordinates

	# def get_ruler(self, coordinates):
	# 	"""Takes all points, extract two with biggest area, and then arranges 
	# 	them by x value ascending"""
	# 	ruler = coordinates[-2:]
	# 	print ruler
	# 	ruler.sort(key=lambda ruler: ruler[0])
	# 	ratio = 500.0/math.sqrt((ruler[1][1]-ruler[0][1])**2+(ruler[1][0]-
	# 		ruler[0][0])**2)
	# 	return ruler, ratio

	# def get_robot(self, coordinates):
	# 	"""Takes list of coordinates, and gives back robot's circle coordinates,
	# 	and robot's center coordinates; [small, medium, big, center]"""
	# 	robot = coordinates[:3]
	# 	center = ((robot[0][0]+robot[1][0])/2.0, (robot[0][1]+robot[1][1])/2.0)
	# 	cv2.circle(self.im,(int(center[0]),int(center[1])), 1, (0,0,255), 3)
	# 	robot.append(center)
	# 	return robot

	# def translation(self, ruler, robot_global):
	# 	"""Translates points from origin coord system, to left point coord
	# 	system coordnate system. New coord sys is paralel with X and Y axis, 
	# 	it's just translated and Y is inverted"""
	# 	robot_local = [(robot_global[i][0] - ruler[0][0], -robot_global[i][1] + 
	# 		ruler[0][1])\
	# 	 for i in range(len(robot_global))]
	# 	return robot_local

	# def rotation(self, robot_local, angle):
	# 	"""Rotates robot points from left point sys to landmark coord sys"""
	# 	rot_mat = np.array([[math.cos(angle), -math.sin(angle)], 
	# 		[math.sin(angle), math.cos(angle)]])
	# 	print 'Rot_mat: ', rot_mat
	# 	robot = np.asarray(robot_local)
	# 	product = [np.dot(rot_mat, robot[i]) for i in range(len(robot))]
	# 	return map(tuple, product)

def get_coordinates(contours2):
		"""Takes contours2 and gives coord of their centers, area ascending"""
		coordinates = []
		for i in range(len(contours2)):
			M = cv2.moments(contours2[i])
			cx = M['m10']/M['m00']
			cy = M['m01']/M['m00']
			coordinates.append((cx, cy)) 
			cv2.circle(res,(int(cx),int(cy)), 1, (0,0,255), 3)
		return coordinates

cap = cv2.VideoCapture(1)

while True:

	_, frame = cap.read()
			
	# Convert BGR to HSV
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

	# define range of blue color in HSV
	lower_blue = np.array([100,50,50])
	upper_blue = np.array([120,255,255])

	# Threshold the HSV image to get only blue colors
	mask = cv2.inRange(hsv, lower_blue, upper_blue)

	# Bitwise-AND mask and original image
	res = cv2.bitwise_and(frame,frame, mask= mask)

	#cv2.imwrite('bluehope.jpg',self.res)

	# Make a copy of res
	#self.im = self.res.copy()

	# Convert picture to grayscale and get contours
	imgray = cv2.cvtColor(res,cv2.COLOR_BGR2GRAY)
	ret, thresh = cv2.threshold(imgray,50,255,0)
	image, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

	# Take only contours with area > 2000 and sort them by area ascending
	contours2 = [contours[i] for i in range(len(contours)) 
		if cv2.contourArea(contours[i]) > 2000]
	contours2.sort(key=lambda contours2: cv2.contourArea(contours2))

	# Makes a list of contours' center coordinates in origin ciird sys
	coordinates = get_coordinates(contours2)
	print "coordinates sorted by area: ", coordinates
	cv2.imshow('res',res)

# # Imports all files from a folder, with the string in its name
# images = sorted(glob.glob('*.jpg')) 

# #images = ['new1.jpg', 'new2.jpg']
# start = images[::2]
# end = images[1::2]

# file = open('results.txt', 'w')	# Creates file for writing down values
# black_canvas  = np.zeros((500,1250,3), np.uint8)
# d_angle = [] # List for angle diference
# d_x = [] # List for x end position in robot coord sys
# d_y = [] # List for y end position in robot coord sys
# dist = [] # Distance traveled
# it = 0

# for i, j in zip(start, end):
# 	it += 1
# 	robot1 = robotClass(i)
# 	print ''
# 	robot2 = robotClass(j,2)

# 	# Calculates relative rotation between start and end
# 	delta_tetha = math.degrees(robot2.robot_slope - robot1.robot_slope)
# 	print '\n', 'Angular error, deg (ccw is positive angle): ', delta_tetha
	
# 	# # Rotates Robot end position to robot coord sys
# 	# delta_end = robot1.rotation(robot2.robot_landmark, -robot1.robot_slope) 
# 	# print 'Robot coords just rotated to robot sys: ', delta_end
# 	# print 'Center point just rotated to robot sys: ', delta_end[3]

# 	# # Translates Robot end position to robot coord sys 
# 	# delta_center = np.asarray(delta_end)-np.asarray(robot1.robot_landmark[3])
# 	# print 'Center point in robot sys (with translation): ', delta_center

# 	# Translates Robot end position to robot coord sys 
# 	delta_end = np.asarray(robot2.robot_landmark)-np.asarray(robot1.\
# 		robot_landmark[3])
# 	print 'Center point in robot sys (with translation): ', delta_end
# 	delta_end = map(tuple, delta_end)

# 	# Rotates Robot end position to robot coord sys
# 	delta_center = robot1.rotation(delta_end, -robot1.robot_slope) 
# 	print 'Robot coords just rotated to robot sys: ', delta_center
# 	print 'Center point just rotated to robot sys: ', delta_center[3]



# 	distance = math.sqrt((delta_center[3][0])**2 + (delta_center[3][1])**2)
# 	d_angle.append(delta_tetha)
# 	d_x.append(delta_center[3][0])
# 	d_y.append(delta_center[3][1])
# 	dist.append(distance)
	
# 	print d_x, d_y, d_angle, dist
# 	for k in range(len(delta_center)):
# 			if k == 2:
# 				cv2.putText(robot2.im, str((('%.3f' % delta_center[k][0], '%.3f'
# 					% delta_center[k][1], '%.3f' % delta_tetha))), 
# 					(int(robot2.robot_global[k][0])-140, 
# 					int(robot2.robot_global[k][1])+240), 
# 					cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 3)
# 			else:
# 				cv2.putText(robot2.im, str((('%.3f' % delta_center[k][0], '%.3f'
# 				% delta_center[k][1]))), (int(robot2.robot_global[k][0])-200, 
# 				int(robot2.robot_global[k][1])+240), cv2.FONT_HERSHEY_SIMPLEX, 
# 				1, (255,255,255), 3)



#  	img1 = cv2.drawContours(robot1.im, robot1.contours2, -1, (0,255,0), 3)
# 	img2 = cv2.drawContours(robot2.im, robot2.contours2, -1, (0,255,0), 3)
#  	cv2.imwrite('test_contures'+str(i)+'.jpg', img1)
#  	cv2.imwrite('test_contures'+str(j)+'.jpg', img2)
#  	rows,cols,dpt = img1.shape
#  	combined = np.hstack((img1[:,:cols/2],img2[:,cols/2:]))
	
# 	# Just finding the end point for axis to draw 
#  	ydraw = (robot1.robot_global[3][1] - robot1.robot_global[2][1]) * \
#  		(robot2.robot_global[2][0]-robot1.robot_global[2][0]) / \
#  		(robot1.robot_global[3][0] - robot1.robot_global[2][0]) + \
#  		robot1.robot_global[2][1]

 	
#  	cv2.line(combined, tuple(map(int,robot1.robot_global[3])), 
#  		(int(robot2.robot_global[2][0])+160, int(ydraw)),(0,0,255), 1, 
#  		cv2.LINE_AA)
#  	cv2.imwrite('test_blend'+str(it)+'.jpg', combined)

# # X axis, Y mark, 1000 mark, value
# cv2.line(black_canvas, (50, 450), (1200, 450) ,(255,255,255), 1, cv2.LINE_AA)
# cv2.line(black_canvas, (50, 450), (50, 100) ,(255,255,255), 1, cv2.LINE_AA)
# cv2.line(black_canvas, (1050, 445), (1050, 455) ,(255,255,255), 1, cv2.LINE_AA)
# cv2.putText(black_canvas, '1000.0', (1025, 470), cv2.FONT_HERSHEY_COMPLEX, 
# 				0.5, (255,255,255), 1, cv2.LINE_AA)

# # Puts point on the image. Y coord multiplied by 4
# for i in range(len(d_x)):
# 	cv2.circle(black_canvas,(int(d_x[i] +50), int(450-d_y[i]*4)), 1, (0,0,255), 3)

# cv2.imwrite('dist.jpg',black_canvas)

# a_stat = np.array([np.mean(d_angle), np.std(d_angle), np.var(d_angle)])
# x_stat = np.array([np.mean(d_x), np.std(d_x), np.var(d_x)])
# y_stat = np.array([np.mean(d_y), np.std(d_y), np.var(d_y)])
# dist_stat = np.array([np.mean(dist), np.std(dist), np.var(dist)])

# d_x, d_y, d_angle = np.asarray(d_x), np.asarray(d_y), np.asarray(d_angle)
# dist = np.asarray(dist)

# d_x = np.concatenate((d_x, x_stat))
# d_y = np.concatenate((d_y, y_stat))
# d_angle = np.concatenate((d_angle, a_stat))
# dist = np.concatenate((dist, dist_stat))

# data = np.array([d_x, d_y, d_angle, dist])
# print data
# data = data.T
# print data
# np.savetxt(file, data, fmt=['%.3f','%.3f','%.3f','%.3f'])

# file.close()

