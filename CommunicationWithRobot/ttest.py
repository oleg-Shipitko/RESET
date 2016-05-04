from collections import deque
import numpy as np
import math
import multiprocessing
import lidar
import time

def time_val(value):	
	return ((ord(value[0])-48)<<18)|((ord(value[1])-48)<<12)|((ord(value[2])-48)<<6)|(ord(value[3])-48)

def dist_val(value):	
	try:	
		return ((ord(value[0])-48)<<12)|((ord(value[1])-48)<<6)|(ord(value[2])-48)
	except IndexError:
		print 'error ', value	
		return 0	

#@profile
def update_di(answer):		
	answer = answer.split('\n')		
	#time = answer.pop(1)[:-1]
	dist = answer[2:-2]
	dist2 = [item[:-1] for item in dist]
	dist4 = ''.join(dist2)			
	step, idxh = 0, 3 			
	polar_graph = deque()
	angle = deque()	
	lend = len(dist4)
	while idxh <= lend:
		point = dist_val(dist4[idxh-3:idxh])
		#strength = dist_val(dist4[idxh:idxh+3])
		if dist_val(dist4[idxh:idxh+3]) > 1100 and point < 3000:	
			polar_graph.append(point)
			angle.append(step)
		#idxl = idxh + 3
		idxh += 6		
		step += 0.004363323129985824
	#print 'point ', polar_graph
	#print 'angle ', angle 
	combined = zip(angle, polar_graph)
	#print 'zip ', combined	
	#print len(combined)
	#x_glob, y_glob = point_transform(combined, pose)
	y = 1
	prev = 0
	beacons = []
	for i in xrange(1,len(combined)):
		if ((combined[i][0] - combined[i-1][0]) > 0.2 or i == (len(combined)-1)) and y<4:
			beacons.append(beacon_num(i,y, prev, combined))
			#print i
			y += 1
			prev = i
	#print 'beacons: ', beacons	
	return beacons, angle, polar_graph

def beacon_num(index, number, prev, combined):
	if number == 1:
		#print 'got into 1'		
		return np.mean(combined[:index], axis = 0)
	if number == 2:
		#print 'got into 2'		
		return np.mean(combined[prev:index], axis = 0)
	if number == 3:
		#print 'got into 3'
		return np.mean(combined[prev:], axis = 0)
	#print type(combined)

# Transforms lidar points to glob coord sys
def point_transform(combined, pose):
	#rot_mat = np.array([[math.cos(pose[2]), -math.sin(pose[2])],
	#			[math.sin(pose[2]), math.cos(pose[2])]])
	x_glob = [(pose[0] + point[1] * math.cos(angle3(pose[2],point[0]))) for point in combined]
	y_glob = [(pose[1] + point[1] * math.sin(angle3(pose[2],point[0]))) for point in combined]
	return x_glob, y_glob 

# Transforms global beacons to robot coord sys
def beacon_transform1(beacons, pose):
	beac = [(beacon[0] - pose[0], beacon[1] - pose[1]) for beacon in beacons]
	rot_mat = np.array([[math.cos(pose[2]), math.sin(pose[2])],
				[-math.sin(pose[2]), math.cos(pose[2])]])
	transform = [np.dot(rot_mat, b) for b in beac]
	return transform

def beacon_transform2(beacons, pose):
	print np.degrees(pose[2])
	rot_mat = np.array([[math.cos(pose[2]), math.sin(pose[2])],
				[-math.sin(pose[2]), math.cos(pose[2])]])
	transform = [np.dot(rot_mat, b) for b in beacons]
	beac = [(beacon[0] - pose[0], beacon[1] - pose[1]) for beacon in transform]
	return transform

# Transforms lidar measurement to xy in robot coord sys
def p_trans(agl, pit):
	x_rob = [pit[i] * math.cos(angle5(agl[i])) for i in xrange(len(agl))]
	y_rob = [pit[i] * math.sin(angle5(agl[i])) for i in xrange(len(agl))]
	return x_rob, y_rob
	
# Transforms lidar point angle in robot coord sys
def angle5(angle):
	if angle >= math.pi/4:
		angle -= math.pi/4
	else:
		angle += 7*math.pi/4
	return angle

# Finds angle of lidar measurement arm from global x axis (takes self.orientation)
def angle3(orientation, angle):
 	start = orientation - math.pi/4
 	if start < 0:
 		start += (2*math.pi)
 	start += angle
	start %= (2*math.pi)
	return start


def update_di2(answer, pose):		
	answer = answer.split('\n')		
	#time = answer.pop(1)[:-1]
	dist = answer[2:-2]
	dist2 = [item[:-1] for item in dist]
	dist4 = ''.join(dist2)			
	step, idxh = 0, 3 			
	polar_graph = deque()
	angle = deque()	
	lend = len(dist4)
	while idxh <= lend:
		point = dist_val(dist4[idxh-3:idxh])
		#strength = dist_val(dist4[idxh:idxh+3])
		if dist_val(dist4[idxh:idxh+3]) > 1100 and point < 3200:	
			polar_graph.append(point)
			angle.append(step)
		idxh += 6		
		step += 0.004363323129985824
	#print 'point ', polar_graph
	#print 'angle ', angle 
	combined = zip(angle, polar_graph)
	#print 'zip ', combined	
	#print len(combined)
	x_glob, y_glob = point_transform(combined, pose)
	print x_glob, y_glob
	return angle, polar_graph, x_glob, y_glob
	 

if __name__ == '__main__':
	lock = multiprocessing.Lock()
	shared = multiprocessing.Array('f', [0.0, 0.0, 0.0])
	l = multiprocessing.Process(target=lidar.localisation, args=(lock,shared))
	l.start()
	
	while 1:
		try:
			#with lock:
			#	print 'this is main process'
			print shared[:]
			#time.sleep(1)
			#lock.release()
			time.sleep(0.1)
		except KeyboardInterrupt:
			l.terminate()
			l.join()
# i need to build beacons on the assumption what robot sees

# fov_start = self.orientation - 135
# fov_end = self.orientation + 135

# def angle(pt1, pt2):
#     x1, y1 = pt1
#     x2, y2 = pt2
#     inner_product = x1*x2 + y1*y2
#     len1 = math.hypot(x1, y1)
#     len2 = math.hypot(x2, y2)
#     return math.acos(inner_product/(len1*len2))

# def pol2cart(rho, phi):
#     x = rho * np.cos(phi)
#     y = rho * np.sin(phi)
#     return(x, y)

# def rtpi(x, y):
#     hypotenuse = math.hypot(x, y)
#     angle = round(math.degrees(math.atan2(y, x)))
#     if angle<0:
#         angle += 360
#     return angle

# def angle_conv(y, x):
# 	angle = math.atan2(y, x)
# 	if angle<0:
# 		angle += (2*math.pi)
# 	return angle

# def angle(measurement):
# 	if measurement < 0:
# 		measurement += (2*math.pi)	
# 	return measurement

# def angle2(measurement):
# 	if measurement < 3*math.pi/4:
# 		measurement += (5*math.pi/4)
# 		return measurement
# 	else:
# 		measurement -= (3*math.pi/4)
# 		return measurement

# # SORTING BEACONS

# # Finds angle of lidar start arm from global x axis (takes self.orientation)
# def angle3(orientation):
# 	start = orientation - 3*math.pi/4
# 	if start < 0:
# 		start += (2*math.pi)
# 	return start

# # Finds angle between angle3 and beacon (takes angle3 of orientation, and angle_conv of beacon point)
# def angle4(arm, beacon):
# 	angle = beacon - arm
# 	if angle < 0:
# 		angle += (2*math.pi)
# 	return angle

# def sort_beacons(orientation,BEACONS,x,y):
# 	arm = angle3(orientation)
# 	print 'arm: ', arm
# 	beacons = [angle_conv(BEACONS[i][1] - y, BEACONS[i][0] - x) for i in xrange(3)]
# 	print 'beacon relative angle to robot position: ', beacons
# 	order =[angle4(arm,beacon) for beacon in beacons]
# 	print 'angle between arm and beacons: ', order
# 	print 'sorted: ', sorted(zip(order,BEACONS))
# 	beacons_sort = [BEACONS for (order,BEACONS) in sorted(zip(order,BEACONS)) if order <= 3*math.pi/2]
# 	return beacons_sort

# orientation = 3*math.pi/2
# BEACONS = [(3000,0),(3000,2000),(0,1000)]
# x = 1500
# y = 0.000
# print sort_beacons(orientation,BEACONS,x,y)
