import time
import socket
import math
import random
import serialWrapper
import packetBuilder
import packetParser
import struct
from multiprocessing.pool import ThreadPool
from collections import deque
from serial.tools import list_ports


#Hokuyo socket parameters 
TCP_IP = '192.168.0.10'
TCP_PORT = 10940
BUFFER_SIZE = 8192 #4096

# PC server address and  port
HOST = '192.168.1.4'
PORT = 9997

#STM 32 board parameters
VID = 1155
PID = 22336
SNR = '3677346C3034'

#Number of particles
N = 200
#Resampling trashold
n_trash = N/3
# Dimensions of the playing field  
WORLD_X = 3500
WORLD_Y = 2100
# Beacon location: 1(left middle), 2(right lower), 3(right upper)
BEACONS = [0, 0, 0]
#BEACONS = [(-57,1000),(3065,-65),(3065,2065)] # right (green) starting possition (0,0 in corner near beach huts)
#BEACONS = [(-56,-55),(-56,2056),(3055,1000)] # left (purple) starting possition (0,0 in corner near beach huts)

# Lidar displacement from robo center
DELTA_X = 43.0
DELTA_Y = 0.0
BETA = math.atan(DELTA_Y/DELTA_X)
R = math.sqrt(60.0**2) 

class Robot(object):
	"""Robot class. Represent particles in monte carlo localisation"""
	def __init__(self, first, start_position = None):
		"""Initialize robot/particle with random position"""		
		if first:
			self.x = random.gauss(start_position[0]*1000+60, 20)
			self.y = random.gauss(start_position[1]*1000, 20)
			self.orientation = random.gauss(start_position[2], 0.1)


	def set(self, x_new, y_new, orientation_new):
		"""Set particle position on the field"""
		if -100 <= x_new <= WORLD_X:
			self.x = x_new
		else:
			self.x = random.gauss(2800.0, 5) 
		if -100 <= y_new <= WORLD_Y:
			self.y = y_new
		else:
			self.y = random.gauss(756.5, 5)
		self.orientation = orientation_new % (2 * math.pi)
		
	def move(self, delta):
		"""Move particle by creating new one and setting position"""
		x_new = self.x + delta[0] + random.gauss(0, 10)
		y_new = self.y + delta[1] + random.gauss(0, 10)
		orientation_new = self.orientation + delta[2] + random.gauss(0, 0.03)	
		new_robot = Robot(False)
		new_robot.set(x_new, y_new, orientation_new)
		return new_robot

	def pose(self):
		"""Return particle pose"""
		return self.x, self.y, self.orientation

	def weight(self, x_rob, y_rob, BEACONS):		
		"""Calculate particle weight based on its pose and lidar data"""
		# calculate beacon position relative to robot, translation and rotation
		temp_beac = [(beacon[0] - self.x, beacon[1] - self.y) for beacon in BEACONS] 
		beacons = [(math.cos(self.orientation)*beac[0] + math.sin(self.orientation)*beac[1],
				-math.sin(self.orientation)*beac[0] + math.cos(self.orientation)*beac[1])
				for beac in temp_beac]
		beacon = [0, 0, 0]
		num_point = [0, 0, 0]
		# find min distance from real to measured beacons
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
			# filter particles with distance more than 200 from beacon center
			if lmin > 200:
				continue
			beacon[num] += lmin
			num_point[num] += 1
		median =[(beacon[i]/num_point[i]) if num_point[i] != 0 else (1000) for i in xrange(3)]
		try:
			return 1.0/sum(median)
		except ZeroDivisionError:
			print 'Zero division error in weights'
			return 0

	def __str__(self):
		"""Print statement"""
		return 'Particle pose: x = %.2f mm, y = %.2f mm, theta = %.2f deg' \
			%(self.x, self.y, math.degrees(self.orientation))

###############################################################################

def mean_angl(p, w):
	"""Calculate robot orientation based on particles/weights"""
	x = 0 
	y = 0
	for i in xrange(N):
		x += w[i]*math.cos(p[i].orientation)
		y += w[i]*math.sin(p[i].orientation)
	angle = math.atan2(y,x)
	if angle < 0:
		return angle + (2*math.pi)
	return angle

def lidar_scan(answer):		
	"""Separate data points from lidar"""
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
		# filter based on strength and distance; 1200 strength; 4000mm distance
		if dist_val(dist4[idxh:idxh+3]) > 1200 and point < 4000:	
			append_pg(point)
			append_a(step)
		idxh += 6		
		step += 0.004363323129985824
	return angle, polar_graph

def p_trans(agl, pit):
	"""Transform lidar measurment to xy in robot coord sys"""
	x_rob = [pit[i] * math.cos(angle5(agl[i])) for i in xrange(len(agl))]
	y_rob = [pit[i] * math.sin(angle5(agl[i])) for i in xrange(len(agl))]
	return x_rob, y_rob

def angle5(angle):
	"""Transform lidar points from lidar coord sys to robot cord sys""" 
	#if angle >= math.pi/4:
	#	return angle - math.pi/4
	#else:
	#	return angle + 7*math.pi/4
	return (angle + math.pi/4)%(2*math.pi)

def relative_motion(input_command_queue, reply_to_localization_queue, old, correction_performed, myrobot, cc_robot):
		"""Calculate robot's relative motion"""	
		new = stm_driver(input_command_queue, reply_to_localization_queue,'get_current_coordinates')
		cc_robot[0] = new[0]
		cc_robot[1] = new[1]
		cc_robot[2] = new[2]
		if correction_performed.value == 1:
			old = [myrobot.x/1000, myrobot.y/1000,myrobot.orientation]
			correction_performed.value = 0	
		return [(new[0]-old[0])*1000, (new[1]-old[1])*1000, new[2]-old[2]], new

def dist_val(value):	
	"""Parse raw lidar data"""
	try:	
		return ((ord(value[0])-48)<<12)|((ord(value[1])-48)<<6)|(ord(value[2])-48)
	except IndexError:	
		return 0	

def my_sum(val):
	"""Calculate a sum of two arrays. Not used at all in this script"""
	f = lambda x,y: (x[0]+y[0],x[1]+y[1])
	return reduce(f, val)

def resample(p,w,N):
	"""Random pick algorithm for resempling"""
	sample = []
	index = int(random.random() * N)
	beta = 0.0
	mw = max(w)
	for i in xrange(N):
		beta += random.random() * 2.0 * mw
		while beta > w[index]:
			beta -= w[index]
			index = (index +1)%N
		sample.append(p[index])
	return sample 

def start_lidar(AF_INET, SOCK_STREAM, TCP_IP, TCP_PORT):
	"""Initialize lidar and get test measurements"""
	s = socket.socket(AF_INET, SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))
	time.sleep(0.1)
	s.send('BM\r')
	data = s.recv(BUFFER_SIZE)
	time.sleep(0.1)	
	for i in xrange(5):
		s.send('GE0000108000\r')
		data = s.recv(BUFFER_SIZE)
		time.sleep(0.1)
	print 'lidar initialized'
	return s

def connect_pc(HOST, PORT):
    """Connect to remote pc server"""
    try:    
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.5)
        sock.connect((HOST, PORT))
        print 'PC server connected'
        return sock
    except Exception as err:
        print 'Error in connecting to pc server: ', err
        sock.close()
        return None

def stm_driver(input_command_queue, reply_to_localization_queue, command, parameters = ''):
    """Send drequest to STM board"""
    command = {'request_source': 'localisation', 'command': command, 'parameters': parameters}
    input_command_queue.put(command)
    return reply_to_localization_queue.get()

def lidar_worker(s):
    """Request and retrieve data from lidar"""
    s.send('GE0000108000\r')
    return s.recv(BUFFER_SIZE)
###############################################################################

# main function 
def main(input_command_queue,reply_to_localization_queue, current_coordinatess,
            correction_performed, start_position, cc_robot, start_flag, selected_side,
            flag):
    #global BEACONS
    if selected_side.value == 'green':
        #BEACONS = [(-57,1000),(3065,-65),(3065,2065)] # right (green) starting possition (0,0 in corner near beach huts)
        BEACONS = [(-62,1000),(3062,-62),(3062,2062)] # right (green) starting possition (0,0 in corner near beach huts)
        print 'Beacons for green side'
    else:
        #BEACONS = [(-56,-55),(-56,2056),(3055,1000)] # left (purple) starting possition (0,0 in corner near beach huts)	
        BEACONS = [(-62,-62),(-62,2062),(3062,1000)] # left (purple) starting possition (0,0 in corner near beach huts)	
        print 'Beacons for purple side'
    """Main function for localisation"""
    try: 
        # s connect to lidar, pc connects to pc server
        s = start_lidar(socket.AF_INET, socket.SOCK_STREAM, TCP_IP, TCP_PORT)
        #pc = connect_pc(HOST,PORT)
        # initialize myrobot and particles
        myrobot = Robot(True,start_position)
        #print myrobot
        p = [Robot(True, start_position) for i in xrange(N)]
        # position of robot at the start of the game (important for odometry)
        old = start_position
        # particle weights; w_re is constant that we use after resampling
        w_re = [1.0/N for i in xrange(N)]
        w_prev = [1.0 for i in xrange(N)]
        current_coordinatess[0] = start_position[0]
        current_coordinatess[1] = start_position[1]
        current_coordinatess[2] = start_position[2]
        # create pool for separate lidar thread
        pool = ThreadPool(processes=1)
        #time.sleep(1)
        flag.value = True
        while start_flag.value == True:
            continue
        print "particle filter started", start_flag.value
        try:
            while 1:
                # calculate relative motion of robot
                rel_motion, old2 = relative_motion(input_command_queue, 
                                                    reply_to_localization_queue, 
                                                    old, correction_performed,
                                                    myrobot, cc_robot)	
                # dispatche lidar into separate thread to get data
                lidar_thread = pool.apply_async(lidar_worker, (s,))
                # move particles based on odometry
                # change N to some number < 200 to leave some particles in place
                p2 = [p[i].move(rel_motion) for i in xrange(150)]
                p = p2 + p[150:]
                old = old2
                # return data from lidar
                data_lidar = lidar_thread.get()
                # calculate angle and distance from lidar data
                angle, distance = lidar_scan(data_lidar) 
                # calculate x and y position of beacons in robot coord system
                x_rob, y_rob = p_trans(angle, distance)		
                # update weights based on lidar data
                w_next =[p[i].weight(x_rob, y_rob, BEACONS) for i in xrange(N)]
                # normalize weights
                w_n_sum = sum(w_next)
                try:
                    w_n_norm = [i/w_n_sum for i in w_next]
                except ZeroDivisionError:
                    print 'zero weights in lidar'
                    w_n_norm = [0.0 for i in xrange(N)]
                # incorporate new weights with old ones and normalize
                w = [w_prev[i]*w_n_norm[i] for i in xrange(N)]
                w_sum = sum(w)
    
                try:
                    w_norm = [i/w_sum for i in w]
                except ZeroDivisionError:
                    print 'zero weights in lidar'
                    w_norm = [0.0 for i in xrange(N)]
                # calculate weighted mean value of robot pose from particles	
                mean_val = [(p[i].x*w_norm[i], p[i].y*w_norm[i]) for i in xrange(N)]
                mean_orientation = mean_angl(p, w_norm)
                center = reduce(lambda x,y: (x[0] + y[0], x[1] + y[1]), mean_val)
                # set new robot pose 
                myrobot.set(center[0]-R*math.cos(mean_orientation+BETA), 
                            center[1]-R*math.sin(mean_orientation+BETA), 
                            mean_orientation)
                #print myrobot
                current_coordinatess[0] = myrobot.x/1000
                current_coordinatess[1] = myrobot.y/1000
                current_coordinatess[2] = myrobot.orientation
                w_prev = w_norm
###############################################################################
                #UNCOMMENT THIS FOR SENDING DATA TO REMOTE SERVER
                '''data = []
                for part in p:
                    data.extend(part.pose())
                data.extend(w_prew)
                data.extend(x_rob)
                data.extend(y_rob)
                send = struct.pack('4si%if' %len(data),'xxxx',len(data), *data)
                try:
                    pc.sendall(send)
                except:
                    pc = connect_pc(HOST,PORT)'''
###############################################################################
                # calculate efective value and resample if needed
                n_eff = 1.0/(sum([math.pow(i, 2) for i in w_norm]))
                if n_eff < n_trash:# and sum(rel_motion) > 0.1:
                    try:
                        # resamples new particles based on probability
                        p3 = resample(p, w_norm, N)
                        p = p3
                        # sets new weights for resampled particles
                        w_prev = w_re
                    except:
                        print 'error with choice'
        except:		
            # close connection with lidar and pc	
            s.shutdown(2)
            s.close()
            #pc.close()
    except:		
        # close connection with lidar and pc	
        s.shutdown(2)
        s.close()
        #pc.close()