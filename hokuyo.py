import socket
import binascii as baci
from collections import deque
import matplotlib.pyplot as plt
import numpy as np

TCP_IP = '192.168.0.10'
TCP_PORT = 10940
BUFFER_SIZE = 8192 #4096
version = 'VV\r'
parameters = 'PP\r'
state = 'II\r'
scan = 'BM\r'
stop = 'QT\r'
values = 'GD0000108000\r'
state2 = '%ST\r'
cont = 'GE0000108000\r'

def time_val(value):	
	return ((ord(value[0])-48)<<18)|((ord(value[1])-48)<<12)|((ord(value[2])-48)<<6)|(ord(value[3])-48)

def dist_val(value):	
	try:	
		return ((ord(value[0])-48)<<12)|((ord(value[1])-48)<<6)|(ord(value[2])-48)
	except IndexError:
		print 'error ', value	
		return 0		
		#s.shutdown(2)
		#s.close()	
		#plt.close('all')	
		#print 'END'		
		#exit()

def update_d(answer):		
	answer = answer.split('\n')		
	time = answer.pop(1)[:-1]
	dist = answer[1:-2]
	dist2 = [item[:-1] for item in dist]
	dist4 = ''.join(dist2)			
	idxl = 0			
	idxh= 3	
	polar_graph = deque()	
	angle = np.arange(0,len(dist4)/12.0,0.25)
	angle = np.radians(angle)	
	while idxh <= len(dist4):
		point = dist_val(dist4[idxl:idxh])
		polar_graph.append(point)
		idxl = idxh
		idxh += 3				
	if len(angle) != len(polar_graph):
		print 'not same length'
		return None	
	update_polar_plot(angle, polar_graph)

def update_di(answer):		
	answer = answer.split('\n')		
	time = answer.pop(1)[:-1]
	dist = answer[1:-2]
	dist2 = [item[:-1] for item in dist]
	dist4 = ''.join(dist2)			
	idxl, idxh = 0, 3			
	polar_graph = deque()
	angle = deque()
	step = 0	
	while idxh <= len(dist4):
		point = dist_val(dist4[idxl:idxh])
		strength = dist_val(dist4[idxh:idxh+3])
		if strength > 1600:	
			polar_graph.append(point)
			angle.append(step)
		idxl = idxh + 3
		idxh += 6		
		step += 0.004363323129985824
	try:
		update_polar_plot(angle, polar_graph)
	except:
		print 'not same length'

	
def init_xy_plot():
	""" setup an XY plot canvas """

	plt.ion()
	figure = plt.figure(figsize=(6, 6),
						dpi=160,
						facecolor="w",
						edgecolor="k")
	ax = figure.add_subplot(111)
	lines, = ax.plot([],[],linestyle="none",
						marker=".",
						markersize=3,
						markerfacecolor="blue")
	ax.set_xlim(-5000, 5000)
	ax.set_ylim(-5000, 5000)
	ax.grid()
	
def update_xy_plot():
	""" re-draw the XY plot with new current_frame """

	lines.set_xdata(self.current_frame.x)
	lines.set_ydata(self.current_frame.y)
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
	ax.set_rmax(500)
	ax.set_theta_direction(1) #set to clockwise
	ax.set_theta_offset(-np.pi/4) #offset by 90 degree so that 0 degree is at 12 o'clock
	#ax.grid()
	return figure, lines

def update_polar_plot(angle, dist):
	""" re-draw the polar plot with new current_frame """

	lines.set_xdata(angle)
	lines.set_ydata(dist)
	figure.canvas.draw()

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((TCP_IP, TCP_PORT))

figure, lines = init_polar_plot()

while True:
	try:
		msg = raw_input('Command: ')
		if msg == 'l':
			s.send(state2)
			data = s.recv(BUFFER_SIZE)	
			print data			
		if msg == 'v':
			s.send(version)
			data = s.recv(BUFFER_SIZE)	
			print data
		elif msg ==	'p':
			s.send(parameters)
			data = s.recv(BUFFER_SIZE)
			print data
		elif msg ==	's':
			s.send(state)
			data = s.recv(BUFFER_SIZE)
			print data	
		elif msg ==	'b':
			s.send(scan)
			data = s.recv(BUFFER_SIZE)
			print data
		elif msg ==	'q':
			s.send(stop)
			data = s.recv(BUFFER_SIZE)
			print data
		elif msg ==	'g':
			s.send(values)
			answer = s.recv(BUFFER_SIZE)
			print type(answer)
			print answer
			d = deque(answer)
			print type(d)
			print d			
			answer = answer.split('\n')
			print 'data ', answer	
			print 'len ', len(answer)		
			time = answer.pop(1)[:-1]
			print time			
			print time_val(time)
			print 'data ', answer
			dist = answer[1:-2]
			print dist
			dist2 = [item[:-1] for item in dist]
			print 'list comprehension ', dist2
			dist3 = deque(''.join(dist2))
			dist4 = ''.join(dist2)			
			#dist3 = [dist_val(dist
			idxl = 0			
			idxh= 3	
			polar_graph = deque()	
			while idxh <= len(dist4):
				point = dist_val(dist4[idxl:idxh])
				polar_graph.append(point)
				idxl = idxh
				idxh += 3	
			print 'polar ', len(polar_graph), '\n', polar_graph		
			angle = np.arange(0,len(polar_graph)/4.0,0.25)
			angle = np.radians(angle)			
			print angle	
			print 'len dist %i, len angle %i' %(len(polar_graph), len(angle))			
			update_polar_plot(angle, polar_graph)
		elif msg == 'c':
			while True:
				s.send(values)
				answer = s.recv(BUFFER_SIZE)
				update_d(answer)
		elif msg == 'n':
			while True:
				s.send(cont)
				answer = s.recv(BUFFER_SIZE)
				update_di(answer)	
		elif msg == 'k':
			s.send(cont)
			answer = s.recv(BUFFER_SIZE)		
			answer = answer.split('\n')		
			print 'answer ', answer
			time = answer.pop(1)[:-1]
			dist = answer[1:-2]
			dist2 = [item[:-1] for item in dist]
			dist4 = ''.join(dist2)
			print 'dist4 ', dist4
			print 'len dist4 ', len(dist4)			
			idxl, idxh, maxs = 0, 3, 0			
			polar_graph = deque()
			#angle = np.arange(0,len(dist4)/24.0,0.25)
			#angle = np.radians(angle)
			angle = deque()
			step = 0
			print 'len angle ', len(angle)	
			while idxh <= len(dist4):
				point = dist_val(dist4[idxl:idxh])
				strength = dist_val(dist4[idxh:idxh+3])
				if strength > maxs:
					maxs = strength
				#print 'strenght ', strength
				if strength > 1600:	
					polar_graph.append(point)
					angle.append(step)
				idxl = idxh + 3
				idxh += 6		
				step += np.radians(0.25)	
			if len(angle) != len(polar_graph):
				print 'not same length, angle len %i, polar_graph len %i' %(len(angle), len(polar_graph))
			try:
				update_polar_plot(angle, polar_graph)
				print 'max strength ', maxs
			except:
				print 'not same length'	
	except KeyboardInterrupt:
		s.shutdown(2)
		s.close()	
		plt.close('all')	
		print 'END'
		break
