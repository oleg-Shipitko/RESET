import socket

TCP_IP = '192.168.0.10'
TCP_PORT = 10940
BUFFER_SIZE = 4096
version = 'VV\r'
parameters = 'PP\r'
state = 'II\r'
scan = 'BM\r'
stop = 'QT\r'
values = 'GD0000010000\r'
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print 'maki3'
s.connect((TCP_IP, TCP_PORT))
print 'maki2'
print 'maki'

while True:
	try:
		msg = raw_input('Command: ')
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
			data = s.recv(BUFFER_SIZE)
			print data	
	except KeyboardInterrupt:
		s.shutdown(2)
		s.close()		
		print 'END'
