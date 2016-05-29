
import socket, time
import sys
import multiprocessing

def readlines(sock, recv_buffer=4096, delim='\n'):
	buffer = ''
	data = True
	while data:
		data = sock.recv(recv_buffer)
		buffer += data
		#print 'in first while loop'
		while buffer.find(delim) != -1:
			#print 'in second while loop'
			line, buffer = buffer.split('\n', 1)
			yield line
	return

def main(data_queue):
	HOST = ''      # Symbolic name meaning all available interfaces
	PORT = 9999            # Arbitrary non-privileged port
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind((HOST, PORT))
	s.listen(1)
	conn, addr = s.accept()
	#print 'Connected by', addr
	message = readlines(conn)
	while 1:
		try:
			#print 'what?'
			data_queue.put(eval(message.next()))
		except Exception as err:
			print 'Error on server: ', err
			conn.close()
			sys.exit()





#     data = conn.recv(4096)
#     if not data: break
#     conn.send(data)
# conn.close()