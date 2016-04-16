#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket
import time
X = 100
Y = 200
a = 30

sock = socket.socket()
sock.connect(('10.30.65.226', 9090))
while True:
	sock.send(str(X))
	#sock.send(str(Y))
	#sock.send(str(a))
	#time.sleep(1)
	data = sock.recv(1024)
#sock.close()

print data