#!/usr/bin/env python
# -*- coding: utf-8 -*-
import socket

sock = socket.socket()
sock.bind(("10.30.65.227",9090)) #host and port (0-1023 - some special priority), limit - 65535
sock.listen(1)
conn, addr = sock.accept()

print 'connected:', addr

def cooors():
	data = conn.recv(1024)
	return data

conn.close()