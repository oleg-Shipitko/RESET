# import socket
# import sys
# import time

# HOST, PORT = "localhost", 9999
# i = 0
# data = [i]

# # Create a socket (SOCK_STREAM means a TCP socket)
# sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# sock.connect((HOST, PORT))
# try:
    
#     while i < 100:
#         # Connect to server and send data
        sock.sendall(str(data)+'\n')
#         i +=1
#         data.append(i)
#         print("Sent: %d [%d]" % (i, time.time()))    # Receive data from the server and shut down
#     #received = sock.recv(1024)
# except Exception as err:
#     print('Err:', err)
#     sock.close()

# print "Sent:     {}".format(data)
# #print "Received: {}".format(received)

# def  StairCase(n):
#     level = [i for i in xrange(n-1,-1,-1)]
#     inv = [i for i in xrange(1,n+1)]
#     k = zip(level,inv)
#     #print k
#     for idx,j in k:
#         print idx*' '+ j*'#'
#         #print 'maki'

# StairCase(6)

import numpy as np

