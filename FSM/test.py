import serialWrapper
import packetBuilder
import packetParser
import socket
import Queue
import sys
import multiprocessing
import time

from serial.tools import list_ports

commands_to_stm = packetBuilder.CommandsList()
    
def get_com_port_number():
    vid = 1155
    pid = 22336
    snr = '3677346C3034'

    for port in list_ports.comports():
        if (port.serial_number == snr) and (port.pid == pid) and (port.vid == vid): 
            return '/dev/' + port.name

port_number = get_com_port_number()
port = serialWrapper.SerialWrapper(port_number)

	# Build packet for sending to robot	
packet = packetBuilder.BuildPacket(commands_to_stm.switchOnPid)
		
	# send packet to port. sendRequest method will wait answer from robot. In case 
	# if you don't need answer possible to use 'sendData' method (just send data, 
	# without waiting for answer)
startT = time.time()	
recievedPacket = port.sendRequest(packet.bytearray)
endT = time.time()
print 'Recieved ', (endT - startT)
if recievedPacket.reply == 'Ok':
	print 'PID controller On'		
else:
	raise Exception('switchOnPid failed')
	
packet = packetBuilder.BuildPacket(commands_to_stm.switchOnTrajectoryRegulator)
tartT = time.time()	
recievedPacket = port.sendRequest(packet.bytearray)
endT = time.time()
print 'Recieved ', (endT - startT)
if recievedPacket.reply == 'Ok':
	print 'Trajectory regulator ON'
else:
	raise Exception('switchOnTrajectoryRegulator failed')
	
packet = packetBuilder.BuildPacket(commands_to_stm.switchOnKinematicCalculation)
startT = time.time()
recievedPacket = port.sendRequest(packet.bytearray)
endT = time.time()
print 'Recieved ', (endT - startT)
if recievedPacket.reply == 'Ok':
	print 'Kinematics ON'
else:
	raise Exception('switchOnKinematicCalculation failed')

#packet = packetBuilder.BuildPacket(commands_to_stm.setCoordinates, [0.1525, 0.72, 0])
#port.sendRequest(packet.bytearray)
packet = packetBuilder.BuildPacket(commands_to_stm.addPointToStack, [0.1725, 0.72, 0.0, 1])
recievedPacket = port.sendRequest(packet.bytearray)
time.sleep(2)
packet = packetBuilder.BuildPacket(commands_to_stm.getNumberOfReachedPoints)
reply = port.sendRequest(packet.bytearray).reply