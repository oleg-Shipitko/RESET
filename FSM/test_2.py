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

packet = packetBuilder.BuildPacket(commands_to_stm.switchOnPid)
recievedPacket = port.sendRequest(packet.bytearray)


packet = packetBuilder.BuildPacket(commands_to_stm.switchOnTrajectoryRegulator)
recievedPacket = port.sendRequest(packet.bytearray)

packet = packetBuilder.BuildPacket(commands_to_stm.switchOnKinematicCalculation)
recievedPacket = port.sendRequest(packet.bytearray)

packet = packetBuilder.BuildPacket(commands_to_stm.setCoordinates, [0.1, 0.72, 0])
recievedPacket = port.sendRequest(packet.bytearray)

packet = packetBuilder.BuildPacket(commands_to_stm.addPointToStack, [0.2, 0.72, 0, 0])
recievedPacket = port.sendRequest(packet.bytearray)

packet = packetBuilder.BuildPacket(commands_to_stm.addPointToStack, [0.2, 0.72, 0, 0])
recievedPacket = port.sendRequest(packet.bytearray)

packet = packetBuilder.BuildPacket(commands_to_stm.addPointToStack, [0.4, 0.72, 0.0, 1])
recievedPacket = port.sendRequest(packet.bytearray)

packet = packetBuilder.BuildPacket(commands_to_stm.getNumberOfReachedPoints)
reply = port.sendRequest(packet.bytearray).reply

print 'answer: ', reply