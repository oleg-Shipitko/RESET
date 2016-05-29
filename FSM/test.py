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

packet = packetBuilder.BuildPacket(commands_to_stm.openConeCrasher)
reply = port.sendRequest(packet.bytearray).reply
print reply

raw_input('PressEnter')

packet = packetBuilder.BuildPacket(commands_to_stm.closeConeCrasher)
reply = port.sendRequest(packet.bytearray).reply
print reply

raw_input('PressEnter')
while True:
    angle = float(raw_input('Angle: '))
    packet = packetBuilder.BuildPacket(commands_to_stm.setManipulatorAngle, angle)
    reply = port.sendRequest(packet.bytearray).reply
    print reply

packet = packetBuilder.BuildPacket(commands_to_stm.setManipulatorAngle, 180)
reply = port.sendRequest(packet.bytearray).reply
raw_input('Enter')

packet = packetBuilder.BuildPacket(commands_to_stm.openCubeManipulatorBigAngle)
reply = port.sendRequest(packet.bytearray).reply
raw_input('Enter')

packet = packetBuilder.BuildPacket(commands_to_stm.openCubeCollector)
reply = port.sendRequest(packet.bytearray).reply
raw_input('Enter')

while True:
    angle = float(raw_input('Angle: '))
    packet = packetBuilder.BuildPacket(commands_to_stm.setManipulatorAngle, angle)
    reply = port.sendRequest(packet.bytearray).reply
    print reply

packet = packetBuilder.BuildPacket(commands_to_stm.openCubeBorder)
reply = port.sendRequest(packet.bytearray).reply
raw_input('Enter')

packet = packetBuilder.BuildPacket(commands_to_stm.setManipulatorAngle, 283)
reply = port.sendRequest(packet.bytearray).reply
raw_input('Enter')

packet = packetBuilder.BuildPacket(commands_to_stm.closeCubeCollector)
reply = port.sendRequest(packet.bytearray).reply
raw_input('Enter')

packet = packetBuilder.BuildPacket(commands_to_stm.setManipulatorAngle, 290)
reply = port.sendRequest(packet.bytearray).reply

while True:
    time.sleep(0.5)
    packet = packetBuilder.BuildPacket(commands_to_stm.getManipulatorAngle)
    reply = port.sendRequest(packet.bytearray).reply
    print reply

reply = 0

while reply < 209:
    packet = packetBuilder.BuildPacket(commands_to_stm.getManipulatorAngle)
    reply = port.sendRequest(packet.bytearray).reply
    print reply

raw_input('Press to get IK data')

parameters = [0.0, 0.0, 0.0, 0.0, 0.1, 1.57, 1]
packet = packetBuilder.BuildPacket(commands_to_stm.moveWithCorrection, parameters)
reply = port.sendRequest(packet.bytearray).reply
print reply

raw_input('Press to get US data')
packet = packetBuilder.BuildPacket(commands_to_stm.getDataUSSensors)
reply = port.sendRequest(packet.bytearray).reply
print reply

while True:
    raw_input('switched on. Press to switched off')
    packet = packetBuilder.BuildPacket(commands_to_stm.getDataUSSensors)
    reply = port.sendRequest(packet.bytearray).reply
    print reply



raw_input('switched on. Press to switched off')

packet = packetBuilder.BuildPacket(commands_to_stm.closeCubeCollector)
reply = port.sendRequest(packet.bytearray).reply

print reply

raw_input('switched on. Press to switched off')

packet = packetBuilder.BuildPacket(commands_to_stm.openCubeCollector)
reply = port.sendRequest(packet.bytearray).reply

print reply

raw_input('switched on. Press to switched off')


'''packet = packetBuilder.BuildPacket(commands_to_stm.setManipulatorAngle, 270)
reply = port.sendRequest(packet.bytearray).reply

print reply

raw_input('setManipulatorAngle. 270')

packet = packetBuilder.BuildPacket(commands_to_stm.setManipulatorAngle, 193)
reply = port.sendRequest(packet.bytearray).reply

print reply

raw_input('setManipulatorAngle. 193')

packet = packetBuilder.BuildPacket(commands_to_stm.setManipulatorAngle, 160)
reply = port.sendRequest(packet.bytearray).reply

print reply

raw_input('setManipulatorAngle. 160')

packet = packetBuilder.BuildPacket(commands_to_stm.setManipulatorAngle, 125)
reply = port.sendRequest(packet.bytearray).reply

print reply

raw_input('setManipulatorAngle. 125')


packet = packetBuilder.BuildPacket(commands_to_stm.switchOnCollisionAvoidance)
reply = port.sendRequest(packet.bytearray).reply

print reply

raw_input('switched on. Press to switched off')

packet = packetBuilder.BuildPacket(commands_to_stm.switchOffCollisionAvoidance)
reply = port.sendRequest(packet.bytearray).reply

print reply

while True:
    raw_input('Press enter')
    packet = packetBuilder.BuildPacket(commands_to_stm.closeCubeCollector)
    recievedPacket = port.sendRequest(packet.bytearray).reply
    print recievedPacket
    raw_input('Press enter')
    packet = packetBuilder.BuildPacket(commands_to_stm.openCubeCollector)
    recievedPacket = port.sendRequest(packet.bytearray).reply
    print recievedPacket

raw_input('Press enter')
packet = packetBuilder.BuildPacket(commands_to_stm.addPointToStack, [0.1725, 0.72, 0.0, 1])
recievedPacket = port.sendRequest(packet.bytearray)


#packet = packetBuilder.BuildPacket(commands_to_stm.setCoordinates, [0.1525, 0.72, 0])
#port.sendRequest(packet.bytearray)
#packet = packetBuilder.BuildPacket(commands_to_stm.addPointToStack, [0.1725, 0.72, 0.0, 1])
#recievedPacket = port.sendRequest(packet.bytearray)
#time.sleep(2)
#packet = packetBuilder.BuildPacket(commands_to_stm.getNumberOfReachedPoints)
reply = port.sendRequest(packet.bytearray).reply'''