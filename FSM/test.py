import serialWrapper
import packetBuilder
import packetParser
import socket
import Queue
import sys
import multiprocessing

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

while True:
    a = float(raw_input())
    packet = packetBuilder.BuildPacket(commands_to_stm.setManipulatorAngle, a)
    print port.sendRequest(packet.bytearray).reply