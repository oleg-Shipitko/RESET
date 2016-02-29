import serialWrapper 
import packetBuilder
import packetParser

# COM port initialization
computerPort = serialWrapper.SerialWrapper('/dev/ttyACM0')

# we will choose commands which we want to send from this list
commands = packetBuilder.CommandsList()

# build packet for sending to robot. 'switchOffPid' command doesn't require parameters
packet = packetBuilder.BuildPacket(commands.switchOffPid)
# send packet to port. sendRequest method will wait answer from robot. In case if you don't need answer possible to use 'sendData' method (just send data, without waiting for answer)
recievedPacket = computerPort.sendRequest(packet.bytearray)

# print recieved answer
print recievedPacket.reply

coordinates = [0.1, 0.2, 0.3]
# build packet for sending to robot. 'setCoordinates' command require 3 parameters (coordinates). 
packet = packetBuilder.BuildPacket(commands.setCoordinates, coordinates)
recievedPacket = computerPort.sendRequest(packet.bytearray)

print recievedPacket.reply

directionBit = 1
# build packet for sending to robot. 'setDirectionBit' command require 1 parameter (directionBit). 
packet = packetBuilder.BuildPacket(commands.setDirectionBit, directionBit)
recievedPacket = computerPort.sendRequest(packet.bytearray)

print recievedPacket.reply


