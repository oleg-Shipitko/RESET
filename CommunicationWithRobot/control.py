import serialWrapper 
import packetBuilder
import packetParser

def initPTC():
	"""Initialize PID, Trajectory, Kinematics"""
	# build packet for sending to robot. 'switchOffPid' command doesn't require parameters
	packet = packetBuilder.BuildPacket(commands.switchOnPid)
	# send packet to port. sendRequest method will wait answer from robot. In case 
	# if you don't need answer possible to use 'sendData' method (just send data, 
	# without waiting for answer)
	recievedPacket = computerPort.sendRequest(packet)
	# If don't get right response, raise exception
	if recievedPacket.reply != 'Ok':
		raise Exception('switchOnPid failed')

	packet = packetBuilder.BuildPacket(commands.switchOnTrajectoryRegulator)
	recievedPacket = computerPort.sendRequest(packet)
	if recievedPacket.reply != 'Ok':
		raise Exception('switchOnTrajectoryRegulator failed')

	packet = packetBuilder.BuildPacket(commands.switchOnKinematicCalculation)
	recievedPacket = computerPort.sendRequest(packet)
	if recievedPacket.reply != 'Ok':
		raise Exception('switchOnKinematicCalculation failed')


# Get port from user, example '/dev/ttyACM2'
def port_number():
	port = raw_input('Is port COM3, [y/n]? ')
	if port == 'y':
		return port
	else:
		port = raw_input('Define new port number: ')
		return port

port = port_number()

# COM port initialization 
computerPort = serialWrapper.SerialWrapper(port)

# we will choose commands which we want to send from this list
commands = packetBuilder.CommandsList()

# Initialize PID, Trajectory and Kinematics
initPTC()

while True:
	# Get user input for movement
	coordinates = raw_input('X, Y, angle: ')

	# build packet for sending to robot. 'setCoordinates' command require 
	# 3 parameters (x , y, angle(rad)). 
	packet = packetBuilder.BuildPacket(commands.addPointToStack, coordinates)
	recievedPacket = computerPort.sendRequest(packet)

	if recievedPacket.reply != 'Ok':
			raise Exception('setCoordinates failed')

# directionBit = 1
# # build packet for sending to robot. 'setDirectionBit' command require 
# # 1 parameter (directionBit). 
# packet = packetBuilder.BuildPacket(commands.setDirectionBit, directionBit)
# recievedPacket = computerPort.sendRequest(packet)

# print recievedPacket.reply



	
	
	

	


