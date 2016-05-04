import packetBuilder
import robotInitialization
import BigRobotTrajectories
import time
import lidar3
import traceback
import multiprocessing
import math

computerPort = serialWrapper.SerialWrapper("/dev/ttyACM0")
commands = packetBuilder.CommandsList()

raw_input("Press_Enter....switchOnBelts")

packet = packetBuilder.BuildPacket(commands.getADCPinState, 2)
recievedPacket = computerPort.sendRequest(packet.bytearray)
print recievedPacket.reply


raw_input("Press_Enter....switchOnVibrationTable")


packet = packetBuilder.BuildPacket(commands.switchOnVibrationTable)
recievedPacket = computerPort.sendRequest(packet.bytearray)

print recievedPacket.reply

raw_input("Press_Enter....switchOffBelts")

packet = packetBuilder.BuildPacket(commands.switchOffBelts)
recievedPacket = computerPort.sendRequest(packet.bytearray)

print recievedPacket.reply

raw_input("Press_Enter....switchOffVibrationTable")

packet = packetBuilder.BuildPacket(commands.switchOffVibrationTable)
recievedPacket = computerPort.sendRequest(packet.bytearray)

raw_input("Press_Enter....")

packet = packetBuilder.BuildPacket(commands.switchOffBelts)
recievedPacket = computerPort.sendRequest(packet.bytearray)

raw_input("Press_Enter....")


packet = packetBuilder.BuildPacket(commands.setManipulatorAngle, 165.0)
recievedPacket = computerPort.sendRequest(packet.bytearray)

time.sleep(1)


packet = packetBuilder.BuildPacket(commands.closeCubeCollector)
recievedPacket = computerPort.sendRequest(packet.bytearray)

time.sleep(1)

packet = packetBuilder.BuildPacket(commands.setManipulatorAngle, 270.0)
recievedPacket = computerPort.sendRequest(packet.bytearray)

time.sleep(1)

packet = packetBuilder.BuildPacket(commands.openCubeCollector)
recievedPacket = computerPort.sendRequest(packet.bytearray)

time.sleep(1)

packet = packetBuilder.BuildPacket(commands.setManipulatorAngle, 130.0)
recievedPacket = computerPort.sendRequest(packet.bytearray)

time.sleep(1)

packet = packetBuilder.BuildPacket(commands.closeCubeCollector)
recievedPacket = computerPort.sendRequest(packet.bytearray)

print recievedPacket.reply

raw_input("Press_Enter....")



packet = packetBuilder.BuildPacket(commands.setManipulatorAngle, 190.0)
recievedPacket = computerPort.sendRequest(packet.bytearray)

raw_input("Press_Enter....")


packet = packetBuilder.BuildPacket(commands.openCubeCollector)
recievedPacket = computerPort.sendRequest(packet.bytearray)

raw_input("Press_Enter....")


packet = packetBuilder.BuildPacket(commands.setManipulatorAngle, 180.0)
recievedPacket = computerPort.sendRequest(packet.bytearray)

raw_input("Press_Enter....")


packet = packetBuilder.BuildPacket(commands.closeCubeCollector)
recievedPacket = computerPort.sendRequest(packet.bytearray)

print recievedPacket.reply

raw_input("Press_Enter....")

packet = packetBuilder.BuildPacket(commands.setManipulatorAngle, 270.0)
recievedPacket = computerPort.sendRequest(packet.bytearray)

time.sleep(1)


packet = packetBuilder.BuildPacket(commands.openCubeCollector)
recievedPacket = computerPort.sendRequest(packet.bytearray)

raw_input("Press_Enter....")

MoveToPointState(0.40, 0.18, -1.57),
MoveToPointState(0.40, 0.18, -3.14), # clse the doors
MoveToPointState(0.40, 0.12, -3.14),
MoveToPointState(0.40, 0.18, -3.14),
MoveToPointState(0.40, 0.18, -1.57),
CorrectCoordinatesState_1(-0.03, 0.935, -1.57),