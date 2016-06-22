import serialWrapper
import packetBuilder
import packetParser
import socket
import Queue
import sys
import multiprocessing
    
from serial.tools import list_ports

commands_to_stm = packetBuilder.CommandsList()
current_module = sys.modules[__name__]
com_port = None

def create_connection_to_stm():
    port_number = get_com_port_number()
    return serialWrapper.SerialWrapper(port_number)

def get_com_port_number():
    vid = 1155
    pid = 22336
    snr = '3677346C3034'

    for port in list_ports.comports():
        print port
        if (port.serial_number == snr) and (port.pid == pid) and (port.vid == vid): 
            return '/dev/' + port.name

def process_request(command, parameters):
    if parameters is '':
        reply = getattr(current_module, command)()
    else:
        reply = getattr(current_module, command)(parameters)
    return reply

def send_request(packet):
    return com_port.sendRequest(packet.bytearray).reply

def switch_on_pid():
    packet = packetBuilder.BuildPacket(commands_to_stm.switchOnPid)
    reply_on_request = send_request(packet)
    return reply_on_request

def switch_off_pid():
    packet = packetBuilder.BuildPacket(commands_to_stm.switchOffPid)
    reply_on_request = send_request(packet)
    return reply_on_request

def switch_off_robot():
    packet = packetBuilder.BuildPacket(commands_to_stm.switchOffRobot)
    reply_on_request = send_request(packet)
    return reply_on_request

def switch_on_tajectory_regulator():
    packet = packetBuilder.BuildPacket(commands_to_stm.switchOnTrajectoryRegulator)
    reply_on_request = send_request(packet)
    return reply_on_request

def switch_off_tajectory_regulator():
    packet = packetBuilder.BuildPacket(commands_to_stm.switchOffTrajectoryRegulator)
    reply_on_request = send_request(packet)
    return reply_on_request

def switch_on_kinematic_calculator():
    packet = packetBuilder.BuildPacket(commands_to_stm.switchOnKinematicCalculation)
    reply_on_request = send_request(packet)
    return reply_on_request

def switch_off_kinematic_calculator():
    packet = packetBuilder.BuildPacket(commands_to_stm.switchOffKinematicCalculation)
    reply_on_request = send_request(packet)
    return reply_on_request

def set_coordinates_without_movement(parameters):
    packet = packetBuilder.BuildPacket(commands_to_stm.setCoordinates, parameters)
    reply_on_request = send_request(packet)
    return reply_on_request

def set_coordinates_with_movement(parameters):
    packet = packetBuilder.BuildPacket(commands_to_stm.setCorectCoordinates, parameters)
    reply_on_request = send_request(packet)
    return reply_on_request

def get_current_coordinates():
    packet = packetBuilder.BuildPacket(commands_to_stm.getCurentCoordinates)
    reply_on_request = send_request(packet)
    return reply_on_request

def set_movement_speed(parameters):
    packet = packetBuilder.BuildPacket(commands_to_stm.setMovementSpeed, parameters)
    reply_on_request = send_request(packet)
    return reply_on_request

def go_to_global_point(parameters):
    packet = packetBuilder.BuildPacket(commands_to_stm.addPointToStack, parameters)
    reply_on_request = send_request(packet)
    return reply_on_request

def set_cube_manipulator_angle(parameters):
    packet = packetBuilder.BuildPacket(commands_to_stm.setManipulatorAngle, parameters)
    reply_on_request = send_request(packet)
    return reply_on_request 

def close_cube_collector():
    packet = packetBuilder.BuildPacket(commands_to_stm.closeCubeCollector)
    reply_on_request = send_request(packet)
    return reply_on_request

def open_cube_collector():
    packet = packetBuilder.BuildPacket(commands_to_stm.openCubeCollector)
    reply_on_request = send_request(packet)
    return reply_on_request

def switch_on_vibration_table(parameter): 
    packet = packetBuilder.BuildPacket(commands_to_stm.switchOnVibrationTable, parameter)
    reply_on_request = send_request(packet)
    return reply_on_request

def switch_off_vibration_table(): 
    packet = packetBuilder.BuildPacket(commands_to_stm.switchOffVibrationTable)
    reply_on_request = send_request(packet)
    return reply_on_request

def open_cubes_border(): 
    packet = packetBuilder.BuildPacket(commands_to_stm.openCubeBorder)
    reply_on_request = send_request(packet)
    return reply_on_request

def close_cubes_border(): 
    packet = packetBuilder.BuildPacket(commands_to_stm.closeCubeBorder)
    reply_on_request = send_request(packet)
    return reply_on_request

def is_point_was_reached():
    packet = packetBuilder.BuildPacket(commands_to_stm.isPointWasReached)
    reply_on_request = send_request(packet)
    return reply_on_request

def switch_on_collision_avoidance():
    packet = packetBuilder.BuildPacket(commands_to_stm.switchOnCollisionAvoidance)
    reply_on_request = send_request(packet)
    return reply_on_request

def switch_off_collision_avoidance():
    packet = packetBuilder.BuildPacket(commands_to_stm.switchOffCollisionAvoidance)
    reply_on_request = send_request(packet)
    return reply_on_request

def check_collision_avoidance():
    packet = packetBuilder.BuildPacket(commands_to_stm.checkCollisionAvoidanceFlag)
    reply_on_request = send_request(packet)
    return reply_on_request

def get_ik_data():
    packet = packetBuilder.BuildPacket(commands_to_stm.getDataIKSensors)
    reply_on_request = send_request(packet)
    return reply_on_request

def clean_point_stack():
    packet = packetBuilder.BuildPacket(commands_to_stm.cleanPointsStack)
    reply_on_request = send_request(packet)
    return reply_on_request

def get_us_data():
    packet = packetBuilder.BuildPacket(commands_to_stm.getDataUSSensors)
    reply_on_request = send_request(packet)
    return reply_on_request

def get_robot_speeds():
    packet = packetBuilder.BuildPacket(commands_to_stm.getCurrentSpeed)
    reply_on_request = send_request(packet)
    return reply_on_request

def move_with_correction(parameters):
    packet = packetBuilder.BuildPacket(commands_to_stm.moveWithCorrection, parameters)
    reply_on_request = send_request(packet)
    return reply_on_request

def get_manipulator_angle():
    packet = packetBuilder.BuildPacket(commands_to_stm.getManipulatorAngle)
    reply_on_request = send_request(packet)
    return reply_on_request

def open_manipulator_big_angle():
    packet = packetBuilder.BuildPacket(commands_to_stm.openCubeManipulatorBigAngle)
    reply_on_request = send_request(packet)
    return reply_on_request

def open_cone_crasher():
    packet = packetBuilder.BuildPacket(commands_to_stm.openConeCrasher)
    reply_on_request = send_request(packet)
    return reply_on_request

def close_cone_crasher():
    packet = packetBuilder.BuildPacket(commands_to_stm.closeConeCrasher)
    reply_on_request = send_request(packet)
    return reply_on_request

def close_doors():
    packet = packetBuilder.BuildPacket(commands_to_stm.closeDoors)
    reply_on_request = send_request(packet)
    return reply_on_request

def open_doors():
    packet = packetBuilder.BuildPacket(commands_to_stm.openDoors)
    reply_on_request = send_request(packet)
    return reply_on_request

def close_doors():
    packet = packetBuilder.BuildPacket(commands_to_stm.closeDoors)
    reply_on_request = send_request(packet)
    return reply_on_request

def open_doors():
    packet = packetBuilder.BuildPacket(commands_to_stm.openDoors)
    reply_on_request = send_request(packet)
    return reply_on_request

def slightly_open_doors():
    packet = packetBuilder.BuildPacket(commands_to_stm.slightlyOpenDoors)
    reply_on_request = send_request(packet)
    return reply_on_request

'''def get_doors_state():
    packet = packetBuilder.BuildPacket(commands_to_stm.####)
    reply_on_request = send_request(packet)
    return reply_on_request'''

def pull_predators_mouth():
    packet = packetBuilder.BuildPacket(commands_to_stm.pullPredatorsMouth)
    reply_on_request = send_request(packet)
    return reply_on_request

def push_predators_mouth():
    packet = packetBuilder.BuildPacket(commands_to_stm.pushPredatorsMouth)
    reply_on_request = send_request(packet)
    return reply_on_request

def close_predators_mouth():
    packet = packetBuilder.BuildPacket(commands_to_stm.closePredatorsMouth)
    reply_on_request = send_request(packet)
    return reply_on_request

def switch_on_pneumo():
    packet = packetBuilder.BuildPacket(commands_to_stm.switchOnPneumo)
    reply_on_request = send_request(packet)
    return reply_on_request

def switch_off_pneumo():
    packet = packetBuilder.BuildPacket(commands_to_stm.switchOffPneumo)
    reply_on_request = send_request(packet)
    return reply_on_request

def unload_middle():
    packet = packetBuilder.BuildPacket(commands_to_stm.unloadMiddle)
    reply_on_request = send_request(packet)
    return reply_on_request

def open_cubes_border(): 
    packet = packetBuilder.BuildPacket(commands_to_stm.openHolder)
    reply_on_request = send_request(packet)
    return reply_on_request

def close_cubes_border(): 
    packet = packetBuilder.BuildPacket(commands_to_stm.closeHolder)
    reply_on_request = send_request(packet)
    return reply_on_request	

def stmMainLoop(input_command_queue, reply_to_fsm_queue, reply_to_localization_queue):
    global com_port
    com_port = create_connection_to_stm()

    while(True):
        incoming_command = input_command_queue.get()
        #print incoming_command
        if incoming_command['request_source'] == 'fsm':
            reply = process_request(incoming_command['command'], 
                                    incoming_command['parameters'])
            reply_to_fsm_queue.put(reply)
        elif incoming_command['request_source'] == 'localisation':
            reply = process_request(incoming_command['command'], 
                                    incoming_command['parameters'])
            reply_to_localization_queue.put(reply)



