import stmDriver
import time
import multiprocessing
import math
import localisation
import socket

# PC server address and  port
HOST = '192.168.1.146'
PORT = 9090

input_command_queue = multiprocessing.Queue()
reply_to_fsm_queue = multiprocessing.Queue()
reply_to_localization_queue = multiprocessing.Queue()

request_source = 'fsm'
start_time = time.time()
last_update_for_robot = time.time()
check_start_time = True
start_game_time = None
current_coordinatess_from_robot = multiprocessing.Array('d', [0.0, 0.0, 0.0])
current_coordinatess = multiprocessing.Array('d', [0.0, 0.0, 0.0])
correction_performed = multiprocessing.Value('i', 0)

cubes_in_trunk = 0
trunk_capacity = 12
unloading_cubes_position = 0
taken_cubes_number = None

#start_position = [2.847, 0.77, -3.14]
server_ip = '192.168.1.146'
start_position = [0.1525, 0.72, 0.0]

class SwitchOnKinematicsAction(object):
    def run_action(self):
        stm_driver('switch_on_kinematic_calculator')

    def check_action(self):
        print 'Kinematics switched on'
        return True

class SwitchOnPidAction(object):
    def run_action(self):
        stm_driver('switch_on_pid')

    def check_action(self):
        print 'Pid switched on'
        return True

class SwitchOnTrajectoryAction(object):
    def run_action(self):
        stm_driver('switch_on_tajectory_regulator')

    def check_action(self):
        print 'Trajectory switched on'
        return True

class OpenCubesManipulatorAction(object):
    def run_action(self):
        #stm_driver('open_cube_collector')
        self.start_time = time.time()

    def check_action(self):
        if time.time() - self.start_time > 1:
            print time.time() - self.start_time
            return True

class SetCubesManipulatorAngleAction(object):
    def __init__(self, manipulator_angle):
        self.manipulator_angle = manipulator_angle

    def run_action(self):
        stm_driver('set_cube_manipulator_angle', self.manipulator_angle)
        self.start_time = time.time()

    def check_action(self):
        if time.time() - self.start_time > 1:
            return True

class SwitchOnVibrationTableAction():
    def run_action(self):
        stm_driver('switch_on_vibration_table')

    def check_action(self):
            print 'Vibration table was activated'
            return True

class OpenCubesBorderAction():
    def run_action(self):
        stm_driver('open_cubes_border')

    def check_action(self):
            print 'Cubes border is opened'
            return True

class SwitchOffVibrationTableAction():
    def run_action(self):
        stm_driver('switch_off_vibration_table')

    def check_action(self):
            print 'Vibration table was disactivated'
            return True

class CloseCubesBorderAction():
    def run_action(self):
        print 'close'
        #stm_driver('close_cubes_border')

    def check_action(self):
            print 'Cubes border was closed'
            return True

class CloseCubesManipulatorAction(object):
    def __init__(self):
        self.start_time = time.time()

    def run_action(self):
        global taken_cubes_number
        taken_cubes_number = stm_driver('close_cube_collector')

    def check_action(self):
        if time.time() - self.start_time > 1:
            return True

class WaitTimeAction(object):
    def __init__(self, time_delay):
        self.time_delay = time_delay
        self.start_time = None

    def run_action(self):
        self.start_time = time.time()

    def check_action(self):
        if time.time() - self.start_time > self.time_delay:
            return True

class SlowMoveToFinalPointAction(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def run_action(self):
        print 'goto point', self.x, self.y, self.theta
        parameters = [self.x, self.y, self.theta, 4]
        stm_driver('go_to_global_point', parameters)
        time.sleep(0.2)

    def check_action(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        if is_point_was_reached == 1:
            #raw_input("Press Enter...")
            return True

class SuperFastMoveToFinalPointAction(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def run_action(self):
        print 'goto point', self.x, self.y, self.theta
        parameters = [self.x, self.y, self.theta, 7]
        stm_driver('go_to_global_point', parameters)
        time.sleep(0.2)

    def check_action(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        #print 'check is_point_was_reached:', is_point_was_reached
        if is_point_was_reached == 1:
            #raw_input("Press Enter...")
            return True

class FastMoveToFinalPointAction(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def run_action(self):
        print 'goto point', self.x, self.y, self.theta
        parameters = [self.x, self.y, self.theta, 1]
        stm_driver('go_to_global_point', parameters)
        time.sleep(0.2)

    def check_action(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        #print 'check is_point_was_reached:', is_point_was_reached
        if is_point_was_reached == 1:
            #raw_input("Press Enter...")
            return True

class SlowMoveToIntermediaryPointAction(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def run_action(self):
        print 'goto point', self.x, self.y, self.theta
        parameters = [self.x, self.y, self.theta, 3]
        stm_driver('go_to_global_point', parameters)
        time.sleep(0.2)

    def check_action(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        #print 'check is_point_was_reached:', is_point_was_reached
        if is_point_was_reached == 1:
            #raw_input("Press Enter...")
            return True

class FastMoveToIntermediaryPointAction(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def run_action(self):
        print 'goto point', self.x, self.y, self.theta
        parameters = [self.x, self.y, self.theta, 0]
        stm_driver('go_to_global_point', parameters)
        time.sleep(0.2)

    def check_action(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        #print 'check is_point_was_reached:', is_point_was_reached
        if is_point_was_reached == 1:
            #raw_input("Press Enter...")
            return True

class SetCorrectCoordinatesAction():
    def run_action(self):
        global current_coordinatess, correction_performed
        correction_performed.value = 1
        self.x = current_coordinatess[0]
        self.y = current_coordinatess[1]
        self.theta = current_coordinatess[2]

        stm_driver('set_coordinates_with_movement', [self.x, self.y, self.theta])
        print 'Coordinates ', [self.x, self.y, self.theta], 'were seted'

    def check_action(self):
        return True

class SetCorrectCoordinatesWithoutMovementAction():
    def run_action(self):
        global current_coordinatess, correction_performed
        correction_performed.value = 1
        self.x = current_coordinatess[0]
        self.y = current_coordinatess[1]
        self.theta = current_coordinatess[2]

        stm_driver('set_coordinates_without_movement', [self.x, self.y, self.theta])
        print 'Coordinates ', [self.x, self.y, self.theta], 'were seted without movement'

    def check_action(self):
        return True

class WaitTimeTask(object):
    def __init__(self, time_delay):
        self.time_delay = time_delay
        self.start_time = None

    def run_task(self):
        self.start_time = time.time()

    def check_task(self):
        if time.time() - self.start_time > self.time_delay:
            return True

class ThrowCubesTask(object):
    all_actions_were_completed = False

    def __init__(self, initial_coordinates):
        self.future_actions= [
            SuperFastMoveToFinalPointAction(initial_coordinates[0], initial_coordinates[1]-0.01, initial_coordinates[2]),
            SuperFastMoveToFinalPointAction(initial_coordinates[0], initial_coordinates[1], initial_coordinates[2]),
            SuperFastMoveToFinalPointAction(initial_coordinates[0], initial_coordinates[1]-0.01, initial_coordinates[2]),
            SuperFastMoveToFinalPointAction(initial_coordinates[0], initial_coordinates[1], initial_coordinates[2]),
            FastMoveToFinalPointAction(initial_coordinates[0]-0.1, initial_coordinates[1], initial_coordinates[2]),
            SuperFastMoveToFinalPointAction(initial_coordinates[0]-0.1, initial_coordinates[1]-0.01, initial_coordinates[2]),
            SuperFastMoveToFinalPointAction(initial_coordinates[0]-0.1, initial_coordinates[1], initial_coordinates[2]),
            SuperFastMoveToFinalPointAction(initial_coordinates[0]-0.1, initial_coordinates[1]-0.01, initial_coordinates[2]),
            SuperFastMoveToFinalPointAction(initial_coordinates[0]-0.1, initial_coordinates[1], initial_coordinates[2]),
            FastMoveToFinalPointAction(initial_coordinates[0]-0.2, initial_coordinates[1], initial_coordinates[2]),
            SuperFastMoveToFinalPointAction(initial_coordinates[0]-0.2, initial_coordinates[1]-0.01, initial_coordinates[2]),
            SuperFastMoveToFinalPointAction(initial_coordinates[0]-0.2, initial_coordinates[1], initial_coordinates[2]),
            SuperFastMoveToFinalPointAction(initial_coordinates[0]-0.2, initial_coordinates[1]-0.01, initial_coordinates[2]),
            SuperFastMoveToFinalPointAction(initial_coordinates[0]-0.2, initial_coordinates[1], initial_coordinates[2]),]
        self.future_actions.reverse()
        self.current_action = self.future_actions.pop()

    def run_task(self):
        self.current_action.run_action()

    def check_task(self):
        self.check_actions_in_task()
        if self.all_actions_were_completed is True:
            return True

    def check_actions_in_task(self):
        if self.current_action.check_action() is True:
            if len(self.future_actions) is not 0:
                self.current_action = self.future_actions.pop()
                self.current_action.run_action()
            else:
                self.all_actions_were_completed = True

class FastMoveToIntermediaryPointTask(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def run_task(self):
        print 'goto point', self.x, self.y, self.theta
        parameters = [self.x, self.y, self.theta, 0]
        stm_driver('go_to_global_point', parameters)
        time.sleep(0.2)

    def check_task(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        #print 'check is_point_was_reached:', is_point_was_reached
        if is_point_was_reached == 1:
            #raw_input("Press Enter...")
            return True

class SlowMoveToIntermediaryPointTask(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def run_task(self):
        print 'goto point', self.x, self.y, self.theta
        parameters = [self.x, self.y, self.theta, 3]
        stm_driver('go_to_global_point', parameters)
        time.sleep(0.2)

    def check_task(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        #print 'check is_point_was_reached:', is_point_was_reached
        if is_point_was_reached == 1:
            #raw_input("Press Enter...")
            return True

class SuperFastMoveToIntermediaryPointTask(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def run_task(self):
        print 'goto point', self.x, self.y, self.theta
        parameters = [self.x, self.y, self.theta, 6]
        stm_driver('go_to_global_point', parameters)
        time.sleep(0.2)

    def check_task(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        #print 'check is_point_was_reached:', is_point_was_reached
        if is_point_was_reached == 1:
            #raw_input("Press Enter...")
            return True

class FastMoveToFinalPointTask(object):
    all_actions_were_completed = False

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.future_actions= [
            FastMoveToFinalPointAction(self.x, self.y, self.theta),
            SetCorrectCoordinatesAction()]
        self.future_actions.reverse()
        self.current_action = self.future_actions.pop()

    def run_task(self):
        self.current_action.run_action()

    def check_task(self):
        self.check_actions_in_task()
        if self.all_actions_were_completed is True:
            return True

    def check_actions_in_task(self):
        if self.current_action.check_action() is True:
            if len(self.future_actions) is not 0:
                self.current_action = self.future_actions.pop()
                self.current_action.run_action()
            else:
                self.all_actions_were_completed = True

class SuperFastMoveToFinalPointTask(object):
    all_actions_were_completed = False

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.future_actions= [
            SuperFastMoveToFinalPointAction(self.x, self.y, self.theta),
            SetCorrectCoordinatesAction()]
        self.future_actions.reverse()
        self.current_action = self.future_actions.pop()

    def run_task(self):
        self.current_action.run_action()

    def check_task(self):
        self.check_actions_in_task()
        if self.all_actions_were_completed is True:
            return True

    def check_actions_in_task(self):
        if self.current_action.check_action() is True:
            if len(self.future_actions) is not 0:
                self.current_action = self.future_actions.pop()
                self.current_action.run_action()
            else:
                self.all_actions_were_completed = True

class SlowMoveToFinalPointTask(object):
    all_actions_were_completed = False

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.future_actions= [
            SlowMoveToFinalPointAction(self.x, self.y, self.theta),
            SetCorrectCoordinatesAction()]
        self.future_actions.reverse()
        self.current_action = self.future_actions.pop()

    def run_task(self):
        self.current_action.run_action()
        #time.sleep(2)

    def check_task(self):
        self.check_actions_in_task()
        if self.all_actions_were_completed is True:
            return True

    def check_actions_in_task(self):
        if self.current_action.check_action() is True:
            if len(self.future_actions) is not 0:
                self.current_action = self.future_actions.pop()
                self.current_action.run_action()
            else:
                self.all_actions_were_completed = True

class FastMoveToFinalPointWithCorrectionTask(object):
    all_actions_were_completed = False

    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        self.future_actions= [
            FastMoveToFinalPointAction(self.x, self.y, self.theta),
            SetCorrectCoordinatesWithoutMovementAction()]
        self.future_actions.reverse()
        self.current_action = self.future_actions.pop()

    def run_task(self):
        self.current_action.run_action()
        #time.sleep(2)

    def check_task(self):
        self.check_actions_in_task()
        if self.all_actions_were_completed is True:
            return True

    def check_actions_in_task(self):
        if self.current_action.check_action() is True:
            if len(self.future_actions) is not 0:
                self.current_action = self.future_actions.pop()
                self.current_action.run_action()
            else:
                self.all_actions_were_completed = True

class TakeCubesTask(object):
    all_actions_were_completed = False
    angle_for_closed_manipulator = 275
    default_angle = 275
    task_was_completed_succesfully = None

    def __init__(self, layer, expected_number_of_cubes = 1):
        self.layer = layer
        self.expected_number_of_cubes = expected_number_of_cubes
        self.manipulator_angle = 270
        self.time = time.time()
        print 'Start time', self.time
        
        if self.layer is 3:
            self.manipulator_angle = 193
        elif self.layer is 2:
            self.manipulator_angle = 160
        elif self.layer is 1:
            self.manipulator_angle = 125

        self.future_actions  = [
            OpenCubesManipulatorAction(),
            SetCubesManipulatorAngleAction(self.manipulator_angle),
            CloseCubesManipulatorAction(),
            SetCubesManipulatorAngleAction(self.angle_for_closed_manipulator),
            OpenCubesManipulatorAction(),
            SetCubesManipulatorAngleAction(self.default_angle)]
        
        self.future_actions.reverse()
        self.current_action = self.future_actions.pop()

    def run_task(self):
        self.current_action.run_action()

    def check_task(self):
        self.check_actions_in_task()
        if self.all_actions_were_completed is True:
            print 'All cubes were taken: ', self.task_was_completed_succesfully
            return True

    def check_actions_in_task(self):
        if self.current_action.check_action() is True:
            if self.current_action.__class__.__name__ is 'CloseCubesManipulatorAction':
                if taken_cubes_number == self.expected_number_of_cubes:
                    self.task_was_completed_succesfully = True
                else:
                    self.task_was_completed_succesfully = False

            if len(self.future_actions) is not 0:
                self.current_action = self.future_actions.pop()
                self.current_action.run_action()
            else:
                self.all_actions_were_completed = True

class RetakeCubesTask(object):
    def __init__(self, layer, expected_number_of_cubes):
        self.layer = layer
        self.expected_number_of_cubes = expected_number_of_cubes

class ActivateRobotRegulatorsTask(object):
    all_actions_were_completed = False

    def __init__(self):
        self.future_actions= [
            SwitchOnKinematicsAction(),
            SwitchOnPidAction(),
            SwitchOnTrajectoryAction()]
        self.future_actions.reverse()
        self.current_action = self.future_actions.pop()

    def run_task(self):
        self.current_action.run_action()

    def check_task(self):
        self.check_actions_in_task()
        if self.all_actions_were_completed is True:
            return True

    def check_actions_in_task(self):
        if self.current_action.check_action() is True:
            if len(self.future_actions) is not 0:
                self.current_action = self.future_actions.pop()
                self.current_action.run_action()
            else:
                self.all_actions_were_completed = True

class SetInitialCoordinateTask(object):
    def __init__(self, initial_coordinates):
        self.initial_coordinates = initial_coordinates

    def run_task(self):
        self.reply = stm_driver('set_coordinates_without_movement', self.initial_coordinates)
        time.sleep(3)

    def check_task(self):
            #print 'coordinates', self.initial_coordinates, 'were setted as initial coordinates'
            return True

class ActivateCubesUnloadingTask(object):
    all_actions_were_completed = False

    def __init__(self):
        self.future_actions = [
            SwitchOnVibrationTableAction(),
            OpenCubesBorderAction()]
        self.future_actions.reverse()
        self.current_action = self.future_actions.pop()

    def run_task(self):
        self.current_action.run_action()

    def check_task(self):
        self.check_actions_in_task()
        if self.all_actions_were_completed is True:
            return True

    def check_actions_in_task(self):
        if self.current_action.check_action() is True:
            if len(self.future_actions) is not 0:
                self.current_action = self.future_actions.pop()
                self.current_action.run_action()
            else:
                self.all_actions_were_completed = True


class DisactivateCubesUnloadingTask(object):
    all_actions_were_completed = False

    def __init__(self):
        self.future_actions = [
            SwitchOffVibrationTableAction(),
            CloseCubesBorderAction()]
        self.future_actions.reverse()
        self.current_action = self.future_actions.pop()

    def run_task(self):
        self.current_action.run_action()

    def check_task(self):
        self.check_actions_in_task()
        if self.all_actions_were_completed is True:
            return True

    def check_actions_in_task(self):
        if self.current_action.check_action() is True:
            if len(self.future_actions) is not 0:
                self.current_action = self.future_actions.pop()
                self.current_action.run_action()
            else:
                self.all_actions_were_completed = True

class ResetCoordinatesTask(object):
    def __init__(self, real_coordinates):
        self.coordinates_to_set = real_coordinates

    def run_task(self):
        stm_driver('set_coordinates_without_movement', self.coordinates_to_set)
    
    def check_task(self):
        return True

class OpenManipulatorTask(object):
    def __init__(self, angle):
        self.angle = angle

    def run_task(self):
        stm_driver('set_cube_manipulator_angle', self.angle)
    
    def check_task(self):
        return True

class MainState(object):
    def __init__(self, states_list):
        states_list.reverse()
        self.current_state = states_list.pop()
        self.future_states = states_list
        self.interrupted_state = None
        self.robot_state = RobotState()
    
    def run_game(self):
        self.current_state.run_state()
        while True:
            current_state_status = self.current_state.check_state()
            if current_state_status is True:
                if self.interrupted_state == None:
                    if len(self.future_states) is not 0:
                        self.current_state = self.future_states.pop()
                        print 'Go to the next state', self.current_state 
                        self.current_state.run_state()
                else:
                    self.current_state = self.interrupted_state
                    self.interrupted_state = None
                    self.current_state.run_state()

            new_state = self.check_states()
            if new_state is not None:
                self.interrupted_state = self.current_state
                self.current_state = new_state
                self.current_state.run_state()

    def check_states(self):
        self.robot_state.send_data_to_socket()
        for method in dir(self):
            if not method.startswith('check_') or method == 'check_states':
                continue
            new_state = getattr(self, method)()
            if new_state is not None:
                return new_state

    def check_game_start_time(self):
        global check_start_time
        if check_start_time is True:
            robot_speeds = stm_driver('get_robot_speeds')
            if robot_speeds[0] > 0.05 or robot_speeds[1] > 0.05 or robot_speeds[2] > 0.05:
                global start_game_time
                start_game_time = time.time()
                check_start_time = False

    def check_game_time(self):
        global start_game_time
        current_time = time.time()
        if start_game_time is not None and (current_time - start_game_time >= 89 and check_start_time is False):
            stm_driver('switch_off_tajectory_regulator')
            stm_driver('set_movement_speed', [0.0, 0.0, 0.0])

    def check_time_and_send_data_to_socket(self):
        global last_update_for_robot
        if time.time() - last_update_for_robot > 1:
            try:
                a = time.time() - last_update_for_robot
                print 'time difference', a
                last_update_for_robot = time.time()
                self.robot_state.update_robot_state()
                self.robot_state.send_data_to_socket()
            except:
                pass

class InitializeRobotState(MainState):
    all_tasks_were_completed = False

    def __init__(self):
        self.future_tasks = [
            ActivateRobotRegulatorsTask(),
            SetInitialCoordinateTask(start_position)]
        self.future_tasks.reverse()
        self.current_task = self.future_tasks.pop()

    def run_state(self):
        self.current_task.run_task()

    def check_state(self):
        self.check_tasks_in_state()
        if self.all_tasks_were_completed is True:
            return True

    def check_tasks_in_state(self):
        if self.current_task.check_task() is True:
            if len(self.future_tasks) is not 0:
                self.current_task = self.future_tasks.pop()
                self.current_task.run_task()
            else:
                self.all_tasks_were_completed = True

class BrokeMiddleWallState(MainState):
    all_tasks_were_completed = False

    def __init__(self): 
        self.future_tasks = [
            SuperFastMoveToFinalPointTask(1.95, 0.4, -3.14),
            SuperFastMoveToFinalPointTask(1.88, 0.36, -2.6),
            SuperFastMoveToFinalPointTask(1.88, 0.23, -2.6),
            SlowMoveToIntermediaryPointTask(1.7, 0.23, -2.6),
            FastMoveToFinalPointWithCorrectionTask(1.32, 0.22, -3.34),
            FastMoveToIntermediaryPointTask(1.5, 0.47, -1.57)]
        self.future_tasks.reverse()
        self.current_task = self.future_tasks.pop()

    def run_state(self):
        print 'Move wall state'
        self.current_task.run_task()

    def check_state(self):
        self.check_tasks_in_state()
        if self.all_tasks_were_completed is True:
            return True

    def check_tasks_in_state(self):
        if self.current_task.check_task() is True:
            if len(self.future_tasks) is not 0:
                self.current_task = self.future_tasks.pop()
                self.current_task.run_task()
            else:
                print 'Wall was moved'
                self.all_tasks_were_completed = True

class CollectCubesStates(MainState):
    all_tasks_were_completed = False
    
    def __init__(self):
        print 'Run collect cubes state'

    def run_state(self):
        self.future_tasks = self.choose_collect_cubes_option()
        if self.future_tasks is None:
            print "All tasks for Collect Cubes State were completed"
            self.all_tasks_were_completed = True
        else:
            self.current_task = self.future_tasks.pop()
            self.current_task.run_task()

    def check_state(self):
        self.check_tasks_in_state()
        if self.all_tasks_were_completed is True:
            self.all_tasks_were_completed = False
            return True

    def check_tasks_in_state(self):
        if self.current_task.check_task() is True:
            if len(self.future_tasks) is not 0:
                self.current_task = self.future_tasks.pop()
                self.current_task.run_task()
            else:
                self.all_tasks_were_completed = True

            if self.current_task.check_task() is False:
                if self.current_task.__class__.__name___ is 'TakeCubesTask':
               	    self.current_task = RetakeCubesTask(self.current_task.layer, self.current_task.expected_number_of_cubes)
               	    self.current_task.run_task()

    def choose_collect_cubes_option(self):
        print 'trying to choose cubes for collectioning'
        choosen_option_priority = 100
        option_number = 0
        choosen_option_number = None
        global collect_cubes_options
        for option in collect_cubes_options:
            if option['priority'] < choosen_option_priority and option['priority'] is not 0:
                choosen_option_priority = option['priority']
                choosen_option_number = option_number
            option_number = option_number + 1
        if choosen_option_number is not None:
            collect_cubes_options[choosen_option_number]['priority'] = 0
            option = collect_cubes_options[choosen_option_number]['tasks_list']
            option.reverse()
            return option
        else:
            return None

class UnloadCubesState(MainState):
    all_tasks_were_completed = False

    def __init__(self):
        global unloading_cubes_position
        self.coordinates_for_unloading_cubes =  [
            [1.95, 0.617, -1.57],
            [0.9655, 1.015, -3.14],
            [0.9655, 1.115, -3.14]]
        self.future_tasks = [
            FastMoveToFinalPointTask(self.coordinates_for_unloading_cubes[unloading_cubes_position][0], self.coordinates_for_unloading_cubes[unloading_cubes_position][1], self.coordinates_for_unloading_cubes[unloading_cubes_position][2]),
            ActivateCubesUnloadingTask(),
            ThrowCubesTask(self.coordinates_for_unloading_cubes[unloading_cubes_position]),
            DisactivateCubesUnloadingTask()]
        if unloading_cubes_position < len(self.coordinates_for_unloading_cubes):
            unloading_cubes_position = unloading_cubes_position + 1
        self.future_tasks.reverse()
        self.current_task = self.future_tasks.pop()

    def run_state(self):
        self.current_task.run_task()

    def check_state(self):
        self.check_tasks_in_state()
        if self.all_tasks_were_completed is True:
            return True

    def check_tasks_in_state(self):
        if self.current_task.check_task() is True:
            if len(self.future_tasks) is not 0:
                self.current_task = self.future_tasks.pop()
                self.current_task.run_task()
            else:
                self.all_tasks_were_completed = True

class CloseDoorsState(object):
    all_tasks_were_completed = False

    def __init__(self): 
        self.future_tasks = [
            SuperFastMoveToIntermediaryPointTask(2.4, 0.47, -1.57),
            FastMoveToFinalPointTask(2.4, 0.1, -1.57),
            FastMoveToIntermediaryPointTask(2.4, 0.23, -1.57),
            FastMoveToIntermediaryPointTask(2.6, 0.23, -1.57),
            FastMoveToFinalPointTask(2.6, 0.1, -1.57)]
        self.future_tasks.reverse()
        self.current_task = self.future_tasks.pop()

    def run_state(self):
        print 'Starting close door state'
        self.current_task.run_task()

    def check_state(self):
        self.check_tasks_in_state()
        if self.all_tasks_were_completed is True:
            return True

    def check_tasks_in_state(self):
        if self.current_task.check_task() is True:
            if len(self.future_tasks) is not 0:
                self.current_task = self.future_tasks.pop()
                self.current_task.run_task()
            else:
                print 'All close doors tasks were completed'
                self.all_tasks_were_completed = True

class TestState(object):
    all_tasks_were_completed = False

    def __init__(self): 
        self.future_tasks = [FastMoveToFinalPointTask(0.4, 0.45, -1.57)]
            #MoveToIntermediaryPointTask(0.1525, 0.72, 0.0),
            #MoveToFinalPointTask(0.3, 0.72, 0)] 
            #SetInitialCoordinateTask([0.3, 0.72, 0])]
        self.future_tasks.reverse()
        self.current_task = self.future_tasks.pop()

    def run_state(self):
        self.current_task.run_task()

    def check_state(self):
        self.check_tasks_in_state()
        if self.all_tasks_were_completed is True:
            return True

    def check_tasks_in_state(self):
        if self.current_task.check_task() is True:
            if len(self.future_tasks) is not 0:
                self.current_task = self.future_tasks.pop()
                self.current_task.run_task()
            else:
                self.all_tasks_were_completed = True

class DragCubesState(object):
    all_tasks_were_completed = False

    def __init__(self): 
        self.future_tasks = [
            SuperFastMoveToFinalPointTask(2.8, 0.88, -3.14),
            FastMoveToFinalPointTask(2.4, 0.88, -3.14),
            FastMoveToFinalPointTask(2.0, 0.88, -3.14)]
        self.future_tasks.reverse()
        self.current_task = self.future_tasks.pop()

    def run_state(self):
        self.current_task.run_task()

    def check_state(self):
        self.check_tasks_in_state()
        if self.all_tasks_were_completed is True:
            return True

    def check_tasks_in_state(self):
        if self.current_task.check_task() is True:
            if len(self.future_tasks) is not 0:
                self.current_task = self.future_tasks.pop()
                self.current_task.run_task()
            else:
                self.all_tasks_were_completed = True

class RobotState(object):
    def __init__(self):
        global current_coordinatess_from_robot
        self.current_coordinatess_from_robot = current_coordinatess_from_robot
        global current_coordinatess
        self.current_coordinatess = current_coordinatess
        self.collision_avoidance = 1
        self.pc = self.connect_pc(HOST, PORT)

    def connect_pc(self, HOST, PORT):
        try:    
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((HOST, PORT))
            return sock
        except Exception as err:
            print 'Error in connecting to pc server: ', err
            sock.close()
    
    def update_robot_state(self):
        print 'we are in update_robot_state method'
        global current_coordinatess_from_robot
        self.current_coordinatess_from_robot = current_coordinatess_from_robot
        global current_coordinatess
        self.current_coordinatess = current_coordinatess
        self.collision_avoidance = 1

    def send_data_to_socket(self):
        data_to_send = self.current_coordinatess[0], self.current_coordinatess[1], self.current_coordinatess[2], self.current_coordinatess_from_robot[0], self.current_coordinatess_from_robot[1], self.current_coordinatess_from_robot[2], self.collision_avoidance
        string_to_send = str(data_to_send)
        print 'String to send:', string_to_send
        #self.pc.sendall(string_to_send +'\n')
        '''
        try:
            data_to_send = self.current_coordinatess[0], self.current_coordinatess[1], self.current_coordinatess[2], self.current_coordinatess_from_robot[0], self.current_coordinatess_from_robot[1], self.current_coordinatess_from_robot[2], self.collision_avoidance
            string_to_send = str(data_to_send)
            print 'String to send:', string_to_send
            self.pc.sendall(string_to_send +'\n')
        except:
            print 'exception occured'
            pass'''

def stm_driver(command, parameters = ''):
    command = {'request_source': 'fsm', 'command': command, 'parameters': parameters}
    input_command_queue.put(command)
    return reply_to_fsm_queue.get()

def get_angles_diff(a1, a2):
    target = (a1 + 180) % 360 - 180
    cur = (a2 + 180) % 360 - 180
    return (target - cur + 180) % 360 - 180

collect_cubes_options = [{ 
        'priority': 3, 
        'tasks_list': [
            FastMoveToFinalPointTask(0.60, 0.13, -1.57),
            SlowMoveToFinalPointTask(0.67, 0.13, -1.57),
            FastMoveToFinalPointTask(0.67, 0.38, -1.57),
            FastMoveToFinalPointTask(0.95, 0.38, -1.57),
            #OpenManipulatorTask(193),
            SlowMoveToFinalPointTask(0.95, 0.30, -1.57),
            SlowMoveToFinalPointTask(0.925, 0.31, -1.57),
            #TakeCubesTask(3),
            #TakeCubesTask(2),
            SlowMoveToFinalPointTask(0.925, 0.325, -1.57),
            #OpenManipulatorTask(130),
            SlowMoveToFinalPointTask(0.925, 0.31, -1.57),
            #TakeCubesTask(1),
            FastMoveToIntermediaryPointTask(0.925, 0.45, -1.57)]},
        { 
        'priority': 4, 
        'tasks_list': [
            FastMoveToFinalPointTask(1.48, 0.43, -1.57),
            SlowMoveToFinalPointTask(1.5, 0.44, -1.57),
            #TakeCubesTask(1),
            SlowMoveToFinalPointTask(1.48, 0.35, -1.57),
            #TakeCubesTask(3)
            ]},
        { 
        'priority': 1, 
        'tasks_list': [
            SuperFastMoveToIntermediaryPointTask(0.45, 0.43, 0),
            SuperFastMoveToIntermediaryPointTask(2.0, 0.43, -1.57),
            SlowMoveToFinalPointTask(2.05, 0.43, -1.57),
            #OpenManipulatorTask(193),
            WaitTimeTask(1),
            SlowMoveToFinalPointTask(2.125, 0.43, -1.57),
            SlowMoveToFinalPointTask(2.125, 0.29, -1.57),
            SlowMoveToFinalPointTask(2.12, 0.31, -1.57),
            #TakeCubesTask(3),
            #TakeCubesTask(2),
            SlowMoveToFinalPointTask(2.12, 0.325, -1.57),
            #OpenManipulatorTask(130),
            SlowMoveToFinalPointTask(2.12, 0.31, -1.57),
            SuperFastMoveToIntermediaryPointTask(2.125, 0.43, -1.57)]}]

stm = multiprocessing.Process(target=stmDriver.stmMainLoop, args=(input_command_queue, reply_to_fsm_queue, reply_to_localization_queue))
localisation = multiprocessing.Process(target=localisation.main, args=(input_command_queue, reply_to_localization_queue, current_coordinatess, correction_performed, start_position, current_coordinatess_from_robot))
stm.start()
#time.sleep(2)
localisation.start()
states_list = [
    InitializeRobotState(),
    #BrokeMiddleWallState(),
    CollectCubesStates(),
    UnloadCubesState(),
    CloseDoorsState(),
    CollectCubesStates(),
    DragCubesState()]
'''states_list = [InitializeRobotState(), TestState()]#, CloseDoorsState(), CollectCubesStates(), CollectCubesStates()]'''
MainState(states_list).run_game()