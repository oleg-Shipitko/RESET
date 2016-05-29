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
start_game_time = time.time()
current_coordinatess_from_robot = multiprocessing.Array('d', [0.0, 0.0, 0.0])
current_coordinatess = multiprocessing.Array('d', [0.0, 0.0, 0.0])
correction_performed = multiprocessing.Value('i', 0)

cubes_in_trunk = 0
trunk_capacity = 12
unloading_cubes_position = 0
taken_cubes_number = None

#start_position = [0.1525, 0.72, 0.0]
start_position = [2.847, 0.72, 0.0]

'''class PrintCoordinatesAction(object):
    def run_action(self):
        print 'print coordinates action'

    def check_action(self):
        global current_coordinatess, correction_performed
        correction_performed.value = 1
        self.x = current_coordinatess[0]
        self.y = current_coordinatess[1]
        self.theta = current_coordinatess[2]
        stm_driver('set_coordinates_with_movement', [self.x, self.y, self.theta])

        print 'Robot coordinates: ', current_coordinatess_from_robot[0], current_coordinatess_from_robot[1], current_coordinatess_from_robot[2]
        print 'Lidar coordinates: ', current_coordinatess[0], current_coordinatess[1], current_coordinatess[2]
        time.sleep(10)'''

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
        stm_driver('open_cube_collector')
        self.start_time = time.time()

    def check_action(self):
        if time.time() - self.start_time > 0.5:
            print time.time() - self.start_time
            return True

class SetCubesManipulatorAngleAction(object):
    def __init__(self, manipulator_angle, timeout):
        self.manipulator_angle = manipulator_angle
        self.timeout = timeout

    def run_action(self):
        stm_driver('set_cube_manipulator_angle', self.manipulator_angle)
        self.start_time = time.time()

    def check_action(self):
        if time.time() - self.start_time > self.timeout:
            return True

class SwitchOnVibrationTableAction():
    def __init__(self, time):
        self.time = time

    def run_action(self):
        stm_driver('switch_on_vibration_table', self.time)

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
        stm_driver('close_cubes_border')

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
        if time.time() - self.start_time > 0.8:
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
        print 'Coordinates ', [self.x, self.y, self.theta], 'were set'

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
        print 'Coordinates ', [self.x, self.y, self.theta], 'were set without movement'

    def check_action(self):
        return True

class FastMoveWithCorrectionTask():
    def __init__(self, next_x, next_y, next_thata):
        self.next_x = next_x
        self.next_y = next_y
        self.next_theta = next_thata

    def run_task(self):
        global current_coordinatess
        current_x = current_coordinatess[0]
        current_y = current_coordinatess[1]
        current_theta = current_coordinatess[2]
        stm_driver('move_with_correction', [current_x, current_y, current_theta, self.next_x, self.next_y, self.next_theta, 1])
        time.sleep(0.2)

    def check_task(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        if is_point_was_reached == 1:
            return True

class FastMoveWithCorrectionIntermediatePoinTask():
    def __init__(self, next_x, next_y, next_thata):
        self.next_x = next_x
        self.next_y = next_y
        self.next_theta = next_thata

    def run_task(self):
        global current_coordinatess
        current_x = current_coordinatess[0]
        current_y = current_coordinatess[1]
        current_theta = current_coordinatess[2]
        stm_driver('move_with_correction', [current_x, current_y, current_theta, self.next_x, self.next_y, self.next_theta, 0])
        time.sleep(0.2)

    def check_task(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        if is_point_was_reached == 1:
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

class SwitchOnCollisionAvoidanceTask(object):
    def run_task(self):
        stm_driver('switch_on_collision_avoidance')

    def check_task(self):
        print 'Collision Avoidance is On'
        return True

class SwitchOffCollisionAvoidanceTask(object):
    def run_task(self):
        stm_driver('switch_off_collision_avoidance')

    def check_task(self):
        print 'Collision Avoidance is Off'
        return True

class ThrowCubesTask(object):
    all_actions_were_completed = False

    def __init__(self, initial_coordinates):
        self.future_actions= [
            WaitTimeAction(1.0),
            SlowMoveToFinalPointAction(initial_coordinates[0] + 0.08, initial_coordinates[1], initial_coordinates[2]),
            WaitTimeAction(1.0),
            SlowMoveToFinalPointAction(initial_coordinates[0] + 0.16, initial_coordinates[1], initial_coordinates[2]),
            WaitTimeAction(1.0),
            SlowMoveToFinalPointAction(initial_coordinates[0] + 0.24, initial_coordinates[1], initial_coordinates[2]),
            WaitTimeAction(1.0),
            SlowMoveToFinalPointAction(initial_coordinates[0] + 0.32, initial_coordinates[1], initial_coordinates[2]),
            WaitTimeAction(1.0),
            SlowMoveToFinalPointAction(initial_coordinates[0] + 0.40, initial_coordinates[1], initial_coordinates[2])]
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

class TakeCubesTask(object):
    all_actions_were_completed = False
    angle_for_closed_manipulator = 283
    default_angle = 283
    task_was_completed_succesfully = None

    def __init__(self, layer, expected_number_of_cubes = 1):
        self.layer = layer
        self.expected_number_of_cubes = expected_number_of_cubes
        self.manipulator_angle = 283
        self.time = time.time()
        self.timeout = 0
        print 'Start time', self.time
        
        if self.layer is 3:
            self.manipulator_angle = 196
            self.timeout = 0.8
        elif self.layer is 2:
            self.manipulator_angle = 160
            self.timeout = 1.3
        elif self.layer is 1:
            self.manipulator_angle = 143
            self.timeout = 1.8

        self.future_actions  = [
            OpenCubesManipulatorAction(),
            SetCubesManipulatorAngleAction(self.manipulator_angle, self.timeout),
            CloseCubesManipulatorAction(),
            SetCubesManipulatorAngleAction(self.angle_for_closed_manipulator, self.timeout),
            OpenCubesManipulatorAction(),
            SwitchOnVibrationTableAction(5),
            SetCubesManipulatorAngleAction(self.default_angle, 0)]
        
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
            return True

class ActivateCubesUnloadingTask(object):
    all_actions_were_completed = False

    def __init__(self, time):
        self.future_actions = [
            SwitchOnVibrationTableAction(time),
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
        #self.robot_state = RobotState()
    
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
            if robot_speeds[0] > 0.1 or robot_speeds[1] > 0.1 or robot_speeds[2] > 0.1:
                global start_game_time
                start_game_time = time.time()
                check_start_time = False

    def check_game_time(self):
        global start_game_time
        current_time = time.time()
        #print 'time difference: ', current_time - start_game_time
        if start_game_time is not None and (current_time - start_game_time >= 89 and check_start_time is False):
            reply = stm_driver('switch_off_robot')
            print 'reply:', reply
            print 'Robot was stopped'

    '''def check_time_and_send_data_to_socket(self):
        global last_update_for_robot
        if time.time() - last_update_for_robot > 1:
            last_update_for_robot = time.time()
            self.robot_state.update_robot_state()
            self.robot_state.send_data_to_socket()
            try:
                last_update_for_robot = time.time()
                self.robot_state.update_robot_state()
                self.robot_state.send_data_to_socket()
            except:
                pass'''

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
            SuperFastMoveToIntermediaryPointTask(1.95, 0.4, 0),
            FastMoveToFinalPointTask(1.88, 0.36, 0.54),
            SwitchOffCollisionAvoidanceTask(),
            FastMoveWithCorrectionTask(1.88, 0.1, 0.54),
            FastMoveToIntermediaryPointTask(1.7, 0.09, 0.54),
            FastMoveToFinalPointWithCorrectionTask(1.23, 0.09, 0),
            FastMoveWithCorrectionIntermediatePoinTask(1.8, 0.40, -1.57),
            SwitchOnCollisionAvoidanceTask()]
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

    def __init__(self, time):
        global unloading_cubes_position
        self.coordinates_for_unloading_cubes =  [
            [1.82, 1.00, 0]]
        self.future_tasks = [
            FastMoveToFinalPointTask(self.coordinates_for_unloading_cubes[unloading_cubes_position][0], self.coordinates_for_unloading_cubes[unloading_cubes_position][1], self.coordinates_for_unloading_cubes[unloading_cubes_position][2]),
            ActivateCubesUnloadingTask(time),
            ThrowCubesTask(self.coordinates_for_unloading_cubes[unloading_cubes_position]),
            DisactivateCubesUnloadingTask(),
            FastMoveToFinalPointTask(2.1, 1.00, 0)]
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
            SwitchOffCollisionAvoidanceTask(),
            FastMoveWithCorrectionIntermediatePoinTask(2.72, 0.40, -1.57),
            FastMoveWithCorrectionTask(2.72, 0.40, -1.57),
            FastMoveToFinalPointWithCorrectionTask(2.72, 0.09, -1.57),
            FastMoveWithCorrectionTask(2.72, 0.25, -1.57),
            FastMoveWithCorrectionTask(2.45, 0.25, -1.57),
            FastMoveToFinalPointWithCorrectionTask(2.45, 0.06, -1.57)]
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

class DragCubesState(object):
    all_tasks_were_completed = False

    def __init__(self): 
        self.future_tasks = [
            FastMoveToFinalPointTask(2.49, 0.43, -1.57),
            FastMoveToFinalPointTask(2.49, 0.95, -1.57),
            SwitchOffCollisionAvoidanceTask(),
            FastMoveToFinalPointWithCorrectionTask(1.60, 0.95, -1.57)]
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

class TestState(object):
    all_tasks_were_completed = False

    def __init__(self):
        self.future_tasks = [SlowMoveToFinalPointTask(0.3525, 0.72, 0.0)] 
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


def stm_driver(command, parameters = ''):
    command = {'request_source': 'fsm', 'command': command, 'parameters': parameters}
    input_command_queue.put(command)
    return reply_to_fsm_queue.get()

def get_angles_diff(a1, a2):
    target = (a1 + 180) % 360 - 180
    cur = (a2 + 180) % 360 - 180
    return (target - cur + 180) % 360 - 180

class RobotState(object):
    def __init__(self):
        global current_coordinatess_from_robot
        self.current_coordinatess_from_robot = current_coordinatess_from_robot
        global current_coordinatess
        self.current_coordinatess = current_coordinatess
        self.collision_avoidance = 1
        self.pc = self.connect_pc(HOST, PORT)
        self.ik_data = [0.0, 0.0, 0.0, 0.0]
        self.us_data = [0.0, 0.0, 0.0, 0.0, 0.0]

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
        self.ik_data = stm_driver('get_ik_data')
        self.us_data = stm_driver('get_us_data')


    def send_data_to_socket(self):
        print 'we are in send_data_to_socket method'
        #data_to_send = self.current_coordinatess[0], self.current_coordinatess[1], self.current_coordinatess[2], self.current_coordinatess_from_robot[0], self.current_coordinatess_from_robot[1], self.current_coordinatess_from_robot[2], self.collision_avoidance, self.ik_data [0], self.ik_data [1], self.ik_data [2], self.ik_data[3], self.us_data[0], self.us_data[1], self.us_data[2], self.us_data[3], self.us_data[4]
        #string_to_send = str(data_to_send)
        #print 'String to send:', string_to_send
        #self.pc.sendall(string_to_send +'\n')
        '''try:
            data_to_send = self.current_coordinatess[0], self.current_coordinatess[1], self.current_coordinatess[2], self.current_coordinatess_from_robot[0], self.current_coordinatess_from_robot[1], self.current_coordinatess_from_robot[2], self.collision_avoidance, self.ik_data [0], self.ik_data [1], self.ik_data [2], ik_data[3], self.us_data[0], self.us_data[1], self.us_data[2], self.us_data[3], self.us_data[4]
            string_to_send = str(data_to_send)
            print 'String to send:', string_to_send
            self.pc.sendall(string_to_send +'\n')
        except: 
            print 'exception occured'
            pass'''

collect_cubes_options = [{ 
        'priority': 3, 
        'tasks_list': [
            SlowMoveToFinalPointTask(2.48, 0.1, 1.57),
            SlowMoveToFinalPointTask(2.35, 0.1, 1.57),
            FastMoveToIntermediaryPointTask(2.35, 0.45, 1.57),
            FastMoveToFinalPointTask(1.96, 0.45, 1.57),
            FastMoveToFinalPointTask(1.96, 0.45, -1.57),
            OpenManipulatorTask(213),
            SlowMoveToIntermediaryPointTask(1.96, 0.315, -1.57),
            SuperFastMoveToFinalPointTask(2.085, 0.315, -1.57),
            TakeCubesTask(2),
            SlowMoveToIntermediaryPointTask(2.085, 0.33, -1.57),
            OpenManipulatorTask(150),
            SlowMoveToFinalPointTask(2.085, 0.32, -1.57),
            TakeCubesTask(1),
            FastMoveToIntermediaryPointTask(2.085, 0.45, -1.57),
            SwitchOnCollisionAvoidanceTask()]},
        { 
        'priority': 2, 
        'tasks_list': [
            SlowMoveToFinalPointTask(1.37, 0.42, -1.57),
            TakeCubesTask(1),
            SlowMoveToFinalPointTask(1.37, 0.37, -1.57),
            TakeCubesTask(2),
            FastMoveToIntermediaryPointTask(1.37, 0.46, -1.57)]},
        { 
        'priority': 1, 
        'tasks_list': 
            [
            SuperFastMoveToIntermediaryPointTask(2.48, 0.43, -1.57),
            FastMoveToFinalPointTask(2.14, 0.43, -1.57),
            SlowMoveToFinalPointTask(2.14, 0.43, -1.57),
            OpenManipulatorTask(203),
            SlowMoveToIntermediaryPointTask(2.14, 0.43, -1.57),
            SlowMoveToIntermediaryPointTask(2.13, 0.24, -1.57),
            FastMoveToFinalPointTask(2.12, 0.30, -1.57),
            TakeCubesTask(3),
            FastMoveToFinalPointTask(2.12, 0.335, -1.57),
            OpenManipulatorTask(160),
            WaitTimeTask(2),
            FastMoveToFinalPointTask(2.12, 0.31, -1.57),
            TakeCubesTask(2),
            FastMoveToFinalPointTask(2.12, 0.335, -1.57),
            OpenManipulatorTask(143),
            WaitTimeTask(2),
            FastMoveToFinalPointTask(2.12, 0.32, -1.57),
            TakeCubesTask(1),
            SuperFastMoveToIntermediaryPointTask(2.12, 0.43, -1.57)]}]

stm = multiprocessing.Process(target=stmDriver.stmMainLoop, args=(input_command_queue,reply_to_fsm_queue, reply_to_localization_queue))
localisation = multiprocessing.Process(target=localisation.main, args=(input_command_queue,reply_to_localization_queue, current_coordinatess,correction_performed, start_position, current_coordinatess_from_robot))
stm.start()
#time.sleep(2)
localisation.start()
states_list = [
    InitializeRobotState(),
    #BrokeMiddleWallState(),
    CloseDoorsState(),
    CollectCubesStates(),
    DragCubesState(),
    UnloadCubesState(15)]
'''states_list = [InitializeRobotState(), TestState()]'''
MainState(states_list).run_game()   