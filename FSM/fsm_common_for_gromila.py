import stmDriver
import time
import multiprocessing
import math
import localisation
import socket
import ctypes

selected_side = multiprocessing.Value(ctypes.c_char_p, 'green') #right
#selected_side = multiprocessing.Value(ctypes.c_char_p, 'purple') #left

# PC server address and  port
HOST = '192.168.1.146'
PORT = 9090

input_command_queue = multiprocessing.Queue()
reply_to_fsm_queue = multiprocessing.Queue()
reply_to_localization_queue = multiprocessing.Queue()

request_source = 'fsm'
start_time = time.time()
last_update_for_robot = time.time()
check_start_time = multiprocessing.Value('b', True)
start_game_time = time.time()
current_coordinatess_from_robot = multiprocessing.Array('d', [0.0, 0.0, 0.0])
current_coordinatess = multiprocessing.Array('d', [0.0, 0.0, 0.0])
correction_performed = multiprocessing.Value('i', 0)

cubes_in_trunk = 0
trunk_capacity = 12
unloading_cubes_position = 0
taken_cubes_number = None

if (selected_side.value == 'green'):
    start_position = [0.14, 0.76, 0.0]
else:
    start_position = [2.86, 0.76, 0.0]

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
        if time.time() - self.start_time > 0.3:
            print time.time() - self.start_time
            return True

class BigOpenCubesManipulatorAction(object):
    def run_action(self):
        stm_driver('open_manipulator_big_angle')

    def check_action(self):
            time.sleep(0.2)
            return True

class WideOpenManipulatorAction(object):
    def run_action(self):
        stm_driver('open_manipulator_big_angle') 
        time.sleep(0.2)

    def check_action(self):
            return True

class CloseCubesManipulatorAngleAction(object):
    def run_action(self):
        stm_driver('set_cube_manipulator_angle', 285)
        self.start_time = time.time()

    def check_action(self):
        time.sleep(0.7)
        return True

class MoveToPointAction(object):
    def __init__(self, x, y, theta, movement_type, 
                with_stop = True, correction_at_end_point = True, correction_while_mooving = False, correction_without_mooving = False):
        self.correction_while_moving = correction_while_mooving
        self.correction_without_mooving = correction_without_mooving
        self.movement_type =  movement_type
        self.correction_at_end_point = correction_at_end_point
        if with_stop:
            self.movement_type = movement_type + 1
        self.x = x
        self.y = y
        self.theta = theta

    def run_action(self):
        if self.correction_while_moving:
            global current_coordinatess
            current_x = current_coordinatess[0]
            current_y = current_coordinatess[1]
            current_theta = current_coordinatess[2]
            stm_driver('move_with_correction', [current_x, current_y, current_theta, self.x, self.y, self.theta, self.movement_type])
        else:
            parameters = [self.x, self.y, self.theta, self.movement_type ]
            stm_driver('go_to_global_point', parameters)
        time.sleep(0.2)

    def check_action(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        if is_point_was_reached == 1:
            global current_coordinatess, correction_performed
            if self.correction_at_end_point:
                correction_performed.value = 1
                self.x = current_coordinatess[0]
                self.y = current_coordinatess[1]
                self.theta = current_coordinatess[2]
                stm_driver('set_coordinates_with_movement', [self.x, self.y, self.theta])
            if self.correction_without_mooving:
                correction_performed.value = 1
                self.x = current_coordinatess[0]
                self.y = current_coordinatess[1]
                self.theta = current_coordinatess[2]
                stm_driver('set_coordinates_without_movement', [self.x, self.y, self.theta])
                print 'Coordinates ', [self.x, self.y, self.theta], 'were set without movement'

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
            print 'Vibration table was activated for ', self.time, ' sec'
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
        print 'collision avoidance switched on'
        #stm_driver('switch_on_collision_avoidance')

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
        if (selected_side.value == 'green'): 
            self.future_actions= [
                WaitTimeAction(0.5),
                MoveToPointAction(initial_coordinates[0] - 0.16, initial_coordinates[1], initial_coordinates[2], movement_type['slow']),
                WaitTimeAction(0.5),
                MoveToPointAction(initial_coordinates[0] - 0.24, initial_coordinates[1], initial_coordinates[2], movement_type['slow']),
                WaitTimeAction(0.5)]
        else:
            self.future_actions= [
                WaitTimeAction(0.5),
                MoveToPointAction(initial_coordinates[0] + 0.16, initial_coordinates[1], initial_coordinates[2], movement_type['slow']),
                WaitTimeAction(0.5),
                MoveToPointAction(initial_coordinates[0] + 0.24, initial_coordinates[1], initial_coordinates[2], movement_type['slow']),
                WaitTimeAction(0.5)]
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

class TakeCubesTask(object):
    all_actions_were_completed = False
    angle_for_closed_manipulator = 295
    default_angle = 295
    task_was_completed_succesfully = None

    def __init__(self, layer):
        self.layer = layer
        self.manipulator_angle = 295
        self.time = time.time()
        self.timeout = 0
        self.should_retake_cubes = False

        if self.layer is 3:
            self.manipulator_angle = 190
            self.timeout = 0.8
        elif self.layer is 2:
            self.manipulator_angle = 160
            self.timeout = 1.3
        elif self.layer is 1:
            self.manipulator_angle = 133
            self.timeout = 1.7

        self.future_actions  = [
            OpenCubesManipulatorAction(),
            SetCubesManipulatorAngleAction(self.manipulator_angle, self.timeout),
            BigOpenCubesManipulatorAction(),
            CloseCubesManipulatorAction(),
            SetCubesManipulatorAngleAction(self.angle_for_closed_manipulator, self.timeout),
            OpenCubesManipulatorAction(),
            SwitchOnVibrationTableAction(3),
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
            if len(self.future_actions) is not 0:
                self.current_action = self.future_actions.pop()
                self.current_action.run_action()
            else:
                self.all_actions_were_completed = True

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

class OpenCollectorTask(object):
    def __init__(self, angle):
        self.angle = angle

    def run_task(self):
        stm_driver('set_cube_manipulator_angle', self.angle)
        time.sleep(1.3)
    
    def check_task(self):
        return True


class WideOpenCollectorTask(object):
    all_actions_were_completed = False

    def __init__(self, angle):
        self.future_actions = [
            SetCubesManipulatorAngleAction(185, 0.5),
            WideOpenManipulatorAction(),
            SetCubesManipulatorAngleAction(angle, 0.8)]
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

class OpenManipulatorTask(object):
    def run_task(self):
        stm_driver('open_cube_collector')
    
    def check_task(self):
        return True

class CloseManipulatorTask(object):
    def run_task(self):
        stm_driver('close_cube_collector')
    
    def check_task(self):
        return True

class MoveToPointTask(object):
    def __init__(self, x, y, theta, movement_type, 
                with_stop = True, correction_at_end_point = True, correction_while_mooving = False, correction_without_mooving = False):
        self.correction_while_moving = correction_while_mooving
        self.correction_without_mooving = correction_without_mooving
        self.movement_type =  movement_type
        self.correction_at_end_point = correction_at_end_point
        if with_stop:
            self.movement_type = movement_type + 1
        self.x = x
        self.y = y
        self.theta = theta

    def run_task(self):
        print 'go to point', self.x, self.y, self.theta, self.movement_type
        if self.correction_while_moving:
            global current_coordinatess
            current_x = current_coordinatess[0]
            current_y = current_coordinatess[1]
            current_theta = current_coordinatess[2]
            stm_driver('move_with_correction', [current_x, current_y, current_theta, self.x, self.y, self.theta, self.movement_type])
        else:
            parameters = [self.x, self.y, self.theta, self.movement_type ]
            stm_driver('go_to_global_point', parameters)
        time.sleep(0.2)

    def check_task(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        if is_point_was_reached == 1:
            global current_coordinatess, correction_performed
            if self.correction_at_end_point:
                correction_performed.value = 1
                self.x = current_coordinatess[0]
                self.y = current_coordinatess[1]
                self.theta = current_coordinatess[2]
                stm_driver('set_coordinates_with_movement', [self.x, self.y, self.theta])
                print 'Coordinates ', [self.x, self.y, self.theta], 'were set'
            if self.correction_without_mooving:
                correction_performed.value = 1
                self.x = current_coordinatess[0]
                self.y = current_coordinatess[1]
                self.theta = current_coordinatess[2]
                stm_driver('set_coordinates_without_movement', [self.x, self.y, self.theta])
                print 'Coordinates ', [self.x, self.y, self.theta], 'were set without movement'

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
                #if self.interrupted_state == None:
                if len(self.future_states) is not 0:
                    self.current_state = self.future_states.pop()
                    print 'Go to the next state', self.current_state 
                    self.current_state.run_state()
                '''else:
                    self.current_state = self.interrupted_state
                    self.interrupted_state = None
                    self.current_state.run_state()'''

            new_state = self.check_states()
            if new_state is not None:
                print 'we are going to', new_state
                #self.interrupted_state = self.current_state
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
        if check_start_time.value == True:
            robot_speeds = stm_driver('get_robot_speeds')
            if abs(robot_speeds[0]) > 0.1 or abs(robot_speeds[1]) > 0.1 or abs(robot_speeds[2] > 0.1):
                global start_game_time
                start_game_time = time.time()
                check_start_time.value = False
                print 'flag was changed to' , check_start_time.value

    def check_game_time(self):
        global start_game_time
        current_time = time.time()
        if start_game_time is not None and (current_time - start_game_time >= 89 and check_start_time is False):
            reply = stm_driver('switch_off_robot')
            print 'reply:', reply
            print 'Robot was stopped'

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
            print 'need to Retake cubes....the next task is RetakeCubesTask' 
            self.current_task = RetakeCubesTask(self.current_task.layer)
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
        if (selected_side.value == 'green'):
            self.coordinates_for_unloading_cubes =  [[1.15, 1.05, -3.14]]
            self.future_tasks = [
                MoveToPointTask(self.coordinates_for_unloading_cubes[unloading_cubes_position][0], self.coordinates_for_unloading_cubes[unloading_cubes_position][1], self.coordinates_for_unloading_cubes[unloading_cubes_position][2], movement_type['fast']),
                ActivateCubesUnloadingTask(time),
                ThrowCubesTask(self.coordinates_for_unloading_cubes[unloading_cubes_position]),
                DisactivateCubesUnloadingTask(),
                MoveToPointTask(1.1, 1.05, -3.14, movement_type['fast'])]
        else:
            self.coordinates_for_unloading_cubes = [[1.82, 1.05, 0]]
            self.future_tasks = [
                MoveToPointTask(self.coordinates_for_unloading_cubes[unloading_cubes_position][0], self.coordinates_for_unloading_cubes[unloading_cubes_position][1], self.coordinates_for_unloading_cubes[unloading_cubes_position][2], movement_type['fast']),
                ActivateCubesUnloadingTask(time),
                ThrowCubesTask(self.coordinates_for_unloading_cubes[unloading_cubes_position]),
                DisactivateCubesUnloadingTask(),
                MoveToPointTask(2.15, 1.05, 0, movement_type['fast'])]
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
        if (selected_side.value == 'green'): 
            self.future_tasks = [
                #SuperFastMoveToIntermediaryPointTask(0.4, 0.47, -1.57),
                SwitchOffCollisionAvoidanceTask(),
                #SwitchOnCollisionAvoidanceTask(),
                MoveToPointTask(0.28, 0.47, -1.57, movement_type['fast'], True, False, False, True),
                #SwitchOffCollisionAvoidanceTask(),
                MoveToPointTask(0.3, 0.09, -1.57, movement_type['fast'], False, False),
                MoveToPointTask(0.3, 0.18, -1.57, movement_type['fast'], False, False),
                MoveToPointTask(0.57, 0.18, -1.57, movement_type['fast'])]
                #FastMoveToIntermediaryPointTask(0.57, 0.11, -1.57)]
        else:
            self.future_tasks = [
                SwitchOffCollisionAvoidanceTask(),
                MoveToPointTask(2.72, 0.40, 0.0, movement_type['fast']),
                MoveToPointTask(2.72, 0.40, -1.57, movement_type['fast']),
                MoveToPointTask(2.72, 0.09, -1.57, movement_type['fast'], True, False, False, True),
                MoveToPointTask(2.72, 0.25, -1.55, movement_type['fast'], False, False),
                MoveToPointTask(2.5, 0.25, -1.55, movement_type['fast'], False, False),
                MoveToPointTask(2.5, 0.06, -1.57, movement_type['fast'], True, False, False, True)]
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

class CollectMiddleFiguresState(object):
    all_tasks_were_completed = False

    def __init__(self):
        if (selected_side.value == 'green'): 
            self.future_tasks = [
                MoveToPointTask(0.35, 0.65, -1.57, movement_type['fast']),
                #SwitchOffCollisionAvoidanceTask(),
                MoveToPointTask(0.35, 0.985, -1.57, movement_type['fast']),
                MoveToPointTask(1.28, 0.985, -1.57, movement_type['fast'], True, False, False, True),
                MoveToPointTask(1.05, 1.0, -1.57, movement_type['fast'], False)]
                #SwitchOnCollisionAvoidanceTask()]
        else:
            self.future_tasks = [
                MoveToPointTask(2.52, 0.43, -1.57, movement_type['fast']),
                MoveToPointTask(2.52, 1.02, -1.57, movement_type['fast']),
                #SwitchOffCollisionAvoidanceTask(),
                MoveToPointTask(1.7, 1.02, -1.57, movement_type['fast'], True, False, False, True),
                MoveToPointTask(1.9, 1.02, -1.57, movement_type['fast'])]
                #SwitchOnCollisionAvoidanceTask()]
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
        self.future_tasks = [TakeCubesTask(2), TakeCubesTask(1)]
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

        if self.current_task.check_task() is False:
            print 'need to Retake cubes....the next task is RetakeCubesTask' 
            self.current_task = RetakeCubesTask(self.current_task.layer)
            self.current_task.run_task()

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

movement_type = {'slow': 3, 'fast': 0, 'super_fast': 6} 
if (selected_side.value == 'green'):
    collect_cubes_options = [{ 
            'priority': 1, 
            'tasks_list':[ 
                MoveToPointTask(0.60, 0.09, -1.57, movement_type['fast'], True, True, False, True),
                MoveToPointTask(0.60, 0.13, -1.57, movement_type['slow'], True),
                MoveToPointTask(0.67, 0.13, -1.57, movement_type['slow']),
                #SwitchOnCollisionAvoidanceTask(),
                MoveToPointTask(0.67, 0.4, -1.57, movement_type['fast']),
                MoveToPointTask(0.935, 0.4, -1.57, movement_type['slow']),
                #SwitchOffCollisionAvoidanceTask(),
                OpenCollectorTask(227),
                MoveToPointTask(0.935, 0.313, -1.57, movement_type['slow']),
                MoveToPointTask(1.035, 0.313, -1.57, movement_type['super_fast']),
                MoveToPointTask(0.912, 0.322, -1.57, movement_type['slow']),
                TakeCubesTask(3),
                MoveToPointTask(0.912, 0.34, -1.57, movement_type['slow']),
                WideOpenCollectorTask(160),
                MoveToPointTask(0.912, 0.32, -1.57, movement_type['slow']),
                TakeCubesTask(2),
                MoveToPointTask(0.912, 0.335, -1.57, movement_type['slow']),
                WideOpenCollectorTask(133),
                MoveToPointTask(0.912, 0.325, -1.57, movement_type['slow']),
                TakeCubesTask(1),
                #SwitchOnCollisionAvoidanceTask(),
                MoveToPointTask(0.912, 0.45, -1.57, movement_type['fast'], False)]}]
else:
    collect_cubes_options = [{ 
            'priority': 1, 
            'tasks_list': 
                [
                MoveToPointTask(2.48, 0.35, -1.57, movement_type['fast']),
                MoveToPointTask(2.14, 0.35, -1.57, movement_type['fast']),
                MoveToPointTask(2.14, 0.35, -1.57, movement_type['slow']),
                OpenCollectorTask(220),
                MoveToPointTask(2.14, 0.35, -1.57, movement_type['slow'], False, False, True),
                MoveToPointTask(2.13, 0.24, -1.57, movement_type['slow'], False, False, True),
                MoveToPointTask(2.12, 0.30, -1.57, movement_type['fast']),
                TakeCubesTask(3),
                MoveToPointTask(2.12, 0.335, -1.57, movement_type['fast']),
                WideOpenCollectorTask(160),
                MoveToPointTask(2.12, 0.322, -1.57, movement_type['fast']),
                TakeCubesTask(2),
                MoveToPointTask(2.12, 0.35, -1.57, movement_type['fast']),
                WideOpenCollectorTask(133),
                MoveToPointTask(2.12, 0.333, -1.57, movement_type['fast']),
                TakeCubesTask(1),
                MoveToPointTask(2.12, 0.43, -1.57, movement_type['super_fast'], False, False, True)]}]

stm = multiprocessing.Process(target=stmDriver.stmMainLoop, args=(input_command_queue,reply_to_fsm_queue, reply_to_localization_queue))
localisation = multiprocessing.Process(target=localisation.main, args=(input_command_queue,reply_to_localization_queue, current_coordinatess,correction_performed, start_position, current_coordinatess_from_robot,check_start_time, selected_side))
stm.start()
#time.sleep(2)
localisation.start()
states_list = [
    InitializeRobotState(),
    #BrokeMiddleWallState(),
    CloseDoorsState(),
    CollectCubesStates(),
    #CollectCubesStates(),
    #UnloadCubesState(15),
    #CollectCubesStates(),
    DragCubesState(),
    UnloadCubesState(15)]
'''states_list = [InitializeRobotState(), TestState()]'''
MainState(states_list).run_game()   