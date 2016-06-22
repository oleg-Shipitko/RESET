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
taken_cubes_number = None

collision_avoidance_activated = 0
collision_avoidance_time = None
collision_avoidance_time_difference = None

strategy_number = 1

lidar_started = multiprocessing.Value('b', False)

if (selected_side.value == 'green'):
    start_position = [0.14, 0.76, -3.14]
else:
    start_position = [2.86, 0.76, 0]

doors_catch_figures = False
cubes_catcher_catch_figures = False

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

class SwitchOnPneumoAction(object):
    def run_action(self):
        stm_driver('switch_on_pneumo')

    def check_action(self):
        print 'Pneumo switched on'
        return True

class SwitchOffPneumoAction(object):
    def run_action(self):
        stm_driver('switch_off_pneumo')

    def check_action(self):
        print 'Pneumo switched off'
        return True

class OpenDoorsAction(object):
    def run_action(self):
        stm_driver('open_doors')

    def check_action(self):
        print 'open_doors'
        return True

class SlightlyOpenDoorsAction(object):
    def run_action(self):
        stm_driver('slightly_open_doors')

    def check_action(self):
        print 'slightly open doors action'
        return True

class CloseDoorsAction(object):
    def __init__(self):
        self.start_time = time.time()

    def run_action(self):
        stm_driver('close_doors')

    def check_action(self):
        if time.time() - self.start_time > 1.0:
            return True

#######REMOVE
class OpenDoorsTask(object):
    def run_task(self):
        stm_driver('open_doors')

    def check_task(self):
        print 'open_doors'
        return True

class SlightlyOpenDoorsTask(object):
    def run_task(self):
        stm_driver('slightly_open_doors')

    def check_task(self):
        print 'slightly open doors action'
        return True

class CloseDoorsTask(object):
    def run_task(self):
        self.start_time = time.time()
        stm_driver('close_doors')

    def check_task(self):
        if time.time() - self.start_time > 1.5:
            return True

class MoveLeftTask(object):
    def run_task(self):
        stm_driver('go_to_global_point', [current_coordinatess_from_robot[0] - 0.04, current_coordinatess_from_robot[1] + 0.10, current_coordinatess_from_robot[2] + 0.78, 0])
        stm_driver('go_to_global_point', [current_coordinatess_from_robot[0] - 0.14, current_coordinatess_from_robot[1] + 0.14, current_coordinatess_from_robot[2] + 1.54, 1])

    def check_task(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        #print 'is point was reached', is_point_was_reached
        if is_point_was_reached == 1:
            return True

class MoveRightTask(object):
    def run_task(self):
        stm_driver('go_to_global_point', [current_coordinatess_from_robot[0] - 0.04, current_coordinatess_from_robot[1] - 0.10, current_coordinatess_from_robot[2] - 0.78, 0])
        stm_driver('go_to_global_point', [current_coordinatess_from_robot[0] - 0.14, current_coordinatess_from_robot[1] - 0.14, current_coordinatess_from_robot[2] - 1.54, 1])

    def check_task(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        print 'is point was reached', is_point_was_reached
        if is_point_was_reached == 1:
            return True

#TODO convert result to true or false
class CheckDoorsCatchSomethingAction(object):
    def run_action(self):
        global doors_catch_figures
        doors_catch_figures = stm_driver('get_doors_state')
        doors_catch_figures = True

    def check_action(self):
        return True

class SwitchOnTrajectoryAction(object):
    def run_action(self):
        stm_driver('switch_on_tajectory_regulator')

    def check_action(self):
        print 'Trajectory switched on'
        return True

class ClosePredatorMouthCatcherAction(object):
    def __init__(self):
        self.start_time = time.time()

    def run_action(self):
        global taken_cubes_number
        taken_cubes_number = stm_driver('close_predators_mouth')

    def check_action(self):
        if time.time() - self.start_time > 1.0:
            return True

class PushPredatorMouthCatcherAction(object):
    def run_action(self):
        print 'push_predators_mouth'
        stm_driver('push_predators_mouth')
        self.start_time = time.time()

    def check_action(self):
        if time.time() - self.start_time > 0.5:
            print time.time() - self.start_time
            return True

class PullPredatorMouthCatcherAction(object):
    def __init__(self):
        self.start_time = time.time()

    def run_action(self):
        global taken_cubes_number
        print 'pull_predators_mouth'
        taken_cubes_number = stm_driver('pull_predators_mouth')

    def check_action(self):
        if time.time() - self.start_time > 0.8:
            return True

class CloseCubesBorderAction(object):
    def run_action(self):
        global taken_cubes_number
        taken_cubes_number = stm_driver('close_cubes_border')
        self.start_time = time.time()

    def check_action(self):
        if time.time() - self.start_time > 0.5:
            return True

class ClosePredatorMouthCatcherTask(object):
    def __init__(self):
        self.start_time = time.time()

    def run_task(self):
        global taken_cubes_number
        taken_cubes_number = stm_driver('close_predators_mouth')

    def check_task(self):
        if time.time() - self.start_time > 1.0:
            return True


class PushPredatorMouthCatcherTask(object):
    def run_task(self):
        print 'push_predators_mouth'
        stm_driver('push_predators_mouth')
        self.start_time = time.time()

    def check_task(self):
        if time.time() - self.start_time > 0.5:
            print time.time() - self.start_time
            return True

class PullPredatorMouthCatcherTask(object):
    def __init__(self):
        self.start_time = time.time()

    def run_task(self):
        global taken_cubes_number
        print 'pull_predators_mouth'
        taken_cubes_number = stm_driver('pull_predators_mouth')

    def check_task(self):
        if time.time() - self.start_time > 0.8:
            return True

class CloseCubesBorderTask(object):
    def run_task(self):
        global taken_cubes_number
        taken_cubes_number = stm_driver('close_cubes_border')
        self.start_time = time.time()

    def check_task(self):
        if time.time() - self.start_time > 0.5:
            return True

class OpenCubesBorderAction(object):
    def run_action(self):
        global taken_cubes_number
        taken_cubes_number = stm_driver('open_cubes_border')
        self.start_time = time.time()

    def check_action(self):
        if time.time() - self.start_time > 0.5:
            return True

#TODO convert result to true or false
class CheckCubesCatcherAction(object):
    def run_action(self):
        global cubes_catcher_catch_figures
        cubes_catcher_catch_figures = stm_driver('get_cubes_catcher_state')
        cubes_catcher_catch_figures = True

    def check_action(self):
        return True

class MoveLeftAction(object):
    def run_action(self):
        stm_driver('go_to_global_point', [current_coordinatess_from_robot[0] - 0.04, current_coordinatess_from_robot[1] + 0.10, current_coordinatess_from_robot[2] + 0.78])
        stm_driver('go_to_global_point', [current_coordinatess_from_robot[0] - 0.14, current_coordinatess_from_robot[1] + 0.14, current_coordinatess_from_robot[2] + 1.54])

    def check_action(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        print 'is point was reached', is_point_was_reached
        if is_point_was_reached == 1:
            return True

class MoveRightAction(object):
    def run_action(self):
        stm_driver('go_to_global_point', [current_coordinatess_from_robot[0] - 0.04, current_coordinatess_from_robot[1] - 0.10, current_coordinatess_from_robot[2] - 0.78])
        stm_driver('go_to_global_point', [current_coordinatess_from_robot[0] - 0.14, current_coordinatess_from_robot[1] - 0.14, current_coordinatess_from_robot[2] - 1.54])

    def check_action(self):
        is_point_was_reached = stm_driver('is_point_was_reached')
        print 'is point was reached', is_point_was_reached
        if is_point_was_reached == 1:
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
            parameters = [self.x, self.y, self.theta, self.movement_type]
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

class WaitTimeAction(object):
    def __init__(self, time_delay):
        self.time_delay = time_delay
        self.start_time = None

    def run_action(self):
        self.start_time = time.time()

    def check_action(self):
        if time.time() - self.start_time > self.time_delay:
            return True

class ReachMiddleTask(object):
    def __init__(self):
        self.reached_points = 0

    def run_task(self):
        if (selected_side.value == 'green'):
            stm_driver('go_to_global_point', [0.8, 0.52, -3.14, 6])
            stm_driver('go_to_global_point', [1.58, 0.52, -3.14, 7])
        else:
            stm_driver('go_to_global_point', [2.5, 0.57, 0.0, 6])
            stm_driver('go_to_global_point', [1.4, 0.4, 0.0, 7])

    def check_task(self):
        #print 'check point'
        is_point_was_reached = stm_driver('is_point_was_reached')
        #print 'is point was reached', is_point_was_reached
        if is_point_was_reached == 1:
            return True

class WaitTimeTask(object):
    def __init__(self, time_delay):
        self.time_delay = time_delay
        self.start_time = None

    def run_task(self):
        self.start_time = time.time()

    def check_task(self):
        print 'Wait time task', time.time() - self.start_time
        if time.time() - self.start_time > self.time_delay:
            return True

class SwitchOnCollisionAvoidanceTask(object):
    def run_task(self):
        print 'collision avoidance switched on'
        stm_driver('switch_on_collision_avoidance')

    def check_task(self):
        print 'Collision Avoidance is On'
        return True

class SwitchOffCollisionAvoidanceTask(object):
    def run_task(self):
        print 'collision avoidance switched off'
        stm_driver('switch_off_collision_avoidance')

    def check_task(self):
        print 'Collision Avoidance is Off'
        return True

class OpenDoorsTask(object):
    def run_task(self):
        stm_driver('open_doors')

    def check_task(self):
        print 'open_doors'
        return True

#TODO convert result to true or false
class CheckDoorsCatchSomethingTask(object):
    def run_task(self):
        global doors_catch_figures
        doors_catch_figures = stm_driver('get_doors_state')
        doors_catch_figures = True

    def check_task(self):
        return True

class SetInitialCoordinateTask(object):
    def __init__(self, initial_coordinates):
        self.initial_coordinates = initial_coordinates

    def run_task(self):
        self.reply = stm_driver('set_coordinates_without_movement', self.initial_coordinates)
        time.sleep(1)

    def check_task(self):
            return True

class SwitchOnPneumoTask(object):
    def run_task(self):
        stm_driver('switch_on_pneumo')

    def check_task(self):
        print 'Pneumo switched on'
        return True

class SwitchOffPneumoTask(object):
    def run_task(self):
        stm_driver('switch_off_pneumo')

    def check_task(self):
        print 'Pneumo switched off'
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
        while lidar_started.value == False:
            continue
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

class CloseCubesBorderTask(object):
    def run_task(self):
        global taken_cubes_number
        taken_cubes_number = stm_driver('close_cubes_border')
        self.start_time = time.time()

    def check_task(self):
        if time.time() - self.start_time > 0.5:
            return True

class OpenCubesBorderTask(object):
    def run_task(self):
        global taken_cubes_number
        taken_cubes_number = stm_driver('open_cubes_border')
        self.start_time = time.time()

    def check_task(self):
        if time.time() - self.start_time > 0.5:
            return True

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


class TakeMiddleCubesTask(object):
    all_actions_were_completed = False

    def __init__(self):
        if (selected_side.value == 'green'): 
            self.future_actions  = [
                    MoveToPointAction(1.517, 0.5, -1.57, movement_type['fast'], True, True),
                    WaitTimeAction(0.5),
                    MoveToPointAction(1.517, 0.46, -1.57, movement_type['slow'], True, True),
                    PushPredatorMouthCatcherAction(),
                    MoveToPointAction(1.517, 0.325, -1.57, movement_type['slow'], True, True),
                    ClosePredatorMouthCatcherAction(),
                    MoveToPointAction(1.517, 0.35, -1.57, movement_type['slow'], True, True),
                    PullPredatorMouthCatcherAction(),
                    SwitchOnPneumoAction(),
                    MoveToPointAction(1.517, 0.37, -1.57, movement_type['slow'], True, True),
                    MoveToPointAction(1.517, 0.12, -1.57, movement_type['slow'], True, True, False, True),
                    MoveToPointAction(1.517, 0.3, -1.57, movement_type['slow'], True, False),
                    CloseCubesBorderAction(),
                    SwitchOnPneumoAction()]
        else:
            self.future_actions  = [
                    MoveToPointAction(1.517, 0.46, -1.57, movement_type['slow'], True, True),
                    WaitTimeAction(1),
                    MoveToPointAction(1.517, 0.46, -1.57, movement_type['slow'], True, True),
                    MoveToPointAction(1.517, 0.48, -1.57, movement_type['slow'], True, True),
                    PushPredatorMouthCatcherAction(),
                    MoveToPointAction(1.517, 0.325, -1.57, movement_type['slow'], True, True),
                    WaitTimeAction(1),
                    ClosePredatorMouthCatcherAction(),
                    MoveToPointAction(1.517, 0.35, -1.57, movement_type['slow'], True, True),
                    PullPredatorMouthCatcherAction(),
                    SwitchOnPneumoAction(),
                    MoveToPointAction(1.517, 0.37, -1.57, movement_type['slow'], True, True),
                    MoveToPointAction(1.517, 0.12, -1.57, movement_type['slow'], True, True, False, True),
                    MoveToPointAction(1.517, 0.3, -1.57, movement_type['slow'], True, False),
                    CloseCubesBorderAction(),
                    SwitchOnPneumoAction()]
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

#Wrong, because we have to take cubes simultaneousley
class ThrowCubesWithDoorsTask(object):
    all_actions_were_completed = False

    def __init__(self, initial_coordinates):
        if (selected_side.value == 'green'): 
            self.future_actions= [
                    MoveToPointTask[0.98, 0.45, 3.14, movement_type['fast']],
                    MoveToPointTask[0.65, 0.45, 3.14, movement_type['fast']],
                    MoveToPointTask[0.65, 0.94, 3.14, movement_type['fast']],
                    MoveToPointTask[1.1, 0.94, 3.14, movement_type['fast']],
                    OpenDoorsTask(), 
                    MoveToPointTask[1.0, 0.94, 3.14, movement_type['fast']]]
        else:
            self.future_actions= [
                    MoveToPointTask[2.02, 0.45, 3.14, movement_type['fast']],
                    MoveToPointTask[2.35, 0.45, 3.14, movement_type['fast']],
                    MoveToPointTask[2.35, 0.94, 3.14, movement_type['fast']],
                    MoveToPointTask[1.9, 0.94, 3.14, movement_type['fast']],
                    OpenDoorsAction(), 
                    MoveToPointTask[2.0, 0.94, 3.14, movement_type['fast']]]
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

class UnloadMiddleCubesState():
    all_tasks_were_completed = False

    def run_state(self):
        global strategy_number
        
        print 'STRATEGY #', strategy_number
        if (selected_side.value == 'green'):
            if strategy_number == 1:
                self.future_tasks = [
                MoveToPointTask(0.97, 0.6, -1.57, movement_type['fast'], False, False, True),
                MoveToPointTask(0.7, 0.6, -1.57, movement_type['fast'], False, False, True),
                MoveToPointTask(0.7, 1.08, -1.57, movement_type['fast'], False, False, True),
                MoveToPointTask(0.7, 1.08, 0.0, movement_type['slow']),
                SwitchOffPneumoTask(),
                MoveToPointTask(1.19, 1.08, 0.0, movement_type['fast'], True, False),
                MoveToPointTask(1.15, 1.08, 0.0, movement_type['fast'], True, False),
                WaitTimeTask(2),
                OpenCubesBorderTask(),
                MoveToPointTask(0.97, 1.08, 0.0, movement_type['fast'], True, False),
                PushPredatorMouthCatcherTask(),
                MoveToPointTask(0.87, 1.08, 0.0, movement_type['fast'], True, False),
                ClosePredatorMouthCatcherTask(),
                PullPredatorMouthCatcherTask(),
                #MoveToPointAction(0.8, 1.13, -3.14, movement_type['slow'], True, False),
                #OpenDoorsAction(),
                MoveToPointTask(0.75, 1.08, -1.57, movement_type['fast'], True, True, False, True)]
            else:
                self.future_tasks = [
                    MoveToPointTask(0.98, 0.45, 3.14, movement_type['slow']),
                    MoveToPointTask(0.6, 0.45, 3.14, movement_type['fast'], False, False),
                    MoveToPointTask(0.6, 1.1, 3.14, movement_type['fast'], False, False),
                    MoveToPointTask(1.1, 1.1, 3.14, movement_type['fast'], True, False),
                    OpenDoorsTask(), 
                    MoveToPointTask(0.9, 1.1, 3.14, movement_type['fast'], True, False),
                    CloseDoorsTask(),
                    MoveToPointTask(0.82, 1.1, 3.14, movement_type['fast'], False, False)]
        else:
            if strategy_number == 1:
                self.future_tasks = [
                    MoveToPointTask(2.03, 0.55, -1.57, movement_type['fast'], False, False, True),
                    MoveToPointTask(2.28, 0.55, -1.57, movement_type['fast'], False, False, True),
                    MoveToPointTask(2.28, 1.06, -1.57, movement_type['fast'], True, True),
                    MoveToPointTask(2.28, 1.06, -3.14, movement_type['slow']),
                    SwitchOffPneumoTask(),
                    MoveToPointTask(1.85, 1.06, -3.14, movement_type['fast'], True, False),
                    MoveToPointTask(1.9, 1.06, -3.14, movement_type['fast'], True, False),
                    WaitTimeTask(2),
                    OpenCubesBorderTask(),
                    MoveToPointTask(2.05, 1.06, -3.14, movement_type['fast'], True, False),
                    PushPredatorMouthCatcherTask(),
                    MoveToPointTask(2.1, 1.06, -3.14, movement_type['fast'], True, False),
                    ClosePredatorMouthCatcherTask(),
                    PullPredatorMouthCatcherTask(),
                    #MoveToPointAction(0.8, 1.13, -3.14, movement_type['slow'], True, False),
                    #OpenDoorsAction(),
                    MoveToPointTask(2.1, 1.06, 1.57, movement_type['fast'], True, True)]
            else:
                self.future_tasks = [
                    MoveToPointTask(2.06, 0.48, 0.0, movement_type['slow'], True, False),
                    MoveToPointTask(2.43, 0.55, 0.0, movement_type['fast'], False, False),
                    MoveToPointTask(2.43, 1.1, 0.0, movement_type['fast'], False, False),
                    MoveToPointTask(1.9, 1.1, 0.0, movement_type['fast'], False, False),
                    OpenDoorsTask(), 
                    MoveToPointTask(2.05, 1.1, 0.0, movement_type['fast'], False, False),
                    CloseDoorsTask()]

        self.future_tasks.reverse()
        self.current_task = self.future_tasks.pop()
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

class MainState(object):
    def __init__(self, states_list):
        states_list.reverse()
        self.current_state = states_list.pop()
        self.future_states = states_list
        self.interrupted_state = None
    
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
        #print 'SGT:', start_game_time
        print 'GameTime:', current_time - start_game_time
        #print 'check_start_time', check_start_time.value
        if (start_game_time is not None) and (current_time - start_game_time >= 89) and (check_start_time.value is 0):
            reply = stm_driver('switch_off_robot')
            print 'Robot was stopped!!!!!!!'

    def check_collision_avoidance(self):
        global collision_avoidance_activated, collision_avoidance_time, collision_avoidance_time_difference
        collision_avoidance_activated = stm_driver('check_collision_avoidance')
        print 'colission!!!', collision_avoidance_activated
        if collision_avoidance_activated == 1:
            if collision_avoidance_time == None:
                collision_avoidance_time = time.time()
            collision_avoidance_time_difference = time.time() - collision_avoidance_time
        else:
            collision_avoidance_time = None

class InitializeRobotState(MainState):
    all_tasks_were_completed = False

    def __init__(self):
        self.future_tasks = [
            ActivateRobotRegulatorsTask(),
            SetInitialCoordinateTask(start_position),
            SwitchOnCollisionAvoidanceTask()]
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

class CloseDoorsState(object):
    all_tasks_were_completed = False

    def __init__(self):
        if (selected_side.value == 'green'): 
            self.future_tasks = [
                #SuperFastMoveToIntermediaryPointTask(0.4, 0.47, -1.57),
                SwitchOffCollisionAvoidanceTask(),
                #SwitchOnCollisionAvoidanceTask(),0.9, 1.05, -1.57
                MoveToPointTask(0.7, 0.75, -1.57, movement_type['fast'], False, False),
                MoveToPointTask(0.5, 0.47, -1.57, movement_type['fast'], True, False),
                MoveToPointTask(0.5, 0.47, 1.57, movement_type['fast'], True, True),
                MoveToPointTask(0.35, 0.47, 1.57, movement_type['fast'], False, False),
                #SwitchOffCollisionAvoidanceTask(),
                MoveToPointTask(0.35, 0.06, 1.57, movement_type['fast'], True, True, False, True),
                MoveToPointTask(0.35, 0.25, 1.57, movement_type['fast'], False, False),
                MoveToPointTask(0.6, 0.25, 1.57, movement_type['fast'], False, False),
                MoveToPointTask(0.6, 0.08, 1.57, movement_type['fast'], True, True, False, True),
                MoveToPointTask(0.6, 0.25, 1.57, movement_type['fast'], True, False),
                MoveToPointTask(0.2, 0.76, -3.14, movement_type['fast'], True, False)]

        else:
            self.future_tasks = [
                #SuperFastMoveToIntermediaryPointTask(0.4, 0.47, -1.57),
                SwitchOffCollisionAvoidanceTask(),
                #SwitchOnCollisionAvoidanceTask(),0.9, 1.05, -1.57
                MoveToPointTask(2.22, 1.08, 1.57, movement_type['fast'], False, False),
                MoveToPointTask(2.4, 0.7, 1.57, movement_type['fast'], False, False),
                MoveToPointTask(2.7, 0.47, 1.57, movement_type['fast'], True, True),
                #SwitchOffCollisionAvoidanceTask(),
                MoveToPointTask(2.7, 0.08, 1.57, movement_type['fast'], True, True, False, True),
                MoveToPointTask(2.7, 0.25, 1.57, movement_type['fast'], False, False),
                MoveToPointTask(2.45, 0.25, 1.57, movement_type['fast'], False, False),
                MoveToPointTask(2.45, 0.06, 1.57, movement_type['fast'], True, True),
                MoveToPointTask(2.45, 0.25, 1.57, movement_type['fast'], True, False),
                MoveToPointTask(2.86, 0.76, 0.0, movement_type['fast'], True, False)]
            '''self.future_tasks = [
                SwitchOffCollisionAvoidanceTask(),
                MoveToPointTask(2.4, 0.96, 0.0, movement_type['fast']),
                MoveToPointTask(2.72, 0.40, 0.0, movement_type['fast']),
                MoveToPointTask(2.72, 0.40, -1.57, movement_type['fast']),
                MoveToPointTask(2.72, 0.09, -1.57, movement_type['fast'], True, False, False, True),
                MoveToPointTask(2.72, 0.25, -1.55, movement_type['fast'], False, False),
                MoveToPointTask(2.5, 0.25, -1.55, movement_type['fast'], False, False),
                MoveToPointTask(2.5, 0.06, -1.57, movement_type['fast'], True, False, False, True)]'''
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

class CollectCubesInTheMiddleState(object):
    all_tasks_were_completed = False

    def __init__(self):
        if (selected_side.value == 'green'): 
            self.future_tasks = [ 
                ReachMiddleTask(),
                SwitchOffCollisionAvoidanceTask(),
                TakeMiddleCubesTask()]
        else:
            self.future_tasks = [
                ReachMiddleTask(),
                SwitchOffCollisionAvoidanceTask(),
                TakeMiddleCubesTask()]
        self.future_tasks.reverse()
        self.current_task = self.future_tasks.pop()

    def run_state(self):
        self.current_task.run_task()

    def check_state(self):
        global strategy_number, collision_avoidance_activated
        print 'ST', start_game_time
        print 'strategy_number', strategy_number
        print 'check_start_time', check_start_time.value
        print 'collision avoidance flag', collision_avoidance_activated
        if (start_game_time is not None) and (time.time() - start_game_time >= 20) and (check_start_time.value is 0) and (strategy_number == 1) and (collision_avoidance_activated == 1):
            strategy_number = 2
            print 'STRATEGY', strategy_number
            self.all_tasks_were_completed = True
            stm_driver('clean_point_stack')
            print 'Coordinates to send', current_coordinatess_from_robot[0], current_coordinatess_from_robot[1], current_coordinatess_from_robot[2]
            reply = stm_driver('set_coordinates_without_movement', [current_coordinatess_from_robot[0], current_coordinatess_from_robot[1], current_coordinatess_from_robot[2]])
            stm_driver('clean_point_stack')
            stm_driver('switch_off_collision_avoidance')
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

class CollectSideCubesState(object):
    all_tasks_were_completed = False

    def __init__(self):
        global doors_catch_figures
        if (selected_side.value == 'green'):
            self.future_tasks = [
                MoveToPointTask(1.0, 0.5, -3.14, movement_type['fast']),
                MoveToPointTask(1.0, 0.5, 1.57, movement_type['fast'], True, False),
                SwitchOffCollisionAvoidanceTask(),
                MoveToPointTask(0.975, 0.4, 1.57, movement_type['fast']),
                OpenDoorsTask(),
                MoveToPointTask(0.975, 0.225, 1.57, movement_type['slow']),
                MoveToPointTask(0.97, 0.225, 1.57, movement_type['slow']),
                MoveToPointTask(1.05, 0.225, 1.57, movement_type['slow']),
                MoveToPointTask(0.975, 0.225, 1.57, movement_type['slow']),
                CloseDoorsTask(),
                WaitTimeTask(2),
                MoveToPointTask(0.975, 0.28, 1.57, movement_type['slow'], True, False),
                SlightlyOpenDoorsTask(),
                MoveToPointTask(0.975, 0.225, 1.57, movement_type['slow'], True, False),
                CloseDoorsTask(),
                WaitTimeTask(2),
                MoveToPointTask(0.975, 0.45, 1.57, movement_type['slow'])]
                #CheckDoorsCatchSomethingAction()]

            '''if doors_catch_figures == True:
                self.future_tasks = [
                MoveToPointTask(0.6, 0.96, -1.57, movement_type['fast'], True, False, False, True),
                MoveToPointTask(0.6, 0.4, -1.57, movement_type['fast'], True, False, False, True)] + self.future_tasks     

                doors_catch_figures = False'''

        else:
            self.future_tasks = [
                MoveToPointTask(2.2, 0.45, 0.0, movement_type['fast']),
                MoveToPointTask(2.055, 0.45, 1.57, movement_type['fast']),
                OpenDoorsTask(),
                MoveToPointTask(2.055, 0.25, 1.57, movement_type['slow']),
                #SlightlyOpenDoorsTask(),
                MoveToPointTask(1.95, 0.25, 1.57, movement_type['slow']),
                MoveToPointTask(2.055, 0.25, 1.57, movement_type['slow']),
                MoveToPointTask(2.055, 0.25, 1.57, movement_type['slow']),
                CloseDoorsTask(),
                MoveToPointTask(2.055, 0.33, 1.57, movement_type['slow'], True, False),
                SlightlyOpenDoorsTask(),
                MoveToPointTask(2.055, 0.24, 1.57, movement_type['slow'], True, False),
                CloseDoorsTask(),
                MoveToPointTask(2.055, 0.45, 1.57, movement_type['slow'])]
                #CheckDoorsCatchSomethingAction()]
                #CheckDoorsCatchSomethingAction()]

            '''if doors_catch_figures == True:
                self.future_tasks = [
                MoveToPointTask(2.4, 0.96, -1.57, movement_type['fast'], True, False, False, True),
                MoveToPointTask(2.4, 0.4, -1.57, movement_type['fast'], True, False, False, True)] + self.future_tasks''' 

        self.future_tasks.reverse()
        self.current_task = self.future_tasks.pop()

    def run_state(self):
        if strategy_number == 2:
            self.current_task.run_task()
        else:
            self.all_tasks_were_completed  = True

    def check_state(self):        
        if self.all_tasks_were_completed is True:
            return True
        self.check_tasks_in_state()

    def check_tasks_in_state(self):
        if self.current_task.check_task() is True:
            if len(self.future_tasks) is not 0:
                self.current_task = self.future_tasks.pop()
                self.current_task.run_task()
            else:
                self.all_tasks_were_completed = True

class UnloadAllCubesState(object):
    all_tasks_were_completed = False
    
    def __init__(self):
        if (selected_side.value == 'green'):
            self.future_tasks = [
                MoveToPointTask(1.505, 0.4, 0.0, movement_type['fast'], True, True),
                MoveToPointTask(0.45, 0.4, 0.0, movement_type['fast'], False, False, True),
                MoveToPointTask(0.45, 1.13, 0.0, movement_type['fast'], True, True),
                UnloadMiddleCubesTask()]
        else:
            self.future_tasks = [
                MoveToPointTask(1.515, 0.4, -3.14, movement_type['slow'], True, True),
                MoveToPointTask(2.55, 0.45, -3.14, movement_type['fast'], True, True),
                MoveToPointTask(2.55, 0.45, -1.57, movement_type['fast'], True, True),
                MoveToPointTask(2.55, 1.09, -1.57, movement_type['fast'], True, True),
                MoveToPointTask(2.55, 1.09, -3.14, movement_type['fast'], True, True),
                UnloadMiddleCubesTask()]

        self.future_tasks.reverse()
        self.current_task = self.future_tasks.pop()
        self.current_task.run_task()

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
        '''self.future_tasks = [
                MoveToPointTask(0.35, 0.4, -3.14, movement_type['fast'], False, False),
                MoveToPointTask(0.9, 0.4, -3.14, movement_type['fast'], True, False),
                SwitchOffCollisionAvoidanceTask(),
                MoveToPointTask(0.98, 0.4, 1.57, movement_type['fast']),
                OpenDoorsTask(),
                MoveToPointTask(0.98, 0.22, 1.57, movement_type['slow']),
                MoveToPointTask(0.97, 0.21, 1.57, movement_type['slow']),
                MoveToPointTask(1.05, 0.22, 1.57, movement_type['slow']),
                MoveToPointTask(0.98, 0.22, 1.57, movement_type['slow']),
                CloseDoorsTask(),
                MoveToPointTask(0.98, 0.26, 1.57, movement_type['slow'], True, False),
                SlightlyOpenDoorsTask(),
                MoveToPointTask(0.98, 0.22, 1.57, movement_type['slow'], True, False),
                CloseDoorsTask(),
                MoveToPointTask(0.98, 0.45, 1.57, movement_type['slow']),
                MoveToPointTask(0.98, 0.45, 3.14, movement_type['slow']),
                MoveToPointTask(0.6, 0.45, 3.14, movement_type['fast'], True, False),
                MoveToPointTask(0.6, 1.05, 3.14, movement_type['fast'], True, False),
                MoveToPointTask(1.1, 1.05, 3.14, movement_type['fast'], True, False),
                OpenDoorsTask(), 
                MoveToPointTask(1.0, 1.05, 3.14, movement_type['fast'], True, False)]'''
        self.future_tasks = [
                SwitchOffCollisionAvoidanceTask(),
                MoveToPointTask(2.65, 0.58, 0.0, movement_type['fast'], False, False),
                MoveToPointTask(2.2, 0.45, 0.0, movement_type['fast'], True, False),
                MoveToPointTask(2.06, 0.45, 1.57, movement_type['fast']),
                OpenDoorsTask(),
                MoveToPointTask(2.06, 0.24, 1.57, movement_type['slow']),
                #SlightlyOpenDoorsTask(),
                MoveToPointTask(1.95, 0.24, 1.57, movement_type['slow']),
                MoveToPointTask(2.06, 0.24, 1.57, movement_type['slow']),
                MoveToPointTask(2.06, 0.23, 1.57, movement_type['slow']),
                CloseDoorsTask(),
                MoveToPointTask(2.06, 0.28, 1.57, movement_type['slow'], True, False),
                SlightlyOpenDoorsTask(),
                MoveToPointTask(2.06, 0.225, 1.57, movement_type['slow'], True, False),
                CloseDoorsTask(),
                MoveToPointTask(2.06, 0.48, 1.57, movement_type['slow'], True, True),
                MoveToPointTask(2.06, 0.48, 0.0, movement_type['slow'], True, False),
                MoveToPointTask(2.4, 0.55, 0.0, movement_type['fast'], True, False),
                MoveToPointTask(2.4, 1.1, 0.0, movement_type['fast'], True, False),
                MoveToPointTask(1.9, 1.1, 0.0, movement_type['fast'], True, False),
                OpenDoorsTask(), 
                MoveToPointTask(2.0, 1.1, 0.0, movement_type['fast'], True, False),
                CloseDoorsTask()]
                #CheckDoorsCatchSomethingAction()]
                #Taking middle wal
        '''self.future_tasks = [
                ReachMiddleTask(),
                SwitchOffCollisionAvoidanceTask(),
                TakeMiddleCubesTask(),
                UnloadMiddleCubesTask()]
                SwitchOffPneumoTask(),
                MoveToPointTask(1.05, 1.13, 0.0, movement_type['fast'], True, False),
                OpenCubesBorderTask(),
                MoveToPointTask(0.95, 1.13, 0.0, movement_type['slow'], True, False, False, True)]
                self.future_tasks = [
                MoveToPointTask(2.65, 0.5, 0.0, movement_type['fast'],False, False),
                MoveToPointTask(1.51, 0.4, 0.0, movement_type['super_fast'], True, False),
                TakeMiddleCubesTask(),
                MoveToPointTask(1.515, 0.4, -3.14, movement_type['slow'], True, True),
                MoveToPointTask(2.55, 0.45, -3.14, movement_type['fast'], True, True),
                MoveToPointTask(2.55, 0.45, -1.57, movement_type['fast'], True, True),
                MoveToPointTask(2.55, 1.09, -1.57, movement_type['fast'], True, True),
                MoveToPointTask(2.55, 1.09, -3.14, movement_type['fast'], True, True),
                SwitchOffPneumoTask(),
                MoveToPointTask(2.05, 1.09, -3.14, movement_type['fast'], True, True),
                OpenCubesBorderTask()]
                #UnloadMiddleCubesTask()]'''
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

movement_type = {'slow': 3, 'fast': 0, 'super_fast': 6} 

stm = multiprocessing.Process(target=stmDriver.stmMainLoop, args=(input_command_queue,reply_to_fsm_queue, reply_to_localization_queue))
localisation = multiprocessing.Process(target=localisation.main, args=(input_command_queue,reply_to_localization_queue, current_coordinatess,correction_performed, start_position, current_coordinatess_from_robot,check_start_time, selected_side, lidar_started))
stm.start()
#time.sleep(2)
localisation.start()
states_list = [InitializeRobotState(), CollectCubesInTheMiddleState(), CollectSideCubesState(), UnloadMiddleCubesState(), CloseDoorsState()]
#states_list = [InitializeRobotState(), TestState()]
MainState(states_list).run_game()