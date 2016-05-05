import stmDriver
import time
import multiprocessing
import math

input_command_queue = multiprocessing.Queue()
reply_to_fsm_queue = multiprocessing.Queue()
reply_to_localization_queue = multiprocessing.Queue()

request_source = 'fsm'
start_time = time.time()
check_time = False
need_to_update_coordinates = False

cubes_in_trunk = 0
trunk_capacity = 12
unloading_cubes_position = 0

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
        if time.time() - self.start_time > 1.5:
            print time.time() - self.start_time
            return True

class SetCubesManipulatorAngleAction(object):
    def __init__(self, manipulator_angle):
        self.manipulator_angle = manipulator_angle

    def run_action(self):
        stm_driver('set_cube_manipulator_angle', self.manipulator_angle)
        self.start_time = time.time()

    def check_action(self):
        if time.time() - self.start_time > 2:
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
        stm_driver('close_cubes_border')

    def check_action(self):
            print 'Cubes border was closed'
            return True

class CloseCubesManipulatorAction(object):
    def __init__(self):
        self.start_time = time.time()

    def run_action(self):
        #TODO: return number of taken cubes
        stm_driver('close_cube_collector')

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

class MoveToPointAction(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def run_action(self):
        parameters = [self.x, self.y, self.theta, 1]
        stm_driver('go_to_global_point', parameters)

    def check_action(self):
        self.current_coordinates = stm_driver('get_current_coordinates')
        if abs(self.current_coordinates[0] - self.x) <= 0.005  and abs(self.current_coordinates[1] - self.y) <=0.005 and abs(get_angles_diff(math.degrees(self.theta),math.degrees(self.current_coordinates[2]))) <= 1:
            return True

class ThrowCubesTask(object):
    all_actions_were_completed = False

    def __init__(self, initial_coordinates):
        self.future_actions= [
            WaitTimeAction(3),
            MoveToPointAction(initial_coordinates[0]-0.1, initial_coordinates[1], initial_coordinates[2]),
            WaitTimeAction(3),
            MoveToPointAction(initial_coordinates[0]-0.2, initial_coordinates[1], initial_coordinates[2]),
            WaitTimeAction(3)]
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

class MoveToPointTask(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def run_task(self):
        parameters = [self.x, self.y, self.theta, 1]
        stm_driver('go_to_global_point', parameters)

    def check_task(self):
        self.current_coordinates = stm_driver('get_current_coordinates')
        if abs(self.current_coordinates[0] - self.x) <= 0.005  and abs(self.current_coordinates[1] - self.y) <=0.005 and abs(get_angles_diff(math.degrees(self.theta),math.degrees(self.current_coordinates[2]))) <= 1:
            return True

class TakeCubesTask(object):
    all_actions_were_completed = False
    angle_for_closed_manipulator = 275
    default_angle = 255

    def __init__(self, layer, expected_number_of_cubes = 1):
        self.layer = layer
        self.expected_number_of_cubes = expected_number_of_cubes
        self.manipulator_angle = 270
        self.time = time.time()
        print 'Start time', self.time
        
        if self.layer is 3:
            self.manipulator_angle = 195
        elif self.layer is 2:
            self.manipulator_angle = 165
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
            return True

    def check_actions_in_task(self):
        if self.current_action.check_action() is True:
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
        stm_driver('set_coordinates_without_movement', self.initial_coordinates)

    def check_task(self):
            print 'coordinates', self.initial_coordinates, 'were setted as initial coordinates'
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
                if self.interrupted_state == None:
                    if len(self.future_states) is not 0:
                        self.current_state = self.future_states.pop()
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

    def check_game_time(self):
        current_time = time.time()
        if (current_time - start_time >= 87 and check_time):
            a = 1

    def check_enemy(self):
        a = 1

    def check_trunk(self):
        a = 1

class InitializeRobotState(MainState):
    all_tasks_were_completed = False

    def __init__(self):
        self.future_tasks = [
            ActivateRobotRegulatorsTask(),
            SetInitialCoordinateTask([0.1525, 0.72, 0])]
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

class CollectCubesStates(MainState):
    all_tasks_were_completed = False
    
    collect_cubes_options = [{ 
    'priority': 1, 
    'tasks_list': [
        MoveToPointTask(0.68, 0.12, -1.57),
        ResetCoordinatesTask([0.68, 0.16, -1.57]),
        MoveToPointTask(0.68, 0.38, -1.57),
        MoveToPointTask(0.81, 0.38, -1.57),
        MoveToPointTask(0.81, 0.24, -1.57),
        TakeCubesTask(1)]},
        { 
    'priority': 2, 
    'tasks_list': [
        MoveToPointTask(0.68, 0.12, -1.57),
        ResetCoordinatesTask([0.68, 0.16, -1.57]),
        MoveToPointTask(0.68, 0.38, -1.57),
        MoveToPointTask(0.81, 0.38, -1.57),
        MoveToPointTask(0.81, 0.24, -1.57),
        TakeCubesTask(1)]}]

    def __init__(self):
        self.future_tasks = self.choose_collect_cubes_option()
        if self.future_tasks is None:
            print "All tasks for Collect Cubes State were completed"
            self.all_tasks_were_completed = True
        else:
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
                if self.current_task.__class__.__name___ is 'TakeCubesTask':
               	    self.current_task = RetakeCubesTask(self.current_task.layer, self.current_task.expected_number_of_cubes)
               	    self.current_task.run_task()

    def choose_collect_cubes_option(self):
        choosen_option_priority = None
        option_number = 0
        choosen_option_number = None
        for option in self.collect_cubes_options:
            if option['priority'] > choosen_option_priority and option['priority'] is not 0:
                choosen_option_priority = option['priority']
                choosen_option_number = option_number
                option_number = option_number + 1 

        if choosen_option_number is not None:
            self.collect_cubes_options[choosen_option_number]['priority'] = 0
            option = self.collect_cubes_options[choosen_option_number]['tasks_list']
            option.reverse()
            return option
        else:
            return None

class UnloadCubesState(MainState):
    all_tasks_were_completed = False

    def __init__(self):
        global unloading_cubes_position
        self.coordinates_for_unloading_cubes =  [
            [0.9655, 0.915, -3.14],
            [0.9655, 1.015, -3.14],
            [0.9655, 1.115, -3.14]]
        self.future_tasks = [
            MoveToPointTask(self.coordinates_for_unloading_cubes[unloading_cubes_position][0], self.coordinates_for_unloading_cubes[unloading_cubes_position][1], self.coordinates_for_unloading_cubes[unloading_cubes_position][2]),
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
            MoveToPointTask(0.35, 0.10, -1.57),
            MoveToPointTask(0.35, 0.18, -1.57),
            MoveToPointTask(0.55, 0.18, -1.57),
            MoveToPointTask(0.55, 0.10, -1.57)]
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

stm = multiprocessing.Process(target=stmDriver.stmMainLoop, args=(input_command_queue,reply_to_fsm_queue))
stm.start()
states_list = [
    InitializeRobotState(),
    CloseDoorsState(),
    CollectCubesStates()]
MainState(states_list).run_game()