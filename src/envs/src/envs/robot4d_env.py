# Code developed by Deepak Gopinath*, Mahdieh Nejati Javaremi* in February 2020. Copyright (c) 2020. Deepak Gopinath, Mahdieh Nejati Javaremi, Argallab. (*) Equal contribution

from Box2D import (b2EdgeShape, b2FixtureDef, b2PolygonShape, b2Random, b2CircleShape, b2Vec2, b2Color)
import Box2D
from backends.rendering import Viewer, Transform
sys.path.append(os.path.join(rospkg.RosPack().get_path('simulators'), 'scripts'))
from corrective_mode_switch_utils import *
import csv
import math
import numpy as np
import collections
import itertools
import rospy
import threading
from envs.srv import OptimalAction, OptimalActionRequest, OptimalActionResponse
import time

class Robot4DEnv(object):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : FPS
    }

    def __init__(self, env_params):
        self.viewer = None

        self.world = Box2D.b2World()
        self.world.gravity = (0,0)
        self.env_params = env_params
        assert self.env_params is not None

        assert 'num_turns' in self.env_params
        assert 'robot_position' in self.env_params
        assert 'goal_position' in self.env_params
        assert 'robot_orientation' in self.env_params
        assert 'goal_orientation' in self.env_params
        assert 'r_to_g_relative_orientation' in self.env_params
        assert 'start_direction' in self.env_params
        assert 'start_mode' in self.env_params
        assert 'location_of_turn' in self.env_params
        assert 'location_of_gripper_action' in self.env_params

        self.num_turns = None
        self.num_locations = None
        self.STATES = None
        self.LOCATIONS = None
        self.ORIENTATIONS = None
        self.DIMENSIONS = None
        self.LOCATIONS_TO_WAYPOINTS_DICT = None
        self.WAYPOINTS_TO_LOCATION_DICT = None
        self.robot = None
        self.waypoints = None
        self.path_points = None
        self.robot_position = None
        self.robot_orientation = None
        self.goal_position = None
        self.goal_orientation = None
        self.r_to_g_relative_orientation = None
        self.start_direction = None
        self.start_mode = None
        self.location_of_turn = None
        self.location_of_gripper_action = None
        self.service_initialized = False

        self.ready_for_new_prompt = True
        self.text_display_delay = 2
        self.start_msg_ind = 0
    
    def update_params(self, env_params):
        self.env_params = env_params
        assert self.env_params is not None

        assert 'num_turns' in self.env_params
        assert 'robot_position' in self.env_params
        assert 'goal_position' in self.env_params
        assert 'robot_orientation' in self.env_params
        assert 'goal_orientation' in self.env_params
        assert 'r_to_g_relative_orientation' in self.env_params
        assert 'start_direction' in self.env_params
        assert 'start_mode' in self.env_params
        assert 'location_of_turn' in self.env_params
        assert 'location_of_gripper_action' in self.env_params
    
    def get_optimal_action(self, req):
        #change this into get stochastic policy. 
        pass
    
    def _check_continuous_position_on_line_joining_waypoints(self, start_position, end_position, current_position):
        '''
        code borrowed from https://kite.com/python/answers/how-to-determine-if-a-point-is-on-a-line-segment-in-python
        '''
        start_x, end_x, curr_x = start_position[0], end_position[0], current_position[0]
        start_y, end_y, curr_y = start_position[1], end_position[1], current_position[1]

        if not start_x == end_x: #slope is not infinite #parallel to y axis
            m = (end_y - start_y)/(end_x - start_x) #compute finite slope
            curr_on = (curr_y - start_y) == m * (curr_x - start_x) #check if the current point is ON the line
            curr_between = (min(start_x, end_x) <= curr_x <= max(start_x, end_x)) and (min(start_y, end_y) <= curr_y <= max(start_y, end_y)) #check if the current point is between the end points
            curr_on_and_between = curr_on and curr_between #Logical AND of ON and BETWEEN
        else:
            curr_on_and_between =  (min(start_x, end_x) <= curr_x <= max(start_x, end_x))  and (min(start_y, end_y) <= curr_y <= max(start_y,end_y))

        position_on_line = PositionOnLine.NOT_ON_LINE
        if curr_on_and_between:
            position_on_line = PositionOnLine.BETWEEN
            if abs(curr_y - end_y) < 0.1 and abs(curr_x - end_x) < 0.1: #TODO (deepak.gopinath) Define these thresholds in utils.py folder.
                position_on_line = PositionOnLine.END
            if abs(curr_y -start_y) < 0.1 and abs(curr_x - start_x) < 0.1:
                position_on_line = PositionOnLine.START

        assert position_on_line is not PositionOnLine.NOT_ON_LINE
        return position_on_line
        
    def _transform_continuous_position_to_discrete_position(self):
        '''
        Transforms continuous robot position to a discrete position id.
        The current position is between two locations pM and pN (assuming pM is earlier in the path than pN), then if the robot position is anywhere between the line segment joining
        pM and pN, the discrete position will still be pM. If the robot position is within some predefined threshold of the pN, then the discrete id will be pN.
        The restricted motion will ensure that the robot stays ON the line segment always.

        Returns - discrete location associated with the continuous position, Boolean indicator for ensure that the robot is 'snapped' into position at the end of the line segment
        '''
        current_position = self.robot.get_position()
        current_orientation = self.robot.get_angle()
        start_index = int(self.current_discrete_state[0][-1]) #get the number in the location id. That is, if location is p3, retrieve 3. This number is a proxy for which waypoint is the robot at
        #only consider line segments from the current index onwards.
        for (turn_id, p, pt) in zip(range(start_index, self.num_locations-1), self.waypoints[start_index:-1], self.waypoints[start_index+1:]):
            if turn_id < self.location_of_turn or turn_id > self.location_of_turn:
                position_on_line =  self._check_continuous_position_on_line_joining_waypoints(p, pt, current_position) #check if the current position of the robot is at the START, BETWEEN or END of the line segment
                if position_on_line == PositionOnLine.START or position_on_line == PositionOnLine.BETWEEN:
                    return self.WAYPOINTS_TO_LOCATION_DICT[tuple(p)], False
                elif position_on_line == PositionOnLine.END:
                    return self.WAYPOINTS_TO_LOCATION_DICT[tuple(pt)], True
            elif turn_id == self.location_of_turn:
                if abs(current_orientation - self.goal_orientation) > 0.1: #hasn't turned or is turning
                    return self.WAYPOINTS_TO_LOCATION_DICT[tuple(p)], False
                elif abs(current_orientation - self.goal_orientation) < 0.1: #finished turning
                    position_on_line =  self._check_continuous_position_on_line_joining_waypoints(p, pt, current_position)
                    if position_on_line == PositionOnLine.START or position_on_line == PositionOnLine.BETWEEN:
                        return self.WAYPOINTS_TO_LOCATION_DICT[tuple(p)], False
                    elif position_on_line == PositionOnLine.END:
                        return self.WAYPOINTS_TO_LOCATION_DICT[tuple(pt)], True

        return self.WAYPOINTS_TO_LOCATION_DICT[tuple(self.waypoints[-1])], True

    def _transform_continuous_orientation_to_discrete_orientation(self):
        '''
        Retrieves discrete orientation from continuous orientation
        '''
        current_orientation = self.robot.get_angle()
        if self.goal_orientation == PI/2:
            assert current_orientation >= 0.0 and current_orientation <= (PI/2 + 0.1)
            if abs(current_orientation - self.goal_orientation) > 0.1:
                return 0.0, False
            else:
                return self.goal_orientation, True
        elif self.goal_orientation == -PI/2:
            assert current_orientation >= (-PI/2 - 0.1) and current_orientation <= 0.0
            if abs(current_orientation - self.goal_orientation) > 0.1:
                return 0.0, False
            else:
                return self.goal_orientation, True
    
    def _transform_continuous_continuous_gripper_angle_to_discrete_gripper_angle(self):
        '''
        Retrieves discrete gripper angle from continuous gripper angle. 
        Return values: Discrete gripper angle and bool indicating whether goal was reached or not. 
        '''
        current_gripper_angle = self.robot.get_gripper_angle()
        return 0.0, False

    def _retrieve_current_allowed_mode(self):
        pass

    def _init_modes_in_which_motion_allowed_dict(self):
        pass

    def _create_state_transition_model():
        for s in self.STATES: #(cartesian product of x, y, orientation angle and gripper angle)
            self.STATE_TRANSITION_MODEL[s] = collections.OrderedDict()
            for u in LOW_LEVEL_COMMANDS: #[hp, hs, ss, sp]
                self.STATE_TRANSITION_MODEL[s][u] = None


    def _init_state_transition_model():
        pass
    
    def _generate_path(self):
        '''
        Function to generate the track boundaries.
        '''
        if self.r_to_g_relative_orientation == RGOrient.TOP_RIGHT or self.r_to_g_relative_orientation == RGOrient.BOTTOM_LEFT:
            left_edge_angle = PI/2 + PI/4.0
            right_edge_angle = -PI/4.0
        elif self.r_to_g_relative_orientation == RGOrient.TOP_LEFT or self.r_to_g_relative_orientation == RGOrient.BOTTOM_RIGHT:
            left_edge_angle = -(PI/2 + PI/4.0)
            right_edge_angle = PI/4.0

        r = math.sqrt(2)*PATH_HALF_WIDTH/SCALE
        for i in range(0, self.num_turns+2):
            self.path_points[i][0] = self.waypoints[i] + (r*math.cos(left_edge_angle), r*math.sin(left_edge_angle))#left edge
            self.path_points[i][1] = self.waypoints[i] + (r*math.cos(right_edge_angle), r*math.sin(right_edge_angle))

    def _generate_way_points(self):
        '''
        Function to generate the waypoints given the params of the path
        '''
        self.waypoints[0] = self.robot_position
        self.waypoints[-1] = self.goal_position
        #for odd num_turns, both directions are divided using the same divisor.
        #for even num_turns, one dimensions uses divisor and the other uses divisor-1
        if self.num_turns % 2 == 0:
            divisor = self.num_turns/2 + 1
            if self.start_direction == StartDirection.X: #move along x direction first
                x_inc = (self.waypoints[-1][0] - self.waypoints[0][0])/divisor
                y_inc = (self.waypoints[-1][1] - self.waypoints[0][1])/(divisor - 1)
            elif self.start_direction == StartDirection.Y: #move along y direction first
                x_inc = (self.waypoints[-1][0] - self.waypoints[0][0])/(divisor - 1)
                y_inc = (self.waypoints[-1][1] - self.waypoints[0][1])/divisor
        else: #odd number of turns
            divisor = (self.num_turns + 1)/2
            x_inc = (self.waypoints[-1][0] - self.waypoints[0][0])/divisor
            y_inc = (self.waypoints[-1][1] - self.waypoints[0][1])/divisor

        if self.start_direction == StartDirection.X:
            for i in range(1, self.num_turns+1):
                if i % 2 == 0:
                    self.waypoints[i][0] = self.waypoints[i-1][0]
                    self.waypoints[i][1] = self.waypoints[i-1][1] + y_inc
                else:
                    self.waypoints[i][0] = self.waypoints[i-1][0] + x_inc
                    self.waypoints[i][1] = self.waypoints[i-1][1]
        elif self.start_direction == StartDirection.Y:
            for i in range(1, self.num_turns+1):
                if i % 2 == 0:
                    self.waypoints[i][0] = self.waypoints[i-1][0] + x_inc
                    self.waypoints[i][1] = self.waypoints[i-1][1]
                else:
                    self.waypoints[i][0] = self.waypoints[i-1][0]
                    self.waypoints[i][1] = self.waypoints[i-1][1] + y_inc

    def start_countdown(self):
        if self.ready_for_new_prompt:
            self.msg_prompt = EXPERIMENT_START_COUNTDOWN[self.start_msg_ind]
            self.start_msg_ind += 1
            self.ts = time.time()
            self.ready_for_new_prompt = False
        if (time.time() - self.ts) >= self.text_display_delay:
            self.ready_for_new_prompt = True

        self._render_text(self.msg_prompt, COMMAND_DISPLAY_POSITION)
        self.viewer.render(False)

        if self.start_msg_ind == len(EXPERIMENT_START_COUNTDOWN):
            return 1
        else:
            return 0

    def initialize_viewer(self):
        if self.viewer is None:
            self.viewer = Viewer(VIEWPORT_W, VIEWPORT_H)
            self.viewer.set_bounds(0, VIEWPORT_W/SCALE, 0, VIEWPORT_H/SCALE)
            self.viewer.window.set_location(650, 300)
            self.timer_thread.start()
            
    def _destroy(self):
        if self.robot is None: return
        self.world.DestroyBody(self.robot.robot)
        self.robot = None
        self.waypoints = np.zeros((self.num_locations, 2))
        self.LOCATIONS = []
        self.ORIENTATIONS = []
        self.DIMENSIONS = []
        self.DIMENSION_INDICES = []
        self.STATES = []
        self.path_points = np.zeros((self.num_locations, 2, 2))
    
    def initialize(self):
        self.start_session = self.env_params['start']
        self.period = rospy.Duration(1.0)
        self.timer_thread = threading.Thread(target=self._render_timer, args=(self.period,)) #timer for trial time. Run on separate thread
        self.lock = threading.Lock()
        self.current_time = 0
        self.max_time = 50
    
    def reset(self):
        self._destroy()

        #Grab all trial params from the env_params dict
        self.num_turns = self.env_params['num_turns']
        assert self.num_turns > 0
        self.robot_position = self.env_params['robot_position']
        self.robot_orientation = self.env_params['robot_orientation']
        self.goal_position = self.env_params['goal_position']
        self.goal_orientation = self.env_params['goal_orientation']
        self.r_to_g_relative_orientation = self.env_params['r_to_g_relative_orientation'] #top right, top left, bottom, right, bottom left. relative position of the goal with respect to starting position of robot
        self.start_direction = self.env_params['start_direction']
        self.start_mode = self.env_params['start_mode']
        self.start_gripper_angle = self.env_params['start_gripper_angle']
        self.training = self.env_params['training']
        self.assistance_type = ASSISTANCE_CODE_NAME[self.env_params['assistance_type']] # label assistance type
        self.location_of_turn = self.env_params['location_of_turn'] #corresponds to the index in self.waypoints list
        self.location_of_gripper_action = self.env_params['location_of_gripper_action']
        self.num_locations = self.num_turns + 2 #num turns + start + end point

        self.LOCATIONS = ['p' + str(i) for i in range(self.num_locations)] #create location id. p0, p1, p2....pN
        self.ORIENTATIONS = [0, PI/2, -PI/2] #discrete set of rotations. 0 and pi/2
        self.GRIPPER_ANGLES = [0, PI/2] #open, closed
        self.DIMENSIONS = ['x', 'y', 't', 'gr'] #set of dimensions or modes
        self.DIMENSION_INDICES = np.array([0,1,2,3]) #set of mode indices (needed for look up)
        self.STATES = [s for s in itertools.product(self.LOCATIONS, self.ORIENTATIONS, self.DIMENSIONS, self.GRIPPER_ANGLES)] #list of all discrete states for the given configuration
        self.waypoints = np.zeros((self.num_locations, 2))
        self.path_points = np.zeros((self.num_locations, 2, 2)) #to draw the outline of the path
        self.current_discrete_state = (self.LOCATIONS[0], self.ORIENTATIONS[0], self.start_mode, self.start_gripper_angle) #init current discrete state
        self.current_mode_index = DIM_TO_MODE_INDEX[self.start_mode]
        self.current_mode = self.start_mode
        self.current_gripper_angle = self.start_gripper_angle
        self.mode_display_label = None

        #create and initialize the dictionary which contains info regarding which modes/dimension allow motion for each location
        self.MODES_MOTION_ALLOWED = collections.OrderedDict()
        self._init_modes_in_which_motion_allowed_dict()

        #create robot
        self.robot =  Robot4D(self.world, position=self.robot_position, orientation=self.robot_orientation, gripper_angle=self.start_gripper_angle, robot_color=ROBOT_COLOR_WHEN_COMMAND_REQUIRED,  radius=ROBOT_RADIUS/SCALE)
        self._generate_way_points()
        self._generate_path()

        #create STATE_TRANSITION_MODEL
        self.STATE_TRANSITION_MODEL = collections.OrderedDict()
        self._create_state_transition_model()
        self._init_state_transition_model()

        #TODO change the following to incorporate stochastic policy
        #create optimal next state and optimal next action dict
        self.OPTIMAL_NEXT_STATE_DICT = collections.OrderedDict()
        self.OPTIMAL_ACTION_DICT = collections.OrderedDict()
        self._create_optimal_next_state_dict()
        self._generate_optimal_control_dict()


        #create bidrectional mapping between discrete location ids and the waypoints
        self.LOCATIONS_TO_WAYPOINTS_DICT = collections.OrderedDict()
        for i in range(len(self.LOCATIONS)):
            self.LOCATIONS_TO_WAYPOINTS_DICT[self.LOCATIONS[i]] = tuple(self.waypoints[i])

        self.WAYPOINTS_TO_LOCATION_DICT = collections.OrderedDict()
        for i in range(len(self.LOCATIONS)):
            self.WAYPOINTS_TO_LOCATION_DICT[tuple(self.waypoints[i])] = self.LOCATIONS[i]

        self.ALLOWED_DIRECTIONS_OF_MOTION = collections.OrderedDict()
        self._init_allowed_directions_of_motion()

        if not self.service_initialized:
            rospy.Service('/mode_inference_env/get_stochastic_policy', OptimalAction, self.get_optimal_action)
            self.service_initialized = True

        self.period = rospy.Duration(1.0)
        self.timer_thread = threading.Thread(target=self._render_timer, args=(self.period,))
        self.lock = threading.Lock()
        self.current_time = 0
        self.max_time = 50
    
    def step(self, input_action):
        assert 'human' in input_action.keys()

        #Todo potentially replace all of the 'snapping functionality with some sort of stablization controller which gets activated when in the vicinity of the set point?
        current_discrete_position, should_snap = self._transform_continuous_position_to_discrete_position() #transform the continuous robot position to a discrete state representation. Easier for computing the optimal action etc
        if should_snap:
            #if the robot is very close to the 'next' corner, then artificially snap the position of the robot to that corner. So that the linear motion along the segment is properly completed
            self.robot.set_position(self.LOCATIONS_TO_WAYPOINTS_DICT[current_discrete_position]) #directly set the position of the robot
            current_discrete_position, should_snap = self._transform_continuous_position_to_discrete_position() #recompute the discrete position

        current_discrete_orientation, should_snap = self._transform_continuous_orientation_to_discrete_orientation() #compute discrete orientation state from continuous orientation of the robot
        if should_snap:
            self.robot.set_orientation(current_discrete_orientation) #if snap is true then force orientation to be that of the target orientation
            current_discrete_orientation, should_snap = self._transform_continuous_orientation_to_discrete_orientation() #recompute the discrete orientation

        #restrict the nonzero components of the velocity only to the allowed modes.
        self.current_mode_index = rospy.get_param('mode') #0,1,2,3 #get current mode index
        self.current_mode =  MODE_INDEX_TO_DIM[self.current_mode_index] #x,y,t,g #get current mode
        self.current_discrete_state = (current_discrete_position, current_discrete_orientation, self.current_mode) #update the current discrete state.
        current_allowed_mode = self._retrieve_current_allowed_mode() #x,y,t #for the given location, retrieve what is the allowed mode of motion.
        current_allowed_mode_index = DIM_TO_MODE_INDEX[current_allowed_mode] #0,1,2 #get the mode index of the allowed mode of motion
        user_vel = np.array([input_action['human'].velocity.data[0], input_action['human'].velocity.data[1], input_action['human'].velocity.data[2]]) #numpyify the velocity data. note the negative sign on the 3rd component.To account for proper counterclockwise motion
        user_vel[np.setdiff1d(self.DIMENSION_INDICES, current_allowed_mode_index)] = 0.0 #zero out all velocities except the ones for the allowed mode

        #check if the direction of user_vel is correct as well. For each location in the allowed mode/dimension there is proper direction in which the velocity should be.
        current_allowed_direction_of_motion_in_allowed_mode = self._retrieve_allowed_direction_motion_in_allowed_mode()
        if get_sign_of_number(user_vel[current_allowed_mode_index]) != current_allowed_direction_of_motion_in_allowed_mode: #check if the direction velocity component in the allowed mode matches the allowed direction of motion in the allowed mode
            user_vel[current_allowed_mode_index] = 0.0 #if not, zero the velocity out

        rospy.set_param('current_discrete_state', self.current_discrete_state)
        self.robot.robot.linearVelocity = b2Vec2(user_vel[0], user_vel[1]) #update robot velocity
        self.robot.robot.angularVelocity = -user_vel[2]
        if self.current_time >= self.max_time and not self.training:
            self.robot.robot.linearVelocity = b2Vec2(0.0, 0.0)
            self.robot.robot.angularVelocity = 0.0

        self.robot.step(input_action['human'])
        self.world.Step(1.0/FPS, VELOCITY_ITERATIONS, POSITION_ITERATIONS)



