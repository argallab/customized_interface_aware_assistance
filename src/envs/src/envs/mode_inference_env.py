from Box2D import (b2EdgeShape, b2FixtureDef, b2PolygonShape, b2Random, b2CircleShape, b2Vec2, b2Color)
import Box2D
from backends.rendering import Viewer, Transform
from utils import RobotSE2, FPS, VELOCITY_ITERATIONS, POSITION_ITERATIONS, SCALE, VIEWPORT_W, VIEWPORT_H
from utils import PI, ROBOT_RADIUS, GOAL_RADIUS, MODE_DISPLAY_RADIUS
from utils import ROBOT_COLOR_WHEN_MOVING, ROBOT_COLOR_WHEN_COMMAND_REQUIRED, ACTIVE_MODE_COLOR, NONACTIVE_MODE_COLOR, TURN_LOCATION_COLOR, MODE_DISPLAY_TEXT_COLOR, MODE_DISPLAY_TEXT_FONTSIZE
from utils import MODE_DISPLAY_CIRCLE_START_POSITION_S, MODE_DISPLAY_CIRCLE_X_OFFSET_S, MODE_DISPLAY_TEXT_START_POSITION, MODE_DISPLAY_TEXT_X_OFFSET, MODE_DISPLAY_TEXT_Y_ANCHOR
from utils import TIMER_DISPLAY_POSITION, TIMER_DISPLAY_FONTSIZE, TIMER_COLOR_NEUTRAL, TIMER_COLOR_WARNING, TIMER_COLOR_DANGER, TIMER_DISPLAY_TEXT_Y_ANCHOR, TIMER_DANGER_THRESHOLD, TIMER_WARNING_THRESHOLD
from utils import TRIAL_OVER_TEXT_DISPLAY_POSITION, TRIAL_OVER_TEXT_FONTSIZE, TRIAL_OVER_TEXT_COLOR, TRIAL_OVER_TEXT_Y_ANCHOR, START_GOAL_TEXT_DISPLACEMENT, MODE_DISPLAY_CIRCLE_Y_OFFSET_S
from utils import WP_RADIUS, INFLATION_FACTOR, PATH_HALF_WIDTH, MODE_INDEX_TO_DIM, DIM_TO_MODE_INDEX
from utils import RGOrient, StartDirection, PositionOnLine
from utils import get_sign_of_number
from utils import LOW_LEVEL_COMMANDS, HIGH_LEVEL_ACTIONS, TRUE_ACTION_TO_COMMAND, TRUE_COMMAND_TO_ACTION, MODE_SWITCH_TRANSITION, TRANSITION_FOR_ACTION
from utils import COMMAND_TEXT_COLOR, COMMAND_DISPLAY_POSITION, COMMAND_DISPLAY_FONTSIZE, LABEL_DISPLAY_POSITION
from utils import EXPERIMENT_START_COUNTDOWN
from utils import ASSISTANCE_CODE_NAME
import csv
import math
import numpy as np
import collections
import itertools
import rospy
import threading
from envs.srv import OptimalAction, OptimalActionRequest, OptimalActionResponse
import time
from IPython import embed

class ModeInferenceEnv(object):
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

    def get_optimal_action(self, req):
        response = OptimalActionResponse()
        current_discrete_state = rospy.get_param('current_discrete_state', ['p0', 0, 't'])
        current_discrete_state = tuple(current_discrete_state)
        if current_discrete_state[0] != self.LOCATIONS[-1]:
            assert self.OPTIMAL_ACTION_DICT is not None and current_discrete_state in self.OPTIMAL_ACTION_DICT
            response.optimal_high_level_action =  self.OPTIMAL_ACTION_DICT[current_discrete_state]
            response.current_mode = current_discrete_state[-1]
            response.status = True
        else:
            response.optimal_high_level_action = 'None'
            response.current_mode = 'None'
            response.status = False
        return response

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
        The current position is between two locations pM and pN (assuming pM is earlier in the path than pN), then if the robot position is anywhere between (and n the ealso on pM) the line segment joining
        pM and pN, the discrete position will still be pM. If the robot position is within some predefined threshold of the pN, then the discrete id will be pN.
        The restricted motion will ensure that the robot stays ON the line segment always.

        Returns - discrete location associated with the continuous position, Boolean indicator for ensure that the robot is 'snapped' into position at the end of the line segment
        '''

        current_position = self.robot.get_position()
        current_orientation = self.robot.get_angle()
        start_index = int(self.current_discrete_state[0][-1]) #get the number in the location id. That is, if location is p3, retrieve 3. This number is a proxy for which waypoint is the robot at
        # print start_index
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
        # assert current_orientation >= 0.0 and current_orientation <= (PI/2 + 0.1)
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

    def _retrieve_current_allowed_mode(self):
        '''
        For a given discrete state, retrieves the mode/dimension in which motion is allowed.
        '''
        location = self.current_discrete_state[0]
        orientation = self.current_discrete_state[1]
        if location != self.LOCATIONS[self.location_of_turn]:
            return self.MODES_MOTION_ALLOWED[location][0]
        else:
            if orientation == 0.0: #still hasn't turned or is still turning. therefore the allowed mode is 't'
                return self.MODES_MOTION_ALLOWED[location][1] #'t'
            else:
                return self.MODES_MOTION_ALLOWED[location][0] #'linear'

    def _retrieve_allowed_direction_motion_in_allowed_mode(self):
        '''
        For a given discrete state, retrieves the direction in which motion is allowed in the allowed mode
        '''
        location = self.current_discrete_state[0]
        orientation = self.current_discrete_state[1]
        if location != self.LOCATIONS[self.location_of_turn]:
            return self.ALLOWED_DIRECTIONS_OF_MOTION[location][0]
        else:
            if orientation == 0.0:
                return self.ALLOWED_DIRECTIONS_OF_MOTION[location][1]
            else:
                return self.ALLOWED_DIRECTIONS_OF_MOTION[location][0]

    #initializer functions for different dictionaries
    def _init_modes_in_which_motion_allowed_dict(self):
        '''
    	Specifies, for each location, the modes in which motion is allowed
    	'''
        for i, s in enumerate(self.LOCATIONS[:-1]):
            if self.start_direction == StartDirection.X:
                if i % 2 == 0:
                    self.MODES_MOTION_ALLOWED[s] = ['x']
                else:
                    self.MODES_MOTION_ALLOWED[s] = ['y']
                if i == self.location_of_turn:
                    self.MODES_MOTION_ALLOWED[s].append('t') #note that the theta dimension is always the second one in the list.
            elif self.start_direction == StartDirection.Y:
                if i % 2 == 0:
                    self.MODES_MOTION_ALLOWED[s] = ['y']
                else:
                    self.MODES_MOTION_ALLOWED[s] = ['x']
                if i == self.location_of_turn:
                    self.MODES_MOTION_ALLOWED[s].append('t')

        self.MODES_MOTION_ALLOWED[self.LOCATIONS[-1]] = self.MODES_MOTION_ALLOWED[self.LOCATIONS[-2]][0] #for the last location, copy the linear mode in which the motion is allowed for the second last location

    def _init_allowed_directions_of_motion(self):
        '''
        For each location (p0,p1...), specifies in which direction is motion allowed in the allowed modes.
        '''
        for i, (loc_p, loc_n) in enumerate(zip(self.LOCATIONS[:-1], self.LOCATIONS[1:])):
            diff_vector = np.array(self.LOCATIONS_TO_WAYPOINTS_DICT[loc_n]) - np.array(self.LOCATIONS_TO_WAYPOINTS_DICT[loc_p]) #find the vector connecting two waypoints
            motion_dimension_index = np.nonzero(diff_vector)[0] #pick which is the dimension in which the vector is pointing. Same as the allowed dimension for motion
            sign_of_motion = np.sign(diff_vector[motion_dimension_index])[0] #determine the sign of the vector (that is if it is y, should the robot be moving up or down?)
            self.ALLOWED_DIRECTIONS_OF_MOTION[loc_p] = [sign_of_motion]
            if i == self.location_of_turn:
                if self.goal_orientation == PI/2:
                    self.ALLOWED_DIRECTIONS_OF_MOTION[loc_p].append(-1.0) #append direction indicator for rotation. counterclockwise is 1.0
                else:
                    self.ALLOWED_DIRECTIONS_OF_MOTION[loc_p].append(1.0)

        self.ALLOWED_DIRECTIONS_OF_MOTION[self.LOCATIONS[-1]] = [self.ALLOWED_DIRECTIONS_OF_MOTION[self.LOCATIONS[-2]][0]] #copy the direction of the linear dimension of the second last waypoint.

    def _create_state_transition_model(self):
        for s in self.STATES:
            self.STATE_TRANSITION_MODEL[s] = collections.OrderedDict()
            for u in LOW_LEVEL_COMMANDS:
                self.STATE_TRANSITION_MODEL[s][u] = None

    def _init_state_transition_model(self):
        rgc = self.r_to_g_relative_orientation
        for s in self.STATE_TRANSITION_MODEL.keys():#for all states in the world
            for u in self.STATE_TRANSITION_MODEL[s].keys():#for all available low-level commands
                if u == 'Hard Puff' or u == 'Hard Sip':
                    self.STATE_TRANSITION_MODEL[s][u] = (s[0], s[1], MODE_SWITCH_TRANSITION[s[2]][u]) #generate new state
                if u == 'Soft Puff' or u == 'Soft Sip': #if sp or ss, this will result in motion if the mode associated with s an allowed mode for motion
                    allowed_modes_for_motion = self.MODES_MOTION_ALLOWED[s[0]] #retrieve all the modes in which motion is allowed for this state. (note that, for the location in which turning happens there will be two modes in which motion is allowed)
                    self.STATE_TRANSITION_MODEL[s][u] = (s[0], s[1], s[2]) #by default store the same state as next state. Because if no motion happens, the resultant state is also the same state
                    for m in allowed_modes_for_motion:
                        if m == s[2]: #make sure that the allowed mode matches the mode in the state s. If it doesn't no motion will happen
                            if m != 't': #allowed motion mode is a linear mode, x or y.
                                if TRANSITION_FOR_ACTION[rgc][u][m] == 'next':
                                    new_loc_next = self.LOCATIONS[min(self.LOCATIONS.index(s[0]) + 1, len(self.LOCATIONS)-1 )]
                                    self.STATE_TRANSITION_MODEL[s][u] = (new_loc_next, s[1], s[2])
                                elif TRANSITION_FOR_ACTION[rgc][u][m] == 'prev':
                                    new_loc_prev = self.LOCATIONS[max(self.LOCATIONS.index(s[0]) - 1, 0 )]
                                    if m == self.MODES_MOTION_ALLOWED[new_loc_prev][0]:
                                        self.STATE_TRANSITION_MODEL[s][u] = (new_loc_prev, s[1], s[2])
                            elif m == 't':# if allowed mode is rotation mode, rotate the angle properly.
                                new_theta = s[1]
                                if TRANSITION_FOR_ACTION[rgc][u][m] == 'next':
                                    new_theta = min(PI/2, s[1] + PI/2) #max angle allowed is PI/2
                                elif TRANSITION_FOR_ACTION[rgc][u][m] == 'prev':
                                    new_theta = max(-PI/2, s[1] - PI/2) #min angle allowed is 0.0

                                self.STATE_TRANSITION_MODEL[s][u] = (s[0], new_theta, s[2])

    def _create_optimal_next_state_dict(self):

        for s in self.STATES:
            if s[0] == self.LOCATIONS[-1]:
                continue
            if self.LOCATIONS.index(s[0]) < self.location_of_turn or self.LOCATIONS.index(s[0]) > self.location_of_turn:
                if s[2] not in self.MODES_MOTION_ALLOWED[s[0]]:
                    if len(self.MODES_MOTION_ALLOWED[s[0]]) == 1:
                        self.OPTIMAL_NEXT_STATE_DICT[s] = (s[0], s[1], self.MODES_MOTION_ALLOWED[s[0]][0])
                else:
                    self.OPTIMAL_NEXT_STATE_DICT[s] = (self.LOCATIONS[min(self.LOCATIONS.index(s[0])+1, self.num_locations)], s[1], s[2])
            elif self.LOCATIONS.index(s[0]) == self.location_of_turn:
                if s[2] != 't':
                    if s[1] != self.goal_orientation:
                        self.OPTIMAL_NEXT_STATE_DICT[s] = (s[0], s[1], 't')
                    else:
                        if s[2] == self.MODES_MOTION_ALLOWED[s[0]][0]:
                            self.OPTIMAL_NEXT_STATE_DICT[s] = (self.LOCATIONS[min(self.LOCATIONS.index(s[0]) + 1, self.num_locations)], s[1], s[2])
                        else:
                            self.OPTIMAL_NEXT_STATE_DICT[s] = (s[0], s[1], self.MODES_MOTION_ALLOWED[s[0]][0])
                else:
                    if s[1] != self.goal_orientation:
                        diff_orientation = self.goal_orientation - s[1]
                        if diff_orientation > 0:
                            diff_orientation = min(PI/2, diff_orientation)
                        else:
                            diff_orientation = max(-PI/2, diff_orientation)
                        self.OPTIMAL_NEXT_STATE_DICT[s] = (s[0], s[1] + diff_orientation, s[2])
                    else:
                        if s[2] == self.MODES_MOTION_ALLOWED[s[0]][0]:
                            self.OPTIMAL_NEXT_STATE_DICT[s] = (self.LOCATIONS[min(self.LOCATIONS.index(s[0]) + 1, self.num_locations)], s[1], s[2])
                        else:
                            self.OPTIMAL_NEXT_STATE_DICT[s] = (s[0], s[1], self.MODES_MOTION_ALLOWED[s[0]][0])

    def _generate_optimal_control_dict(self):
        for s in self.OPTIMAL_NEXT_STATE_DICT:
            sp = self.OPTIMAL_NEXT_STATE_DICT[s]
            self.OPTIMAL_ACTION_DICT[s] = [k for k,v in self.STATE_TRANSITION_MODEL[s].items() if v == sp]
            self.OPTIMAL_ACTION_DICT[s] = TRUE_COMMAND_TO_ACTION[s[2]][self.OPTIMAL_ACTION_DICT[s][0]]

    #RENDER FUNCTIONS
    def _render_goal(self):
        t = Transform(translation=(self.goal_position[0],self.goal_position[1]))
        self.viewer.draw_circle(GOAL_RADIUS/SCALE, 30, True, color=(0.53, 1.0, 0.42)).add_attr(t)
        self.viewer.draw_line(self.goal_position, (self.goal_position[0] + 2*(GOAL_RADIUS/SCALE)*math.cos(self.goal_orientation), self.goal_position[1] + 2*(GOAL_RADIUS/SCALE)*math.sin(self.goal_orientation)), linewidth=3.0)

    def _render_bodies(self):
        for r in [self.robot]:
            if isinstance(r, RobotSE2):
                robot = r.robot
            else:
                robot = r
            for f in robot.fixtures:
                trans = f.body.transform
                if type(f.shape) is b2CircleShape:
                    t = Transform(translation = trans.position)
                    if isinstance(r, RobotSE2):
                        if abs(np.linalg.norm(robot.linearVelocity)) > 0:
                            self.viewer.draw_circle(f.shape.radius, 30, color=r.robot_color, filled=True).add_attr(t)
                        else:
                            self.viewer.draw_circle(f.shape.radius, 30, color=ROBOT_COLOR_WHEN_MOVING, filled=True).add_attr(t)
                    else:
                        self.viewer.draw_circle(f.shape.radius, 30, color=(1.0, 1.0, 1.0), filled=True).add_attr(t)
                elif type(f.shape) is b2EdgeShape:
                    self.viewer.draw_line(f.shape.vertices[0], f.shape.vertices[1])
                else:
                    path = [trans*v for v in f.shape.vertices]
                    self.viewer.draw_polygon(path, color=(1.0, 0.0, 0.0))
                    path.append(path[0])
                    self.viewer.draw_polyline(path, color=(1.0, 0.0, 0.0), linewidth=2)

    def _render_robot_direction_indicators(self):
        ep_markers = self.robot.get_direction_marker_end_points()
        self.viewer.draw_line(ep_markers[0], ep_markers[1], linewidth=3.0)

    def _render_waypoints(self):
        #render the waypoints
        for i in range(len(self.waypoints)):
            t =  Transform(translation=(self.waypoints[i][0], self.waypoints[i][1]))
            self.viewer.draw_circle(WP_RADIUS/SCALE, 30, True, color=(0,0,0)).add_attr(t)

    def _render_path(self):
        for i in range(1, len(self.path_points)):
            self.viewer.draw_line(tuple(self.path_points[i-1][0]), tuple(self.path_points[i][0])) #draw left edge
            self.viewer.draw_line(tuple(self.path_points[i-1][1]), tuple(self.path_points[i][1])) #draw right edge

    def _render_turn_location(self):
        location_of_turn_waypoint = self.waypoints[self.location_of_turn]
        t =  Transform(translation=(location_of_turn_waypoint[0], location_of_turn_waypoint[1]))
        self.viewer.draw_circle(8*WP_RADIUS/SCALE, 4, True, color=TURN_LOCATION_COLOR).add_attr(t) # TODO Look into how to properly render a box instead of a circle with 4 points!

    def _render_mode_display(self):
        for i, d in enumerate(self.DIMENSIONS):
            if d == 'y':
                t = Transform(translation=(MODE_DISPLAY_CIRCLE_START_POSITION_S[0] + i*MODE_DISPLAY_CIRCLE_X_OFFSET_S, MODE_DISPLAY_CIRCLE_START_POSITION_S[1]))
            else:
                t = Transform(translation=(MODE_DISPLAY_CIRCLE_START_POSITION_S[0] + i*MODE_DISPLAY_CIRCLE_X_OFFSET_S, MODE_DISPLAY_CIRCLE_START_POSITION_S[1] - MODE_DISPLAY_CIRCLE_Y_OFFSET_S))

            if d == self.current_mode:
                self.viewer.draw_circle(MODE_DISPLAY_RADIUS/SCALE, 30, True, color=ACTIVE_MODE_COLOR).add_attr(t)
            else:
                self.viewer.draw_circle(MODE_DISPLAY_RADIUS/SCALE, 30, True, color=NONACTIVE_MODE_COLOR).add_attr(t)

    def _render_mode_display_text(self):
        '''
        Note that the coordinates of the text should in real pixels. Which is why here there is a multiplicative factor of SCALE.
        '''
        self.viewer.draw_text("Horizontal", x=MODE_DISPLAY_TEXT_START_POSITION[0], y=MODE_DISPLAY_TEXT_START_POSITION[1] - MODE_DISPLAY_CIRCLE_Y_OFFSET_S*SCALE , font_size=MODE_DISPLAY_TEXT_FONTSIZE, color=MODE_DISPLAY_TEXT_COLOR, anchor_y=MODE_DISPLAY_TEXT_Y_ANCHOR)
        self.viewer.draw_text("Vertical", x=MODE_DISPLAY_TEXT_START_POSITION[0] + MODE_DISPLAY_TEXT_X_OFFSET, y=MODE_DISPLAY_TEXT_START_POSITION[1], font_size=MODE_DISPLAY_TEXT_FONTSIZE, color=MODE_DISPLAY_TEXT_COLOR, anchor_y=MODE_DISPLAY_TEXT_Y_ANCHOR)
        self.viewer.draw_text("Rotation", x=MODE_DISPLAY_TEXT_START_POSITION[0] + 2*MODE_DISPLAY_TEXT_X_OFFSET, y=MODE_DISPLAY_TEXT_START_POSITION[1] - MODE_DISPLAY_CIRCLE_Y_OFFSET_S*SCALE, font_size=MODE_DISPLAY_TEXT_FONTSIZE, color=MODE_DISPLAY_TEXT_COLOR, anchor_y=MODE_DISPLAY_TEXT_Y_ANCHOR)

    def _render_timer_text(self):
        if self.current_time < TIMER_WARNING_THRESHOLD:
            self.viewer.draw_text(str(self.current_time), x=TIMER_DISPLAY_POSITION[0], y=TIMER_DISPLAY_POSITION[1], font_size=TIMER_DISPLAY_FONTSIZE, color=TIMER_COLOR_NEUTRAL, anchor_y=TIMER_DISPLAY_TEXT_Y_ANCHOR, bold=True)
        elif self.current_time < TIMER_DANGER_THRESHOLD:
            self.viewer.draw_text(str(self.current_time), x=TIMER_DISPLAY_POSITION[0], y=TIMER_DISPLAY_POSITION[1], font_size=TIMER_DISPLAY_FONTSIZE, color=TIMER_COLOR_WARNING, anchor_y=TIMER_DISPLAY_TEXT_Y_ANCHOR, bold=True)
        else:
            self.viewer.draw_text(str(self.current_time), x=TIMER_DISPLAY_POSITION[0], y=TIMER_DISPLAY_POSITION[1], font_size=TIMER_DISPLAY_FONTSIZE, color=TIMER_COLOR_DANGER, anchor_y=TIMER_DISPLAY_TEXT_Y_ANCHOR, bold=True)

    def _render_timer(self, period):
        while not rospy.is_shutdown():
            start = rospy.get_rostime()
            self.lock.acquire()
            if self.viewer is not None:
                # self.viewer.draw_text("   ", x = VIEWPORT_W/2, y=VIEWPORT_H/6, font_size=MODE_DISPLAY_TEXT_FONTSIZE, color=MODE_DISPLAY_TEXT_COLOR, anchor_y='top')
                self.current_time += 1
            else:
                pass
            self.lock.release()
            end = rospy.get_rostime()
            if end-start < period:
                rospy.sleep(period - (end-start))
            else:
                rospy.loginfo('took more time')

    def _render_text(self, msg, position, font_size=TRIAL_OVER_TEXT_FONTSIZE, color=COMMAND_TEXT_COLOR):
        self.viewer.draw_text(msg, x=position[0], y=position[1], font_size=font_size, color=color, bold=True)

    def _render_start_end_text(self):

        if self.r_to_g_relative_orientation == RGOrient.TOP_RIGHT:
            if self.start_direction == StartDirection.X:
                start_text_location = (self.robot_position[0]* SCALE - START_GOAL_TEXT_DISPLACEMENT, self.robot_position[1]*SCALE)
                goal_text_location = (self.goal_position[0]* SCALE - START_GOAL_TEXT_DISPLACEMENT, self.goal_position[1]*SCALE)
            elif self.start_direction == StartDirection.Y:
                start_text_location = (self.robot_position[0]* SCALE - START_GOAL_TEXT_DISPLACEMENT, self.robot_position[1]*SCALE)
                goal_text_location = (self.goal_position[0]* SCALE, self.goal_position[1]*SCALE - START_GOAL_TEXT_DISPLACEMENT)
        elif self.r_to_g_relative_orientation == RGOrient.TOP_LEFT:
            if self.start_direction == StartDirection.X:
                start_text_location = (self.robot_position[0]* SCALE + START_GOAL_TEXT_DISPLACEMENT, self.robot_position[1]*SCALE)
                goal_text_location = (self.goal_position[0]* SCALE + START_GOAL_TEXT_DISPLACEMENT, self.goal_position[1]*SCALE)
            elif self.start_direction == StartDirection.Y:
                start_text_location = (self.robot_position[0]* SCALE - START_GOAL_TEXT_DISPLACEMENT, self.robot_position[1]*SCALE)
                goal_text_location = (self.goal_position[0]* SCALE - START_GOAL_TEXT_DISPLACEMENT, self.goal_position[1]*SCALE)
        elif self.r_to_g_relative_orientation == RGOrient.BOTTOM_LEFT:
            if self.start_direction == StartDirection.X:
                start_text_location = (self.robot_position[0]* SCALE, self.robot_position[1]*SCALE - START_GOAL_TEXT_DISPLACEMENT)
                goal_text_location = (self.goal_position[0]* SCALE - START_GOAL_TEXT_DISPLACEMENT, self.goal_position[1]*SCALE)
            elif self.start_direction == StartDirection.Y:
                start_text_location = (self.robot_position[0]* SCALE - START_GOAL_TEXT_DISPLACEMENT, self.robot_position[1]*SCALE)
                goal_text_location = (self.goal_position[0]* SCALE - START_GOAL_TEXT_DISPLACEMENT, self.goal_position[1]*SCALE)
        elif self.r_to_g_relative_orientation == RGOrient.BOTTOM_RIGHT:
            if self.start_direction == StartDirection.X:
                start_text_location = (self.robot_position[0]* SCALE - START_GOAL_TEXT_DISPLACEMENT, self.robot_position[1]*SCALE)
                goal_text_location = (self.goal_position[0]* SCALE - START_GOAL_TEXT_DISPLACEMENT, self.goal_position[1]*SCALE)
            elif self.start_direction == StartDirection.Y:
                start_text_location = (self.robot_position[0]* SCALE - START_GOAL_TEXT_DISPLACEMENT , self.robot_position[1]*SCALE)
                goal_text_location = (self.goal_position[0]* SCALE + START_GOAL_TEXT_DISPLACEMENT, self.goal_position[1]*SCALE)

        self.viewer.draw_text('Start', x=start_text_location[0], y=start_text_location[1], font_size=TRIAL_OVER_TEXT_FONTSIZE-6, color=(0,0,0,255), bold=False)
        self.viewer.draw_text('Goal', x=goal_text_location[0], y=goal_text_location[1], font_size=TRIAL_OVER_TEXT_FONTSIZE-6, color=(0,0,0,255), bold=False)

    def render_clear(self, msg):
        self.viewer.window.clear()
        self._render_text(msg, COMMAND_DISPLAY_POSITION)
        return self.viewer.render(False)

    def close_window(self):
        self.viewer.close()

    def _render_trial_over_text(self):
        self.viewer.draw_text("TRIAL OVER", x=TRIAL_OVER_TEXT_DISPLAY_POSITION[0], y=TRIAL_OVER_TEXT_DISPLAY_POSITION[1], font_size=TRIAL_OVER_TEXT_FONTSIZE, color=TRIAL_OVER_TEXT_COLOR, anchor_y=TRIAL_OVER_TEXT_Y_ANCHOR, bold=True)

    def initialize_viewer(self):
        if self.viewer is None:
            self.viewer = Viewer(VIEWPORT_W, VIEWPORT_H)
            self.viewer.set_bounds(0, VIEWPORT_W/SCALE, 0, VIEWPORT_H/SCALE)
            self.viewer.window.set_location(1650, 300)
            self.timer_thread.start()

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

    def render(self, mode='human'):
        self._render_start_end_text()
        #render location for turning
        self._render_turn_location()
        #render the goal position
        self._render_goal()
        #render bodies
        self._render_bodies()
        #draw robot direction indicator after the robot has been drawn.
        self._render_robot_direction_indicators()
        #render waypoints
        self._render_waypoints()
        #render path
        self._render_path()
        #render virtual mode display
        self._render_mode_display()
        #render dimension text
        self._render_mode_display_text()
        #render timer
        self._render_timer_text()
        #render assistance block label
        self._render_text('Assistance Type: '+self.assistance_type, LABEL_DISPLAY_POSITION, MODE_DISPLAY_TEXT_FONTSIZE+5)

        if self.current_time >= self.max_time: #TODO change this time limit to a param
            self._render_trial_over_text()

        return self.viewer.render(False)

    def _generate_path(self):
        '''
        function to generate the track boundaries.
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

        # embed(banner1='check waypoints')

    #Environment destructor and reset
    def _destroy(self):
        if self.robot is None: return
        self.world.DestroyBody(self.robot.robot)
        self.robot = None
        self.waypoints = np.zeros((self.num_locations, 2))

        # self.timer_thread.join()
        #TODO add all other initializations
        #destory time thread.

    def initialize(self):
        self.start_session = self.env_params['start']

        self.period = rospy.Duration(1.0)
        self.timer_thread = threading.Thread(target=self._render_timer, args=(self.period,))
        self.lock = threading.Lock()
        self.current_time = 0
        self.max_time = 50
        print 'START'


    def reset(self):
        self._destroy()

        self.num_turns = self.env_params['num_turns']
        assert self.num_turns > 0
        self.robot_position = self.env_params['robot_position']
        self.robot_orientation = self.env_params['robot_orientation']
        self.goal_position = self.env_params['goal_position']
        self.goal_orientation = self.env_params['goal_orientation']
        print 'goal_orientation', self.goal_orientation
        self.r_to_g_relative_orientation = self.env_params['r_to_g_relative_orientation'] #top right, top left, bottom, right, bottom left. relative position of the goal with respect to starting position of robot
        self.start_direction = self.env_params['start_direction']
        self.start_mode = self.env_params['start_mode']
        self.assistance_type = ASSISTANCE_CODE_NAME[self.env_params['assistance_type']] # label assistance type
        self.location_of_turn = self.env_params['location_of_turn'] #corresponds to the index in self.waypoints list
        self.num_locations = self.num_turns + 2 #num turns + start + end point

        self.LOCATIONS = ['p' +str(i) for i in range(self.num_locations)] #create location id. p0, p1, p2....pN
        self.ORIENTATIONS = [0, PI/2, -PI/2] #discrete set of rotations. 0 and pi/2
        self.DIMENSIONS = ['x', 'y', 't'] #set of dimensions or modes
        self.DIMENSION_INDICES = np.array([0,1,2]) #set of mode indices (needed for look up)
        self.STATES = [s for s in itertools.product(self.LOCATIONS, self.ORIENTATIONS, self.DIMENSIONS)] #list of all discrete states for the given configuration
        self.waypoints = np.zeros((self.num_locations, 2))
        self.path_points = np.zeros((self.num_locations, 2, 2)) #to draw the outline of the path
        self.current_discrete_state = (self.LOCATIONS[0], self.ORIENTATIONS[0], self.start_mode) #init current discrete state
        self.current_mode_index = DIM_TO_MODE_INDEX[self.start_mode]
        self.current_mode = self.start_mode
        self.mode_display_label = None
        #create and initialize the dictionary which contains info regarding which modes/dimension allow motion for each location
        self.MODES_MOTION_ALLOWED = collections.OrderedDict()
        self._init_modes_in_which_motion_allowed_dict()

        #create robot
        self.robot = RobotSE2(self.world, position=self.robot_position, orientation=self.robot_orientation, robot_color=ROBOT_COLOR_WHEN_COMMAND_REQUIRED, radius=ROBOT_RADIUS/SCALE)
        #create the continuous valued waypoints and the actual path boundaries
        self._generate_way_points()
        self._generate_path()

        #create STATE_TRANSITION_MODEL
        self.STATE_TRANSITION_MODEL = collections.OrderedDict()
        self._create_state_transition_model()
        self._init_state_transition_model()



        #create optimal next state and optimal next action dict
        self.OPTIMAL_NEXT_STATE_DICT = collections.OrderedDict()
        self.OPTIMAL_ACTION_DICT = collections.OrderedDict()
        self._create_optimal_next_state_dict()
        self._generate_optimal_control_dict()

        # print 'SAS', self.STATE_TRANSITION_MODEL
        # print '      '
        # print 'NEXT STATE', self.OPTIMAL_NEXT_STATE_DICT
        # print '       '
        # print 'NEXT OPITMAL', self.OPTIMAL_ACTION_DICT


        #create bidrectional mapping between discrete location ids and the waypoints
        self.LOCATIONS_TO_WAYPOINTS_DICT = collections.OrderedDict()
        for i in range(len(self.LOCATIONS)):
            self.LOCATIONS_TO_WAYPOINTS_DICT[self.LOCATIONS[i]] = tuple(self.waypoints[i])

        self.WAYPOINTS_TO_LOCATION_DICT = collections.OrderedDict()
        for i in range(len(self.LOCATIONS)):
            self.WAYPOINTS_TO_LOCATION_DICT[tuple(self.waypoints[i])] = self.LOCATIONS[i]


        self.ALLOWED_DIRECTIONS_OF_MOTION = collections.OrderedDict()
        self._init_allowed_directions_of_motion()
        # print '       '
        # print 'ALLOWED_DIRECTIONS_OF_MOTION', self.ALLOWED_DIRECTIONS_OF_MOTION
        # print '       '
        # print 'MODES_MOTION_ALLOWED', self.MODES_MOTION_ALLOWED
        # print '               '
        # print 'WP TO LOC', self.WAYPOINTS_TO_LOCATION_DICT
        # print '                 '
        # print ':LOC to WP', self.LOCATIONS_TO_WAYPOINTS_DICT

        if not self.service_initialized:
            rospy.Service('/mode_inference_env/get_optimal_action', OptimalAction, self.get_optimal_action)
            self.service_initialized = True

        self.period = rospy.Duration(1.0)
        self.timer_thread = threading.Thread(target=self._render_timer, args=(self.period,))
        self.lock = threading.Lock()
        self.current_time = 0
        self.max_time = 50
        print 'START'



    def step(self, input_action):
        assert 'human' in input_action.keys()
        # print rospy.get_param('current_discrete_state')
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
        self.current_mode_index = rospy.get_param('mode') #0,1,2 #get current mode index
        self.current_mode =  MODE_INDEX_TO_DIM[self.current_mode_index] #x,y,t, #get current mode
        self.current_discrete_state = (current_discrete_position, current_discrete_orientation, self.current_mode) #update the current discrete state.
        current_allowed_mode = self._retrieve_current_allowed_mode() #x,y,t #for the given location, retrieve what is the allowed mode of motion.
        current_allowed_mode_index = DIM_TO_MODE_INDEX[current_allowed_mode] #0,1,2 #get the mode index of the allowed mode of motion
        # print "current_state", self.current_discrete_state
        # print "current_allowed_mode_index", current_allowed_mode_index
        user_vel = np.array([input_action['human'].velocity.data[0], input_action['human'].velocity.data[1], input_action['human'].velocity.data[2]]) #numpyify the velocity data. note the negative sign on the 3rd component.To account for proper counterclockwise motion
        user_vel[np.setdiff1d(self.DIMENSION_INDICES, current_allowed_mode_index)] = 0.0 #zero out all velocities except the ones for the allowed mode

        #check if the direction of user_vel is correct as well. For each location in the allowed mode/dimension there is proper direction in which the velocity should be.
        current_allowed_direction_of_motion_in_allowed_mode = self._retrieve_allowed_direction_motion_in_allowed_mode()
        # print user_vel
        if get_sign_of_number(user_vel[current_allowed_mode_index]) != current_allowed_direction_of_motion_in_allowed_mode: #check if the direction velocity component in the allowed mode matches the allowed direction of motion in the allowed mode
            # print('NOT ALLOWED')
            user_vel[current_allowed_mode_index] = 0.0 #if not, zero the velocity out

        # print self.current_discrete_state, current_allowed_mode, user_vel
        rospy.set_param('current_discrete_state', self.current_discrete_state)
        self.robot.robot.linearVelocity = b2Vec2(user_vel[0], user_vel[1]) #update robot velocity
        self.robot.robot.angularVelocity = -user_vel[2]
        if self.current_time >= self.max_time:
            self.robot.robot.linearVelocity = b2Vec2(0.0, 0.0)
            self.robot.robot.angularVelocity = 0.0

        self.world.Step(1.0/FPS, VELOCITY_ITERATIONS, POSITION_ITERATIONS) #call box2D step function
        is_done = False
        if self.current_discrete_state[0] == self.LOCATIONS[-1]: #reached the last location
            is_done = True

        return self.robot.get_position(), self.robot.get_angle(), [user_vel[0], user_vel[1]], -user_vel[2], self.current_discrete_state, is_done #slightly hacky for velocities. TODO fix this. for some reason getters for velocities were not working properly
