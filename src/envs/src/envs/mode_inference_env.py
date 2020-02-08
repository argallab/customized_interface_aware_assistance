from Box2D import (b2EdgeShape, b2FixtureDef, b2PolygonShape, b2Random, b2CircleShape, b2Vec2, b2Color)
import Box2D
from backends.rendering import Viewer, Transform
from utils import RobotSE2, FPS, VELOCITY_ITERATIONS, POSITION_ITERATIONS, SCALE, VIEWPORT_W, VIEWPORT_H
from utils import PI, ROBOT_COLOR_WHEN_MOVING, ROBOT_COLOR_WHEN_COMMAND_REQUIRED, ROBOT_RADIUS, GOAL_RADIUS, TURN_LOCATION_COLOR
from utils import WP_RADIUS, INFLATION_FACTOR, PATH_HALF_WIDTH, MODE_INDEX_TO_DIM, DIM_TO_MODE_INDEX
from utils import RGOrient, StartDirection, PositionOnLine
import csv
import math
import numpy as np
import collections
import itertools
import rospy
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


    def _check_continuous_position_on_line_joining_waypoints(self, start_position, end_position, current_position):
        '''
        code borrowed from https://kite.com/python/answers/how-to-determine-if-a-point-is-on-a-line-segment-in-python

        '''
        start_x, end_x, curr_x = start_position[0], end_position[0], current_position[0]
        start_y, end_y, curr_y = start_position[1], end_position[1], current_position[1]

        if not start_x == end_x: #slope is not infinite #parallel to y axis
            m = (end_y - start_y)/(end_x - start_x)
            curr_on = (curr_y - start_y) == m * (curr_x - start_x)
            curr_between = (min(start_x, end_x) <= curr_x <= max(start_x, end_x)) and (min(start_y, end_y) <= curr_y <= max(start_y, end_y))
            curr_on_and_between = curr_on and curr_between
        else:
            # print 'HERE'
            # print curr_x, end_x
            # print (start_y, curr_y, end_y)
            curr_on_and_between = (curr_x == end_x) and (start_y <= curr_y <= end_y)


        position_on_line = PositionOnLine.NOT_ON_LINE
        if curr_on_and_between:
            position_on_line = PositionOnLine.BETWEEN
            if abs(curr_y - end_y) < 0.1 and abs(curr_x - end_x) < 0.1: #maybe use eps
                position_on_line = PositionOnLine.END
            if abs(curr_y -start_y) < 0.1 and abs(curr_x - start_x) < 0.1:
                position_on_line = PositionOnLine.START

        return position_on_line

    def _transform_continuous_position_to_discrete_position(self):
        current_position = self.robot.get_position()
        current_orientation = self.robot.get_angle()
        start_index = int(self.current_discrete_state[0][-1])
        for (turn_id, p, pt) in zip(range(start_index, self.num_locations-1), self.waypoints[start_index:-1], self.waypoints[start_index+1:]):
            print "I", turn_id, self.location_of_turn
            if turn_id < self.location_of_turn or turn_id > self.location_of_turn:
                position_on_line =  self._check_continuous_position_on_line_joining_waypoints(p, pt, current_position)
                # assert position_on_line != PositionOnLine.NOT_ON_LINE
                if position_on_line == PositionOnLine.START or position_on_line == PositionOnLine.BETWEEN:
                    return self.WAYPOINTS_TO_LOCATION_DICT[tuple(p)], False
                elif position_on_line == PositionOnLine.END:
                    return self.WAYPOINTS_TO_LOCATION_DICT[tuple(pt)], True
            elif turn_id == self.location_of_turn:
                print current_orientation
                if abs(current_orientation - PI/2) > 0.1: #hasn't turned or is turning
                    return self.WAYPOINTS_TO_LOCATION_DICT[tuple(p)], False
                elif abs(current_orientation - PI/2) < 0.1: #finished truning
                    position_on_line =  self._check_continuous_position_on_line_joining_waypoints(p, pt, current_position)
                    if position_on_line == PositionOnLine.START or position_on_line == PositionOnLine.BETWEEN:
                        return self.WAYPOINTS_TO_LOCATION_DICT[tuple(p)], False
                    elif position_on_line == PositionOnLine.END:
                        return self.WAYPOINTS_TO_LOCATION_DICT[tuple(pt)], True

        return self.WAYPOINTS_TO_LOCATION_DICT[tuple(self.waypoints[-1])], True

    def _transform_continuous_orientation_to_discrete_orientation(self):
        current_orientation = self.robot.get_angle()
        assert current_orientation >= 0.0 and current_orientation <= (PI/2 + 0.1)
        if abs(current_orientation - PI/2) > 0.1:
            return 0.0, False
        else:
            return PI/2, True

    def _retrieve_current_allowed_mode(self):
        location = self.current_discrete_state[0]
        orientation = self.current_discrete_state[1]
        if location != self.LOCATIONS[self.location_of_turn]:
            return self.MODES_MOTION_ALLOWED[location][0]
        else:
            if orientation == 0.0:
                return self.MODES_MOTION_ALLOWED[location][1] #'t'
            else:
                return self.MODES_MOTION_ALLOWED[location][0] #'linear'

    def _retrieve_allowed_direction_motion_in_allowed_mode(self):
        location = self.current_discrete_state[0]
        orientation = self.current_discrete_state[1]
        if location != self.LOCATIONS[self.location_of_turn]:
            return self.ALLOWED_DIRECTIONS_OF_MOTION[location][0]
        else:
            if orientation == 0.0:
                return self.ALLOWED_DIRECTIONS_OF_MOTION[location][1]
            else:
                return self.ALLOWED_DIRECTIONS_OF_MOTION[location][0]


    def _render_goal(self):
        t = Transform(translation=(self.goal_position[0],self.goal_position[1]))
        self.viewer.draw_circle(GOAL_RADIUS/SCALE, 30, True, color=(0.53, 1.0, 0.42)).add_attr(t)
        self.viewer.draw_line(self.goal_position, (self.goal_position[0] + (GOAL_RADIUS/SCALE)*math.cos(self.goal_orientation), self.goal_position[1] + (GOAL_RADIUS/SCALE)*math.sin(self.goal_orientation)) )

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
        self.viewer.draw_line(ep_markers[0], ep_markers[1])

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
        self.viewer.draw_circle(4*WP_RADIUS/SCALE, 4, True, color=TURN_LOCATION_COLOR).add_attr(t) # TODO Look into how to properly render a box instead of a circle with 4 points!

    def render(self, mode='human'):
        if self.viewer is None:
            self.viewer = Viewer(VIEWPORT_W, VIEWPORT_H)
            self.viewer.set_bounds(0, VIEWPORT_W/SCALE, 0, VIEWPORT_H/SCALE)
            self.viewer.window.set_location(650, 300)

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

        return self.viewer.render(False)

    def _destroy(self):
        if self.robot is None: return
        self.world.DestroyBody(self.robot.robot)
        self.robot = None
        self.waypoints = np.zeros((self.num_turns+2, 2))

    def _init_modes_in_which_motion_allowed_dict(self):
        for i, s in enumerate(self.LOCATIONS[:-1]):
            if self.start_direction == StartDirection.X:
                if i % 2 == 0:
                    self.MODES_MOTION_ALLOWED[s] = ['x']
                else:
                    self.MODES_MOTION_ALLOWED[s] = ['y']
                if i == self.location_of_turn:
                    self.MODES_MOTION_ALLOWED[s].append('t')
            elif self.start_direction == StartDirection.Y:
                if i % 2 == 0:
                    self.MODES_MOTION_ALLOWED[s] = ['y']
                else:
                    self.MODES_MOTION_ALLOWED[s] = ['x']
                if i == self.location_of_turn:
                    self.MODES_MOTION_ALLOWED[s].append('t')

        self.MODES_MOTION_ALLOWED[self.LOCATIONS[-1]] = self.MODES_MOTION_ALLOWED[self.LOCATIONS[-2]][0]

    def _init_allowed_directions_of_motion(self):
        for i, (loc_p, loc_n) in enumerate(zip(self.LOCATIONS[:-1], self.LOCATIONS[1:])):
            diff_vector = np.array(self.LOCATIONS_TO_WAYPOINTS_DICT[loc_n]) - np.array(self.LOCATIONS_TO_WAYPOINTS_DICT[loc_p])
            motion_dimension_index = np.nonzero(diff_vector)[0]
            sign_of_motion = np.sign(diff_vector[motion_dimension_index])[0]
            self.ALLOWED_DIRECTIONS_OF_MOTION[loc_p] = [sign_of_motion]
            if i == self.location_of_turn:
                self.ALLOWED_DIRECTIONS_OF_MOTION[loc_p].append(1.0)

        self.ALLOWED_DIRECTIONS_OF_MOTION[self.LOCATIONS[-1]] = [self.ALLOWED_DIRECTIONS_OF_MOTION[self.LOCATIONS[-2]][0]]

    def reset(self):
        self._destroy()

        self.num_turns = self.env_params['num_turns']
        assert self.num_turns > 0
        self.robot_position = self.env_params['robot_position']
        self.robot_orientation = self.env_params['robot_orientation']
        self.goal_position = self.env_params['goal_position']
        self.goal_orientation = self.env_params['goal_orientation']
        self.r_to_g_relative_orientation = self.env_params['r_to_g_relative_orientation']
        self.start_direction = self.env_params['start_direction']
        self.start_mode = self.env_params['start_mode']
        self.location_of_turn = self.env_params['location_of_turn'] #corresponds to the index in self.waypoints list
        self.num_locations = self.num_turns + 2 #num turns + start + end point
        self.LOCATIONS = ['p' +str(i) for i in range(self.num_locations)]
        self.ORIENTATIONS = [0, PI/2]
        self.DIMENSIONS = ['x', 'y', 't']
        self.DIMENSION_INDICES = np.array([0,1,2])
        self.STATES = [s for s in itertools.product(self.LOCATIONS, self.ORIENTATIONS, self.DIMENSIONS)]
        self.waypoints = np.zeros((self.num_locations, 2))
        self.path_points = np.zeros((self.num_locations, 2, 2)) #to draw the outline of the path
        self.current_discrete_state = (self.LOCATIONS[0], self.ORIENTATIONS[0], self.start_mode)
        self.MODES_MOTION_ALLOWED = collections.OrderedDict()
        self._init_modes_in_which_motion_allowed_dict()
        self.robot = RobotSE2(self.world, position=self.robot_position, orientation=self.robot_orientation, robot_color=ROBOT_COLOR_WHEN_COMMAND_REQUIRED, radius=ROBOT_RADIUS/SCALE)
        self._generate_way_points()
        self._generate_path()
        self.LOCATIONS_TO_WAYPOINTS_DICT = collections.OrderedDict()
        for i in range(len(self.LOCATIONS)):
            self.LOCATIONS_TO_WAYPOINTS_DICT[self.LOCATIONS[i]] = tuple(self.waypoints[i])

        self.WAYPOINTS_TO_LOCATION_DICT = collections.OrderedDict()
        for i in range(len(self.LOCATIONS)):
            self.WAYPOINTS_TO_LOCATION_DICT[tuple(self.waypoints[i])] = self.LOCATIONS[i]

        self.ALLOWED_DIRECTIONS_OF_MOTION = collections.OrderedDict()
        self._init_allowed_directions_of_motion()

    def _get_sign_of_number(self, x):
        if int(x>=0):
            return 1.0
        else:
            return -1.0

    def _generate_path(self):
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

    def step(self, input_action):
        assert 'human' in input_action.keys()

        current_discrete_position, snap_or_not = self._transform_continuous_position_to_discrete_position()
        if snap_or_not:
            self.robot.set_position(self.LOCATIONS_TO_WAYPOINTS_DICT[current_discrete_position])
            current_discrete_position, snap_or_not = self._transform_continuous_position_to_discrete_position()

        current_discrete_orientation, snap_or_not = self._transform_continuous_orientation_to_discrete_orientation()
        if snap_or_not:
            self.robot.set_orientation(current_discrete_orientation) #is snap is true then this oreintation should be PI/2
            current_discrete_orientation, snap_or_not = self._transform_continuous_orientation_to_discrete_orientation()

        current_mode_index = rospy.get_param('mode') #0,1,2
        current_mode =  MODE_INDEX_TO_DIM[current_mode_index] #x,y,t
        self.current_discrete_state = (current_discrete_position, current_discrete_orientation, current_mode)
        current_allowed_mode = self._retrieve_current_allowed_mode() #x,y,t
        current_allowed_mode_index = DIM_TO_MODE_INDEX[current_allowed_mode] #0,1,2
        user_vel = np.array([input_action['human'].velocity.data[0], input_action['human'].velocity.data[1], -input_action['human'].velocity.data[2]]) #numpyify the velocity data
        user_vel[np.setdiff1d(self.DIMENSION_INDICES, current_allowed_mode_index)] = 0.0 #zero out all velocities except the ones for the allowed mode

        #check if the direction of user_vel is correct as well. For each location there is proper direction in which the velocity should be.
        current_allowed_direction_of_motion_in_allowed_mode = self._retrieve_allowed_direction_motion_in_allowed_mode()
        print current_allowed_direction_of_motion_in_allowed_mode, current_allowed_mode_index, type(current_allowed_mode_index)
        print self._get_sign_of_number(user_vel[current_allowed_mode_index])
        if self._get_sign_of_number(user_vel[current_allowed_mode_index]) != current_allowed_direction_of_motion_in_allowed_mode:
            user_vel[current_allowed_mode_index] = 0.0

        print self.current_discrete_state, current_allowed_mode, user_vel
        self.robot.robot.linearVelocity = b2Vec2(user_vel[0], user_vel[1])
        self.robot.robot.angularVelocity = user_vel[2]
        self.world.Step(1.0/FPS, VELOCITY_ITERATIONS, POSITION_ITERATIONS)
