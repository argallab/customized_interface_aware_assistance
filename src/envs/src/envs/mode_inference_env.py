from Box2D import (b2EdgeShape, b2FixtureDef, b2PolygonShape, b2Random, b2CircleShape, b2Vec2, b2Color)
import Box2D

from backends.rendering import Viewer, Transform
from utils import RobotSE2, FPS, VELOCITY_ITERATIONS, POSITION_ITERATIONS, SCALE, VIEWPORT_W, VIEWPORT_H
from utils import PI, ROBOT_COLOR_WHEN_MOVING, ROBOT_COLOR_WHEN_COMMAND_REQUIRED, ROBOT_RADIUS, GOAL_RADIUS
from utils import WP_RADIUS, INFLATION_FACTOR, PATH_HALF_WIDTH
from utils import RGOrient, StartDirection
import csv
import math
import numpy as np

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


        self.num_turns = None
        self.robot = None
        self.waypoints = None
        self.path_points = None
        self.robot_position = None
        self.robot_orientation = None
        self.goal_position = None
        self.goal_orientation = None
        self.r_to_g_relative_orientation = None
        self.start_direction = None


    def render(self, mode='human'):
        if self.viewer is None:
            self.viewer = Viewer(VIEWPORT_W, VIEWPORT_H)
            self.viewer.set_bounds(0, VIEWPORT_W/SCALE, 0, VIEWPORT_H/SCALE)
            self.viewer.window.set_location(650, 300)

        #render the goal position
        t = Transform(translation=(self.goal_position[0],self.goal_position[1]))
        self.viewer.draw_circle(GOAL_RADIUS/SCALE, 30, True, color=(0.53, 1.0, 0.42)).add_attr(t)
        self.viewer.draw_line(self.goal_position, (self.goal_position[0] + (GOAL_RADIUS/SCALE)*math.cos(self.goal_orientation), self.goal_position[1] + (GOAL_RADIUS/SCALE)*math.sin(self.goal_orientation)) )

        #render bodies
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
                        self.viewer.draw_circle(f.shape.radius, 30, color=r.robot_color, filled=True).add_attr(t)
                    else:
                        self.viewer.draw_circle(f.shape.radius, 30, color=(1.0, 1.0, 1.0), filled=True).add_attr(t)
                elif type(f.shape) is b2EdgeShape:
                    self.viewer.draw_line(f.shape.vertices[0], f.shape.vertices[1])
                else:
                    path = [trans*v for v in f.shape.vertices]
                    self.viewer.draw_polygon(path, color=(1.0, 0.0, 0.0))
                    path.append(path[0])
                    self.viewer.draw_polyline(path, color=(1.0, 0.0, 0.0), linewidth=2)

        #draw robot direction indicator after the robot has been drawn.
        ep_markers = self.robot.get_direction_marker_end_points()
        self.viewer.draw_line(ep_markers[0], ep_markers[1])

        #render the waypoints
        for i in range(len(self.waypoints)):
            t =  Transform(translation=(self.waypoints[i][0], self.waypoints[i][1]))
            self.viewer.draw_circle(WP_RADIUS/SCALE, 30, True, color=(0,0,0)).add_attr(t)

        # for i in range(1, len(self.waypoints)):
        #     self.viewer.draw_line(tuple(self.waypoints[i-1]), tuple(self.waypoints[i]))

        #render path
        for i in range(1, len(self.path_points)):
            self.viewer.draw_line(tuple(self.path_points[i-1][0]), tuple(self.path_points[i][0])) #draw left edge
            self.viewer.draw_line(tuple(self.path_points[i-1][1]), tuple(self.path_points[i][1])) #draw right edge

        return self.viewer.render(False)


    def _destroy(self):
        if self.robot is None: return
        self.world.DestroyBody(self.robot.robot)
        self.robot = None
        self.waypoints = np.zeros((self.num_turns+2, 2))

    def reset(self):
        self._destroy()

        self.num_turns = self.env_params['num_turns']
        assert self.num_turns > 0
        self.waypoints = np.zeros((self.num_turns + 2, 2))
        self.path_points = np.zeros((self.num_turns + 2, 2, 2))

        self.robot_position = self.env_params['robot_position']
        self.robot_orientation = self.env_params['robot_orientation']
        self.goal_position = self.env_params['goal_position']
        self.goal_orientation = self.env_params['goal_orientation']
        self.r_to_g_relative_orientation = self.env_params['r_to_g_relative_orientation']
        self.start_direction = self.env_params['start_direction']

        self.robot = RobotSE2(self.world, position=self.robot_position, orientation=self.robot_orientation, robot_color=ROBOT_COLOR_WHEN_COMMAND_REQUIRED, radius=ROBOT_RADIUS/SCALE)
        self._generate_way_points()
        self._generate_path()

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
        self.robot.robot.linearVelocity = b2Vec2(input_action['human'].velocity.data[0], input_action['human'].velocity.data[1])
        self.robot.robot.angularVelocity = input_action['human'].velocity.data[2]
        self.world.Step(1.0/FPS, VELOCITY_ITERATIONS, POSITION_ITERATIONS)
