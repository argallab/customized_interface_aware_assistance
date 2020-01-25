import Box2D
from Box2D import (b2EdgeShape, b2FixtureDef, b2PolygonShape, b2Random, b2CircleShape, b2Vec2, b2Color)

from backends.rendering import Viewer, Transform
from utils import Robot2D, FPS, VELOCITY_ITERATIONS, POSITION_ITERATIONS, SCALE, VIEWPORT_W, VIEWPORT_H
from utils import ROBOT_RADIUS, GOAL_RADIUS, GOAL_SHAPES, GOAL_COLORS, PI, HUMAN_ROBOT_COLOR, AUTONOMY_ROBOT_COLOR, TRIANGLE_L
import numpy as np
import os
import math


class SimpleHA2DEnv(object): #TODO: potential change the parent class to an abstract env class from which different environments will be derived
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second' : FPS
    }
    def __init__(self, env_params):
        # assert os.path.exists(trial_info_dir_path) #this directory contains different files of the name format trial_index.pkl
        #each trial_index.pkl contains all info needed for setting up the environment. The info will be stored as a dictionary.
        #

        self.viewer = None

        self.world = Box2D.b2World()
        self.world.gravity = (0,0)
        #load env specific params
        self.autonomy_robot = None
        self.human_robot = None
        self.partition = None

        self.human_robot_position = None
        self.autonomy_robot_position = None
        self.human_goal_positions = None
        self.autonomy_goal_positions = None
        self.robot_list = []

        self.env_params = env_params
        assert self.env_params is not None
        assert 'num_goals' in self.env_params
        assert 'human_robot_position' in self.env_params
        assert 'autonomy_robot_position' in self.env_params
        assert 'human_goal_positions' in self.env_params
        assert 'autonomy_goal_positions' in self.env_params


    def _check_ar_bounds(self):
        #check autonomy_robot bounds
        if (self.autonomy_robot.robot.position.x < VIEWPORT_W/SCALE/2 + ROBOT_RADIUS/SCALE and self.autonomy_robot.robot.linearVelocity.x < 0.0) or (self.autonomy_robot.robot.position.x > VIEWPORT_W/SCALE - ROBOT_RADIUS/SCALE and self.autonomy_robot.robot.linearVelocity.x > 0.0):
            self.autonomy_robot.robot.linearVelocity.x = 0.0
        if (self.autonomy_robot.robot.position.y < ROBOT_RADIUS/SCALE and self.autonomy_robot.robot.linearVelocity.y) < 0.0 or (self.autonomy_robot.robot.position.y > VIEWPORT_H/SCALE - ROBOT_RADIUS/SCALE and self.autonomy_robot.robot.linearVelocity.y > 0.0):
            self.autonomy_robot.robot.linearVelocity.y = 0.0

    def _check_hr_bounds(self):
        if (self.human_robot.robot.position.x < ROBOT_RADIUS/SCALE and self.human_robot.robot.linearVelocity.x < 0.0) or (self.human_robot.robot.position.x > VIEWPORT_W/SCALE/2 - ROBOT_RADIUS/SCALE and self.human_robot.robot.linearVelocity.x > 0.0):
            self.human_robot.robot.linearVelocity.x = 0.0
        if (self.human_robot.robot.position.y < ROBOT_RADIUS/SCALE and self.human_robot.robot.linearVelocity.y) < 0.0 or (self.human_robot.robot.position.y > VIEWPORT_H/SCALE - ROBOT_RADIUS/SCALE and self.human_robot.robot.linearVelocity.y > 0.0):
            self.human_robot.robot.linearVelocity.y = 0.0

    def _render_goal(self, shape, goal_color, goal_position): #potentially add default values for these args
        if shape == 'circle':
            t = Transform(translation=goal_position)
            self.viewer.draw_circle(GOAL_RADIUS/SCALE, 30, color=goal_color, filled=True).add_attr(t)
        elif shape == 'triangle':
            triangle_points = []
            for i in range(3):
                p = (goal_position[0] + TRIANGLE_L/SCALE*math.cos(i*(2*PI/3)), goal_position[1] + TRIANGLE_L/SCALE*math.sin(i*(2*PI/3)) )
                triangle_points.append(p)
            self.viewer.draw_polygon(triangle_points, color=goal_color)
        elif shape == 'rect':
            pass
        elif shape == 'square':
            pass

    def render(self, mode='human'):
        if self.viewer is None:
            self.viewer = Viewer(VIEWPORT_W, VIEWPORT_H)
            self.viewer.set_bounds(0, VIEWPORT_W/SCALE, 0, VIEWPORT_H/SCALE)
            self.viewer.window.set_location(650, 300)

        #Add rendering of the goals
        for r in self.robot_list + [self.partition]:
            if isinstance(r, Robot2D):
                robot = r.robot
            else:
                robot = r
            for f in robot.fixtures:
                trans = f.body.transform
                if type(f.shape) is b2CircleShape:
                    t = Transform(translation=trans.position)
                    if isinstance(r, Robot2D):
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

        #Goal rendering
        for i in range(len(self.human_goal_positions)):
            shape = GOAL_SHAPES[i]
            goal_color = GOAL_COLORS[shape]
            goal_position = tuple(self.human_goal_positions[i])
            self._render_goal(shape, goal_color, goal_position)
            goal_position = tuple(self.autonomy_goal_positions[i])
            self._render_goal(shape, goal_color, goal_position)

        return self.viewer.render(False)

    def _destroy(self):
        if not self.partition: return
        self.world.contactListener = None
        self.world.DestroyBody(self.partition)
        self.partition = None
        for r in self.robot_list:
            self.world.DestroyBody(r.robot)

        self.robot_list = []

        self.human_robot = None
        self.autonomy_robot = None
        self.human_robot_position = None
        self.autonomy_robot_position = None
        self.human_goal_positions = None
        self.autonomy_goal_positions = None

    def reset(self):
        self._destroy()
        self.num_goals = self.env_params['num_goals']
        self.autonomy_robot_position = self.env_params['autonomy_robot_position']
        self.human_robot_position = self.env_params['human_robot_position']

        self.autonomy_goal_positions = self.env_params['autonomy_goal_positions']
        self.human_goal_positions = self.env_params['human_goal_positions']

        self.partition = self.world.CreateStaticBody(
            position = (VIEWPORT_W/SCALE/2, VIEWPORT_H/SCALE/2),
            shapes = [b2EdgeShape(vertices=[(VIEWPORT_W/SCALE/2, 0.0), (VIEWPORT_W/SCALE/2, VIEWPORT_H/SCALE)])]
        )
        self.human_robot = Robot2D(self.world, position=(VIEWPORT_W/SCALE/2 - VIEWPORT_W/SCALE/4, VIEWPORT_H/SCALE/2), radius=ROBOT_RADIUS/SCALE, robot_color=HUMAN_ROBOT_COLOR)
        self.robot_list.append(self.human_robot)
        self.autonomy_robot = Robot2D(self.world, position=(VIEWPORT_W/SCALE/2 + VIEWPORT_W/SCALE/4, VIEWPORT_H/SCALE/2), radius=ROBOT_RADIUS/SCALE, robot_color=AUTONOMY_ROBOT_COLOR)
        self.robot_list.append(self.autonomy_robot)


    def step(self, input_action):
        '''
        input_action: dict containing all necessary control actions.
        '''
        assert 'human' in input_action.keys()
        assert 'autonomy' in input_action.keys()
        self.human_robot.robot.linearVelocity = b2Vec2(input_action['human'].velocity.data[0], input_action['human'].velocity.data[1])
        self.autonomy_robot.robot.linearVelocity = b2Vec2(input_action['autonomy'].velocity.data[0], input_action['autonomy'].velocity.data[1])
        self._check_ar_bounds()
        self._check_hr_bounds()
        self.world.Step(1.0/FPS, VELOCITY_ITERATIONS, POSITION_ITERATIONS)
