from Box2D import b2CircleShape, b2EdgeShape
import math
from enum import Enum

#TODO move defines to a separate file
FPS = 100
VELOCITY_ITERATIONS = 180
POSITION_ITERATIONS = 60
SCALE = 20.0

VIEWPORT_W = 600
PI = math.pi
VIEWPORT_H = 400

#COLORS
HUMAN_ROBOT_COLOR = (0.0, 0.0, 0.0)
AUTONOMY_ROBOT_COLOR = (0.5, 0.5, 0.5)
TURN_LOCATION_COLOR = (173/255.0, 216/255.0, 230/255.0)
ACTIVE_MODE_COLOR = (0, 1.0, 0.0)
NONACTIVE_MODE_COLOR = (0.8,0.8, 0.8)
MODE_DISPLAY_TEXT_COLOR = (0,0,0,255)
ROBOT_COLOR_WHEN_MOVING = (1.0, 105.0/255, 80/255.0)
ROBOT_COLOR_WHEN_COMMAND_REQUIRED = (1.0, 0.0, 0.0)

#GEOMETRY
MODE_DISPLAY_RADIUS = 15
ROBOT_RADIUS = 10
GOAL_RADIUS = 10
TRIANGLE_L = 6
GOAL_COLORS = {'circle': (1.0, 0.0, 0.0), 'triangle': (0.0, 1.0, 0.0), 'rect': (0.0, 0.0, 1.0)}
GOAL_SHAPES = {0:'circle', 1:'triangle', 2:'rect'}
WP_RADIUS = 4
INFLATION_FACTOR = 1.2
PATH_HALF_WIDTH = INFLATION_FACTOR * ROBOT_RADIUS

VIEWPORT_WS = VIEWPORT_W/SCALE
VIEWPORT_HS = VIEWPORT_H/SCALE
ROBOT_RADIUS_S = ROBOT_RADIUS/SCALE
GOAL_RADIUS_S = GOAL_RADIUS/SCALE


MODE_DISPLAY_CIRCLE_START_POSITION_S = [(3*VIEWPORT_WS)/4 +ROBOT_RADIUS_S, VIEWPORT_HS - 2*ROBOT_RADIUS_S]
MODE_DISPLAY_CIRCLE_X_OFFSET_S = 4*ROBOT_RADIUS_S

MODE_DISPLAY_TEXT_START_POSITION = [(3*VIEWPORT_W)/4 +ROBOT_RADIUS, VIEWPORT_H - 3.5*ROBOT_RADIUS]
MODE_DISPLAY_TEXT_X_OFFSET = 4*ROBOT_RADIUS
MODE_DISPLAY_TEXT_FONTSIZE = 14

#DICTIONARIES
DIM_TO_MODE_INDEX = {'x': 0, 'y':1, 't': 2}
MODE_INDEX_TO_DIM = {v:k for k,v in DIM_TO_MODE_INDEX.items()}
#low level commands issued by the snp interface. hp = hard puff, hs= hard sip, sp = soft puff, ss = soft sip. Also the domain for ui and um
LOW_LEVEL_COMMANDS = ['hp', 'hs', 'sp', 'ss']
#high level actions, move_p = move in positive direction, move_n = move in negative direction, mode_r = switch mode to right, mode_l = switch mode to left. positive and negative is conditioned on mode
HIGH_LEVEL_ACTIONS = ['move_p', 'move_n', 'mode_r', 'mode_l']
#true mapping of a to u
TRUE_ACTION_TO_COMMAND = collections.OrderedDict({'move_p': 'sp', 'move_n':'ss', 'mode_r':'hp', 'mode_l': 'hs'})
#true inverse mapping of u to a
TRUE_COMMAND_TO_ACTION = collections.OrderedDict({v:k for k, v in TRUE_ACTION_TO_COMMAND.items()})
#transition function for mode switches.
MODE_SWITCH_TRANSITION = {'x': {'hp': 'y', 'hs': 't', 'sp': 'x', 'ss': 'x'},
						  'y': {'hp': 't', 'hs': 'x', 'sp': 'y', 'ss': 'y'},
						  't': {'hp': 'x', 'hs': 'y', 'sp': 't', 'ss': 't'}}
#Depending on the configuration of the initial robot position and goal position, the motion commands will result in either moving towards the
#next location or the previous location
TRANSITION_FOR_ACTION =   {'tr': {'sp': {'x': 'next', 'y': 'next', 't': 'next'}, 'ss': {'x': 'prev', 'y': 'prev', 't': 'prev'}},
    					   'tl': {'sp': {'x': 'prev', 'y': 'next', 't': 'next'}, 'ss': {'x': 'next', 'y': 'prev', 't': 'prev'}},
    					   'br': {'sp': {'x': 'next', 'y': 'prev', 't': 'next'}, 'ss': {'x': 'prev', 'y': 'next', 't': 'prev'}},
    					   'bl': {'sp': {'x': 'prev', 'y': 'prev', 't': 'next'}, 'ss': {'x': 'next', 'y': 'next', 't': 'prev'}}
    						}
#Enum defintions
class PositionOnLine(Enum):
    START = 0
    BETWEEN = 1
    END = 2
    NOT_ON_LINE = 3

class StartDirection(Enum):
    X = 0
    Y = 1
    
class RGOrient(Enum):
    '''
    Relative position of the goal with respect to the robot. This information is used for constructing properly shared paths in ModeInferenceEnv
    '''
    TOP_RIGHT = 0
    TOP_LEFT = 1
    BOTTOM_LEFT = 2
    BOTTOM_RIGHT = 3

#utility functions

def get_sign_of_number(x):
    '''
    Utility function for getting the sign of a scalar. +1 for positive, -1 for negative
    '''
    if int(x>=0):
        return 1.0
    else:
        return -1.0


#class definitions
class Robot2D(object):
    def __init__(self, world, position, robot_color=(1.0, 0.0, 0.0), radius=3, type='kinematic'):
        self.robot = world.CreateKinematicBody(position=position,shapes=[b2CircleShape(pos=position, radius=radius)])
        self.robot_color = robot_color

class RobotSE2(object):
    def __init__(self, world, position, orientation, robot_color=(1.0, 0.0, 0.0), radius=3, type='kinematic'):
        # self.direction_vec = world.CreateKinematicBody(position=position, shape=[b2EdgeShape(vertices=[position, (radius*math.cos(orientation), radius*math.sin(orientation))])])
        self.robot = world.CreateKinematicBody(position=position, angle=orientation, shapes=[b2CircleShape(pos=position, radius=radius)])
        self.robot_color = robot_color
        self.radius = radius

    def set_robot_color(self, robot_color):
        self.robot_color = robot_color

    #GETTERS
    def get_direction_marker_end_points(self):
        return (self.robot.position[0], self.robot.position[1]), (self.robot.position[0] + self.radius*math.cos(self.robot.angle), self.robot.position[1] + self.radius*math.sin(self.robot.angle))

    def get_position(self):
        return self.robot.position

    def get_angle(self):
        return self.robot.angle

    def set_position(self, position):
        self.robot.position = position

    def set_orientation(self, orientation):
        self.robot.angle = orientation

class Goal(object):
    """docstring forGoal."""
    def __init__(self, arg):
        pass
