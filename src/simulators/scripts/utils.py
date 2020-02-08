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

HUMAN_ROBOT_COLOR = (0.0, 0.0, 0.0)
AUTONOMY_ROBOT_COLOR = (0.5, 0.5, 0.5)
TURN_LOCATION_COLOR = (173/255.0, 216/255.0, 230/255.0)

ROBOT_RADIUS = 10
GOAL_RADIUS = 10
TRIANGLE_L = 6
GOAL_COLORS = {'circle': (1.0, 0.0, 0.0), 'triangle': (0.0, 1.0, 0.0), 'rect': (0.0, 0.0, 1.0)}
GOAL_SHAPES = {0:'circle', 1:'triangle', 2:'rect'}

ROBOT_COLOR_WHEN_MOVING = (1.0, 105.0/255, 80/255.0)
ROBOT_COLOR_WHEN_COMMAND_REQUIRED = (1.0, 0.0, 0.0)
WP_RADIUS = 4
INFLATION_FACTOR = 1.2

PATH_HALF_WIDTH = INFLATION_FACTOR * ROBOT_RADIUS

DIM_TO_MODE_INDEX = {'x': 0, 'y':1, 't': 2}
MODE_INDEX_TO_DIM = {v:k for k,v in DIM_TO_MODE_INDEX.items()}

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
