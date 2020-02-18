#!/usr/bin/env python

# This is a python script for measuring stochastic deviations of input commands from intended commands
# This was written for the paper titled "Do You Really Want To Do that?Customized Handling of Unintended Actions InAssistive Robots" to
# model personalized distributions for p(u_i|u_m) from user data

import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from simulators.msg import Command
from envs.sprite_test import ActionEnv
from utils import LOW_LEVEL_COMMANDS, EXPERIMENT_START_COUNTDOWN
import pyglet
import sys
from random import randrange
import threading


class CommandFollowing(object):
    def __init__(self):

        # initialization
        rospy.init_node("command_following")

        self.env = ActionEnv()


    def shutdown_hook(self):
        pass


if __name__ == '__main__':
    CommandFollowing()
    rospy.spin()

