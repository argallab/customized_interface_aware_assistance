#!/usr/bin/env python

# This is a python script for modeling the internal mapping of measured commands from intended action
# This was written for the paper titled "Do You Really Want To Do that?Customized Handling of Unintended Actions InAssistive Robots" to
# model personalized distributions for p(u_m|a) from user data

import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from simulators.msg import Command
from envs.action_to_command_mapping_env import ActionEnv
from utils import LOW_LEVEL_COMMANDS, EXPERIMENT_START_COUNTDOWN
import pyglet
import sys
from random import randrange
import threading




class ActionCommandModelling(object):
    def __init__(self, iterations=1, blocks=1):

        # initialization
        rospy.init_node("action_to_command_modelling")
        rospy.on_shutdown(self.shutdown_hook)

        self.initialize_subscribers()

        self.iterations = iterations
        self.blocks = blocks

        self.action_list = []

        # To do: change to input argumen using argparse
        file_dir = '/home/corrective_mode_switch_assistance/src/envs/sprites/actions'
        self.actions= ['clockwise', 'counterclockwise', 'down', 'up', 'left', 'right',
                        'mode_switch_right_1', 'mode_switch_right_2', 'mode_switch_right_3',
                        'mode_switch_left_1', 'mode_switch_left_2', 'mode_switch_left_3']

        env_params = dict()
        env_params['file_dir'] = file_dir
        env_params['img_prompt'] = ''

        self.env = ActionEnv(env_params)
        self.env.initialize_publishers('action_prompt')

        self.generate_action_list()

        r = rospy.Rate(100)

        while not rospy.is_shutdown(): 
            self.env.step()
            self.env.render()
            r.sleep()

    def initialize_subscribers(self):
        rospy.Subscriber('/keyboard_entry', String, self.keyboard_callback)

    # start experiment
    def keyboard_callback(self, msg): 
        # Start experiment 
        if msg.data == 's':            
            self.env.env_params['start_prompt'] = True        
            self.env.reset()

        # # User input used to turn on radio buttons
        # if msg.data == '1' or msg.data == '2' or msg.data == '3' or msg.data == '4': 
        #     self.env.env_params['user_input'] = msg.data 

    # randomize actions
    def generate_action_list(self): 
        for i in range(len(self.iterations)): 
            actions = self.actions[:]
            for j in range(len(actions)): 
                rand_ind = randrange(len(actions))
                self.action_list.append(actions[rand_ind])
                actions.pop(rand_ind)
        self.env.env_params['action_prompts'] = self.action_list[:]
        self.env.reset()

    def shutdown_hook(self):
        pass


if __name__ == '__main__':
    ActionCommandModelling(sys.argv[1], sys.argv[2])
    rospy.spin()

