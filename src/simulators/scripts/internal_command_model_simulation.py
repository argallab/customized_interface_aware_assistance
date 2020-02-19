#!/usr/bin/env python

# This is a python script for modeling the internal mapping of measured commands from intended action
# This was written for the paper titled "Do You Really Want To Do that?Customized Handling of Unintended Actions InAssistive Robots" to
# model personalized distributions for p(u_m|a) from user data

import rospy
from simulators.msg import Command
from envs.action_to_command_mapping_env import ActionEnv
import sys
from random import randrange
from pyglet.window import key

class ActionCommandModelling(object):
    def __init__(self, iterations=1, blocks=1):

        # initialization
        rospy.init_node("action_to_command_modelling")
        rospy.on_shutdown(self.shutdown_hook)

        # self.initialize_subscribers()

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

        self.env.initialize_viewer()
        self.env.viewer.window.on_key_press = self.key_press_callback

        while not rospy.is_shutdown(): 
            self.env.step()
            self.env.render()
            r.sleep()


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

    def key_press_callback(self, k, mode): 
        if k==key.S: 
            self.env.env_params['start_prompt'] = True
            print 'started key'

        if k==49: 
            self.env.env_params['next_prompt'] = True

        if k==50:
            self.env.env_params['next_prompt'] = True

        if k==51:
            self.env.env_params['next_prompt'] = True

        if k==52: 
            self.env.env_params['next_prompt'] = True


        self.env.reset()

    def shutdown_hook(self):
        pass


if __name__ == '__main__':
    ActionCommandModelling(sys.argv[1], sys.argv[2])
    rospy.spin()

