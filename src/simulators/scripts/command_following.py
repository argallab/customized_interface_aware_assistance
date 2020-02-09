#!/usr/bin/env python

# This is a python script for measuring stochastic deviations of input commands from intended commands
# This was written for the paper titled "Do You Really Want To Do that?Customized Handling of Unintended Actions InAssistive Robots" to 
# model personalized distributions for p(u_i|u_m) from user data 

import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from envs.text_window_env import TextWindowEnv
from utils import LOW_LEVEL_CONTROL_COMMANDS, EXPERIMENT_START_COUNTDOWN
import pyglet
import sys 
from random import randrange

class CommandFollowing(object):    
    def __init__(self, duration=1.0, iterations=1):
            
        rospy.init_node("command_following")	
        self.initialize_subscribers()

        self.duration = float(duration) # duration command text is displayed on screen
        self.iterations = int(iterations) # number of iterations each command is to be displayed on screen 

        self.isStart = False # boolean experiment start 

        self.env_params = None

        self.env_params = dict()
        self.env_params['text'] = 'Command Following'
        
        self.env = TextWindowEnv(self.env_params)
        self.env.reset()


    def initialize_subscribers(self): 
        rospy.Subscriber('/keyboard_entry', String, self.keyboard_callback)

    # start experiment
    def keyboard_callback(self, msg): 
        # Start experiment 
        if msg.data == 's':            
            for i in range(len(EXPERIMENT_START_COUNTDOWN)): 
                self.call_render(EXPERIMENT_START_COUNTDOWN[i], self.duration)
            self.command_following()


    # randomize commands and display for desired duration and iterations
    def command_following(self): 
        for i in range(self.iterations): 
            commands = LOW_LEVEL_CONTROL_COMMANDS[:]
            for j in range(len(commands)): 
                rand_index = randrange(len(commands))
                self.call_render(LOW_LEVEL_CONTROL_COMMANDS[rand_index], self.duration)
                commands.pop(rand_index)
        self.call_render('ALL DONE :D', self.duration)

    # set new text message and render            
    def call_render(self, msg, duration): 
            self.env_params['text'] = msg
            self.env.reset()
            self.env.render()
            rospy.sleep(duration)


    def shutdown_hook(self):
        pass


if __name__ == '__main__':
    CommandFollowing(sys.argv[1], sys.argv[2])
    rospy.spin()
