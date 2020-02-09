#!/usr/bin/env python

# This is a python script for measuring stochastic deviations of input commands from intended commands
# This was written for the paper titled "Do You Really Want To Do that?Customized Handling of Unintended Actions InAssistive Robots" to 
# model personalized distributions for p(u_i|u_m) from user data 

import rospy
import time
from sensor_msgs.msg import Joy
from envs.text_window_env import TextWindowEnv
import pickle
import os
import pyglet

class CommandFollowing(object):    
    def __init__(self, duration=1, iterations=1):
            
        rospy.init_node("command_following")	
        # r = rospy.Rate(100)

        self.duration = duration # duration command text is displayed on screen
        self.iterations = iterations # number of iterations each command is to be displayed on screen 

        self.env_params = None

        self.env_params = dict()
        self.env_params['text'] = 'Command Following'
        
        self.text_window = TextWindowEnv(self.env_params)
        	
        self.display_text()


    def display_text(self):
        
        self.text_window.reset()
        pyglet.app.run()





if __name__ == '__main__':
    CommandFollowing()
    rospy.spin()
