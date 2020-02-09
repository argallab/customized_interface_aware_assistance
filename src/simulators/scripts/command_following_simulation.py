#!/usr/bin/env python

# This is a python script for measuring stochastic deviations of input commands from intended commands
# This was written for the paper titled "Do You Really Want To Do that?Customized Handling of Unintended Actions InAssistive Robots" to 
# model personalized distributions for p(u_i|u_m) from user data 

import rospy
import time
from sensor_msgs.msg import Joy
from envs.command_following_env import CommandFollowingEnv
import pickle
import os
import pyglet

class CommandFollowing(object):
	def __init__(self, duration=1, iterations=1, trial_info_dir_path=None):
    		
		# initializaiton 
		rospy.init_node("command_following")
		rospy.on_shutdown(self.shutdown_hook)

		self.initialize_subscribers()
		
		self.duration = duration 	# duration command text is displayed on screen
		self.iterations = iterations 	# number of iterations each command is to be displayed on screen 
	
		# TO DO: load list of text commands to be displayed 
		self.trial_info_dir_path = trial_info_dir_path
		if self.trial_info_dir_path is not None and os.path.exists(self.trial_info_dir_path):
			trial_info_filename = str(self.trial_index) + '.pkl'
			trial_info_filepath = os.path.join(self.trial_info_dir_path, trial_info_filename)
			with open(trial_info_filepath, 'rb') as fp:
				trial_info_dict = pickle.load(fp) #this dict could have other info related to autonomy params. We are only interested in the environment params for the time being

			assert 'env_params' in trial_info_dict
			self.env_params = trial_info_dict['env_params']
		else:
			self.env_params = dict()
			self.env_params['text_x_pos'] = VIEWPORT_W/2
			self.env_params['text_y_pos'] = VIEWPORT_H/4
			self.env_params['text'] = 'Hello, World!'
	
		self.env = CommandFollowingEnv(self.env_params)
		self.env.reset()
		self.env.render()

		self.input_command = {}
		# self.input_command['human'] = user_vel


		r = rospy.Rate(100)
		self.trial_start_time = time.time()

		# while not rospy.is_shutdown():
		# 	# self.env.step(self.input_command)
		# 	self.env.render()
		# 	r.sleep()

	def initialize_subscribers(self): 
		rospy.Subscriber('/joy', Joy, self.joy_callback)

	def joy_callback(self, msg):
		self.input_command = msg

	def shutdown_hook(self):
		pass

if __name__ == '__main__':
	CommandFollowing()
	rospy.spin()
