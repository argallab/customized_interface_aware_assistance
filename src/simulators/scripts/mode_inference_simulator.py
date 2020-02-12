#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from sensor_msgs.msg import Joy
from envs.mode_inference_env import ModeInferenceEnv
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from teleop_nodes.msg import CartVelCmd
from teleop_nodes.srv import SetMode, SetModeRequest, SetModeResponse
from pyglet.window import key
import numpy as np
import pickle
import os
from utils import Robot2D, FPS, VELOCITY_ITERATIONS, POSITION_ITERATIONS, SCALE, VIEWPORT_W, VIEWPORT_H, DIM_TO_MODE_INDEX
from utils import ROBOT_RADIUS, GOAL_RADIUS, GOAL_SHAPES, GOAL_COLORS, PI, HUMAN_ROBOT_COLOR, AUTONOMY_ROBOT_COLOR, TRIANGLE_L
from utils import RGOrient, StartDirection, AssistanceType
from IPython import embed

class Simulator(object):
	def __init__(self, dim=3, trial_index = 22, trial_info_dir_path=None):
		#TODO pass args as a dict
		super(Simulator, self).__init__()
		rospy.init_node("Simulator")
		rospy.on_shutdown(self.shutdown_hook)
		# rospy.Subscriber('/joy', Joy, self.joy_callback)
		self.dim = dim
		user_vel = CartVelCmd()
		_dim = [MultiArrayDimension()]
		_dim[0].label = 'cartesian_velocity'
		_dim[0].size = dim
		_dim[0].stride = dim
		user_vel.velocity.layout.dim = _dim
		user_vel.velocity.data = np.zeros(self.dim)
		user_vel.header.stamp = rospy.Time.now()
		user_vel.header.frame_id = 'human_control'


		self.input_action = {}
		self.input_action['human'] = user_vel

		rospy.Subscriber('/user_vel', CartVelCmd, self.joy_callback)
		self.trial_index = trial_index

		# self.input_action_initialized = False

		self.env_params = None
		self.trial_info_dir_path = os.path.join(os.path.dirname(__file__), 'trial_dir')


		if self.trial_info_dir_path is not None and os.path.exists(self.trial_info_dir_path):
			print ('IN HERE')
			trial_info_filename = str(self.trial_index) + '.pkl'
			trial_info_filepath = os.path.join(self.trial_info_dir_path, trial_info_filename)
			assert os.path.exists(trial_info_filepath) is not None
			with open(trial_info_filepath, 'rb') as fp:
				trial_info_dict = pickle.load(fp) #this dict could have other info related to autonomy params. We are only interested in the environment params for the time being

			assert 'env_params' in trial_info_dict
			self.env_params = trial_info_dict['env_params']
			print "ENV PARAMS", self.env_params
		else:
			self.env_params = dict()
			self.env_params['num_turns'] = 1
			self.env_params['robot_position'] = ((VIEWPORT_W)/4/SCALE, (3*VIEWPORT_H)/4/SCALE)
			self.env_params['goal_position'] = ((3*VIEWPORT_W)/4/SCALE, (VIEWPORT_H)/4/SCALE)
			self.env_params['robot_orientation'] = 0.0
			self.env_params['goal_orientation'] = PI/2
			self.env_params['r_to_g_relative_orientation'] = RGOrient.BOTTOM_RIGHT
			self.env_params['start_direction'] = StartDirection.Y
			self.env_params['start_mode'] = 't'
			self.env_params['location_of_turn'] = 1
			self.env_params['assistance_type'] = 0
			assert self.env_params['location_of_turn'] > 0 and self.env_params['location_of_turn'] <= self.env_params['num_turns'] #can't be the first or last location

		rospy.set_param('assistance_type', self.env_params['assistance_type'])

		rospy.loginfo("Waiting for teleop_node ")
		rospy.wait_for_service("/teleop_node/set_mode")
		rospy.loginfo("teleop_node node service found! ")


		#set starting mode for the trial
		self.set_mode_srv = rospy.ServiceProxy('/teleop_node/set_mode', SetMode)
		self.set_mode_request = SetModeRequest()
		self.set_mode_request.mode_index = DIM_TO_MODE_INDEX[self.env_params['start_mode']]

		status = self.set_mode_srv(self.set_mode_request)

		self.env = ModeInferenceEnv(self.env_params)
		self.env.reset()
		self.env.render()

		r = rospy.Rate(100)
		self.trial_start_time = time.time()

		while not rospy.is_shutdown():
			self.env.step(self.input_action)
			self.env.render()
			r.sleep()

		isStart = True

	def joy_callback(self, msg):
		self.input_action['human'] = msg

	def shutdown_hook(self):
		pass

if __name__ == '__main__':
	#TODO parse num turns from launch file

	Simulator()
	rospy.spin()
