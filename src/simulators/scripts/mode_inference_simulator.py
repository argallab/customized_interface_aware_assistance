#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from sensor_msgs.msg import Joy
from envs.mode_inference_env import ModeInferenceEnv
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from simulators.msg import CartVelCmd
from pyglet.window import key
import numpy as np
import pickle
import os
from utils import Robot2D, FPS, VELOCITY_ITERATIONS, POSITION_ITERATIONS, SCALE, VIEWPORT_W, VIEWPORT_H
from utils import ROBOT_RADIUS, GOAL_RADIUS, GOAL_SHAPES, GOAL_COLORS, PI, HUMAN_ROBOT_COLOR, AUTONOMY_ROBOT_COLOR, TRIANGLE_L
from utils import RGOrient, StartDirection

class Simulator(object):
	def __init__(self, dim=3, trial_index = 0, trial_info_dir_path=None):
		#TODO pass args as a dict
		super(Simulator, self).__init__()
		rospy.init_node("Simulator")
		rospy.on_shutdown(self.shutdown_hook)
		rospy.Subscriber('/joy', Joy, self.joy_callback)
		self.trial_index = trial_index
		self.dim = dim

		self.env_params = None
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
			self.env_params['num_turns'] = 3
			self.env_params['robot_position'] = ((3*VIEWPORT_W)/4/SCALE, (VIEWPORT_H)/4/SCALE)
			self.env_params['goal_position'] = (VIEWPORT_W/4/SCALE, (3*VIEWPORT_H)/4/SCALE)
			self.env_params['robot_orientation'] = 0.0
			self.env_params['goal_orientation'] = 0.0
			self.env_params['r_to_g_relative_orientation'] = RGOrient.TOP_LEFT
			self.env_params['start_direction'] = StartDirection.Y

		self.env = ModeInferenceEnv(self.env_params)
		self.env.reset()
		self.env.render()


		self.user_vel = CartVelCmd()
		_dim = [MultiArrayDimension()]
		_dim[0].label = 'cartesian_velocity'
		_dim[0].size = 2
		_dim[0].stride = 2
		self.user_vel.velocity.layout.dim = _dim
		self.user_vel.velocity.data = np.zeros(self.dim)
		self.user_vel.header.stamp = rospy.Time.now()
		self.user_vel.header.frame_id = 'human_control'

		self.input_action = {}
		self.input_action['human'] = self.user_vel

		if rospy.has_param('max_cart_vel'):
			self._max_cart_vel = np.array(rospy.get_param('max_cart_vel'))
		else:
			self._max_cart_vel = 10*np.ones(self.dim)
			rospy.logwarn('No rosparam for max_cart_vel found...Defaulting to max linear velocity of 50 cm/s and max rotational velocity of 50 degrees/s')

		r = rospy.Rate(100)
		self.trial_start_time = time.time()

		while not rospy.is_shutdown():
			self.env.step(self.input_action)
			self.env.render()
			r.sleep()

		isStart = True

	def joy_callback(self, msg):
		_axes = np.array(msg.axes)
		for i in range(self.dim):
			self.user_vel.velocity.data[i] = 0.0

		self.user_vel.velocity.data[0] = -_axes[0] * self._max_cart_vel[0]
		self.user_vel.velocity.data[1] = _axes[1] * self._max_cart_vel[1]
		self.user_vel.velocity.data[2] = _axes[2] * self._max_cart_vel[2] * 0.3
		self.user_vel.header.stamp = rospy.Time.now()
		self.input_action['human'] = self.user_vel

	def shutdown_hook(self):
		pass

if __name__ == '__main__':
	#TODO parse num turns from launch file

	Simulator()
	rospy.spin()
