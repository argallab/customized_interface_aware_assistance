#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from sensor_msgs.msg import Joy
from envs.simple_ha2d_env import SimpleHA2DEnv
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from teleop_nodes.msg import CartVelCmd
from pyglet.window import key
import numpy as np
import os
from utils import VIEWPORT_W, VIEWPORT_H, ROBOT_RADIUS, SCALE
import pickle

class Simulator(object):
	"""docstring for Simulator"""
	def __init__(self, dim, trial_index = 0, trial_info_dir_path=None):
		super(Simulator, self).__init__()
		rospy.init_node("Simulator")
		rospy.on_shutdown(self.shutdown_hook)
		# rospy.Subscriber('/joy', Joy, self.joy_callback)
		rospy.Subscriber('/user_vel', CartVelCmd, self.joy_callback)
		rospy.Subscriber('/autonomy_vel', CartVelCmd, self.autonomy_vel_callback)


		self.trial_index = trial_index

		self.env_params = None
		self.trial_info_dir_path = trial_info_dir_path
		#load up env params from the appropriate pickle file corresponding to trial_index
		if self.trial_info_dir_path is not None and os.path.exists(self.trial_info_dir_path):
			trial_info_filename = str(self.trial_index) + '.pkl'
			trial_info_filepath = os.path.join(self.trial_info_dir_path, trial_info_filename)
			with open(trial_info_filepath, 'rb') as fp:
				trial_info_dict = pickle.load(fp)

			assert 'env_params' in trial_info_dict
			self.env_params = trial_info_dict['env_params']
		else:
			self.env_params = dict()
			self.num_goals = 2
			self.env_params['num_goals'] = self.num_goals
			self.env_params['human_robot_position']= (VIEWPORT_W/SCALE/2 - VIEWPORT_W/SCALE/4, VIEWPORT_H/SCALE/2)
			self.env_params['autonomy_robot_position'] = (VIEWPORT_W/SCALE/2 + VIEWPORT_W/SCALE/4, VIEWPORT_H/SCALE/2)
			self.human_goal_positions = np.zeros((self.num_goals, 2))
			self.autonomy_goal_positions = np.zeros((self.num_goals, 2))
			for i in range(self.num_goals):
				x = ROBOT_RADIUS/SCALE + np.random.random()*(VIEWPORT_W/2/SCALE - ROBOT_RADIUS/SCALE)
				y = ROBOT_RADIUS/SCALE + np.random.random()*(VIEWPORT_H/SCALE - ROBOT_RADIUS/SCALE)
				self.human_goal_positions[i] = (x,y)

				x = ((VIEWPORT_W/2) + ROBOT_RADIUS)/SCALE + np.random.random()*(VIEWPORT_W/2/SCALE - ROBOT_RADIUS/SCALE)
				y = ROBOT_RADIUS/SCALE + np.random.random()*(VIEWPORT_H/SCALE - ROBOT_RADIUS/SCALE)
				self.autonomy_goal_positions[i] = (x,y)

			self.env_params['human_goal_positions'] = self.human_goal_positions
			self.env_params['autonomy_goal_positions'] = self.autonomy_goal_positions


		self.env = SimpleHA2DEnv(self.env_params)
		self.env.reset()
		self.env.render()
		self.dim = dim

		# self.user_vel = CartVelCmd()
		# _dim = [MultiArrayDimension()]
		# _dim[0].label = 'cartesian_velocity'
		# _dim[0].size = 2
		# _dim[0].stride = 2
		# self.user_vel.velocity.layout.dim = _dim
		# self.user_vel.velocity.data = np.zeros(self.dim)
		# self.user_vel.header.stamp = rospy.Time.now()
		# self.user_vel.header.frame_id = 'human_control'

		# self.autonomy_vel = CartVelCmd()
		# _dim = [MultiArrayDimension()]
		# _dim[0].label = 'cartesian_velocity'
		# _dim[0].size = 2
		# _dim[0].stride = 2
		# self.autonomy_vel.velocity.layout.dim = _dim
		# self.autonomy_vel.velocity.data = np.zeros(self.dim)
		# self.autonomy_vel.header.stamp = rospy.Time.now()
		# self.autonomy_vel.header.frame_id = 'autonomy_control'

		self.input_action = {}
		self.input_action['human'] = CartVelCmd()
		self.input_action['autonomy'] = CartVelCmd()

		# if rospy.has_param('max_cart_vel'):
		# 	self._max_cart_vel = np.array(rospy.get_param('max_cart_vel'))
		# else:
		# 	self._max_cart_vel = 10*np.ones(self.dim)
		# 	rospy.logwarn('No rosparam for max_cart_vel found...Defaulting to max linear velocity of 50 cm/s and max rotational velocity of 50 degrees/s')

		r = rospy.Rate(100)
		self.trial_start_time = time.time()

		while not rospy.is_shutdown():
			self.env.step(self.input_action)
			self.env.render()
			r.sleep()

		isStart = True


	def joy_callback(self, msg):
		# _axes = np.array(msg.axes)
		# for i in range(self.dim):
		# 	self.user_vel.velocity.data[i] = 0.0
		#
		# self.user_vel.velocity.data[0] = -_axes[0] * self._max_cart_vel[0]
		# self.user_vel.velocity.data[1] = _axes[1] * self._max_cart_vel[1]
		# self.user_vel.header.stamp = rospy.Time.now()
		self.input_action['human'] = msg

	def autonomy_vel_callback(self, msg):
		pass

	def shutdown_hook(self):
		pass

if __name__ == '__main__':
	#TODO parse arguments for num dimensions etc. To be included in launch file
	Simulator(2)
	rospy.spin()
