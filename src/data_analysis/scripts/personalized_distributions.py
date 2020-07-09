#!/usr/bin/env python

# Code developed by Deepak Gopinath*, Mahdieh Nejati Javaremi* in February 2020. Copyright (c) 2020. Deepak Gopinath, Mahdieh Nejati Javaremi, Argallab. (*) Equal contribution

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import pandas as pd
import numpy  as np
import pickle
import sys
import os
import argparse
import rospkg
import collections

class PersonalizedDistributions(object):
	def __init__(self, args):

		# load p_um_given_a and p_um_given_ui
		self.id = args.id
		self.distribution = args.dist
		self.save_fig = args.save

		self.P_UM_GIVEN_A = None
		self.P_UM_GIVEN_UI = None
		self.P_UI_GIVEN_A = None

		self.personalized_distributions_dir = os.path.join(rospkg.RosPack().get_path('inference_and_correction'), 'personalized_distributions')

		for dist in self.distribution:
			assert os.path.exists(os.path.join(self.personalized_distributions_dir, str(self.id)+str(dist)+'.pkl'))
			print('LOADING PERSONALIZED ', dist)
			with open(os.path.join(self.personalized_distributions_dir, str(self.id)+str(dist)+'.pkl'), 'rb') as fp:
				if dist == '_p_um_given_a':
					self.P_UM_GIVEN_A  = pickle.load(fp) #assumes that the conditional probability distribution is stored as a collections.OrderedDict conditioned on the mode
					# self._analyze_p_um_given_a()
				elif dist == '_p_um_given_ui':
					self.P_UM_GIVEN_UI  = pickle.load(fp)
					# self._analyze_p_um_given_ui()
				elif dist == '_p_ui_given_a':
					self.P_UI_GIVEN_A  = pickle.load(fp)
					# self._analyze_p_ui_given_a()

		for dist in self.distribution:
			if dist == '_p_um_given_a':
				self._analyze_p_um_given_a()
			elif dist == '_p_um_given_ui':
				self._analyze_p_um_given_ui()
			elif dist == '_p_ui_given_a':
				self._analyze_p_ui_given_a()

	def _analyze_p_um_given_ui(self):

		# for dist in self.distribution:
		#create a new directory
		folder = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir)), '/plots/')

		for i, key in enumerate(self.P_UM_GIVEN_UI.keys()):
			plt.subplot(len(self.P_UM_GIVEN_UI.keys()), 1, i+1)
			plt.bar(range(len(self.P_UM_GIVEN_UI[key])), list(self.P_UM_GIVEN_UI[key].values()), align='center')
			plt.xticks(range(len(self.P_UM_GIVEN_UI[key])), list(self.P_UM_GIVEN_UI[key].keys()))
			plt.ylim(0, 1)
			plt.ylabel('probability')
			plt.title('UI='+key)
		plt.suptitle('Personalized P(UM|UI)')
		plt.show()
		if self.save_fig:
			try:	#else already exists
				os.makedirs(folder)
			except:
				pass
			filename = folder + self.id  + '_p_um_given_ui.png'
			print filename
			plt.savefig(filename)


	def _analyze_p_um_given_a(self):

		fig = plt.figure()
		#gridspec inside gridspec

		outer_grid = gridspec.GridSpec(3, 4, wspace=0.25, hspace=.25)

		for i in range(12):
			inner_grid = gridspec.GridSpecFromSubplotSpec(4, 1, subplot_spec=outer_grid[i], wspace=0.0, hspace=0.25)
			for mode in self.P_UM_GIVEN_A.keys():
				for action in self.P_UM_GIVEN_A[mode].keys():
					for j, command in enumerate(self.P_UM_GIVEN_A[mode][action].keys()):
						ax = plt.Subplot(fig, inner_grid[j])
						ax.bar(range(len(self.P_UM_GIVEN_A[mode][action])), list(self.P_UM_GIVEN_A[mode][action].values()), align='center')
						ax.set_xticks(range(len(self.P_UM_GIVEN_A[mode][action])), list(self.P_UM_GIVEN_A[mode][action].keys()))
						ax.set_ylim(0, 1)
						ax.set_ylabel('probability')
						ax.set_title('Action =' + command)
						fig.add_subplot(ax)
						print mode
						print action

		all_axes = fig.get_axes()

		# show only the outside spines
		for ax in all_axes:
		    for sp in ax.spines.values():
		        sp.set_visible(False)
		    if ax.is_first_row():
		        ax.spines['top'].set_visible(True)
		    if ax.is_last_row():
		        ax.spines['bottom'].set_visible(True)
		    if ax.is_first_col():
		        ax.spines['left'].set_visible(True)
		    if ax.is_last_col():
		        ax.spines['right'].set_visible(True)
		plt.show()

	def _analyze_p_ui_given_a(self):

		fig = plt.figure()
		#gridspec inside gridspec

		outer_grid = gridspec.GridSpec(3, 4, wspace=0.25, hspace=.25)

		for i in range(12):
			inner_grid = gridspec.GridSpecFromSubplotSpec(4, 1, subplot_spec=outer_grid[i], wspace=0.0, hspace=0.25)
			for mode in self.P_UI_GIVEN_A.keys():
				for action in self.P_UI_GIVEN_A[mode].keys():
					for j, command in enumerate(self.P_UI_GIVEN_A[mode][action].keys()):
						ax = plt.Subplot(fig, inner_grid[j])
						ax.bar(range(len(self.P_UI_GIVEN_A[mode][action])), list(self.P_UI_GIVEN_A[mode][action].values()), align='center')
						ax.set_xticks(range(len(self.P_UI_GIVEN_A[mode][action])), list(self.P_UI_GIVEN_A[mode][action].keys()))
						ax.set_ylim(0, 1)
						ax.set_ylabel('probability')
						ax.set_title('Action =' + command)
						fig.add_subplot(ax)
						print mode
						print action

		all_axes = fig.get_axes()

		# show only the outside spines
		for ax in all_axes:
		    for sp in ax.spines.values():
		        sp.set_visible(False)
		    if ax.is_first_row():
		        ax.spines['top'].set_visible(True)
		    if ax.is_last_row():
		        ax.spines['bottom'].set_visible(True)
		    if ax.is_first_col():
		        ax.spines['left'].set_visible(True)
		    if ax.is_last_col():
		        ax.spines['right'].set_visible(True)
		plt.show()




if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('-id', help='subject id', type=str)
	parser.add_argument('-d', '--dist', help='distribution of interest in array form', type=str, default=['_p_um_given_a', '_p_um_given_ui', '_p_ui_given_a'])
	parser.add_argument('-s', '--save', help='save generated plots', type=bool, default=False)
	args = parser.parse_args()
	pd = PersonalizedDistributions(args)
	# pd.analyze_distributions()
