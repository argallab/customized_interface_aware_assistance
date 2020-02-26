#!/usr/bin/env python

import os
import argparse
import pandas as pd
import numpy as numpy
import matplotlib
import matplotlib.pyplot as plt 
from IPython import embed

# per individual and over all subjects
# load dataframes in folder 
# for each dataframe, calculate metric of interest (e.g. completion time or number of mode switches)
# do post-processing on data: completion time: deal with time-outs (unsuccesfsul, max out)
# 							  mode switches: remove service call mode switches 
# group into no assistance, filtered, or corrected
# scatter plot
# annova and post-hoc analysis

class CompareAssistanceParadigms(object):
	def __init__(self, metrics, *subject_id):


		self.directory_list = list()
		self.metrics = metrics
		print self.metrics
		if subject_id: # if looking at one subject
			self.subject_id = subject_id[0]
			self.data_dir = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir)), 'data', self.subject_id)
			self.directory_list.append(self.data_dir)
			print self.data_dir
		else: 
			self.compute_averages = True # if looking over all subjects, need to get averages (?)
			self.data_dir = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir)), 'data')
			self.get_recursive_folders()
			print self.data_dir	
		

	def get_recursive_folders(self):
		# get all folders in directory (assumes each subject has one folder containing all trial data)
		# this is for when not looking at individual subject but all participants
		for root, dirs, files in os.walk(self.data_dir, topdown=False): 
			for name in dirs: 
				self.directory_list.append(os.path.join(root, name))
		print self.directory_list	
		# To do: maybe add check if directory is empty, or loop until empty directory


	def load_trial_data(self): 
		# get list of all files for all folders in the directory
		self.files_name_list = []
		self.files_path_list = []
		for folder in self.directory_list: 
			[self.files_name_list.append(f) for f in os.listdir(folder) if os.path.isfile(os.path.join(folder, f))]
			[self.files_path_list.append(os.path.join(folder, f)) for f in os.listdir(folder) if os.path.isfile(os.path.join(folder, f))]

		
	def compute_metric(self, metric, file): 
		# compute the measure of interest

		df = pd.read_csv(file, header = 0)

		if metric == 'time': 
			start_ind = df[df['trial_marker'] == 'start'].index.tolist()
			end_ind = df[df['trial_marker'] == 'end'].index.tolist()
			value = df.loc[end_ind, 'time'].item() - df.loc[start_ind, 'time'].item()

		if metric == 'mode_switches': 
			mode_switches = df[(df['mode'].notnull()) & (df['mode_frame_id']=='user')].index.tolist() # mode swithces from user or autonomy and not service calls
		 	value = len(mode_switches)

		if metric == 'corrections': 
			if 'is_corrected_or_filtered' in df.columns: 
				corrections = df[df['is_corrected_or_filtered'] == True].index.tolist()
			else: 
				corrections = [] # no assistance has no corrections
			value = len(corrections)

		# TO DO: Other things to add, entropy for corrections how many um!=ui

		return value


	def group_per_metric(self): 
	# for each file, get the metric of interest and store in corresponding array
		for metric in self.metrics: 

			# To Do: maybe save these as globals for each metric? 
			no_assistance = list()
			filtered = list()
			corrected = list()

			for i, file in enumerate(self.files_name_list): 
				self.trial_name_info = file.split('_')
				assistance_type_ind = self.trial_name_info.index('assistance')-1

				value = self.compute_metric(metric, self.files_path_list[i])

				if self.trial_name_info[assistance_type_ind] == 'no': 
					no_assistance.append(value)
				elif self.trial_name_info[assistance_type_ind] == 'filter': 
					filtered.append(value)
				elif self.trial_name_info[assistance_type_ind] == 'corrective': 
					corrected.append(value)
				else:
					print '[warning:] unexpected assistance type' 

			self.plot_box_plot([no_assistance, filtered, corrected], ['No Assistance', 'Filtered', 'Corrective'], metric)


	def plot_box_plot(self, data, ticks, title):

		plt.boxplot(data)
		plt.xticks(range(1,len(ticks)+1), ticks, rotation=25)
		plt.title(title)
		plt.show() 
		fig = plt.gcf()
		plot_folder = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir)), 'plots')
		fig_name = os.path.join(plot_folder, title+'.png')
		plt.savefig(fig_name)

		# TO DO: add argumen to save plot


if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('-id', '--subject_id', help='experiment block: subject_id_type_assistance_block', type=str)
	parser.add_argument('-m', '--metrics', help='metrics to analyze', nargs='+', default=['time', 'mode_switches', 'corrections'])
	args = parser.parse_args()
	if args.subject_id: 
		comp_assistance = CompareAssistanceParadigms(args.metrics, args.subject_id)

	else: 
		comp_assistance = CompareAssistanceParadigms(args.metrics)
	comp_assistance.load_trial_data()
	comp_assistance.group_per_metric()
