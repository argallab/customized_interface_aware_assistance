#!/usr/bin/env python3

import os
import argparse
import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt 
import seaborn as sns
from IPython import embed
import sys
import rospkg
# sys.path.append(os.path.join(rospkg.RosPack().get_path('simulators'), 'scripts'))
sys.path.append(os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir, os.pardir)), 'simulators/scripts/'))
import utils 
import itertools
import scipy.stats as ss
import statsmodels.api as sa
import scikit_posthocs as sp

# import plotly.express as px


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
		print(self.metrics)
		if subject_id: # if looking at one subject
			self.subject_id = subject_id[0]
			self.data_dir = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir)), 'data', self.subject_id)
			self.directory_list.append(self.data_dir)
			print(self.data_dir)
		else: 
			self.compute_averages = True # if looking over all subjects, need to get averages (?)
			self.data_dir = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir)), 'data')
			self.get_recursive_folders()
			print(self.data_dir)

		self.labels= ['No Assistance', 'Filtered', 'Corrective']
		self.label_to_plot_pos = {'No Assistance': 0, 'Filtered': 1 , 'Corrective': 2}
		self.v_strong_alpha = 0.001
		self.strong_alpha = 0.01
		self.alpha = 0.05
		

	def get_recursive_folders(self):
		# get all folders in directory (assumes each subject has one folder containing all trial data)
		# this is for when not looking at individual subject but all participants
		for root, dirs, files in os.walk(self.data_dir, topdown=False): 
			for name in dirs: 
				self.directory_list.append(os.path.join(root, name))
		print(self.directory_list)
		# To do: maybe add check if directory is empty, or loop until empty directory


	def load_trial_data(self): 
		# get list of all files for all folders in the directory
		self.files_name_list = []
		self.files_path_list = []
		for folder in self.directory_list: 
			[self.files_name_list.append(f) for f in os.listdir(folder) if os.path.isfile(os.path.join(folder, f))]
			[self.files_path_list.append(os.path.join(folder, f)) for f in os.listdir(folder) if os.path.isfile(os.path.join(folder, f))]

	def load_trial_metadata(self, trial_index): 
		self.metadata_dir = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir, os.pardir)), 'simulators/scripts/trial_dir', trial_index+'.pkl')
		trial_metadata = pd.read_pickle(self.metadata_dir)
		return trial_metadata

	def get_trial_index_from_file(self, file): 
		trial_index= file.split('/')[-1].split('_')[-1].split('.')[0]
		return trial_index

		
	def _task_completion_time(self, df): 
		start_ind = df[df['trial_marker'] == 'start'].index.tolist()
		end_ind = df[df['trial_marker'] == 'end'].index.tolist()
		time = df.loc[end_ind, 'time'].item() - df.loc[start_ind, 'time'].item()
		return time

	def _distance_to_end(self, df, file): 
		trial_index = self.get_trial_index_from_file(file)
		metadata = self.load_trial_metadata(trial_index) 
		goal_position = metadata['env_params']['goal_position'] # get goal position
		# TO DO: fix main_study_concat so these aren't strings
		final_position_str = df.loc[df['robot_continuous_position'].last_valid_index(),'robot_continuous_position'].split('[')[-1].split(']')[0].split(',') # get last robot position
		final_position = [float(final_position_str[0]), float(final_position_str[1])]
		manhattan_dist = np.linalg.norm(goal_position[0]-final_position[0]) + np.linalg.norm(goal_position[1]-final_position[1]) # manhattan distance
		return manhattan_dist


	def is_successful_trial(self, df, file): 
		# either get time and if time is >= 50 then didn't complete, or if gola postiont or orientation doesn't match
		# more accurate to do the position since can technically be successful with 50
		# no need for orientation becuaase orientation is already before goal, so if not at goal, that's all we need to know
		time = self._task_completion_time(df)
		manhattan_dist = self._distance_to_end(df, file)
		if manhattan_dist <= 0.1 and time <= 50: # successful trial if less than max time and at goal
			return 1
		else: 
			return 0 	


	def compute_metric(self, metric, file): 
		# compute the measure of interest

		df = pd.read_csv(file, header = 0)

		if metric == 'time': 
			value = self._task_completion_time(df)

		if metric == 'mode_switches': 
			if self.is_successful_trial(df, file): 
				mode_switches = df[(df['mode'].notnull()) & (df['mode_frame_id']=='user')].index.tolist() # mode swithces from user or autonomy and not service calls
				value = len(mode_switches)
			else: 
				value = 'nan'

		if metric == 'corrections': 
			if 'is_corrected_or_filtered' in df.columns: 
				corrections = df[df['is_corrected_or_filtered'] == True].index.tolist()
			else: 
				corrections = [] # no assistance has no corrections
			value = len(corrections)

		if metric == 'success': 			
			value = self.is_successful_trial(df, file)

		if metric == 'distance':
			value = self._distance_to_end(df, file) 
			

		# TO DO: Other things to add, entropy for corrections how many um!=ui
		return value


	def group_per_metric(self, metric): 
	
		# To Do: maybe save these as globals for each metric? 
		no_assistance = list()
		filtered = list()
		corrected = list()

		for i, file in enumerate(self.files_name_list): 
			self.trial_name_info = file.split('_')
			assistance_type_ind = self.trial_name_info.index('assistance')-1

			value = self.compute_metric(metric, self.files_path_list[i])

			if value != 'nan': 
				if self.trial_name_info[assistance_type_ind] == 'no': 
					no_assistance.append(value)
				elif self.trial_name_info[assistance_type_ind] == 'filter': 
					filtered.append(value)
				elif self.trial_name_info[assistance_type_ind] == 'corrective': 
					corrected.append(value)
				else:
					print('[warning:] unexpected assistance type')
			else: 
				print('[warning: ]'+metric+' value is nan. make sure this is what you expected.')

		return no_assistance, filtered, corrected


	def create_dataframe(self, data, metric): 
		# assumes labels are in the same order of the data
		# assign the assistance condition label to each data in the arrays so we can add it as a column in the dataframe
		condition = []
		for i in range(len(self.labels)): 
			for j in range(len(data[i])):  
				condition.append(self.labels[i])
		df = pd.DataFrame({metric: list(itertools.chain(*data)) , 'condition': condition}) # flatten the data and create dataframe so each value corresponds with assistance condition
		return df 


	def data_analysis(self): 
		# for each file, get the metric of interest and store in corresponding array
		for metric in self.metrics: 
			no_assistance, filtered, corrected = self.group_per_metric(metric)
			data = [no_assistance, filtered, corrected]

			self.parametric_anova_with_post_hoc(data, metric)
		# TO DO: maybe make a main dataframe that holds all the metrics we looked at and save to pkl or something


	def plot_with_significance(self, df, metric, *args, **kwargs): 

		pairs = kwargs.get('pairs', None)
		p_values = kwargs.get('p_values', None)

		plt.style.use('ggplot')
		sns.set(style="whitegrid")
		ax = sns.boxplot(x=df["condition"], y=df[metric]) 	
		ax = sns.swarmplot(x=df["condition"], y=df[metric], color=".4")	

		# If significance exists, plot it
		if not pairs == None: 
			y_min =  round(df[metric].max()) # get maximum data value (max y)
			h = y_min*0.1
			y_min = y_min + h

			sig_text = []
			for i in p_values: 
				if i <= self.v_strong_alpha: 
					sig_text.append('***')
				elif i <= self.strong_alpha: 
					sig_text.append('**')
				elif i <= self.alpha: 
					sig_text.append('*')		

			# get y position for all the p-value pairs based on location and significacne
			sig_df = pd.DataFrame({'pair':pairs, 'p_value': p_values, 'text': sig_text})
			sig_df = sig_df.sort_values(by=['pair']) # sort so it looks neat when plotting. convert to data frame so can get sig_text with the pairs after sorting
			sig_df.reset_index(drop=True, inplace=True) # reindex after sorting

			for i in range(len(pairs)): 
				y_pos = [y_min+(h*i)]*2 # start and end is same height so *2 
				text_pos_x = sum(sig_df.loc[i, 'pair'])/2 # text position should be in the center of the line connecting the pairs 
				text_pos_y = y_min+(h*i)+0.25
				plt.plot(sig_df.loc[i, 'pair'], y_pos, lw=1.5, c='k')		
				plt.text(text_pos_x, text_pos_y, sig_df.loc[i, 'text'], ha='center', va='bottom', color='k')
		
		plt.show()

		# save to folder
		plot_folder = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir)), 'plots')
		fig_name = os.path.join(plot_folder, metric+'.png')
		plt.savefig(fig_name)


	def get_significant_pairs(self, df, metric): 

		pairwise_comparisons = sp.posthoc_conover(df, val_col=metric, group_col='condition', p_adjust='holm') 
		# embed()
		# TO DO: Wilcoxon won't work for mode switches because not truly paired test (conditions have different lengths)
		# pairwise_comparisons = sp.posthoc_wilcoxon(df, val_col=metric, group_col='condition', p_adjust='holm')

		groups = pairwise_comparisons.keys().to_list() 
		combinations = list(itertools.combinations(groups, 2)) # possible combinations for pairwise comparison 
		pairs = []
		p_values = []
		# get pairs for x: 
		for i in range(len(combinations)):
			if pairwise_comparisons.loc[combinations[i][0], combinations[i][1]] <= self.alpha:  # if signifcane between the two pairs is alot, add position
				pairs.append([self.label_to_plot_pos[combinations[i][0]], self.label_to_plot_pos[combinations[i][1]]])
				p_values.append(pairwise_comparisons.loc[combinations[i][0], combinations[i][1]])

		return pairs, p_values


	def parametric_anova_with_post_hoc(self, data, metric):

		df = self.create_dataframe(data, metric)	

		# non parametric kruskal wallis test
		H, p = ss.kruskal(*data)
		embed()
		# if can reject null hypothesis that population medians of all groups are equel, 
		if p<= self.alpha: 
			# do posthoc test to learn which groups differ in their medians
			pairs, p_values = self.get_significant_pairs(df, metric)
			self.plot_with_significance(df, metric, pairs=pairs, p_values=p_values)	

		else: 
			print(metric + ' failed hypothesis test.')
			self.plot_with_significance(df, metric)	
		
			




if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('-id', '--subject_id', help='experiment block: subject_id_type_assistance_block', type=str)
	parser.add_argument('-m', '--metrics', help='metrics to analyze', nargs='+', default=['time', 'mode_switches', 'corrections', 'success', 'distance'])
	args = parser.parse_args()
	if args.subject_id: 
		comp_assistance = CompareAssistanceParadigms(args.metrics, args.subject_id)

	else: 
		comp_assistance = CompareAssistanceParadigms(args.metrics)
	comp_assistance.load_trial_data()
	comp_assistance.data_analysis()
