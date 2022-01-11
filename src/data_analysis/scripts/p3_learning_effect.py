#!/usr/bin/env python3

# Code developed by Deepak Gopinath*, Mahdieh Nejati Javaremi* in February 2020. Copyright (c) 2020. Deepak Gopinath, Mahdieh Nejati Javaremi, Argallab. (*) Equal contribution
import os
import argparse
import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir, os.pardir)), 'simulators/scripts/'))
import corrective_mode_switch_utils
import utils
import itertools
import scipy.stats as ss
import statsmodels.api as sa
import scikit_posthocs as sp

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

# per individual and over all subjects

from IPython import embed


class CompareAssistanceParadigms(object):
	def __init__(self, metrics, directory_path, *subject_id):
		self.directory_list = list()
		self.metrics = metrics
		self.directory_path = directory_path

		self.compute_averages = True # if looking over all subjects, need to get averages (?)
		self.data_dir = os.path.join(self.directory_path, 'trial_csvs')
		print(self.data_dir)
		self.directory_list = self.get_recursive_folders(self.data_dir)
		

		self.trial_fraction_metric = ['success'] # metrics that are calculated over all trials for a signle condition for each person
		self.labels= ['No Assistance', 'Filtered', 'Corrective']
		self.assistance_cond = ['no', 'filter', 'corrective']
		self.label_to_plot_pos = {'No Assistance': 0, 'Filtered': 1 , 'Corrective': 2}
		self.v_strong_alpha = 0.001
		self.strong_alpha = 0.01
		self.alpha = 0.05


	def get_recursive_folders(self, data_dir):
		# get all folders in directory (assumes each subject has one folder containing all trial data)
		# this is for when not looking at individual subject but all participants
		directory_list = list()
		for root, dirs, files in os.walk(data_dir, topdown=False):
			for name in dirs:
				directory_list.append(os.path.join(root, name))
		return directory_list
		# print(self.directory_list)
		# To do: maybe add check if directory is empty, or loop until empty directory

	def load_trial_data(self):
		# get list of all files for all folders in the directory
		self.files_name_list = []
		self.files_path_list = []
		for folder in self.directory_list:
			[self.files_name_list.append(f) for f in os.listdir(folder) if os.path.isfile(os.path.join(folder, f))]
			[self.files_path_list.append(os.path.join(folder, f)) for f in os.listdir(folder) if os.path.isfile(os.path.join(folder, f))]


	def load_trial_metadata(self, trial_index):
		self.metadata_dir = os.path.join(self.directory_path, 'trial_dir', trial_index+'.pkl') 
		trial_metadata = pd.read_pickle(self.metadata_dir)
		return trial_metadata

	def get_trial_index_from_file(self, file):
		trial_index= file.split('/')[-1].split('_')[-1].split('.')[0]
		return trial_index

	def get_all_block_orders(self, id): 

		# only way to get the timestamp is from the bag files stored in /study_bag_files
		bag_file_path = os.path.join(self.directory_path, 'study_bag_files', id)
		# list of study participant folders that contain all block bag files of that participant
		# bag_file_folder_list = self.get_recursive_folders(bag_file_path)

		# for folder in bag_file_path:  #bag_file_folder_list: 
		all_bag_files = os.listdir(bag_file_path)
		block_bag_files = [filename for filename in all_bag_files for assistance in ['filter', 'corrective', 'no_assistance'] if assistance in filename]
		files_in_block = [None]*len(block_bag_files)
		# now sort the files based on the timestamps at the end of the bag file name
		for i in range(len(block_bag_files)): 
			files_in_block[i] = block_bag_files[i].split('_')[-1] # segment out the part of filename that contains the date-time
		enum_files_in_block = list(enumerate(files_in_block))
		sorted_files_by_time = sorted(enum_files_in_block, key=lambda enum_files_in_block: enum_files_in_block[1])
		# get the sorted indices: 
		sorted_indices = []
		sorted_files_in_block = []
		for index, element in sorted_files_by_time: 
			sorted_indices.append(index)
			sorted_files_in_block.append(block_bag_files[index].split('_mode')[0])
		#returns block order from first (top) to last (bottom)
		return list(enumerate(sorted_files_in_block))

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
		# print(metric)
		# print(file)
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


	def group_per_trial(self, metric):

		# TO DO: this needs to be looked at, not as generci as it seems, rewally only for percentages within a trial, so if you want mean over traia, need to fix this
		# no_assistance_first = list()
		# filtered_first = list()
		# corrected_first = list()

		# no_assistance_second = list()
		# filtered_second = list()
		# corrected_second = list()
		data = []
		assistance_label = []
		block_label = []

		if metric in self.trial_fraction_metric:

			id_list = []
			for i, file in enumerate(self.files_name_list):
				subject_id = file.split('_')[0]
				if subject_id not in id_list:
					id_list.append(subject_id) # get the list of unique ids

			for unique_id in id_list: # for each user

				# print('unique_id', unique_id)

				# get block orders
				block_orders = self.get_all_block_orders(unique_id)

				indices = [] 	# get the index of all files that belong to that user
				for ind, file in enumerate(self.files_name_list):
					if unique_id in file:
						indices.append(ind)

				for cond in self.assistance_cond:
					for block in range(2): # block zero or 1
						value_list = []
						# get order of block: 

						for i in indices: # for all tirals of that user, get the trials belogning to certian assistance condition
							if self.files_name_list[i].split('_')[self.files_name_list[i].split('_').index('assistance')-1] == cond:
								if self.files_name_list[i].split('_')[3] == str(block):
									value_list.append(self.compute_metric(metric, self.files_path_list[i])) # find value given path file corresponding to filename

						# calculated percentage:
						value = 100*sum(value_list)/len(value_list)

						for index, element in block_orders: 
							if element == unique_id+'_'+cond+'_assistance_'+str(block): 
								block_index = index 
							elif element == unique_id+'_'+cond+'_assistance_'+str(abs(block-1)): 
								other_block_index = index 

						data.append(value)	
						if block_index < other_block_index: 	
							block_label.append('first')
						elif block_index > other_block_index:
							block_label.append('second')			
						if cond == 'no':
							assistance_label.append('No Assistance')
						elif cond == 'filter':
							assistance_label.append('Filtered')
						elif cond == 'corrective':
							assistance_label.append('Corrective')

		else: 
			for i, file in enumerate(self.files_name_list):
				self.trial_name_info = file.split('_') # split up info contained in files_name
				assistance_type_ind = self.trial_name_info.index('assistance')-1 # get the index that contains the word assistance, minus one to get the assistance type
				id = self.trial_name_info[0]
				block = self.trial_name_info[3]
				block_orders = self.get_all_block_orders(id)
				separator = '_'
				for index, element in block_orders: 
					if element == separator.join(self.trial_name_info[0:4]):  
						block_index = index 
					elif element == separator.join(self.trial_name_info[0:3])+'_'+str(abs(int(block)-1)): 
						other_block_index = index 

				value = self.compute_metric(metric, self.files_path_list[i])

				if value != 'nan':
					data.append(value)
					if block_index < other_block_index: 	
							block_label.append('First')
					elif block_index > other_block_index:
						block_label.append('Second')
					# TO DO: Use self.assistance_cond instead of if els
					if self.trial_name_info[assistance_type_ind] == 'no':
						assistance_label.append('No Assistance')
					elif self.trial_name_info[assistance_type_ind] == 'filter':
						assistance_label.append('Filtered')
					elif self.trial_name_info[assistance_type_ind] == 'corrective':
						assistance_label.append('Corrective')

				else:
					print('[warning]: '+metric+' value is nan. make sure this is what you expected.')


		return data, assistance_label, block_label


	def data_analysis(self):
		# for each file, get the metric of interest and store in corresponding array
		for metric in self.metrics:

			data, assistance_label, block_label = self.group_per_trial(metric)

			df = pd.DataFrame({metric: data , 'condition': assistance_label, 'block': block_label}) 
			data = df.groupby(['condition', 'block'])[metric].apply(list)

			label_to_plot_pos = {'No Assistance First': -0.2, 'No Assistance Second': 0.2, 'Filtered First': 0.8, 'Filtered Second': 1.2 , 'Corrective First': 1.8, 'Corrective Second': 2.2}
			self.parametric_anova_with_post_hoc(data, df, metric, label_to_plot_pos)
		# TO DO: maybe make a main dataframe that holds all the metrics we looked at and save to pkl or something

	def create_dataframe(self, data, metric):
		# assumes labels are in the same order of the data
		# assign the assistance condition label to each data in the arrays so we can add it as a column in the dataframe
		condition = []
		for i in range(len(self.labels)):
			for j in range(len(data[i])):
				condition.append(self.labels[i])
		df = pd.DataFrame({metric: list(itertools.chain(*data)) , 'condition': condition}) # flatten the data and create dataframe so each value corresponds with assistance condition
		return df

	def plot_with_significance(self, df, metric, *args, **kwargs):

		embed()

		pairs = kwargs.get('pairs', None)
		p_values = kwargs.get('p_values', None)

		sns.set_style("dark")
		sns.set_context("paper")
		sns.set_palette("colorblind")

		# ax = sns.barplot(x=df["condition"], y=df[metric], data=df)

		# ax = sns.boxplot(x=df["condition"], y=df[metric])
		ax = sns.boxplot(x="condition", y=metric, hue="block", order=["No Assistance", "Filtered", "Corrective"], data=df)
		# ax = sns.swarmplot(x=df["condition"], y=df[metric], color=".4")
		font_size = 20
		ax.tick_params(labelsize=font_size)

		plt.subplots_adjust(left=.17)
		plt.subplots_adjust(right=.96)

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
				plt.plot(sig_df.loc[i, 'pair'], y_pos, lw=2, c='k')
				plt.text(text_pos_x, text_pos_y, sig_df.loc[i, 'text'], ha='center', va='bottom', color='k', fontsize=font_size-2)

		plt.xlabel('')
		

		# To do: Clean lablels:
		if metric == 'time':
			plt.ylabel('Total Trial Time (s)' , fontsize=font_size)
		elif metric == 'mode_switches':
			plt.ylabel('Successful Trial Mode Switch Count' , fontsize=font_size)
			plt.ylim(0, 30)
		elif metric == 'corrections':
			plt.ylabel('Average Intervention Counts', fontsize=font_size)
		elif metric == 'success':
			plt.ylabel('Percentage of Successful Trials (%)', fontsize=font_size)
		elif metric == 'distance':
			plt.ylabel('Distance to Goal', fontsize=font_size)

		# save to folder
		plot_folder = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir)), 'plots')
		fig_name = os.path.join(plot_folder,'learning_effect_'+metric+'.png')
		plt.savefig(fig_name)

		plt.show()

		# save to folder
		# plot_folder = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir)), 'plots')
		# fig_name = os.path.join(plot_folder, metric+'.png')
		# plt.savefig(fig_name)


	def get_significant_pairs(self, df, metric, label_to_plot_pos):

		df["trial"] = df["condition"]+" "+df["block"]
		pairwise_comparisons = sp.posthoc_conover(df, val_col=metric, group_col='trial', p_adjust='holm')
		# TO DO: Wilcoxon won't work for mode switches because not truly paired test (conditions have different lengths)
		# pairwise_comparisons = sp.posthoc_wilcoxon(df, val_col=metric, group_col='condition', p_adjust='holm')

		# groups = pairwise_comparisons.keys().to_list()
		# combinations = list(itertools.combinations(groups, 2)) # possible combinations for pairwise comparison
		combinations = [('Corrective First', 'Corrective Second'),('Filtered First', 'Filtered Second'),('No Assistance First', 'No Assistance Second')]
		pairs = []
		p_values = []
		# get pairs for x:
		for i in range(len(combinations)):
			if pairwise_comparisons.loc[combinations[i][0], combinations[i][1]] <= self.alpha:  # if signifcane between the two pairs is alot, add position
				pairs.append([label_to_plot_pos[combinations[i][0]], label_to_plot_pos[combinations[i][1]]])
				p_values.append(pairwise_comparisons.loc[combinations[i][0], combinations[i][1]])

		return pairs, p_values


	def parametric_anova_with_post_hoc(self, data, df, metric, label_to_plot_pos):
		# non parametric kruskal wallis test
		H, p = ss.kruskal(*data)
		# if can reject null hypothesis that population medians of all groups are equel,
		if p<= self.alpha:
			# do posthoc test to learn which groups differ in their medians
			pairs, p_values = self.get_significant_pairs(df, metric, label_to_plot_pos)
			self.plot_with_significance(df, metric, pairs=pairs, p_values=p_values)

		else:
			print(metric + ' failed hypothesis test.')
			self.plot_with_significance(df, metric)


if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('-id', '--subject_id', help='experiment block: subject_id_type_assistance_block', type=str)
	parser.add_argument('-m', '--metrics', help='metrics to analyze', nargs='+', default=['time', 'mode_switches', 'corrections', 'success', 'distance'])
	parser.add_argument('-p', '--file_path', help='path to folder which contains both trial_csvs and trial_dir folders', default=os.path.join('N:','2020-IROS-UnintendedCommandAssistance', 'data'))
	args = parser.parse_args()
	if args.subject_id:
		comp_assistance = CompareAssistanceParadigms(args.metrics, args.file_path, args.subject_id)

	else:
		comp_assistance = CompareAssistanceParadigms(args.metrics, args.file_path)
	comp_assistance.load_trial_data()
	comp_assistance.data_analysis()
