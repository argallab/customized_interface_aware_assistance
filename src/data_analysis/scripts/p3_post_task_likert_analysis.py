#!/usr/bin/python

# Code developed by Deepak Gopinath*, Mahdieh Nejati Javaremi* in February 2020. Copyright (c) 2020. Deepak Gopinath, Mahdieh Nejati Javaremi, Argallab. (*) Equal contribution
import os
import argparse
import pandas as pd
import numpy as np
import pickle
from ast import literal_eval
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.colors as mcolors
import seaborn as sns
import itertools

matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42

class PostTaskLikertAnalysis(object):
	def __init__(self, filename, *subject_id):

		# get path to csv file of interest
		self.filename = args.filename
		# To DO: something about this path
		self.file_path = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir, os.pardir)), 'data_processing/raw_data/qualtrics', self.filename)

		assert os.path.exists(self.file_path)

		# only columns of interest
		columns = ['Progress', 'Duration (in seconds)', 'Finished', 'RecordedDate', 'ID', 'Assistance_Type', 'Block',
					'Q1', 'Q2', 'Q3', 'Q4', 'Q5', 'Q7', 'Q8', 'Q9', 'Q10', 'Q11', 'Q13', 'Q14'] # Q12 IS THE OPTIONAL QUESTION NOT LIKERT


		# read csv file ask dataframe
		# skip rows 1 and 2 so the default qualtrics stuff doesn't mix up the data type from int to object
		self.df = pd.read_csv(self.file_path , header = 0, usecols=columns, skiprows=[2])

		# self.question_num = ['Q1', 'Q2', 'Q3', 'Q4', 'Q5', 'Q7', 'Q8', 'Q9', 'Q10', 'Q11', 'Q13', 'Q14']
		# self.question_num = ['Q1', 'Q2', 'Q3', 'Q4', 'Q9', 'Q10', 'Q11']
		self.question_num = ['Q2', 'Q3', 'Q4', 'Q9', 'Q10']



		self.question_text = self.df.loc[0, self.question_num] # first row is questions save as array
		for i in range(len(self.question_text)):
			self.question_text[i] = self.question_text[i].split('Note')[0]

		self.df = self.df.drop(0) # now remove first row from main dataframe so it's clearn
		self.df.reset_index(drop=True, inplace=True) # now reset indices so starts from 0 again

		if subject_id: # if looking at one subject
			self.subject_id = subject_id[0]
			self.df = self.df.loc[self.df['ID'] == self.subject_id]
		else:
			self.skip_ids() # skip test subjects

		self.labels = ['Strongly Agree', 'Agree', 'Somewhat Agree', 'Neutral', 'Somewhat Disagree', 'Disagree', 'Strongly Disagree']
		self.label_to_score = {'Strongly Agree': 3, 'Agree': 2, 'Somewhat Agree': 1, 'Neutral': 0, 'Somewhat Disagree': -1, 'Disagree': -2, 'Strongly Disagree': -3}
		self.conditions= ['No Assistance', 'Filtered', 'Corrective']

	def skip_ids(self):
		# Id's to skip (test id's, manual cleaning)
		# To do: instead of hardcode add input argument
		ids = ['dan', 'deepak', 'andrew']
		for i in range(len(ids)):
			self.df = self.df[self.df.ID != ids[i]]
		self.df.reset_index(drop=True, inplace=True)


	def get_percentages(self, data):

		data.reset_index(drop=True, inplace=True)

		# an array for each questions. Each qesition is an array of 7, from strongly agree to strongly disagree
		# percentes = [question1, question2, etc] where qeustion1=[%(strongly agree), %(agree), %(somewhat agree), %(neutral), etc]
		percentages = []


		total_resondants = float(len(data)) # total number of people who repsonded to quesitonnaire

		for i in self.question_num:
			q_responses = list()
			for j in self.labels:
				reponse_percent = 100.0*len(data[data[i]==j].index.tolist())/total_resondants
				q_responses.append(reponse_percent)
			percentages.append(q_responses)
		return percentages


	def get_ranks_to_score(self, data):

		data.reset_index(drop=True, inplace=True)

		# an array for each quesitons containing the label-to-score value of each respondant's response to that quesiton
		scores = []
		for i in self.question_num:
			q_score = list()
			for j in range(len(data)):
				q_score.append(self.label_to_score[data.loc[j, i]])
			scores.append(q_score)
		return scores


	def plot_mean_rank_horizontal_bar_plot(self, responses, num_resp, title):

		plt.rcdefaults()
		fig, ax = plt.subplots()

		x_data = np.mean(responses, axis=1)
		error = np.std(responses, axis=1)/num_resp
		x_pos = self.label_to_score.values()
		y_pos = np.arange(len(self.question_text))

		ax.barh(y_pos, x_data, xerr=error, align='center')
		ax.set_yticks(y_pos)
		ax.set_yticklabels(self.question_text)
		# ax.set_yticklabels(self.question_num)
		ax.invert_yaxis()  # labels read top-to-bottom
		ax.set_xticks(x_pos)
		ax.set_xticklabels(self.label_to_score.keys())
		ax.set_title(title)



		plt.show()


	def plot_stacked_horizontal_bar_plot(self, df):


		sns.set_style("dark")
		sns.set_context("paper")
		sns.set_palette("colorblind")

		ax = sns.barplot(x='score', y='question', hue='condition', data=df)


		font_size = 20
		ax.tick_params(labelsize='xx-large')


		# TO DO: use gloabl var
		ax.set_xticklabels(['Strongly Disagree', 'Disagree', 'Neutral', 'Somewhat Agree', 'Agree', 'Strongly Agree'])
		plt.xlabel('')
		plt.ylabel('')
		plt.subplots_adjust(left=.49)
		plt.subplots_adjust(right=.97)
		plt.setp(ax.get_legend().get_texts(), fontsize=font_size) # for legend text
		plt.show()


	def group_per_assitance_condition(self):

		# To do: instead of this get Assistance_Type column index levels like R or make key with self.condition
		assistance_condition = ['Noas', 'Filtas', 'Coras']

		df = pd.DataFrame(columns=['score', 'question', 'condition'])

		for i, cond in enumerate(assistance_condition):
			sub_df = self.df[self.df['Assistance_Type'] == cond]
			responses = self.get_ranks_to_score(sub_df)
			question = []
			condition = []
			for j in range(len(responses)): # number of questions
				for k in range(len(responses[j])): # number of respondants
					question.append(self.question_text[j])
					condition.append(self.conditions[i])
			cond_df = pd.DataFrame({'score': list(itertools.chain(*responses)) , 'question': question, 'condition': condition}) # flatten the data and create dataframe so each value corresponds with assistance condition
			df = pd.concat([df, cond_df], join='inner')
			df.reset_index(drop=True, inplace=True)
		self.plot_stacked_horizontal_bar_plot(df)
			# self.plot_mean_rank_horizontal_bar_plot(responses, len(sub_df), i)





if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('-f', '--filename', help='qaultrics post-task survey', default='post_task_survey.csv', type=str) # has defualt,
	parser.add_argument('-id', '--subject_id', help='experiment block: subject_id_type_assistance_block', type=str) # no default but optional
	args = parser.parse_args()
	if args.subject_id:
		likert = PostTaskLikertAnalysis(args.filename, args.subject_id)
	else:
		likert = PostTaskLikertAnalysis(args.filename)

	likert.group_per_assitance_condition()
	# likert.plot_percentage_bar_plot()
