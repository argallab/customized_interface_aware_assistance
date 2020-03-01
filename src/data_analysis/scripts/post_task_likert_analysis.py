#!/usr/bin/python

import os
import argparse
import pandas as pd
import numpy as np
import pickle  
from ast import literal_eval 
from IPython import embed
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.colors as mcolors
import plotly.express as px
import plotly.graph_objects as go


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

		self.question_num = ['Q1', 'Q2', 'Q3', 'Q4', 'Q5', 'Q7', 'Q8', 'Q9', 'Q10', 'Q11', 'Q13', 'Q14']
		self.question_text = self.df.loc[0, self.question_num] # first row is questions save as array
		self.df = self.df.drop(0) # now remove first row from main dataframe so it's clearn
		self.df.reset_index(drop=True, inplace=True) # now reset indices so starts from 0 again

		if subject_id: # if looking at one subject
			self.subject_id = subject_id[0]
			self.df = self.df.loc[self.df['ID'] == self.subject_id]
		else: 
			self.skip_ids() # skip test subjects

		self.labels = ['Strongly Agree', 'Agree', 'Somewhat Agree', 'Neutral', 'Somewhat Disagree', 'Disagree', 'Strongly Disagree']
		self.label_to_score = {'Strongly Agree': 3, 'Agree': 2, 'Somewhat Agree': 1, 'Neutral': 0, 'Somewhat Disagree': -1, 'Disagree': -2, 'Strongly Disagree': -3}
			   
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


	def get_mean_rank(self, data): 

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


	def group_per_assitance_condition(self): 

		# To do: instead of this get Assistance_Type column index levels like R
		assistance_condition = ['Noas', 'Filtas', 'Coras']

		for i in assistance_condition: 
			sub_df = self.df[self.df['Assistance_Type']==i]
			responses = self.get_mean_rank(sub_df)
			self.plot_mean_rank_horizontal_bar_plot(responses, len(sub_df), i)


	def plot_percentage_bar_plot(self): 
		embed(banner1='plotlty')
		
		top_labels = ['Strongly<br>agree', 'Agree', 'Somewhat<br>agree', 'Neutral', 'Somewhat<br>disagree', 'Disagree',
			  'Strongly<br>disagree']

		colors = ['rgba(45, 24, 74, 0.8)', 'rgba(54, 44, 44, 0.8)',
					'rgba(38, 24, 74, 0.8)', 'rgba(71, 58, 131, 0.8)',
					'rgba(122, 120, 168, 0.8)', 'rgba(164, 163, 204, 0.85)',
					'rgba(190, 192, 213, 1)']

		y_data = self.question_text.values.tolist()

		x_data = self.get_percentages()

		fig = go.Figure()

		for i in range(0, len(x_data[0])):
			for xd, yd in zip(x_data, y_data):
				fig.add_trace(go.Bar(
					x=[xd[i]], y=[yd],
					orientation='h',
					marker=dict(
						color=colors[i],
						line=dict(color='rgb(248, 248, 249)', width=1)
					)
				))

		fig.update_layout(
			xaxis=dict(
				showgrid=False,
				showline=False,
				showticklabels=False,
				zeroline=False,
				domain=[0.15, 1]
			),
			yaxis=dict(
				showgrid=False,
				showline=False,
				showticklabels=False,
				zeroline=False,
			),
			barmode='stack',
			paper_bgcolor='rgb(248, 248, 255)',
			plot_bgcolor='rgb(248, 248, 255)',
			margin=dict(l=120, r=10, t=140, b=80),
			showlegend=False,
		)

		annotations = []

		for yd, xd in zip(y_data, x_data):
			# labeling the y-axis
			annotations.append(dict(xref='paper', yref='y',
									x=0.14, y=yd,
									xanchor='right',
									text=str(yd),
									font=dict(family='Arial', size=14,
											  color='rgb(67, 67, 67)'),
									showarrow=False, align='right'))
			# labeling the first percentage of each bar (x_axis)
			annotations.append(dict(xref='x', yref='y',
									x=xd[0] / 2, y=yd,
									text=str(xd[0]) + '%',
									font=dict(family='Arial', size=14,
											  color='rgb(248, 248, 255)'),
									showarrow=False))
			# labeling the first Likert scale (on the top)
			if yd == y_data[-1]:
				annotations.append(dict(xref='x', yref='paper',
										x=xd[0] / 2, y=1.1,
										text=top_labels[0],
										font=dict(family='Arial', size=14,
												  color='rgb(67, 67, 67)'),
										showarrow=False))
			space = xd[0]
			for i in range(1, len(xd)):
					# labeling the rest of percentages for each bar (x_axis)
					annotations.append(dict(xref='x', yref='y',
											x=space + (xd[i]/2), y=yd,
											text=str(xd[i]) + '%',
											font=dict(family='Arial', size=14,
													  color='rgb(248, 248, 255)'),
											showarrow=False))
					# labeling the Likert scale
					if yd == y_data[-1]:
						annotations.append(dict(xref='x', yref='paper',
												x=space + (xd[i]/2), y=1.1,
												text=top_labels[i],
												font=dict(family='Arial', size=14,
														  color='rgb(67, 67, 67)'),
												showarrow=False))
					space += xd[i]

		fig.update_layout(annotations=annotations)

		fig.show()

		embed()




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
	