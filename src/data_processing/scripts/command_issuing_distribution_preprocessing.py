#!/usr/bin/env python

import csv 
import sys
import argparse 
import pandas as pd 
import matplotlib
import matplotlib.pyplot as plt 
import numpy as np

def build_parser(): 
	# Builds the parser for reading the command line arguments 
	parser = argparse.ArgumentParser(description='Script to process command issuing experiment data')
	parser.add_argument('-path', help = 'Path to csv files to read', type=str)
	parser.add_argument('-command_prompt', help='name of /command_prompt file', nargs='*')
	parser.add_argument('-input', help='name of /joy_sip_puff file', nargs='*')
	parser.add_argument('-o', '--output', help='name of the output file', nargs='*')

	return parser 

def read_csv_files(path, command_prompt, user_input, output): 
	user_input_file = path + '/' + user_input
	command_prompt_file = path + '/' + command_prompt
	command_prompt_df = pd.read_csv(command_prompt_file, header = 0)
	user_input_df = pd.read_csv(user_input_file, header = 0)
	return command_prompt_df, user_input_df

# To do: computationally effecient way of doing this, instad of looping everytime
def get_user_response_block(time_s, time_e, user_input): 
	for i in range (len(user_input)): 
		


def build_probabilities(command_prompt, user_input):
	# To do: Change to 7 for separating deadbands
	u_hp = np.zeros(6)
	u_hs = np.zeros(6)
	u_sp = np.zeros(6)
	u_ss = np.zeros(6)

	COMMAND_TO_ARRAY_DICT = {'Hard Puff': u_hp, 'Hard Sip': u_hs, 'Soft Puff': u_sp, 'Soft Sip': u_ss}
	for i in range(len(command_prompt)):
		key =  command_prompt.get_value(i, 'command').replace('"','')
		time_s =  command_prompt.get_value(i, 'rosbagTimestamp')
		# To do: Change time end to the empty command
		if i < len(command_prompt)-1: 
			time_e = command_prompt.get_value(i+1, 'rosbagTimestamp')
			print command_prompt.get_value(i+1, 'seq')

			get_user_response_block(time_s, time_e, user_input)

		# COMMAND_TO_ARRAY_DICT[key][0] += 1

		# print key, COMMAND_TO_ARRAY_DICT[key]

def plot_response_curves(): 
	pass 


if __name__ == '__main__': 

	parser = build_parser()
	args = parser.parse_args()
	path = args.path
	output = args.output
	command_prompt = args.command_prompt[0]
	user_input = args.input[0]

	topics = read_csv_files(path, command_prompt, user_input, output)
	# print topics[1].head()
	# print topics[1][['axes']]
	build_probabilities(topics[0], topics[1])
	


