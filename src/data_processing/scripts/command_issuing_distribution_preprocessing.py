#!/usr/bin/env python

import csv
import sys
import argparse
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import bisect
from IPython import embed
import pickle  

time_synch_difference = 0.01

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

def ensure_ascending_time(time_stamp_array): 
	for t_array in time_stamp_array: 
		previous = t_array.rosbagTimestamp[0]
		for number in t_array.rosbagTimestamp: 
			if number < previous: 
				sys.exit('Times are not in ascending order. Fix data before proceeding')
			previous = number
		plt.figure()
    	plt.plot(range(0,len(t_array.rosbagTimestamp)), t_array.rosbagTimestamp)
    	plt.show()

def get_nearest_time_stamp(tq, time_stamp_array):
	'''
	assumes that time_stamp_array is a sorted (ascending) list.
	returns: (time stamp closest to tq, index of the time stamp closest to tq in time_stamp_array)
	'''
	idx = bisect.bisect(time_stamp_array, tq) #idx is the index in the time_stamp_array whose value is immediately greater than tq
	if idx == 0: #if requested timestamp is smaller than the first element of the list then return the first element of the list and the index is 0
		return time_stamp_array[idx], idx
	elif idx == len(time_stamp_array): #if tq is greater than the biggest timestamp in the list, then retirn the last timestamp and the index of len-1 or idx-1
		return time_stamp_array[idx-1], idx-1
	else:
		prev_t = time_stamp_array[idx - 1]
		next_t = time_stamp_array[idx]
		if next_t - tq <= tq - prev_t: #if tq is closer to the next value
			return time_stamp_array[idx], idx
		else: #tq is closer to the previous value
			return time_stamp_array[idx-1], idx-1


def get_user_response_block_indices(time_s, time_e, user_input):
	assert 'rosbagTimestamp' in user_input
	time_s_u, index_of_time_s_u = get_nearest_time_stamp(time_s, user_input.rosbagTimestamp)
	time_e_u, index_of_time_e_u = get_nearest_time_stamp(time_e, user_input.rosbagTimestamp)

	assert time_e_u > time_s_u #sanity checks
	assert index_of_time_e_u > index_of_time_s_u #sanity check

	user_response_block_indices = range(index_of_time_s_u, index_of_time_e_u) #list of indices for lookup
	return user_response_block_indices

def populate_probabilities_from_count(array, user_input): 
	LABEL_TO_ARRAY_DICT = {'"Hard Puff"': 0, '"Hard Sip"': 1, '"Soft Puff"': 2, '"Soft Sip"':3, '"Zero Band"': 4, '"Soft-Hard Puff Deadband"': 5, '"Soft-Hard Sip Deadband"': 6  }
	
	counts_array = np.zeros(7)
	for label in user_input: 
		if label == '"input stopped"': 
			pass
		else: 
			counts_array[LABEL_TO_ARRAY_DICT[label]] += 1

	norm_counts_array = counts_array/len(user_input)
	array = (array + norm_counts_array)/2
	return array

	# for label in user_input: 
	# 	switch (label) {
	# 		case '"Hard Puf"': 
	# 		case '"Hard Sip"': 
	# 		case '"Soft Puff"': 	
	# 		case '"Soft Sip"': 	
	# 		case '"Zero Band"': 	
	# 		case '"Soft-Hard Puff Deadband"': 	
	# 		case '"Soft-Hard Sip Deadband"': 				
	# 	}


def build_probabilities(command_prompt, user_input):

	# h_p, h_s, s_p, s_s, zero, s-h_p, s-h_s
	u_hp = np.zeros(7)
	u_hs = np.zeros(7)
	u_sp = np.zeros(7)
	u_ss = np.zeros(7)

	u_hp_profile = []
	u_hs_profile = []
	u_sp_profile = []
	u_ss_profile = []

	COMMAND_TO_ARRAY_DICT = {'Hard Puff': u_hp, 'Hard Sip': u_hs, 'Soft Puff': u_sp, 'Soft Sip': u_ss}
	COMMAND_TO_PROFILE_ARRAY_DICT = {'Hard Puff': u_hp_profile, 'Hard Sip': u_hs_profile, 'Soft Puff': u_sp_profile, 'Soft Sip': u_ss_profile}
	for i in range(0,len(command_prompt)-1,2):
		key =  command_prompt.at[i, 'command'].replace('"','')
		comm_start_t =  command_prompt.at[i, 'rosbagTimestamp']
		comm_end_t =  command_prompt.at[i+1, 'rosbagTimestamp']

		user_response_block_indices = get_user_response_block_indices(comm_start_t, comm_end_t, user_input)

		user_response_header = user_input['frame_id'][user_response_block_indices]

		COMMAND_TO_ARRAY_DICT[key] = populate_probabilities_from_count(COMMAND_TO_ARRAY_DICT[key], user_response_header)

		COMMAND_TO_PROFILE_ARRAY_DICT[key][] 


	pickle.dump(u_hp, open("u_hp.p", "wb"))
	pickle.dump(u_hs, open("u_hs.p", "wb"))
	pickle.dump(u_sp, open("u_sp.p", "wb"))
	pickle.dump(u_ss, open("u_ss.p", "wb"))

	# plot_response_curves()

	return u_hp, u_hs, u_sp, u_ss 


def plot_response_curves():
	pass

# def plot_timestamps(): 

if __name__ == '__main__':

	parser = build_parser()
	args = parser.parse_args()
	path = args.path
	output = args.output
	command_prompt = args.command_prompt[0]
	user_input = args.input[0]

	topics = read_csv_files(path, command_prompt, user_input, output)
	# topics_scaled = scale_times(topics[0], topics[1], topics[2])
	ensure_ascending_time(topics)
	build_probabilities(topics[0], topics[1])


	# How to run:
	# python command_issuing_distribution_preprocessing.py -path mahdieh_command_issuing_full -command_prompt _slash_command_prompt.csv -input _slash_joy_sip_puff.csv 
