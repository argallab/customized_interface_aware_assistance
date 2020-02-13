#!/usr/bin/env python

import csv 
import sys
import argparse 
import pandas as pd 

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
	command_prompt_df = pd.read_csv(command_prompt_file)
	user_input_df = pd.read_csv(user_input_file)
	return command_prompt_df, user_input_df

def build_probabilities(): 
	

if __name__ == '__main__': 
	parser = build_parser()
	args = parser.parse_args()
	path = args.path
	output = args.output
	command_prompt = args.command_prompt[0]
	user_input = args.input[0]

	topics = read_csv_files(path, command_prompt, user_input, output)
	print("In command_issuing_distribution_processing:", topics)
	print sys.argv[1], sys.argv[2]

