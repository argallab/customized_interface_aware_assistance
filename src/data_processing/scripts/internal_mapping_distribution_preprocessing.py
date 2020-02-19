#!/usr/bin/env python

import os
import csv
import sys
import argparse
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from IPython import embed
import pickle  
import itertools
import collections 


class DataParser(object): 
	def __init__(self, file_dir):
		super(DataParser, self).__init__()

		results_files = os.listdir(file_dir) 
		action_prompt_file = os.path.join(file_dir, '_slash_action_prompt.csv') 
		user_response_file = os.path.join(file_dir, '_slash_user_response.csv') 

		self.action_prompt_df = self.read_csv_files(action_prompt_file)
		self.user_response_df = self.read_csv_files(user_response_file)

	def read_csv_files(self, file_path):

		df = pd.read_csv(file_path, header = 0)
		return df 

class IntendedCommandGivenActionAnalysis(object): 
	def __init__(self, args):

		self.file_dir = args.path
		self.id = args.id
		self.data_parser = DataParser(self.file_dir)
		
		
	def build_distributions(self): 
		pass 


if __name__ == '__main__': 
	parser = argparse.ArgumentParser()
	parser.add_argument('-path', help = 'Path to csv files to read', type=str)
	parser.add_argument('-id', help='subject id', type=str)
	args = parser.parse_args()
	# embed(banner1="before initialization")
	puia = IntendedCommandGivenActionAnalysis(args)
	puia.build_distributions()

	# python command_issuing_distribution_preprocessing.py -path deepak -command_prompt _slash_command_prompt.csv -input _slash_joy_sip_puff.csv 