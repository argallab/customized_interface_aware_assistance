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
		results_files = [os.path.join(file_dir, f) for f in results_files]

		self.read_csv_files(results_files[0])

	def read_csv_files(self, file_path):
		embed()
		# read tsv file from qualtrics ex
		self.results_df = pd.read_csv(file_path, header = 0, sep='\t')

class IntendedCommandGivenActionAnalysis(object): 
	def __init__(self, args):
		self.file_dir = args.qualtrics_results_dir
		self.id = args.id
		self.data_parser = DataParser(self.file_dir)
		embed()
		

# /home/corrective_mode_switch_assistance/src/data_processing/raw_data
# /home/corrective_mode_switch_assistance/src/data_processing/scripts/raw_data




if __name__ == '__main__': 
	parser = argparse.ArgumentParser()
	parser.add_argument('-p', '--path', dest='qualtrics_results_dir',default=os.path.abspath(os.path.join(os.pardir,'raw_data')), help="The directory where the qualtrics data stored")
	parser.add_argument('-id', help='subject id', type=str)
	args = parser.parse_args()
	# embed(banner1="before initialization")
	puia = IntendedCommandGivenActionAnalysis(args)