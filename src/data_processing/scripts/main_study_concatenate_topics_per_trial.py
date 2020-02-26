#!/usr/bin/env python

import os
import argparse
import pandas as pd
import pickle  
from ast import literal_eval 
from IPython import embed

# read in topic csvs of interest with name and index
# remove unused columns from topic csv
# ensure time is ascending
# merge/concatenate all with single rosbagTimeStamp column
# divide up into trial dataframes
# save as csv with corresponding pkl file name (subject_#_assistance_block_#_trial_#.csv)

class ConcatenateMainStudyTopicsPerTrial(object):
    def __init__(self, args):

        # get path to block of interest 
        self.trial_block = args.block

        self.block_dir = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir)), 'raw_data', self.trial_block)

        assert os.path.exists(self.block_dir)
        
        # To Do: make these inputs to function 
        self.topics = {	'infer_correct_info': '_infer_correct_info.csv', 
      			  		'joy_sip_puff': '_joy_sip_puff.csv', 
      			  		'joy_sip_puff_before': '_joy_sip_puff_before.csv',
      			  		'mode_switches': '_mode_switches.csv', 
      			  		'robot_state': '_robot_state.csv', 
      			  		'trial_index': '_trial_index.csv', 
      			  		'trial_marker': '_trial_marker.csv', 
      			  		'user_vel': '_user_vel.csv' }

      	self.sub_topics = {'infer_correct_info':['rosbagTimestamp', 'optimal_a', 'u_intended', 'normalized_h', 'u_corrected', 'is_corrected_or_filtered', 'is_u_intended_equals_um'], 
	      				  'joy_sip_puff': ['rosbagTimestamp', 'frame_id', 'axes', 'buttons'],
	      				  'joy_sip_puff_before': ['rosbagTimestamp', 'frame_id'], 
	      				  'mode_switches':['rosbagTimestamp', 'mode'], 
	      				  'robot_state': ['rosbagTimestamp', 'robot_continuous_position', 'robot_continuous_orientation', 'robot_linear_velocity', 'robot_angular_velocity', 'robot_discrete_state', 'discrete_location', 'discrete_orientation', 'mode'], 
	      				  'trial_index': ['rosbagTimestamp', 'data'], 
	      				  'trial_marker': ['rosbagTimestamp', 'data'], 
	      				  'user_vel': ['rosbagTimestamp', 'data']}
       
    def load_topic_csvs(self): 
    	# read in topic csvs of interest with name and index
    	
    	# create dict to store topics as variables in loop
    	self.pd_dict = {}

    	for t in self.topics.keys(): 

    		folder = os.path.join(self.block_dir, self.topics[t])
    		
    		if os.path.exists(folder): 
    			print('Loading '+t)		
    		
    			df = pd.read_csv(folder, header = 0, usecols=self.sub_topics[t])
    			
    			keys = [key for key in dict(df.dtypes) if dict(df.dtypes)[key] in ['object']]
    			for key in keys: 
    				df[key].apply(lambda s:s.replace('"', "")) # remove double quotation
    				df[key] = df[key].apply(literal_eval) #convert data types in [] where some rows are zero (considers mixed so dtype is object instead of int or float array)

    			assert self.ensure_ascending(df.rosbagTimestamp) # sanity check, ensure rosbag times in ascending order
    			self.pd_dict[t] = df 
    			# print self.pd_dict[t].dtypes

    		else: 
    			print ('[warning] '+t+' does not exist!!!') # for example, no_assistance wouldn't ahve infer_correct_infor. that is fine

    def rename_df_column(self): 	
    	# specific corrections: 
    	# robot_state/mode --> robot_state_mode
    	# joy_sip_puff_before/frame_id ---> joy_sip_puff_before
    	# user_vel/data --> user_vel

    	# To Do: make this an input to function 
    	self.pd_dict['joy_sip_puff'].rename(columns={'frame_id':'sip_puff_frame_id', 'axes':'sip_puff_axes', 
    		'buttons':'sip_puff_buttons'}, inplace=True)
    	self.pd_dict['joy_sip_puff_before'].rename(columns={'frame_id':'sip_puff_before_frame_id'}, inplace=True)
    	self.pd_dict['robot_state'].rename(columns={'mode':'robot_state_mode'}, inplace=True)
    	self.pd_dict['user_vel'].rename(columns={'data':'user_vel'}, inplace=True)
    	self.pd_dict['trial_marker'].rename(columns={'data':'trial_marker'}, inplace=True)
    	self.pd_dict['trial_index'].rename(columns={'data':'trial_index'}, inplace=True)

    	for df in self.pd_dict.keys(): 
    		self.pd_dict[df].rename(columns={'rosbagTimestamp':'time'}, inplace=True)

    def ensure_ascending(self, data): 

    	flag = 0
    	if (all(data[i] <= data[i+1] for i in range(len(data)-1))): 
    		flag = 1
    	return flag

    def create_csv_per_trial(self): 

    	self.load_topic_csvs() # load data
    	self.rename_df_column() # prevent identical column names before merging 

    	# first combine all data together using the time stamp
    	frames = [self.pd_dict[key] for key in self.pd_dict.keys()]

    	frankenstein_df = frames[0]
    	for i in range(1,len(frames)):  
    		frankenstein_df = frankenstein_df.merge(frames[i], how='outer', sort=True)

    	assert self.ensure_ascending(frankenstein_df.time) # sanity check
    	
    	embed()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-block', help='experiment block: subject_id_type_assistance_block', type=str)
    args = parser.parse_args()
    block_to_trial = ConcatenateMainStudyTopicsPerTrial(args)
    block_to_trial.create_csv_per_trial()

    # python main_study_concatenate_topics_per_trial -id subject_id
