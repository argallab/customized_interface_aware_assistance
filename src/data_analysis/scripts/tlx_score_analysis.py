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


# read in qualtrics ranking csv with filename
# remove unused columns from topic csv
# count rank counts
# plot ranks
# maybe save as pkl file? not sure if necessary
# TO DO: get ranks per person (right now only group)

class TLXCompareAssistanceParadigms(object):
    def __init__(self, filename, metrics, *subject_id):

        self.metrics = metrics
        # get path to csv file of interest 
        self.filename = args.filename
        # To DO: something about this path 
        self.block_dir = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir, os.pardir)), 'data_processing/raw_data/qualtrics', self.filename)

        assert os.path.exists(self.block_dir)

        # only columns of interest
        columns = ['Progress', 'Duration (in seconds)', 'Finished', 'RecordedDate', 'ID', 'Assistance_Type', 'Block',
                    'P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7', 'P8', 'P9', 'P10', 'P11', 'P12', 'P13', 'P14', 'P15', 
                    'Mental_1', 'Frustration_1', 'Physical_1', 'Effort_1', 'Temporal_1', 'Performance_1']

        self.pairwise = ['P1', 'P2', 'P3', 'P4', 'P5', 'P6', 'P7', 'P8', 'P9', 'P10', 'P11', 'P12', 'P13', 'P14', 'P15']
        self.features = ['Mental_1', 'Frustration_1', 'Physical_1', 'Effort_1', 'Temporal_1', 'Performance_1']
        # read csv file ask dataframe
        # skip rows 1 and 2 so the default qualtrics stuff doesn't mix up the data type from int to object
        self.df = pd.read_csv(self.block_dir , header = 0, usecols=columns, skiprows=[1,2])

        if subject_id: # if looking at one subject
            self.subject_id = subject_id[0]
            self.df = self.df.loc[self.df['ID'] == self.subject_id]
        else: 
            self.skip_ids() # skip test subjects               


    def skip_ids(self):
        # Id's to skip (test id's, manual cleaning) 
        # To do: instead of hardcode add input argument
        ids = ['dan', 'deepak', 'andrew']
        for i in range(len(ids)): 
            self.df = self.df[self.df.ID != ids[i]]
        self.df.reset_index(drop=True, inplace=True)
        

    def compute_tlx(self, trial_data): 
        
        # first find weight of each feature (number of times selected in the 15 pairwise comparisons)

        weight_dict = {'Mental Demand': 0, 'Frustration': 0, 'Physical Demand': 0, 'Effort': 0, 'Temporal Demand': 0, 'Performance':0}
        for p in self.pairwise: 
            # for all pairwise comparisons, get the winner and add one to it's cound in the dict
            weight_dict[trial_data.loc[p].splitlines()[0]]+=1 # zero because only interested in first element after spliliens

        # find weighted sum of the rank scales, divided by 15
        weights = pd.Series(weight_dict.values(), index=self.features)
        weighted_ranks = trial_data[self.features]*weights
        tlx = weighted_ranks.sum()/len(self.pairwise)

        return tlx


    def compute_raw_tlx(self, trial_data):
        
        # raw tlx, sum of the ranks divided by the number of features which is 6
        raw_tlx = trial_data[self.features].sum()/len(self.features)
        return raw_tlx


    def group_per_metric(self): 
    # for each file, get the metric of interest and store in corresponding array
        for metric in self.metrics: 

            # To Do: maybe save these as globals for each metric? 
            no_assistance = list()
            filtered = list()
            corrected = list()

            # for every trial (which is a row in the dataframe)
            for i in range(len(self.df)): 

                # compute either tlx or raw_tlx
                if metric == 'tlx': 
                    value = self.compute_tlx(self.df.loc[i])
                if metric == 'raw_tlx':
                    value = self.compute_raw_tlx(self.df.loc[i])

                if self.df.loc[i, 'Assistance_Type'] == 'Noas': 
                    no_assistance.append(value)
                elif self.df.loc[i, 'Assistance_Type'] == 'Filtas': 
                    filtered.append(value)
                elif self.df.loc[i, 'Assistance_Type'] == 'Coras': 
                    corrected.append(value)
                else:
                    print '[warning:] unexpected assistance type' 

            self.plot_box_plot([no_assistance, filtered, corrected], ['No Assistance', 'Filtered', 'Corrective'], metric)

            # df = pd.DataFrame(dict(linear=no_assistance, linear=filtered, corrected=corrected)).melt(var_name="quartilemethod")
            # self.plotly_box_plot(df)

    def plot_box_plot(self, data, ticks, title):
        plt.boxplot(data)
        plt.xticks(range(1,len(ticks)+1), ticks, rotation=0)
        plt.title(title)
        plt.ylim(0,100)
        plt.show() 

        fig = plt.gcf()
        plot_folder = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir)), 'plots')
        fig_name = os.path.join(plot_folder, title+'.png')



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--filename', help='qaultrics post-task survey', default='post_task_survey.csv', type=str) # has defualt,
    parser.add_argument('-id', '--subject_id', help='experiment block: subject_id_type_assistance_block', type=str) # no default but optional
    parser.add_argument('-m', '--metrics', help='metrics to analyze', nargs='+', default=['tlx', 'raw_tlx']) # has default
    args = parser.parse_args()
    if args.subject_id: 
        tlx = TLXCompareAssistanceParadigms(args.filename, args.metrics, args.subject_id)
    else: 
        tlx = TLXCompareAssistanceParadigms(args.filename, args.metrics)
    tlx.group_per_metric()


