#!/usr/bin/python3

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
import itertools
import scipy.stats as ss
import statsmodels.api as sa
import scikit_posthocs as sp
import seaborn as sns

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

        self.labels= ['No Assistance', 'Filtered', 'Corrective']
        self.label_to_plot_pos = {'No Assistance': 0, 'Filtered': 1 , 'Corrective': 2}
        self.v_strong_alpha = 0.001
        self.strong_alpha = 0.01
        self.alpha = 0.05            


    def skip_ids(self):
        # Id's to skip (test id's, manual cleaning) 
        # To do: instead of hardcode add input argument
        ids = ['dan', 'deepak', 'andrew', '0']
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
        weights = pd.Series(list(weight_dict.values()), index=self.features)
        weighted_ranks = trial_data[self.features]*weights
        tlx = weighted_ranks.sum()/len(self.pairwise)

        return tlx


    def compute_raw_tlx(self, trial_data):
        
        # raw tlx, sum of the ranks divided by the number of features which is 6
        raw_tlx = trial_data[self.features].sum()/len(self.features)
        return raw_tlx


    def group_per_metric(self, metric): 
    # for each file, get the metric of interest and store in corresponding array

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
                print('[warning:] unexpected assistance type') 

        return no_assistance, filtered, corrected


    def create_dataframe(self, data, metric): 
        # assumes labels are in the same order of the data
        # assign the assistance condition label to each data in the arrays so we can add it as a column in the dataframe
        condition = []
        for i in range(len(self.labels)): 
            for j in range(len(data[i])): 
                condition.append(self.labels[i])
        df = pd.DataFrame({metric: list(itertools.chain(*data)), 'condition': condition})
        return df 


    def data_analysis(self): 
        # for each file, get the metric of interest and store in corresponding array
        for metric in self.metrics: 
            no_assistance, filtered, corrected = self.group_per_metric(metric)
            data = [no_assistance, filtered, corrected]

            self.parametric_anova_with_post_hoc(data, metric)


    def plot_with_significance(self, df, metric, pairs, p_values): 

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

        plt.style.use('ggplot')

        sns.set(style="whitegrid")
        ax = sns.boxplot(x=df["condition"], y=df[metric])    
        ax = sns.swarmplot(x=df["condition"], y=df[metric], color=".4")    

        # get y position for all the p-value pairs based on location and significacne
        sig_df = pd.DataFrame({'pair':pairs, 'p_value': p_values, 'text': sig_text})
        sig_df = sig_df.sort_values(by=['pair']) # sort so it looks neat when plotting. convert to data frame so can get sig_text with the pairs after sorting
        sig_df.reset_index(drop=True, inplace=True) # reindex after sorting

        for i in range(len(pairs)): 
            y_pos = [y_min+(h*i)]*2 # start and end is same height so *2 
            text_pos_x = sum(sig_df.loc[i, 'pair'])/2 # text position should be in the center of the line connecting the pairs 
            text_pos_y = y_min+(h*i)+0.25
            plt.plot(sig_df.loc[i, 'pair'], y_pos, lw=1.5, c='k')       
            plt.text(text_pos_x, text_pos_y, sig_df.loc[i, 'text'], ha='center', va='bottom', color='k')
        
        plt.show()

        plot_folder = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir)), 'plots')
        fig_name = os.path.join(plot_folder, metric+'.png')
        plt.savefig(fig_name)


    def get_significant_pairs(self, df, metric): 

        pairwise_comparisons = sp.posthoc_conover(df, val_col=metric, group_col='condition', p_adjust='holm') 
        # pairwise_comparisons = sp.posthoc_wilcoxon(df, val_col=metric, group_col='condition', p_adjust='holm')

        groups = pairwise_comparisons.keys().to_list() 
        combinations = list(itertools.combinations(groups, 2)) # possible combinations for pairwise comparison 
        pairs = []
        p_values = []
        # get pairs for x: 
        for i in range(len(combinations)):
            if pairwise_comparisons.loc[combinations[i][0], combinations[i][1]] <= self.alpha:  # if signifcane between the two pairs is alot, add position
                pairs.append([self.label_to_plot_pos[combinations[i][0]], self.label_to_plot_pos[combinations[i][1]]])
                p_values.append(pairwise_comparisons.loc[combinations[i][0], combinations[i][1]])
            
        return pairs, p_values


    def parametric_anova_with_post_hoc(self, data, metric):

        df = self.create_dataframe(data, metric)    

        # non parametric kruskal wallis test
        H, p = ss.kruskal(*data)
        # if can reject null hypothesis that population medians of all groups are equel, 
        if p<= self.alpha: 
            # do posthoc test to learn which groups differ in their medians
            pairs, p_values = self.get_significant_pairs(df, metric)
        
        self.plot_with_significance(df, metric, pairs, p_values)        



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
    tlx.data_analysis()


