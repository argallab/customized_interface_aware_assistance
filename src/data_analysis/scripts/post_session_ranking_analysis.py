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

class PostSessionRankingSurvey(object):
    def __init__(self, args):

        # get path to csv file of interest 
        self.filename = args.filename
        # To DO: something about this path 
        self.block_dir = os.path.join(os.path.abspath(os.path.join(os.getcwd(), os.pardir, os.pardir)), 'data_processing/raw_data/qualtrics', self.filename)

        assert os.path.exists(self.block_dir)

        # only columns of interest
        columns = ['Progress', 'Duration (in seconds)', 'Finished', 'RecordedDate', 'ID', 'Preference_1', 'Preference_2', 'Preference_3', 
                    'Difficulty_1', 'Difficulty_2', 'Difficulty_3', 'Understandability_1', 'Understandability_2', 'Understandability_3', 
                    'Helpfulness_1', 'Helpfulness_2', 'Helpfulness_3']

        # read csv file ask dataframe
        # skip rows 1 and 2 so the default qualtrics stuff doesn't mix up the data type from int to object
        self.df = pd.read_csv(self.block_dir , header = 0, usecols=columns, skiprows=[1,2])
        

    def skip_ids(self):
        pass
        # Id's to skip (test id's, manual cleaning) 

    def get_response_per_category(self): 
        # array: [0, 1, 2] TO DO: Change to [1, 2, 3]

        first_rank = 1  # First
        second_rank = 2# Second
        third_rank = 3  # Third
        # get number of respondants who ranked noas difficulty as 0, 1, and 2 each... repeat for all

        self.preference_noas = [len(self.df[self.df.Preference_1 == first_rank].index.tolist()),    # First
                                len(self.df[self.df.Preference_1 == second_rank].index.tolist()),   #Second
                                len(self.df[self.df.Preference_1 == third_rank].index.tolist())]    #Third
        self.preference_filtas = [len(self.df[self.df.Preference_2 == first_rank].index.tolist()), 
                                len(self.df[self.df.Preference_2 == second_rank].index.tolist()), 
                                len(self.df[self.df.Preference_2 == third_rank].index.tolist())] 
        self.preference_coras = [len(self.df[self.df.Preference_3 == first_rank].index.tolist()), 
                                len(self.df[self.df.Preference_3 == second_rank].index.tolist()), 
                                len(self.df[self.df.Preference_3 == third_rank].index.tolist())] 
        

        self.difficulty_noas = [len(self.df[self.df.Difficulty_1 == first_rank].index.tolist()), 
                                len(self.df[self.df.Difficulty_1 == second_rank].index.tolist()), 
                                len(self.df[self.df.Difficulty_1 == third_rank].index.tolist())] 
        self.difficulty_filtas = [len(self.df[self.df.Difficulty_2 == first_rank].index.tolist()), 
                                len(self.df[self.df.Difficulty_2 == second_rank].index.tolist()), 
                                len(self.df[self.df.Difficulty_2 == third_rank].index.tolist())] 
        self.difficulty_coras = [len(self.df[self.df.Difficulty_3 == first_rank].index.tolist()), 
                                len(self.df[self.df.Difficulty_3 == second_rank].index.tolist()), 
                                len(self.df[self.df.Difficulty_3 == third_rank].index.tolist())] 

        
        self.understandability_noas = [len(self.df[self.df.Understandability_1 == first_rank].index.tolist()), 
                                        len(self.df[self.df.Understandability_1 == second_rank].index.tolist()), 
                                        len(self.df[self.df.Understandability_1 == third_rank].index.tolist())]
        self.understandability_filtas = [len(self.df[self.df.Understandability_2 == first_rank].index.tolist()), 
                                        len(self.df[self.df.Understandability_2 == second_rank].index.tolist()), 
                                        len(self.df[self.df.Understandability_2 == third_rank].index.tolist())] 
        self.understandability_coras = [len(self.df[self.df.Understandability_3 == first_rank].index.tolist()), 
                                        len(self.df[self.df.Understandability_3 == second_rank].index.tolist()), 
                                        len(self.df[self.df.Understandability_3 == third_rank].index.tolist())] 


        self.helpfulness_noas = [len(self.df[self.df.Helpfulness_1 == first_rank].index.tolist()), 
                                len(self.df[self.df.Helpfulness_1 == second_rank].index.tolist()), 
                                len(self.df[self.df.Helpfulness_1 == third_rank].index.tolist())] 
        self.helpfulness_filtas = [len(self.df[self.df.Helpfulness_2 == first_rank].index.tolist()), 
                                len(self.df[self.df.Helpfulness_2 == second_rank].index.tolist()), 
                                len(self.df[self.df.Helpfulness_2 == third_rank].index.tolist())] 
        self.helpfulness_coras = [len(self.df[self.df.Helpfulness_3 == first_rank].index.tolist()), 
                                len(self.df[self.df.Helpfulness_3 == second_rank].index.tolist()), 
                                len(self.df[self.df.Helpfulness_3 == third_rank].index.tolist())]                         

        self.data = np.concatenate((self.preference_noas, self.preference_filtas, self.preference_coras, 
                                self.difficulty_noas, self.difficulty_filtas, self.difficulty_coras, 
                                self.understandability_noas, self.understandability_filtas, self.understandability_coras, 
                                self.helpfulness_noas, self.helpfulness_filtas, self.helpfulness_coras), axis=0)

    def plot_ranks_together(self): 
        # TO DO: Cleaner way to do this! Yikes
        x = [.5, .5, .5,  1, 1, 1, 1.5, 1.5, 1.5 , 2.5, 2.5, 2.5 , 3, 3, 3, 3.5, 3.5, 3.5, 4.5, 4.5, 4.5, 5, 5, 5, 5.5, 5.5, 5.5, 6.5, 6.5, 6.5, 7, 7, 7, 7.5, 7.5, 7.5]
        # y is 1st, 2nd, 3rd rank
             
        y = []
        for i in range(0, len(x), 3): 
            y.append(3)      # First
            y.append(2)      # Second
            y.append(1)      # Third

        noas_color = 'tab:blue'
        filtas_color = 'tab:orange'
        coras_color = 'tab:green'

        colors = [None]*len(x)
        for i in range(0, len(x), 9): 
            for j in range(3): 
                colors[i+j] = noas_color
            for j in range(3, 6): 
                colors[i+j] = filtas_color
            for j in range(6, 9): 
                colors[i+j] = coras_color

        xtick_loc = [1, 3, 5, 7]
        xtick_labels = ['Preference', 'Difficulty', 'Intuitive', 'Helpful']
        ytick_label = ['3rd', '2nd', '1st']

        embed
        noas_patch = mpatches.Patch(color=noas_color, label='No Assistance')
        filtas_patch = mpatches.Patch(color=filtas_color, label='Filtered')
        coras_patch = mpatches.Patch(color=coras_color, label='Correction')

        plt.scatter(x, y, s=(self.data)*100, c=colors, alpha=0.5)
        plt.xticks(xtick_loc, xtick_labels)
        plt.yticks(np.arange(3)+1, ytick_label)
        plt.ylabel('Ranks')
        plt.title('Post Session Ranking')
        plt.legend(handles=[noas_patch, filtas_patch, coras_patch])
        plt.show()
            
        # scatter plot with weighted area


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--filename', help='qaultrics post-session ranking survey', type=str)
    args = parser.parse_args()
    ranks = PostSessionRankingSurvey(args)
    ranks.get_response_per_category()
    ranks.plot_ranks_together()

    # python main_study_concatenate_topics_per_trial -f qualtrics_file_psot_session
