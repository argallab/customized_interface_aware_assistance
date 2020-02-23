#!/usr/bin/env python

import os
import sys
import argparse
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from IPython import embed
import pickle  
import collections 
import rospkg


sys.path.append(os.path.join(rospkg.RosPack().get_path('simulators'), 'scripts'))
from utils import TRUE_ACTION_TO_COMMAND


class IntendedCommandGivenActionAnalysis(object): 
    def __init__(self, args):

   
        # load p_um_given_a and p_um_given_ui
        self.id = args.id
        self.personalized_distributions_dir = os.path.join(rospkg.RosPack().get_path('inference_and_correction'), 'personalized_distributions')

        assert os.path.exists(os.path.join(self.personalized_distributions_dir, str(self.id)+'_p_um_given_a.pkl'))
        assert os.path.exists(os.path.join(self.personalized_distributions_dir, str(self.id)+'_p_um_given_ui.pkl'))

        print('LOADING PERSONALIZED P_UM_GIVEN_A')
        with open(os.path.join(self.personalized_distributions_dir, str(self.id)+'_p_um_given_a.pkl'), 'rb') as fp:
            self.P_UM_GIVEN_A = pickle.load(fp)#assumes that the conditional probability distribution is stored as a collections.OrderedDict conditioned on the mode

        print('LOADING PERSONALIZED P_UM_GIVEN_UI')
        with open(os.path.join(self.personalized_distributions_dir, str(self.id)+'_p_um_given_ui.pkl'), 'rb') as fp:
            self.P_UM_GIVEN_UI = pickle.load(fp) #assumes that the conditional probability distribution is stored as a collections.OrderedDict


        self._compute_p_ui_given_a()

    
    def _compute_p_ui_given_a(self):         

        keys = ['Hard Puff', 'Hard Sip', 'Soft Puff', 'Soft Sip']
        A = np.array([self.P_UM_GIVEN_UI[keys[0]].values(), self.P_UM_GIVEN_UI[keys[1]].values(), self.P_UM_GIVEN_UI[keys[2]].values(), self.P_UM_GIVEN_UI[keys[3]].values()])
        
        p_ui = collections.OrderedDict()
        for mode in TRUE_ACTION_TO_COMMAND.keys():

            p_ui[mode] = collections.OrderedDict()

            for action in TRUE_ACTION_TO_COMMAND[mode].keys(): 

                p_ui[mode][action] = collections.OrderedDict()
                B = np.array(self.P_UM_GIVEN_A[mode][action].values())
                p_ui_given_a_dist = np.linalg.solve(A,B)

                for ind, key in enumerate(keys): 
                    p_ui[mode][action][key] = p_ui_given_a_dist[ind]

        embed()
        pickle.dump(p_ui, open(os.path.join(personalized_distributions_dir,self.id+'_p_ui_given_a.pkl'), "wb"))


if __name__ == '__main__': 
    parser = argparse.ArgumentParser()
    parser.add_argument('-id', help='subject id', type=str)
    args = parser.parse_args()
    puia = IntendedCommandGivenActionAnalysis(args)
    puia.build_distributions()

    # python p_ui_given_a_distribution_preprocessing.py -path mahdieh_internal_model -id mahdieh
