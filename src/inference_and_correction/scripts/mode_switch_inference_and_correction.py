#!/usr/bin/env python

import rospy
import pickle
import os
import numpy as np
from envs.srv import OptimalAction, OptimalActionRequest, OptimalActionResponse
import collections
from IPython import embed
import math
import sys
sys.path.append('../../simulators/scripts') #GOTTA HAVE A BETTER WATY THAN THIS!!!!
from utils import TRUE_ACTION_TO_COMMAND, LOW_LEVEL_COMMANDS


class ModeSwitchInferenceAndCorrection(object):
    def __init__(self):
        self.subject_id = rospy.get_param('subject_id', 0)
        self.distribution_directory_path = os.getcwd() + 'distribution_directory'
        self.P_UI_GIVEN_A = None
        self.P_UM_GIVEN_UI = None
        self.DEFAULT_UI_GIVEN_A_NOISE = 0.2
        self.DEFAULT_UM_GIVEN_UI_NOISE = 0.2

        if os.path.exists(os.path.join(self.distribution_directory_path, str(self.subject_id)+'_p_ui_given_a.pkl')):
            with open(os.path.join(self.distribution_directory_path, str(self.subject_id)+'_p_ui_given_a.pkl'), 'rb') as fp:
                self.P_UI_GIVEN_A = pickle.load(fp)
        else:
            self.P_UI_GIVEN_A = collections.OrderedDict()
            self._init_p_ui_given_a()

        if os.path.exists(os.path.join(self.distribution_directory_path, str(self.subject_id)+'_p_um_given_ui.pkl')):
            with open(os.path.join(self.distribution_directory_path, str(self.subject_id)+'_p_um_given_ui.pkl'), 'rb') as fp:
                self.P_UM_GIVEN_UI = pickle.load(fp)
        else:
            self.P_UM_GIVEN_UI = collections.OrderedDict()
            self._init_p_um_given_ui()


        #TODO define service to be used by sip and puff node. Alongside define the appropriate service types as well
        #Request will contain um.
        #The function body for the service will query the optimal action. Perform Bayesian inference and do correction and return a correct low_level command

        #TODO define service client to get current optimal action


    def _init_p_ui_given_a(self):
        for k in TRUE_ACTION_TO_COMMAND.keys():
            self.P_UI_GIVEN_A[k] = collections.OrderedDict()
            for u in LOW_LEVEL_COMMANDS:
                if u == TRUE_ACTION_TO_COMMAND[k]:
                    self.P_UI_GIVEN_A[k][u] = 1.0
                else:
                    self.P_UI_GIVEN_A[k][u] = np.random.random()*self.DEFAULT_UI_GIVEN_A_NOISE
            normalization_constant = sum(self.P_UI_GIVEN_A[k].values())
            self.P_UI_GIVEN_A[k] = collections.OrderedDict({u:(v/normalization_constant) for u, v in self.P_UI_GIVEN_A[k].items()})

    def _init_p_um_given_ui(self):
        for i in LOW_LEVEL_COMMANDS:
            self.P_UM_GIVEN_UI[i] = collections.OrderedDict()
            for j in LOW_LEVEL_COMMANDS:
                if i==j:
                    self.P_UM_GIVEN_UI[i][j] = 1.0
                else:
                    self.P_UM_GIVEN_UI[i][j] = np.random.random()*self.DEFAULT_UM_GIVEN_UI_NOISE

            normalization_constant = sum(self.P_UM_GIVEN_UI[i].values())
            self.P_UM_GIVEN_UI[i] = collections.OrderedDict({u:(v/normalization_constant) for u, v in self.P_UM_GIVEN_UI[i].items()})

if __name__ == '__main__':
    s = ModeSwitchInferenceAndCorrection()
