#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
from utils import SCALE, VIEWPORT_W, VIEWPORT_H, ROBOT_RADIUS, GOAL_RADIUS, PI, VIEWPORT_HS, VIEWPORT_WS
from utils import AssistanceType, StartDirection, RGOrient
import numpy as np
import os
import pickle
import collections
import itertools
import argparse
import random
from IPython import embed

ROBOT_RADIUS_S = ROBOT_RADIUS/SCALE
GOAL_RADIUS_S = GOAL_RADIUS/SCALE

ACTIONS = ['UP', 'DOWN', 'RIGHT', 'LEFT', 'COUNTERCLOCKWISE', 'CLOCKWISE', 'MODE_R', 'MODE_L']

START_DIRECTION = {'UP': [StartDirection.Y],
                   'DOWN': [StartDirection.Y],
                   'RIGHT':[StartDirection.X],
                   'LEFT': [StartDirection.X],
                   'COUNTERCLOCKWISE': [StartDirection.X, StartDirection.Y],
                   'CLOCKWISE': [StartDirection.X, StartDirection.Y],
                   'MODE_R': [StartDirection.X], #these last two are inconsequential
                   'MODE_L': [StartDirection.X]}

ALLOWED_MODE_INDEX_DICT = {StartDirection.X: 'x', StartDirection.Y, 'y'}
ANGLES = {'COUNTERCLOCKWISE': PI/2, 'CLOCKWISE': -PI/2}

def generate_p_um_given_a_trials(args):
    num_reps_per_condition = args.num_reps_per_condition
    trial_dir = args.trial_dir
    metadata_dir = args.metadata_dir
    if not os.path.exists(trial_dir):
        os.makedirs(trial_dir)

    if not os.path.exists(metadata_dir):
        os.makedirs(metadata_dir)

    index = 0



if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--trial_dir', dest='trial_dir',default=os.path.join(os.getcwd(), 'p_um_given_a_trial_dir'), help="The directory where trials will be stored are")
    parser.add_argument('--metadata_dir', dest='metadata_dir',default=os.path.join(os.getcwd(), 'metadata_dir'), help="The directory where metadata of trials will be stored")
    parser.add_argument('--num_reps_per_condition', action='store', type=int, default=3, help="number of repetetions for single combination of conditions ")
    args = parser.parse_args()
    generate_experiment_trials(args)

    for a in ACTIONS:
        trial_info_dict = collections.OrderedDict()
        trial_info_dict['env_params'] = collections.OrderedDict()
        trial_info_dict['env_params']['robot_position'] = (VIEWPORT_WS/2, VIEWPORT_HS/2)
        trial_info_dict['env_params']['robot_orientation'] = 0.0
        trial_info_dict['env_params']['start_direction'] = random.choice(START_DIRECTION[a])
        if a == 'COUNTERCLOCKWISE' or a == 'CLOCKWISE':
            trial_info_dict['env_params']['is_rotation'] = True
            trial_info_dict['env_params']['allowed_mode_index'] = 't'
            trial_info_dict['env_params']['goal_orientation']
            trial_info_dict['env_params']['goal_orientation'] = ANGLES[a]
        else:
            trial_info_dict['env_params']['is_rotation'] = False
            trial_info_dict['env_params']['allowed_mode_index'] = ALLOWED_MODE_INDEX_DICT[trial_info_dict['env_params']['start_direction']]
            trial_info_dict['env_params']['goal_orientation'] = 0.0

        if a == 'MODE_L' or a =='MODE_R':
            trial_info_dict['env_params']['is_mode_switch'] = True
        else:
            trial_info_dict['env_params']['is_mode_switch'] = False
            trial_info_dict['env_params']['mode_switch_config'] = None

        trial_info_dict['env_params']
