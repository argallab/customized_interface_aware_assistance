#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Code developed by Deepak Gopinath*, Mahdieh Nejati Javaremi* in February 2020. Copyright (c) 2020. Deepak Gopinath, Mahdieh Nejati Javaremi, Argallab. (*) Equal contribution

import os
import collections
import numpy as np
import sys
import rospkg
sys.path.append(os.path.join(rospkg.RosPack().get_path('simulators'), 'scripts'))
from corrective_mode_switch_utils import VIEWPORT_H, VIEWPORT_W, SCALE, ROBOT_RADIUS, StartDirection, PI, AssistanceType
import itertools
import pickle
import random
import argparse

MAX_SIM_STEPS = 200

NUM_TURNS = 2
NUM_LOCATIONS = NUM_TURNS + 2
#params for generating robot positions. Not used
VIEWPORT_WS = VIEWPORT_W/SCALE
VIEWPORT_HS = VIEWPORT_H/SCALE
ROBOT_RADIUS_S = ROBOT_RADIUS/SCALE
QUADRANT_BOUNDS = collections.OrderedDict()
#array to hold the WAYPOINTS. continuous valued positions
WAYPOINTS = np.zeros((NUM_LOCATIONS, 2))
#The corner at which the robot needs to be turned
LOCATION_OF_TURN = random.choice(range(1, NUM_LOCATIONS-1))
#depending on the relative configuration of the goal with respect to the robot, robot and g will be in different quadrants
R_TO_G_CONFIGS = {'tr':{'r':'3', 'g': '1'},
                  'tl':{'r':'4', 'g': '2'},
                  'br':{'r':'2', 'g': '4'},
                  'bl':{'r':'1', 'g': '3'}}

#1D modes for SNP
MODES = ['x', 'y', 't']
#symbols for different locations.
LOCATIONS = ['p' + str(i) for i in range(NUM_LOCATIONS)]
#two distinct orientations. Initially robot is at 0, after turning the robot will be at PI/2
ORIENTATIONS = [0, PI/2, -PI/2]

TARGET_ORIENTATION = -PI/2
#Generate list of states. State = (location, orientation, mode)
STATES = [s for s in itertools.product(LOCATIONS, ORIENTATIONS, MODES)]
#low level commands issued by the snp interface. hp = hard puff, hs= hard sip, sp = soft puff, ss = soft sip. Also the domain for ui and um
LOW_LEVEL_COMMANDS = ['hp', 'hs', 'sp', 'ss']
#high level actions, move_p = move in positive direction, move_n = move in negative direction, mode_r = switch mode to right, mode_l = switch mode to left. positive and negative is conditioned on mode
HIGH_LEVEL_ACTIONS = ['move_p', 'move_n', 'mode_r', 'mode_l']

#empty dictionary to hold the state transition mode
STATE_TRANSITION_MODEL = collections.OrderedDict()

#transition function for mode switches.
MODE_SWITCH_TRANSITION = {'x': {'hp': 'y', 'hs': 't', 'sp': 'x', 'ss': 'x'},
						  'y': {'hp': 't', 'hs': 'x', 'sp': 'y', 'ss': 'y'},
						  't': {'hp': 'x', 'hs': 'y', 'sp': 't', 'ss': 't'}}

#empty dictionary to store information regarding modes that allow motion. Essentially this is a way to do bound checks in a path
MODES_MOTION_ALLOWED = collections.OrderedDict()

#Depending on the configuration of the initial robot position and goal position, the motion commands will result in either moving towards the
#next location or the previous location
TRANSITION_FOR_ACTION =   {'tr': {'sp': {'x': 'next', 'y': 'next', 't': 'next'}, 'ss': {'x': 'prev', 'y': 'prev', 't': 'prev'}},
							'tl': {'sp': {'x': 'prev', 'y': 'next', 't': 'next'}, 'ss': {'x': 'next', 'y': 'prev', 't': 'prev'}},
							'br': {'sp': {'x': 'next', 'y': 'prev', 't': 'next'}, 'ss': {'x': 'prev', 'y': 'next', 't': 'prev'}},
							'bl': {'sp': {'x': 'prev', 'y': 'prev', 't': 'next'}, 'ss': {'x': 'next', 'y': 'next', 't': 'prev'}}
						}

#true mapping of a to u
TRUE_ACTION_TO_COMMAND = collections.OrderedDict({'move_p': 'sp', 'move_n':'ss', 'mode_r':'hp', 'mode_l': 'hs'})
#true inverse mapping of u to a
TRUE_COMMAND_TO_ACTION = collections.OrderedDict({v:k for k, v in TRUE_ACTION_TO_COMMAND.items()})
#dictionary to hold the optimal next state for a given state. Essentially a one-step optimal plan
OPTIMAL_NEXT_STATE_DICT = collections.OrderedDict()
#dictionary to hold the action to go the optimal next state
OPTIMAL_ACTION_DICT = collections.OrderedDict()

#p(ui|a)
P_UI_GIVEN_A = collections.OrderedDict()
UI_GIVEN_A_NOISE = 0.01 #Lower the number, lower the error. Between 0 and 1. If 0, the p(ui|a) is delta and same as the true mapping

#p(um|ui)
P_UM_GIVEN_UI = collections.OrderedDict()
UM_GIVEN_UI_NOISE = 0.01 #Lower the number, lower the error. Between 0 and 1. If 0, no difference between ui and um

#p(ui|um). dictionary to hold inferred ui
P_UI_GIVEN_UM = collections.OrderedDict()
#entropy threshold for assistance. between 0 and 1
ENTROPY_THRESHOLD = 0.8
#Assistance Type. Choice between Filter and Corrective. TODO. Maybe load it from file
ASSISTANCE_TYPE = AssistanceType.Corrective

def create_state_transition_model():
	global STATE_TRANSITION_MODEL
	for s in STATES:
		STATE_TRANSITION_MODEL[s] = collections.OrderedDict()
		for u in LOW_LEVEL_COMMANDS:
			STATE_TRANSITION_MODEL[s][u] = None

def init_state_transition_model(rgc):
	global STATE_TRANSITION_MODEL, LOCATIONS
	for s in STATE_TRANSITION_MODEL.keys(): #for all states in the world
		for u in STATE_TRANSITION_MODEL[s].keys(): #for all available low-level commands in s
			if u == 'hp' or u == 'hs': #if u is either hard puff or hard sip, only mode switch happens. Use the MODE_SWITCH_TRANSITION dict to pick the next mode
				STATE_TRANSITION_MODEL[s][u] = (s[0], s[1], MODE_SWITCH_TRANSITION[s[2]][u]) #generate new state
			if u == 'sp' or u == 'ss': #if sp or ss, this will result in motion if the mode associated with s an allowed mode for motion
				allowed_modes_for_motion = MODES_MOTION_ALLOWED[s[0]] #retrieve all the modes in which motion is allowed for this state. (note that, for the location in which turning happens there will be two modes in which motion is allowed)
				STATE_TRANSITION_MODEL[s][u] = (s[0], s[1], s[2]) #by default store the same state as next state. Because if no motion happens, the resultant state is also the same state
				for m in allowed_modes_for_motion:
					if m == s[2]: #make sure that the allowed mode matches the mode in the state s. If it doesn't no motion will happen
						if m != 't': #allowed motion mode is a linear mode, x or y.
							if TRANSITION_FOR_ACTION[rgc][u][m] == 'next':
								new_loc_next = LOCATIONS[min(LOCATIONS.index(s[0]) + 1, len(LOCATIONS)-1 )]
								STATE_TRANSITION_MODEL[s][u] = (new_loc_next, s[1], s[2])
							elif TRANSITION_FOR_ACTION[rgc][u][m] == 'prev':
								# print LOCATIONS, s
								new_loc_prev = LOCATIONS[max(LOCATIONS.index(s[0]) - 1, 0 )]
								if m == MODES_MOTION_ALLOWED[new_loc_prev][0]:
									STATE_TRANSITION_MODEL[s][u] = (new_loc_prev, s[1], s[2])
						elif m == 't':# if allowed mode is rotation mode, rotate the angle properly.
							new_theta = s[1]
							if TRANSITION_FOR_ACTION[rgc][u][m] == 'next':
								new_theta = min(PI/2, s[1] + PI/2) #max angle allowed is PI/2
							elif TRANSITION_FOR_ACTION[rgc][u][m] == 'prev':
								new_theta = max(-PI/2, s[1] - PI/2) #min angle allowed is 0.0
							STATE_TRANSITION_MODEL[s][u] = (s[0], new_theta, s[2])

def create_bounds_dict():
	q_keys = [1,2,3,4]
	dimensions = ['x', 'y']
	bounds = ['min', 'max']
	for q_key in q_keys:
		QUADRANT_BOUNDS[str(q_key)] = collections.OrderedDict()
		for d in dimensions:
			QUADRANT_BOUNDS[str(q_key)][d] = collections.OrderedDict()
			for b in bounds:
				QUADRANT_BOUNDS[str(q_key)][d][b] = None

def initialize_bounds():

    QUADRANT_BOUNDS['1']['x']['min'] = (2*VIEWPORT_WS)/3 + ROBOT_RADIUS_S
    QUADRANT_BOUNDS['1']['x']['max'] = VIEWPORT_WS - ROBOT_RADIUS_S
    QUADRANT_BOUNDS['1']['y']['min'] = (2*VIEWPORT_HS)/3 + ROBOT_RADIUS_S
    QUADRANT_BOUNDS['1']['y']['max'] = VIEWPORT_HS - ROBOT_RADIUS_S

    #initalize bounds for 'tl'
    QUADRANT_BOUNDS['2']['x']['min'] = ROBOT_RADIUS_S
    QUADRANT_BOUNDS['2']['x']['max'] = VIEWPORT_WS/3 - ROBOT_RADIUS_S
    QUADRANT_BOUNDS['2']['y']['min'] = (2*VIEWPORT_HS)/3 + ROBOT_RADIUS_S
    QUADRANT_BOUNDS['2']['y']['max'] = VIEWPORT_HS - ROBOT_RADIUS_S

    #initialize_bounds for 'bl'
    QUADRANT_BOUNDS['3']['x']['min'] = ROBOT_RADIUS_S
    QUADRANT_BOUNDS['3']['x']['max'] = VIEWPORT_WS/3 - ROBOT_RADIUS_S
    QUADRANT_BOUNDS['3']['y']['min'] = ROBOT_RADIUS_S
    QUADRANT_BOUNDS['3']['y']['max'] = VIEWPORT_HS/3 - ROBOT_RADIUS_S

    #initialize_bounds for 'br'
    QUADRANT_BOUNDS['4']['x']['min'] = (2*VIEWPORT_WS)/3 + ROBOT_RADIUS_S
    QUADRANT_BOUNDS['4']['x']['max'] = VIEWPORT_WS - ROBOT_RADIUS_S
    QUADRANT_BOUNDS['4']['y']['min'] = ROBOT_RADIUS_S
    QUADRANT_BOUNDS['4']['y']['max'] = VIEWPORT_HS/3 - ROBOT_RADIUS_S

def generate_waypoints(start_direction):
	'''
	generates continuous valued waypoints in (x,y) space. Not used in this simulation
	'''
	if NUM_TURNS % 2 == 0:
		divisor = NUM_TURNS/2 + 1
		if start_direction == StartDirection.X:
			x_inc = (WAYPOINTS[-1][0] - WAYPOINTS[0][0])/divisor
			y_inc = (WAYPOINTS[-1][1] - WAYPOINTS[0][1])/(divisor - 1)
		elif start_direction == StartDirection.Y:
			x_inc = (WAYPOINTS[-1][0] - WAYPOINTS[0][0])/(divisor-1)
			y_inc = (WAYPOINTS[-1][1] - WAYPOINTS[0][1])/divisor
	else:
		divisor = (NUM_TURNS + 1)/2
		x_inc = (WAYPOINTS[-1][0] - WAYPOINTS[0][0])/divisor
		y_inc = (WAYPOINTS[-1][1] - WAYPOINTS[0][1])/divisor

	if start_direction == StartDirection.X:
		for i in range(1, NUM_TURNS+1):
			if i%2 == 0:
				WAYPOINTS[i][0] = WAYPOINTS[i-1][0]
				WAYPOINTS[i][1] = WAYPOINTS[i-1][1] + y_inc
			else:
				WAYPOINTS[i][0] = WAYPOINTS[i-1][0] + x_inc
				WAYPOINTS[i][1] = WAYPOINTS[i-1][1]
	elif start_direction == StartDirection.Y:
		for i in range(1, NUM_TURNS+1):
			if i%2 == 0:
				WAYPOINTS[i][0] = WAYPOINTS[i-1][0] + x_inc
				WAYPOINTS[i][1] = WAYPOINTS[i-1][1]
			else:
				WAYPOINTS[i][0] = WAYPOINTS[i-1][0]
				WAYPOINTS[i][1] = WAYPOINTS[i-1][1] + y_inc

def generate_start_and_end_points(rgc):
	'''
	rgc = robot to goal configuration. Depending on the relative configuration going to the next location in the path might require either ss or sp

	Generates initial robot and goal poses in (x,y, theta) space. Not used in this simulation
	'''
	rq = R_TO_G_CONFIGS[rgc]['r']
	gq = R_TO_G_CONFIGS[rgc]['g']
	rx = QUADRANT_BOUNDS[rq]['x']['min'] + np.random.random()*(QUADRANT_BOUNDS[rq]['x']['max']-QUADRANT_BOUNDS[rq]['x']['min'])
	ry = QUADRANT_BOUNDS[rq]['y']['min'] + np.random.random()*(QUADRANT_BOUNDS[rq]['y']['max']-QUADRANT_BOUNDS[rq]['y']['min'])
	gx = QUADRANT_BOUNDS[gq]['x']['min'] + np.random.random()*(QUADRANT_BOUNDS[gq]['x']['max']-QUADRANT_BOUNDS[gq]['x']['min'])
	gy = QUADRANT_BOUNDS[gq]['y']['min'] + np.random.random()*(QUADRANT_BOUNDS[gq]['y']['max']-QUADRANT_BOUNDS[gq]['y']['min'])
	robot_position = (rx, ry)
	goal_position = (gx, gy)
	robot_init_orientation = 0
	goal_orientation = PI/2
	return robot_position, goal_position, robot_init_orientation, goal_orientation

def init_modes_in_which_motion_allowed_dict(start_direction):
	'''
	Specifies, for each location, the modes in which motion is allowed
	'''
	global MODES_MOTION_ALLOWED, LOCATIONS
	for i, s in enumerate(LOCATIONS[:-1]):
		if start_direction == StartDirection.X:
			if i % 2 == 0:
				MODES_MOTION_ALLOWED[s] = ['x'] #Make this a function of start direction
			else:
				MODES_MOTION_ALLOWED[s] = ['y']
			if i == LOCATION_OF_TURN: #if the location also happened to be the location where turning happens, add 't' to the allowed mode
				MODES_MOTION_ALLOWED[s].append('t')
		elif start_direction == StartDirection.Y:
			if i % 2 == 0:
				MODES_MOTION_ALLOWED[s] = ['y'] #Make this a function of start direction
			else:
				MODES_MOTION_ALLOWED[s] = ['x']
			if i == LOCATION_OF_TURN: #if the location also happened to be the location where turning happens, add 't' to the allowed mode
				MODES_MOTION_ALLOWED[s].append('t')

	MODES_MOTION_ALLOWED[LOCATIONS[-1]] = [] #no more modes for the last locations

def create_optimal_next_state_dict():
	'''
	For every state, this function computes what is the optimal next state to be.
	'''
	global STATES, LOCATIONS, LOCATION_OF_TURN, MODES_MOTION_ALLOWED, OPTIMAL_NEXT_STATE_DICT, TARGET_ORIENTATION
	for s in STATES:
		if LOCATIONS.index(s[0]) < LOCATION_OF_TURN or LOCATIONS.index(s[0]) > LOCATION_OF_TURN: #deal with locations before and after the turn location separately as they consist of ONLY linear motion.
			if s[2] not in MODES_MOTION_ALLOWED[s[0]]: #if not in the proper mode, switch to the mode
				if len(MODES_MOTION_ALLOWED[s[0]]) == 1:
					OPTIMAL_NEXT_STATE_DICT[s] = (s[0],s[1],MODES_MOTION_ALLOWED[s[0]][0])
			else: #if already in proper mode, them move!
				OPTIMAL_NEXT_STATE_DICT[s] = (LOCATIONS[min(LOCATIONS.index(s[0]) + 1, NUM_LOCATIONS)], s[1], s[2])
		elif LOCATIONS.index(s[0]) == LOCATION_OF_TURN: #at the location of turning, gotta deal with both turning and then moving in the allowed linear mode, IN THAT ORDER
			if s[2] != 't': #in a linear mode
				if s[1] != TARGET_ORIENTATION: #haven't turned, is not in 't'. Therefore switch to 't'. Because gotta turn first
					OPTIMAL_NEXT_STATE_DICT[s] = (s[0], s[1], 't')
				else: #have already turned. now need to move to the next location in the allowed linear mode
					if s[2] == MODES_MOTION_ALLOWED[s[0]][0]: #check if the linear mode is an allowed motion mode. If so, move
						OPTIMAL_NEXT_STATE_DICT[s] = (LOCATIONS[min(LOCATIONS.index(s[0]) + 1, NUM_LOCATIONS)], s[1], s[2])
					else:# if it is not the allowed motion mode, switch to the allowed motion 'linear mode'.
						OPTIMAL_NEXT_STATE_DICT[s] = (s[0], s[1], MODES_MOTION_ALLOWED[s[0]][0])
			else:
				#already in turning mode
				if s[1] != TARGET_ORIENTATION: #if not turned yet, go ahead and turn
					diff_orientation = TARGET_ORIENTATION - s[1]
					if diff_orientation > 0:
						diff_orientation = min(PI/2, diff_orientation)
					else:
						diff_orientation = max(-PI/2, diff_orientation)

					OPTIMAL_NEXT_STATE_DICT[s] = (s[0], s[1] + diff_orientation, s[2])
				else: #if already turned
					if s[2] == MODES_MOTION_ALLOWED[s[0]][0]: #check if the linear mode is an allowed motion mode. If so, move
						OPTIMAL_NEXT_STATE_DICT[s] = (LOCATIONS[min(LOCATIONS.index(s[0]) + 1, NUM_LOCATIONS)], s[1], s[2])
					else:# if it is not the allowed motion mode, switch to the allowed motion 'linear mode'.
						OPTIMAL_NEXT_STATE_DICT[s] = (s[0], s[1], MODES_MOTION_ALLOWED[s[0]][0])

def generate_optimal_control_dict():
	'''
	Using the inverse mapping and the optimal next state dictionary, generate the optimal action for each state in optimal_next_state_dict
	'''
	global OPTIMAL_ACTION_DICT, OPTIMAL_NEXT_STATE_DICT, STATE_TRANSITION_MODEL
	for s in OPTIMAL_NEXT_STATE_DICT:
		sp = OPTIMAL_NEXT_STATE_DICT[s]
		OPTIMAL_ACTION_DICT[s] = [k for k,v in STATE_TRANSITION_MODEL[s].items() if v == sp]
		OPTIMAL_ACTION_DICT[s] = TRUE_COMMAND_TO_ACTION[OPTIMAL_ACTION_DICT[s][0]]

def init_p_ui_given_a():
	'''
	Generate a random p(ui | a). key = a, subkey = ui
	'''
	global P_UI_GIVEN_A
	for k in TRUE_ACTION_TO_COMMAND.keys():
		P_UI_GIVEN_A[k] = collections.OrderedDict()
		for u in LOW_LEVEL_COMMANDS:
			if u == TRUE_ACTION_TO_COMMAND[k]:
				P_UI_GIVEN_A[k][u] = 1.0 #try to weight the true command more for realistic purposes. Can be offset by using a high UI_GIVEN_A_NOISE
			else:
				# P_UI_GIVEN_A[k][u] = np.random.random()*UI_GIVEN_A_NOISE #IF UI_GIVEN_A_NOISE is 0, then the p(ui|a) is a deterministic mapping
				P_UI_GIVEN_A[k][u] = 0.0
		
		delta_dist = np.array(P_UI_GIVEN_A[k].values())
		uniform_dist = (1.0/len(LOW_LEVEL_COMMANDS)) * np.ones(len(LOW_LEVEL_COMMANDS))
		blended_dist = (1 - UI_GIVEN_A_NOISE) * delta_dist + UI_GIVEN_A_NOISE * uniform_dist #np.array
		for index, u in enumerate(LOW_LEVEL_COMMANDS):
			P_UI_GIVEN_A[k][u] = blended_dist[index]

		# normalization_constant = sum(P_UI_GIVEN_A[k].values())
		#normalize the entries for a valid probability distribution
		# P_UI_GIVEN_A[k] = collections.OrderedDict({u:(v/normalization_constant) for u, v in P_UI_GIVEN_A[k].items()})

def init_p_um_given_ui():
	'''
	Generates a random p(um|ui). key = ui, subkey = um
	'''
	global P_UM_GIVEN_UI
	for i in LOW_LEVEL_COMMANDS: #ui
		P_UM_GIVEN_UI[i] = collections.OrderedDict()
		for j in LOW_LEVEL_COMMANDS: #um
			if i == j:
				P_UM_GIVEN_UI[i][j] = 1.0#try to weight the true command more for realistic purposes. Can be offset by using a high UM_GIVEN_UI_NOISE
			else:
				# P_UM_GIVEN_UI[i][j] = np.random.random()*UM_GIVEN_UI_NOISE#IF UM_GIVEN_UI_NOISE is 0, then the p(um|ui) is a deterministic mapping
				P_UM_GIVEN_UI[i][j] = 0.0
		
		delta_dist = np.array(P_UM_GIVEN_UI[i].values())
		uniform_dist = (1.0/len(LOW_LEVEL_COMMANDS)) * np.ones(len(LOW_LEVEL_COMMANDS))
		blended_dist = (1 - UM_GIVEN_UI_NOISE) * delta_dist + UM_GIVEN_UI_NOISE * uniform_dist #np.array
		for index, j in enumerate(LOW_LEVEL_COMMANDS):
			P_UM_GIVEN_UI[i][j] = blended_dist[index]
		# normalization_constant = sum(P_UM_GIVEN_UI[i].values())
		#normalize the entries for a valid probability distribution
		# P_UM_GIVEN_UI[i] = collections.OrderedDict({u:(v/normalization_constant) for u, v in P_UM_GIVEN_UI[i].items()})
		


#SIMULATED HUMAN
def sample_a_given_s(s): #this is essentially a deterministic look up from optimal_action_dict
	global OPTIMAL_ACTION_DICT
	assert s in OPTIMAL_ACTION_DICT, "Error in key. Current state not in optimal action dict" #sanity check
	a = OPTIMAL_ACTION_DICT[s]
	return a

def sample_ui_given_a(a): #sample from p(ui|a)
	global P_UI_GIVEN_A
	p_vector = P_UI_GIVEN_A[a].values() #list of probabilities for ui
	ui_index_vector = np.random.multinomial(1, p_vector) #sample from the multinomial distribution with distribution p_vector
	ui_index = np.nonzero(ui_index_vector)[0][0] #grab the index of the index_vector which had a nonzero entry
	ui = P_UI_GIVEN_A[a].keys()[ui_index] #retrieve ui using the ui_index
	return ui

def sample_um_given_ui(ui): #sample from p(um|ui)
	global P_UM_GIVEN_UI
	p_vector = P_UM_GIVEN_UI[ui].values() #list of probabilities for um given ui
	um_index_vector = np.random.multinomial(1, p_vector) #sample from the multinomial distribution
	um_index = np.nonzero(um_index_vector)[0][0] #grab the index of the index_vector which had a nonzero entry
	um = P_UM_GIVEN_UI[ui].keys()[um_index] #retrieve um
	return um

def sample_sp_given_s_um(s, u): #return new state given current state and low-level command. If u = None, return current_state
	global STATE_TRANSITION_MODEL
	if u is not None:
		sp = STATE_TRANSITION_MODEL[s][u]
		return sp
	else:
		return s

def compute_p_ui_given_um(a, um): #inference of ui given um
	global P_UI_GIVEN_UM #need to use global because we are modifying a global dict
	for ui in LOW_LEVEL_COMMANDS:
		P_UI_GIVEN_UM[ui] = P_UM_GIVEN_UI[ui][um] * P_UI_GIVEN_A[a][ui] # BAYESIAN INFERENCE!!
	normalization_constant = sum(P_UI_GIVEN_UM.values())
	for u in P_UI_GIVEN_UM.keys(): #NORMALIZE POSTERIOR
		P_UI_GIVEN_UM[u] = P_UI_GIVEN_UM[u]/normalization_constant

def infer_intended_commands(a, um):
	'''
	Assistance algorithm. Follows the pseudocode in paper
	'''
	global P_UI_GIVEN_UM, ENTROPY_THRESHOLD
	compute_p_ui_given_um(a, um)
	p_ui_given_um_vector = np.array(P_UI_GIVEN_UM.values())
	p_ui_given_um_vector = p_ui_given_um_vector + np.finfo(p_ui_given_um_vector.dtype).tiny #need to add realmin to avoid nan issues with entropy calculation is p_ui_given_um_vector is delta distribution
	u_inferred = P_UI_GIVEN_UM.keys()[np.argmax(p_ui_given_um_vector)] #argmax computation for u_inferred

	#uniform distribution for compute max entropy. which is used as a normalizer. Could be moved to global scope
	uniform_distribution = np.array([1.0/p_ui_given_um_vector.size]*p_ui_given_um_vector.size)
	max_entropy = -np.dot(uniform_distribution, np.log2(uniform_distribution))

	#compute entropy
	normalized_h_of_p_ui_given_um = -np.dot(p_ui_given_um_vector, np.log2(p_ui_given_um_vector))/max_entropy
	if u_inferred != um:
		#check entropy to decide whether to intervene or not
		if normalized_h_of_p_ui_given_um < ENTROPY_THRESHOLD: #intervene
			if ASSISTANCE_TYPE == AssistanceType.Filter:
				u_corrected = None
			elif ASSISTANCE_TYPE == AssistanceType.Corrective:
				u_corrected = u_inferred
		else:
			u_corrected = um #Maybe keep this as None? because u intended is not same as um?
	else:
		u_corrected = um

	return u_corrected, (u_corrected == um), u_inferred, normalized_h_of_p_ui_given_um, (normalized_h_of_p_ui_given_um < ENTROPY_THRESHOLD), (u_inferred == um), (normalized_h_of_p_ui_given_um < ENTROPY_THRESHOLD) and (u_inferred != um)

def simulate_snp_interaction(args):
	'''
	Main simulation function.
	Input: args containing directory which contains all different combinations of simulation parameters. num_reps_per_condition = number of repetitions for each combination
	'''
	simulation_trial_dir = args.simulation_trial_dir
	num_reps_per_condition = args.num_reps_per_condition
	simulation_results_dir = args.simulation_results_dir
	if not os.path.exists(simulation_results_dir):
		os.makedirs(simulation_results_dir)

	for index, trial in enumerate(os.listdir(simulation_trial_dir)):
		global NUM_TURNS, NUM_LOCATIONS, UM_GIVEN_UI_NOISE, UI_GIVEN_A_NOISE, ENTROPY_THRESHOLD, TARGET_ORIENTATION, LOCATIONS, LOCATION_OF_TURN, STATES,ASSISTANCE_TYPE
		global P_UI_GIVEN_UM, P_UM_GIVEN_UI, P_UI_GIVEN_A, STATE_TRANSITION_MODEL, OPTIMAL_ACTION_DICT, OPTIMAL_NEXT_STATE_DICT, MODES_MOTION_ALLOWED
		P_UI_GIVEN_A = collections.OrderedDict()
		P_UM_GIVEN_UI =collections.OrderedDict()
		P_UI_GIVEN_UM = collections.OrderedDict()
		STATE_TRANSITION_MODEL = collections.OrderedDict()
		OPTIMAL_ACTION_DICT = collections.OrderedDict()
		OPTIMAL_NEXT_STATE_DICT = collections.OrderedDict()
		MODES_MOTION_ALLOWED = collections.OrderedDict()

		with open(os.path.join(simulation_trial_dir, str(index) + '.pkl'), 'rb') as fp:
			combination_dict = pickle.load(fp)
		print "COMBINATION NUM ", index
		print "      "
		NUM_TURNS = combination_dict['num_turns'] #number of turns in the path
		r_to_g_config = combination_dict['r_to_g_config']
		start_direction = combination_dict['start_direction']
		ASSISTANCE_TYPE = combination_dict['assistance_type']
		start_mode = combination_dict['start_mode']
		UI_GIVEN_A_NOISE = combination_dict['ui_given_a_noise']
		UM_GIVEN_UI_NOISE = combination_dict['um_given_ui_noise']
		ENTROPY_THRESHOLD = combination_dict['entropy_threshold']
		TARGET_ORIENTATION = combination_dict['target_orientation']

		#derived variables

		NUM_LOCATIONS = NUM_TURNS + 2 #total number of 'pitstops' = turns+start+end point
		LOCATION_OF_TURN = random.choice(range(1, NUM_LOCATIONS-1)) #The corner at which the robot needs to be turned. For simulations, this is never the 0th position.
		LOCATIONS = ['p' + str(i) for i in range(NUM_LOCATIONS)]

		#Generate list of states. State = (location, orientation, mode)
		STATES = [s for s in itertools.product(LOCATIONS, ORIENTATIONS, MODES)]

		init_modes_in_which_motion_allowed_dict(start_direction)
		create_state_transition_model()
		init_state_transition_model(r_to_g_config)

		create_optimal_next_state_dict()
		generate_optimal_control_dict()

		init_p_ui_given_a()
		init_p_um_given_ui()
		

		simulation_results = collections.OrderedDict()
		simulation_results['index'] = index
		simulation_results['combination_dict'] = combination_dict
		simulation_results['trials'] = collections.OrderedDict()

		for rep in range(0, num_reps_per_condition):
			current_state = (LOCATIONS[0], 0, start_mode)
			num_steps = 0
			simulation_results['trials'][rep] = collections.defaultdict(list)
			while current_state[0] != LOCATIONS[-1]:
				a = sample_a_given_s(current_state) #sample action given state #deterministic
				ui = sample_ui_given_a(a) #sample ui given action
				um = sample_um_given_ui(ui) #sample um given ui
				simulation_results['trials'][rep]['s'].append(current_state)
				simulation_results['trials'][rep]['a'].append(a)
				simulation_results['trials'][rep]['ui'].append(ui)
				simulation_results['trials'][rep]['um_before'].append(um)
				um_originally_issued_match_with_ground_truth = (um == TRUE_ACTION_TO_COMMAND[a])
				if ASSISTANCE_TYPE != AssistanceType.No_Assistance: #if assistance flag is true, activate assistanced
					u_corrected, is_u_corrected_equals_um, u_inferred, normalized_h_of_p_ui_given_um, is_normalized_entropy_less_than_threshold, is_u_inferred_equals_um, did_assistance_intervene = infer_intended_commands(a, um)

					assistance_match_with_ground_truth = (u_corrected == TRUE_ACTION_TO_COMMAND[a])
					u_inferred_match_with_ground_truth = (u_inferred == TRUE_ACTION_TO_COMMAND[a])
					simulation_results['trials'][rep]['um_originally_issued_match_with_ground_truth'].append(um_originally_issued_match_with_ground_truth) #can happen despite randomness. 
					simulation_results['trials'][rep]['u_inferred_match_with_ground_truth'].append(u_inferred_match_with_ground_truth)
					simulation_results['trials'][rep]['assistance_match_with_ground_truth'].append(assistance_match_with_ground_truth)
					simulation_results['trials'][rep]['u_inferred'].append(u_inferred)
					simulation_results['trials'][rep]['did_assistance_intervene'].append(did_assistance_intervene)
					simulation_results['trials'][rep]['normalized_h_of_p_ui_given_um'].append(normalized_h_of_p_ui_given_um)
					simulation_results['trials'][rep]['is_normalized_entropy_less_than_threshold'].append(is_normalized_entropy_less_than_threshold)
					simulation_results['trials'][rep]['is_u_corrected_equals_um'].append(is_u_corrected_equals_um)
					simulation_results['trials'][rep]['is_u_inferred_equals_um'].append(is_u_inferred_equals_um)
					simulation_results['trials'][rep]['um_after'].append(u_corrected)
					next_state = sample_sp_given_s_um(current_state, u_corrected) #sample next state
					simulation_results['trials'][rep]['sp'].append(next_state)
				else:
					# assistance_match_with_ground_truth = (um == TRUE_ACTION_TO_COMMAND[a]) #the 
					simulation_results['trials'][rep]['um_originally_issued_match_with_ground_truth'].append(um_originally_issued_match_with_ground_truth)
					simulation_results['trials'][rep]['assistance_match_with_ground_truth'].append(None) #NA for No Assistance
					simulation_results['trials'][rep]['u_inferred_match_with_ground_truth'].append(None)
					simulation_results['trials'][rep]['u_inferred'].append(None) #NA for No Assistance
					simulation_results['trials'][rep]['did_assistance_intervene'].append(None)
					simulation_results['trials'][rep]['normalized_h_of_p_ui_given_um'].append(None)
					simulation_results['trials'][rep]['is_normalized_entropy_less_than_threshold'].append(None)
					simulation_results['trials'][rep]['is_u_corrected_equals_um'].append(None)
					simulation_results['trials'][rep]['is_u_inferred_equals_um'].append(None)
					simulation_results['trials'][rep]['um_after'].append(um) #same as um_before for no assistance
					next_state = sample_sp_given_s_um(current_state, um) #sample next state
					simulation_results['trials'][rep]['sp'].append(next_state)
				num_steps += 1 #number of steps.
				current_state = next_state
				if num_steps > MAX_SIM_STEPS:
					print("Reached max steps")
					break

			simulation_results['trials'][rep]['total_steps'] = num_steps

		for k, v in simulation_results['trials'].items():
			individual_sim_result = collections.OrderedDict()
			individual_sim_result['combination_dict'] = combination_dict
			individual_sim_result['data'] = v
			individual_sim_result['index'] = k + index*(num_reps_per_condition)
			print(individual_sim_result['index'])
			simulation_result_file_path = os.path.join(simulation_results_dir, 'sim_' + str(k + index*num_reps_per_condition) + '.pkl')
			with open(simulation_result_file_path, 'wb') as fp:
				pickle.dump(individual_sim_result, fp)

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--simulation_trial_dir', dest='simulation_trial_dir',default=os.path.join(os.path.dirname(os.getcwd()), 'simulated_human_experiments', 'simulation_trial_dir'), help="The directory where trials will be stored are")
	parser.add_argument('--num_reps_per_condition', action='store', type=int, default=10, help="number of repetetions for single combination of conditions ")
	parser.add_argument('--simulation_results_dir', dest='simulation_results_dir',default=os.path.join(os.path.dirname(os.getcwd()), 'simulated_human_experiments', 'simulation_results_with_higher_entropy_with_proper_noise'), help="The directory where the simulation trials will be stored")

	args = parser.parse_args()
	simulate_snp_interaction(args)
