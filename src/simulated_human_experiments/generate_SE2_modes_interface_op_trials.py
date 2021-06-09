import os
import pickle
import itertools
import argparse
import collections
import sys

MAX_SIM_STEPS = 200
NUM_GOALS_LEVELS = [2,3,4]
OCCUPANCY_LEVELS = [0.05, 0.1]
RAND_DIRECTION_LEVELS = [0.0, 0.2, 0.4, 0.8]
PHI_GIVEN_A_NOISE = [i/10.0 for i in range(1, 9, 2)]
PHM_GIVEN_PHI_NOISE = [i/10.0 for i in range(1, 9, 2)]
ENTROPY_THRESHOLD = [i/10.0 for i in range(5,10)]
ASSISTANCE_TYPE = ['no', 'filter', 'corrective']


def generate_SE2_modes_interface_op_simulation_trials(args):
    simulation_trial_dir = args.simulation_trial_dir

    if not os.path.exists(simulation_trial_dir):
        os.makedirs(simulation_trial_dir)
    
    trial_combinations = []
    i = 0
    for (num_goals, occupancy_level, rand_direction_factor, phi_given_a_noise, phm_given_phi_noise, entropy_threshold, assistance_type) in itertools.product(NUM_GOALS_LEVELS, OCCUPANCY_LEVELS, RAND_DIRECTION_LEVELS, PHI_GIVEN_A_NOISE, PHM_GIVEN_PHI_NOISE, ENTROPY_THRESHOLD, ASSISTANCE_TYPE):
        combination_dict = collections.OrderedDict()
        combination_dict['num_goals'] = num_goals
        combination_dict['occupancy_level'] = occupancy_level
        combination_dict['rand_direction_factor'] = rand_direction_factor
        combination_dict['phi_given_a_noise'] = phi_given_a_noise
        combination_dict['phm_given_phi_noise'] = phm_given_phi_noise
        combination_dict['entropy_threshold'] = entropy_threshold
        combination_dict['assistance_type'] = assistance_type

        print('Creating trial combination ', i)
        with open(os.path.join(simulation_trial_dir, str(i) + '.pkl'), 'wb') as fp:
            pickle.dump(combination_dict, fp)
        
        i+=1
        
if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument('--simulation_trial_dir', dest='simulation_trial_dir',default=os.path.join(os.getcwd(), 'se2_simulation_trial_dir'), help="The directory where trials will be stored are")
	args = parser.parse_args()
	generate_SE2_modes_interface_op_simulation_trials(args)
