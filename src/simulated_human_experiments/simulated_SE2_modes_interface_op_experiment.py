import os
import sys
import copy
import argparse
from mdp.mdp_discrete_SE2_gridworld_with_modes import MDPDiscreteSE2GridWorldWithModes
from mdp.mdp_utils import *
import collections 
import pickle
import matplotlib.pyplot as plt

NUM_GOALS = -1
OCCUPANCY_LEVEL = -0.2
MAX_PATCHES = 2

GRID_WIDTH = 10
GRID_HEIGHT = 10
NUM_ORIENTATIONS = 6

#low level commands issued by the snp interface. hp = hard puff, hs= hard sip, sp = soft puff, ss = soft sip. Also the domain for ui and um
INTERFACE_LEVEL_ACTIONS = ['hp', 'hs', 'sp', 'ss']
#high level actions, move_p = move in positive direction, move_n = move in negative direction, mode_r = switch mode to right, mode_l = switch mode to left. positive and negative is conditioned on mode
TASK_LEVEL_ACTIONS = ['move_p', 'move_n', 'to_mode_r', 'to_mode_l']
#true mapping of a to phi
TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP = collections.OrderedDict({'move_p': 'sp', 'move_n':'ss', 'to_mode_r':'hp', 'to_mode_l': 'hs'})
#true inverse mapping of phi to a
TRUE_INTERFACE_ACTION_TO_TASK_ACTION_MAP = collections.OrderedDict({v:k for k, v in TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP.items()})

#p(phii|a)
P_PHI_GIVEN_A = collections.OrderedDict()
PHI_GIVEN_A_NOISE = -0.1 #Lower the number, lower the error. Between 0 and 1. If 0, the p(ui|a) is delta and same as the true mapping

#p(phm|ui)
P_PHM_GIVEN_PHI = collections.OrderedDict()
PHM_GIVEN_PHI_NOISE = -0.1 #Lower the number, lower the error. Between 0 and 1. If 0, no difference between ui and um

P_G_GIVEN_UM = collections.OrderedDict()

ENTROPY_THRESHOLD = -0.5

def visualize_grid(mdp):
    mdp_params = mdp.get_env_params()
    grid = np.zeros((mdp_params['grid_width'], mdp_params['grid_height']))
    for obs in mdp_params['mdp_obstacles']:
        grid[obs[0], obs[1]] = 1
    
    gs = mdp_params['mdp_goal_state']
    grid[gs[0], gs[1]] = 0.5
    grid = grid.T
    grid = np.flipud(grid)
    plt.imshow(grid)
    plt.colorbar()
    plt.show()

def visualize_V_and_policy(mdp):
    mdp_params = mdp.get_env_params()
    robot_type = mdp_params['robot_type'] #SE2
    mode_set_type = mdp_params['mode_set_type'] # assert this is one D. As the task level actions are for 1D interfaces. 
    mode_set = CARTESIAN_MODE_SET_OPTIONS[robot_type][mode_set_type]
    num_modes = len(mode_set)
    
    V = np.array(mdp.get_value_function()).reshape((mdp_params['grid_width'],mdp_params['grid_height'], mdp_params['num_discrete_orientations'], num_modes))
    
    fig, ax = plt.subplots(mdp_params['num_discrete_orientations'], num_modes)
    for i in range(num_modes):
        for j in range(mdp_params['num_discrete_orientations']):
            Va = np.flipud(V[:, :, j, i].T)
            vmin = np.percentile(Va, 1)
            vmax = np.percentile(Va, 99)

            im = ax[j, i].imshow(Va, vmin=vmin, vmax=vmax)
            cbar = ax[j, i].figure.colorbar(im, ax=ax[j, i])
            cbar.ax.set_ylabel("V", rotation=-90, va="bottom")
    
    fig.tight_layout()
    plt.show()

def create_mdp_list(mdp_env_params, visualize=False):
    mdp_list = []
    for i, g in enumerate(mdp_env_params['all_goals']):
        mdp_env_params['mdp_goal_state'] = g
        goals_that_are_obs = [(g_obs[0], g_obs[1]) for g_obs in mdp_env_params['all_goals'] if g_obs != g] #2d goals. 
        mdp_env_params['mdp_obstacles'] = copy.deepcopy(mdp_env_params['original_mdp_obstacles'])
        mdp_env_params['mdp_obstacles'].extend(goals_that_are_obs)
        discrete_se2_modes_mdp = MDPDiscreteSE2GridWorldWithModes(copy.deepcopy(mdp_env_params))

        if visualize:
            visualize_grid(discrete_se2_modes_mdp)
            visualize_V_and_policy(discrete_se2_modes_mdp)
        mdp_list.append(discrete_se2_modes_mdp)
    
    return mdp_list

def create_mdp_env_param_dict():
    
    mdp_env_params = collections.OrderedDict()
    mdp_env_params['rl_algo_type'] = RlAlgoType.ValueIteration
    mdp_env_params['gamma'] = 0.96
    mdp_env_params['grid_width'] = GRID_WIDTH
    mdp_env_params['grid_height'] = GRID_HEIGHT
    mdp_env_params['num_discrete_orientations'] = NUM_ORIENTATIONS
    mdp_env_params['robot_type'] = CartesianRobotType.SE2
    mdp_env_params['mode_set_type'] = ModeSetType.OneD
    num_patches = 2
    if OCCUPANCY_LEVEL == 0.0:
        mdp_env_params['original_mdp_obstacles'] = []
    else:
        mdp_env_params['original_mdp_obstacles'] = create_random_obstacles(width=mdp_env_params['grid_width'],
                                                                height=mdp_env_params['grid_height'],
                                                                occupancy_measure=OCCUPANCY_LEVEL,
                                                                num_obstacle_patches=num_patches)
    
    print('OBSTACLES', mdp_env_params['original_mdp_obstacles'])
    
    goal_list = create_random_goals( width=mdp_env_params['grid_width'],
                                                            height=mdp_env_params['grid_height'],
                                                            num_goals=NUM_GOALS,
                                                            obstacle_list=mdp_env_params['original_mdp_obstacles']) #make the list a tuple
    

    for i, g in enumerate(goal_list):
        g = list(g)
        g.append(np.random.randint(mdp_env_params['num_discrete_orientations']))
        goal_list[i] = tuple(g)
    
    print(goal_list)
    
    mdp_env_params['all_goals'] = goal_list
    mdp_env_params['obstacle_penalty'] = -100
    mdp_env_params['goal_reward'] = 100
    mdp_env_params['step_penalty'] = -10
    mdp_env_params['sparsity_factor'] = 0.0
    mdp_env_params['rand_direction_factor'] = RAND_DIRECTION_FACTOR
    mdp_env_params['mdp_obstacles'] = []

    return mdp_env_params

def init_P_PHI_GIVEN_A():
	'''
	Generate a random p(phi | a). key = a, subkey = ui
	'''
	global P_PHI_GIVEN_A
	for k in TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP.keys(): #task level action
		P_PHI_GIVEN_A[k] = collections.OrderedDict()
		for u in INTERFACE_LEVEL_ACTIONS:
			if u == TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[k]:
				P_PHI_GIVEN_A[k][u] = 1.0 #try to weight the true command more for realistic purposes. Can be offset by using a high PHI_GIVEN_A_NOISE
			else:
				# P_PHI_GIVEN_A[k][u] = np.random.random()*PHI_GIVEN_A_NOISE #IF PHI_GIVEN_A_NOISE is 0, then the p(ui|a) is a deterministic mapping
				P_PHI_GIVEN_A[k][u] = 0.0
		
		delta_dist = np.array(P_PHI_GIVEN_A[k].values())
		uniform_dist = (1.0/len(INTERFACE_LEVEL_ACTIONS)) * np.ones(len(INTERFACE_LEVEL_ACTIONS))
		blended_dist = (1 - PHI_GIVEN_A_NOISE) * delta_dist + PHI_GIVEN_A_NOISE * uniform_dist #np.array
		for index, u in enumerate(INTERFACE_LEVEL_ACTIONS):
			P_PHI_GIVEN_A[k][u] = blended_dist[index]

		# normalization_constant = sum(P_PHI_GIVEN_A[k].values())
		#normalize the entries for a valid probability distribution
		# P_PHI_GIVEN_A[k] = collections.OrderedDict({u:(v/normalization_constant) for u, v in P_PHI_GIVEN_A[k].items()})

def init_P_PHM_GIVEN_PHI():
	'''
	Generates a random p(um|ui). key = ui, subkey = um
	'''
	global P_PHM_GIVEN_PHI
	for i in INTERFACE_LEVEL_ACTIONS: #ui
		P_PHM_GIVEN_PHI[i] = collections.OrderedDict()
		for j in INTERFACE_LEVEL_ACTIONS: #um
			if i == j:
				P_PHM_GIVEN_PHI[i][j] = 1.0#try to weight the true command more for realistic purposes. Can be offset by using a high UM_GIVEN_UI_NOISE
			else:
				# P_PHM_GIVEN_PHI[i][j] = np.random.random()*UM_GIVEN_UI_NOISE#IF UM_GIVEN_UI_NOISE is 0, then the p(um|ui) is a deterministic mapping
				P_PHM_GIVEN_PHI[i][j] = 0.0
		
		delta_dist = np.array(P_PHM_GIVEN_PHI[i].values())
		uniform_dist = (1.0/len(INTERFACE_LEVEL_ACTIONS)) * np.ones(len(INTERFACE_LEVEL_ACTIONS))
		blended_dist = (1 - PHM_GIVEN_PHI_NOISE) * delta_dist + PHM_GIVEN_PHI_NOISE * uniform_dist #np.array
		for index, j in enumerate(INTERFACE_LEVEL_ACTIONS):
			P_PHM_GIVEN_PHI[i][j] = blended_dist[index]
		
def compute_p_g_given_phm(phm, mdp): #inference of ui given um
	global P_G_GIVEN_UM #need to use global because we are modifying a global dict
    # TODO finish inference equation
    # p(gt | phm0:t) = p(g_{t-1}| phm0:{t-1}) * Sig(a)Sig(phi) p(phmt|phi).p(phi|a)p(a|s, gt)
	
        
def sample_phi_given_a(a): #sample from p(phii|a)
	global P_PHI_GIVEN_A
	p_vector = P_PHI_GIVEN_A[a].values() #list of probabilities for phii
	phi_index_vector = np.random.multinomial(1, p_vector) #sample from the multinomial distribution with distribution p_vector
	phi_index = np.nonzero(phi_index_vector)[0][0] #grab the index of the index_vector which had a nonzero entry
	phi = P_PHI_GIVEN_A[a].keys()[phi_index] #retrieve phii using the phi_index
	return phi

def sample_phm_given_phi(phi): #sample from p(phm|phi)
	global P_PHM_GIVEN_PHI
	p_vector = P_PHM_GIVEN_PHI[phi].values() #list of probabilities for phm given phi
	phm_index_vector = np.random.multinomial(1, p_vector) #sample from the multinomial distribution
	phm_index = np.nonzero(phm_index_vector)[0][0] #grab the index of the index_vector which had a nonzero entry
	phm = P_PHM_GIVEN_PHI[phi].keys()[phm_index] #retrieve phm
	return phm

def simulate_snp_interaction(args):
    simulation_trial_dir = args.simulation_trial_dir
    for index, trial in enumerate(os.listdir(simulation_trial_dir)):
        global PHI_GIVEN_A_NOISE, PHM_GIVEN_PHI_NOISE, ENTROPY_THRESHOLD, NUM_GOALS, OCCUPANCY_LEVEL, RAND_DIRECTION_FACTOR
        with open(os.path.join(simulation_trial_dir, str(index) + '.pkl'), 'rb') as fp:
            combination_dict = pickle.load(fp)
        print "COMBINATION NUM ", index
        print "      "

        #grab params for particular simulation trial
        NUM_GOALS = combination_dict['num_goals']
        OCCUPANCY_LEVEL = combination_dict['occupancy_level']
        RAND_DIRECTION_FACTOR = combination_dict['rand_direction_factor']

        PHI_GIVEN_A_NOISE = combination_dict['phi_given_a_noise']
        PHM_GIVEN_PHI_NOISE = combination_dict['phm_given_phi_noise']
        ENTROPY_THRESHOLD = combination_dict['entropy_threshold']

        mdp_env_params = create_mdp_env_param_dict()
        mdp_list = create_mdp_list(mdp_env_params, visualize=False)
        num_reps_per_condition = args.num_reps_per_condition # number of repetitions for a world

        #initalize the distributions
        init_P_PHI_GIVEN_A()
        init_P_PHM_GIVEN_PHI()
        
        # sample a random goal for the trial. We assume that for a given trial, the goal is fixed but unknown.
        random_goal_id = np.random.randint(NUM_GOALS)
        random_goal = mdp_env_params['all_goals'][random_goal_id]
        print('RANDOM GOAL', random_goal)
        mdp = mdp_list[random_goal_id] #mdp corresponding to the sampled goal
        
        for rep in range(num_reps_per_condition):
            print('REP NUM', rep)
            s = mdp.get_random_valid_state() #pick a random state from the mdp that has the random goal.  to start with. 
            # s is state, a = task_level action phi_i = true interface level action, phi_m = corrupted interface level action
            print('current_state', s)
            while not mdp._check_if_state_coord_is_goal_state(s):
                a = mdp.get_optimal_action(s, return_optimal=False) # task level action
                if a is not 'None':
                    phi = sample_phi_given_a(a) # pure interface level action
                    phm = sample_phm_given_phi(phi) # corrupted interface level action
                    applied_a = TRUE_INTERFACE_ACTION_TO_TASK_ACTION_MAP[phm] # task level action corresponding to corrupted interface level action
                    sp = mdp.get_next_state_from_state_action(s, applied_a)
                else:
                    sp = s
                    phi = 'None'
                    phm = 'None'
                    applied_a = 'None'
                
                print('task action, phi, phm, applied_a', a, phi, phm, applied_a)
                print('next_state',sp)
                s = sp
            break #temporary break for testing
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--simulation_trial_dir', dest='simulation_trial_dir',default=os.path.join(os.path.dirname(os.getcwd()), 'simulated_human_experiments', 'se2_simulation_trial_dir'), help="The directory where trials will be stored are")
    parser.add_argument('--num_reps_per_condition', action='store', type=int, default=3, help="number of repetetions for single combination of conditions ")
    args = parser.parse_args()
    simulate_snp_interaction(args)