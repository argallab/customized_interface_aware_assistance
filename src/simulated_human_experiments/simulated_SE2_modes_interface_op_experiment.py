import os
import sys
import copy
import argparse
from mdp.mdp_discrete_SE2_gridworld_with_modes import MDPDiscreteSE2GridWorldWithModes
from mdp.mdp_utils import *
import collections
import pickle
import matplotlib.pyplot as plt

MAX_SIM_STEPS = 200

NUM_GOALS = -1
OCCUPANCY_LEVEL = -0.2
MAX_PATCHES = 2

GRID_WIDTH = 10
GRID_HEIGHT = 10
NUM_ORIENTATIONS = 6

# low level commands issued by the snp interface. hp = hard puff, hs= hard sip, sp = soft puff, ss = soft sip. Also the domain for ui and um
INTERFACE_LEVEL_ACTIONS = ["hp", "hs", "sp", "ss"]
# high level actions, move_p = move in positive direction, move_n = move in negative direction, mode_r = switch mode to right, mode_l = switch mode to left. positive and negative is conditioned on mode
TASK_LEVEL_ACTIONS = ["move_p", "move_n", "to_mode_r", "to_mode_l"]
# true mapping of a to phi
TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP = collections.OrderedDict(
    {"move_p": "sp", "move_n": "ss", "to_mode_r": "hp", "to_mode_l": "hs"}
)
# true inverse mapping of phi to a
TRUE_INTERFACE_ACTION_TO_TASK_ACTION_MAP = collections.OrderedDict(
    {v: k for k, v in TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP.items()}
)

# p(phii|a)
P_PHI_GIVEN_A = collections.OrderedDict()
# Lower the number, lower the error. Between 0 and 1. If 0, the p(ui|a) is delta and same as the true mapping
PHI_GIVEN_A_NOISE = -0.1

# p(phm|ui)
P_PHM_GIVEN_PHI = collections.OrderedDict()
PHM_GIVEN_PHI_NOISE = -0.1  # Lower the number, lower the error. Between 0 and 1. If 0, no difference between ui and um

P_G_GIVEN_PHM = collections.OrderedDict()  # belief using interface aware model
P_G_GIVEN_UM_NOISE_FREE = collections.OrderedDict()  # belief without interface aware model.

ENTROPY_THRESHOLD = -0.5
ASSISTANCE_TYPE = "blah"
RAND_DIRECTION_FACTOR = 0.0
SPARSITY_FACTOR = 0.0
PHI_SPARSE_LEVEL = 0.0
PHM_SPARSE_LEVEL = 0.0


def visualize_grid(mdp):
    mdp_params = mdp.get_env_params()
    grid = np.zeros((mdp_params["grid_width"], mdp_params["grid_height"]))
    for obs in mdp_params["mdp_obstacles"]:
        grid[obs[0], obs[1]] = 1

    gs = mdp_params["mdp_goal_state"]
    grid[gs[0], gs[1]] = 0.5
    grid = grid.T
    grid = np.flipud(grid)
    plt.imshow(grid)
    plt.colorbar()
    plt.show()


def visualize_V_and_policy(mdp):
    mdp_params = mdp.get_env_params()
    robot_type = mdp_params["robot_type"]  # SE2
    mode_set_type = mdp_params[
        "mode_set_type"
    ]  # assert this is one D. As the task level actions are for 1D interfaces.
    mode_set = CARTESIAN_MODE_SET_OPTIONS[robot_type][mode_set_type]
    num_modes = len(mode_set)

    V = np.array(mdp.get_value_function()).reshape(
        (mdp_params["grid_width"], mdp_params["grid_height"], mdp_params["num_discrete_orientations"], num_modes)
    )

    fig, ax = plt.subplots(mdp_params["num_discrete_orientations"], num_modes)
    for i in range(num_modes):
        for j in range(mdp_params["num_discrete_orientations"]):
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
    for i, g in enumerate(mdp_env_params["all_goals"]):
        mdp_env_params["mdp_goal_state"] = g
        goals_that_are_obs = [(g_obs[0], g_obs[1]) for g_obs in mdp_env_params["all_goals"] if g_obs != g]  # 2d goals.
        mdp_env_params["mdp_obstacles"] = copy.deepcopy(mdp_env_params["original_mdp_obstacles"])
        mdp_env_params["mdp_obstacles"].extend(goals_that_are_obs)
        discrete_se2_modes_mdp = MDPDiscreteSE2GridWorldWithModes(copy.deepcopy(mdp_env_params))

        if visualize:
            visualize_grid(discrete_se2_modes_mdp)
            visualize_V_and_policy(discrete_se2_modes_mdp)
        mdp_list.append(discrete_se2_modes_mdp)

    return mdp_list


def create_mdp_env_param_dict():

    mdp_env_params = collections.OrderedDict()
    mdp_env_params["rl_algo_type"] = RlAlgoType.ValueIteration
    mdp_env_params["gamma"] = 0.96
    mdp_env_params["grid_width"] = GRID_WIDTH
    mdp_env_params["grid_height"] = GRID_HEIGHT
    mdp_env_params["num_discrete_orientations"] = NUM_ORIENTATIONS
    mdp_env_params["robot_type"] = CartesianRobotType.SE2
    mdp_env_params["mode_set_type"] = ModeSetType.OneD
    num_patches = 2
    if OCCUPANCY_LEVEL == 0.0:
        mdp_env_params["original_mdp_obstacles"] = []
    else:
        mdp_env_params["original_mdp_obstacles"] = create_random_obstacles(
            width=mdp_env_params["grid_width"],
            height=mdp_env_params["grid_height"],
            occupancy_measure=OCCUPANCY_LEVEL,
            num_obstacle_patches=num_patches,
        )

    print ("OBSTACLES", mdp_env_params["original_mdp_obstacles"])

    goal_list = create_random_goals(
        width=mdp_env_params["grid_width"],
        height=mdp_env_params["grid_height"],
        num_goals=NUM_GOALS,
        obstacle_list=mdp_env_params["original_mdp_obstacles"],
    )  # make the list a tuple

    for i, g in enumerate(goal_list):
        g = list(g)
        g.append(np.random.randint(mdp_env_params["num_discrete_orientations"]))
        goal_list[i] = tuple(g)

    print (goal_list)

    mdp_env_params["all_goals"] = goal_list
    mdp_env_params["obstacle_penalty"] = -100
    mdp_env_params["goal_reward"] = 100
    mdp_env_params["step_penalty"] = -10
    mdp_env_params["sparsity_factor"] = SPARSITY_FACTOR
    mdp_env_params["rand_direction_factor"] = RAND_DIRECTION_FACTOR
    mdp_env_params["mdp_obstacles"] = []

    return mdp_env_params


def init_P_PHI_GIVEN_A():
    """
    Generate a random p(phi | a). key = a, subkey = ui
    """
    global P_PHI_GIVEN_A
    for k in TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP.keys():  # task level action
        P_PHI_GIVEN_A[k] = collections.OrderedDict()
        for u in INTERFACE_LEVEL_ACTIONS:
            if u == TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[k]:
                # try to weight the true command more for realistic purposes. Can be offset by using a high PHI_GIVEN_A_NOISE
                P_PHI_GIVEN_A[k][u] = 1.0
            else:
                # P_PHI_GIVEN_A[k][u] = np.random.random()*PHI_GIVEN_A_NOISE #IF PHI_GIVEN_A_NOISE is 0, then the p(ui|a) is a deterministic mapping
                P_PHI_GIVEN_A[k][u] = 0.0

        delta_dist = np.array(P_PHI_GIVEN_A[k].values())
        uniform_dist = (1.0 / len(INTERFACE_LEVEL_ACTIONS)) * np.ones(len(INTERFACE_LEVEL_ACTIONS))
        blended_dist = (1 - PHI_GIVEN_A_NOISE) * delta_dist + PHI_GIVEN_A_NOISE * uniform_dist  # np.array
        for index, u in enumerate(INTERFACE_LEVEL_ACTIONS):
            P_PHI_GIVEN_A[k][u] = blended_dist[index]

        # normalization_constant = sum(P_PHI_GIVEN_A[k].values())
        # normalize the entries for a valid probability distribution
        # P_PHI_GIVEN_A[k] = collections.OrderedDict({u:(v/normalization_constant) for u, v in P_PHI_GIVEN_A[k].items()})


def init_P_PHM_GIVEN_PHI():
    """
    Generates a random p(um|ui). key = ui, subkey = um
    """
    global P_PHM_GIVEN_PHI
    for i in INTERFACE_LEVEL_ACTIONS:  # ui
        P_PHM_GIVEN_PHI[i] = collections.OrderedDict()
        for j in INTERFACE_LEVEL_ACTIONS:  # um
            if i == j:
                # try to weight the true command more for realistic purposes. Can be offset by using a high UM_GIVEN_UI_NOISE
                P_PHM_GIVEN_PHI[i][j] = 1.0
            else:
                # P_PHM_GIVEN_PHI[i][j] = np.random.random()*UM_GIVEN_UI_NOISE#IF UM_GIVEN_UI_NOISE is 0, then the p(um|ui) is a deterministic mapping
                P_PHM_GIVEN_PHI[i][j] = 0.0

        delta_dist = np.array(P_PHM_GIVEN_PHI[i].values())
        uniform_dist = (1.0 / len(INTERFACE_LEVEL_ACTIONS)) * np.ones(len(INTERFACE_LEVEL_ACTIONS))
        blended_dist = (1 - PHM_GIVEN_PHI_NOISE) * delta_dist + PHM_GIVEN_PHI_NOISE * uniform_dist  # np.array
        for index, j in enumerate(INTERFACE_LEVEL_ACTIONS):
            P_PHM_GIVEN_PHI[i][j] = blended_dist[index]


def init_P_G_GIVEN_PHM():
    """
    Initializes the p(g | phm) dict to uniform dictionary
    """
    global NUM_GOALS, P_G_GIVEN_PHM
    for g in range(NUM_GOALS):
        P_G_GIVEN_PHM[g] = 1.0 / NUM_GOALS

    normalization_constant = sum(P_G_GIVEN_PHM.values())
    for g in P_G_GIVEN_PHM.keys():  # NORMALIZE POSTERIOR
        P_G_GIVEN_PHM[g] = P_G_GIVEN_PHM[g] / normalization_constant


def init_P_G_GIVEN_PHM_NOISE_FREE():
    """
    Initializes the p(g | phm)_noise free dict to uniform dictionary
    """
    global NUM_GOALS, P_G_GIVEN_UM_NOISE_FREE
    for g in range(NUM_GOALS):
        P_G_GIVEN_UM_NOISE_FREE[g] = 1.0 / NUM_GOALS

    normalization_constant = sum(P_G_GIVEN_UM_NOISE_FREE.values())
    for g in P_G_GIVEN_UM_NOISE_FREE.keys():  # NORMALIZE POSTERIOR
        P_G_GIVEN_UM_NOISE_FREE[g] = P_G_GIVEN_UM_NOISE_FREE[g] / normalization_constant


def compute_p_g_given_phm(phm, mdp_list, s):  # inference of ui given um
    global P_G_GIVEN_PHM, P_PHM_GIVEN_PHI, P_PHI_GIVEN_A  # need to use global because we are modifying a global dict
    # p(gt | phm0:t) = p(g_{t-1}| phm0:{t-1}) * Sig(a)Sig(phi) p(phmt|phi).p(phi|a)p(a|s, gt)
    if phm != "None":
        for g in P_G_GIVEN_PHM.keys():
            mdp_g = mdp_list[g]  # get mdp corresponding to g to retrieve p(a|s, g)
            likelihood = 0  # likelihood
            for a in P_PHI_GIVEN_A.keys():
                for phi in P_PHM_GIVEN_PHI.keys():
                    likelihood += P_PHM_GIVEN_PHI[phi][phm] * P_PHI_GIVEN_A[a][phi] * mdp_g.get_prob_a_given_s(s, a)

            P_G_GIVEN_PHM[g] = P_G_GIVEN_PHM[g] * likelihood  # multiply with prior

        normalization_constant = sum(P_G_GIVEN_PHM.values())
        for g in P_G_GIVEN_PHM.keys():  # NORMALIZE POSTERIOR
            P_G_GIVEN_PHM[g] = P_G_GIVEN_PHM[g] / normalization_constant
    else:
        print ("PHM NONE, therefore no belief update")
        # potentially add exp decay to unfirom distribution
        pass


def compute_p_g_given_phm_noise_free(phm, mdp_list, s):
    global P_G_GIVEN_UM_NOISE_FREE
    if phm != "None":
        # noise free assumptions. Therefore ph intended is same as ph measure
        phi = phm
        # task-level action intended is inverse of phi. No noise in retrieval process.
        a = TRUE_INTERFACE_ACTION_TO_TASK_ACTION_MAP[phi]
        for g in P_G_GIVEN_UM_NOISE_FREE.keys():
            mdp_g = mdp_list[g]
            P_G_GIVEN_UM_NOISE_FREE[g] = P_G_GIVEN_UM_NOISE_FREE[g] * mdp_g.get_prob_a_given_s(s, a)
        normalization_constant = sum(P_G_GIVEN_UM_NOISE_FREE.values())
        for g in P_G_GIVEN_UM_NOISE_FREE.keys():
            P_G_GIVEN_UM_NOISE_FREE[g] = P_G_GIVEN_UM_NOISE_FREE[g] / normalization_constant
    else:
        print ("PHM NONE, therefore no belief update")
        # potentially add exp decay to uniform distribution
        pass


def compute_g_a_ph_inferred(mdp_list, s, use_full_model=True):
    if use_full_model:
        global P_G_GIVEN_PHM
        p_g_given_um_vector = np.array(P_G_GIVEN_PHM.values())
        # need to add realmin to avoid nan issues with entropy calculation is p_ui_given_um_vector is delta distribution'
        p_g_given_um_vector = p_g_given_um_vector + np.finfo(p_g_given_um_vector.dtype).tiny
        g_inferred = P_G_GIVEN_PHM.keys()[np.argmax(p_g_given_um_vector)]  # argmax computation for g_inferred
        # retreive optimal task level action corresponding to inferred goal optimal action will always be not None
        a_inferred = mdp_list[g_inferred].get_optimal_action(s, return_optimal=True)
        # retrieve interface level action corresponding to optimal task level action for inferred goal
        # ph_inferred is always going to be not None.
        ph_inferred = TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[a_inferred]
        return g_inferred, a_inferred, ph_inferred, p_g_given_um_vector
    else:
        global P_G_GIVEN_UM_NOISE_FREE
        p_g_given_um_noise_free_vector = np.array(P_G_GIVEN_UM_NOISE_FREE.values())
        p_g_given_um_noise_free_vector = (
            p_g_given_um_noise_free_vector + np.finfo(p_g_given_um_noise_free_vector.dtype).tiny
        )
        g_inferred = P_G_GIVEN_UM_NOISE_FREE.keys()[np.argmax(p_g_given_um_noise_free_vector)]
        a_inferred = mdp_list[g_inferred].get_optimal_action(s, return_optimal=True)
        ph_inferred = TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[a_inferred]
        return g_inferred, a_inferred, ph_inferred, p_g_given_um_noise_free_vector


# def infer_intended_goal_and_commands(phm, mdp_list, s):
#     compute_p_g_given_phm(phm, mdp_list, s) #update goal belief

#     global P_G_GIVEN_PHM, ENTROPY_THRESHOLD, ASSISTANCE_TYPE
#     p_g_given_um_vector = np.array(P_G_GIVEN_PHM.values())
#     p_g_given_um_vector = p_g_given_um_vector + np.finfo(p_g_given_um_vector.dtype).tiny #need to add realmin to avoid nan issues with entropy calculation is p_ui_given_um_vector is delta distribution
#     g_inferred = P_G_GIVEN_PHM.keys()[np.argmax(p_g_given_um_vector)] #argmax computation for g_inferred
#     # retreive optimal task level action corresponding to inffered goal
#     a_inferred = mdp_list[g_inferred].get_optimal_action(s, return_optimal=True)
#     # retrieve interface level action corresponding to optimal task level action for inferred goal
#     ph_inferred = TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[a_inferred]


#     #uniform distribution for compute max entropy. which is used as a normalizer. Could be moved to global scope
#     uniform_distribution = np.array([1.0/p_g_given_um_vector.size]*p_g_given_um_vector.size)
#     max_entropy = -np.dot(uniform_distribution, np.log2(uniform_distribution))
#     normalized_h_of_p_g_given_um = -np.dot(p_g_given_um_vector, np.log2(p_g_given_um_vector))/max_entropy
#     if ph_inferred != phm:
#         #check entropy to decide whether to intervene or not
#         if normalized_h_of_p_g_given_um < ENTROPY_THRESHOLD: #intervene
#             if ASSISTANCE_TYPE == 'filter':
#                 ph_modified = None
#             elif ASSISTANCE_TYPE == 'corrective':
#                 ph_modified = ph_inferred
#         else:
#             ph_modified = phm #Maybe keep this as None? because u intended is not same as um?
#     else:
#         ph_modified = phm

#     return ph_modified, (ph_modified == ph_m), ph_inferred, g_inferred, normalized_h_of_p_g_given_um, (normalized_h_of_p_g_given_um < ENTROPY_THRESHOLD), (ph_inferred == phm), (normalized_h_of_p_g_given_um < ENTROPY_THRESHOLD) and (ph_inferred != phm)


def apply_assistance(g_inferred, a_inferred, ph_inferred, p_g_given_um_vector, phm):
    global ENTROPY_THRESHOLD, ASSISTANCE_TYPE
    print (ASSISTANCE_TYPE)
    # uniform distribution for compute max entropy. which is used as a normalizer. Could be moved to global scope
    uniform_distribution = np.array([1.0 / p_g_given_um_vector.size] * p_g_given_um_vector.size)
    max_entropy = -np.dot(uniform_distribution, np.log2(uniform_distribution))
    normalized_h_of_p_g_given_um = -np.dot(p_g_given_um_vector, np.log2(p_g_given_um_vector)) / max_entropy
    if phm != "None":
        if ph_inferred != phm:
            # check entropy to decide whether to intervene or not
            if normalized_h_of_p_g_given_um < ENTROPY_THRESHOLD:  # intervene
                if ASSISTANCE_TYPE == "filter":
                    print ("in filter")
                    ph_modified = "None"
                elif ASSISTANCE_TYPE == "corrective":
                    print ("in corrective")
                    ph_modified = ph_inferred
            else:
                print ("greater entropy")
                ph_modified = phm  # Maybe keep this as None? because u intended is not same as um?
        else:
            print ("ph inferred  = phm")
            ph_modified = phm
    else:
        print ("PHM = None, no assistance")
        ph_modified = phm

    return (
        ph_modified,
        (ph_modified == phm),
        ph_inferred,
        g_inferred,
        normalized_h_of_p_g_given_um,
        (normalized_h_of_p_g_given_um < ENTROPY_THRESHOLD),
        (ph_inferred == phm),
        (normalized_h_of_p_g_given_um < ENTROPY_THRESHOLD) and (ph_inferred != phm),
    )


def sample_phi_given_a(a):  # sample from p(phii|a)
    global P_PHI_GIVEN_A, PHI_SPARSE_LEVEL
    d = np.random.rand()

    if d < PHI_SPARSE_LEVEL:
        phi = "None"
    else:
        p_vector = P_PHI_GIVEN_A[a].values()  # list of probabilities for phii
        phi_index_vector = np.random.multinomial(
            1, p_vector
        )  # sample from the multinomial distribution with distribution p_vector
        phi_index = np.nonzero(phi_index_vector)[0][0]  # grab the index of the index_vector which had a nonzero entry
        phi = P_PHI_GIVEN_A[a].keys()[phi_index]  # retrieve phii using the phi_index

    return phi


def sample_phm_given_phi(phi):  # sample from p(phm|phi)
    global P_PHM_GIVEN_PHI, PHM_SPARSE_LEVEL
    d = np.random.rand()
    if phi != "None":
        if d < PHM_SPARSE_LEVEL:
            phm = "None"
        else:
            p_vector = P_PHM_GIVEN_PHI[phi].values()  # list of probabilities for phm given phi
            phm_index_vector = np.random.multinomial(1, p_vector)  # sample from the multinomial distribution
            # grab the index of the index_vector which had a nonzero entry
            phm_index = np.nonzero(phm_index_vector)[0][0]
            phm = P_PHM_GIVEN_PHI[phi].keys()[phm_index]  # retrieve phm
    else:
        print ("Sampled phi is None, therefore phm is None")
        phm = "None"

    return phm


def simulate_snp_interaction(args):
    simulation_trial_dir = args.simulation_trial_dir
    simulation_results_dir = args.simulation_results_dir
    if not os.path.exists(simulation_results_dir):
        os.makedirs(simulation_results_dir)
    num_reps_per_condition = args.num_reps_per_condition  # number of repetitions for a world
    list_of_simulation_conditions = os.listdir(simulation_trial_dir)

    random.shuffle(list_of_simulation_conditions)
    for index in range(len(list_of_simulation_conditions)):
        global PHI_GIVEN_A_NOISE, PHM_GIVEN_PHI_NOISE, ENTROPY_THRESHOLD, NUM_GOALS, OCCUPANCY_LEVEL, RAND_DIRECTION_FACTOR, PHI_SPARSE_LEVEL, PHM_SPARSE_LEVEL, SPARSITY_FACTOR, ASSISTANCE_TYPE
        with open(os.path.join(simulation_trial_dir, str(index) + ".pkl"), "rb") as fp:
            combination_dict = pickle.load(fp)
        print "COMBINATION NUM ", index
        print "      "

        # grab params for particular simulation trial
        NUM_GOALS = combination_dict["num_goals"]
        OCCUPANCY_LEVEL = combination_dict["occupancy_level"]
        # randomness and sparsity factor for task-level action a
        RAND_DIRECTION_FACTOR = combination_dict["rand_direction_factor"]
        SPARSITY_FACTOR = combination_dict["sparsity_factor"]

        # sparsity levels for interface level actions
        PHI_SPARSE_LEVEL = combination_dict["phi_sparse_level"]
        PHM_SPARSE_LEVEL = combination_dict["phm_sparse_level"]

        # randomness levels for interface level actions
        PHI_GIVEN_A_NOISE = combination_dict["phi_given_a_noise"]
        PHM_GIVEN_PHI_NOISE = combination_dict["phm_given_phi_noise"]

        # entropy level for assistance algorithm
        ENTROPY_THRESHOLD = combination_dict["entropy_threshold"]
        # type of assistance provided for a trial.
        ASSISTANCE_TYPE = combination_dict["assistance_type"]

        # generate the MDP config dict
        mdp_env_params = create_mdp_env_param_dict()

        # create the list of MDPs for the configuration
        mdp_list = create_mdp_list(mdp_env_params, visualize=False)

        # initalize the interface related conditional distributions
        init_P_PHI_GIVEN_A()
        init_P_PHM_GIVEN_PHI()

        # init dict for simulation results
        simulation_results = collections.OrderedDict()
        simulation_results["index"] = index
        simulation_results["combination_dict"] = combination_dict
        simulation_results["trials"] = collections.OrderedDict()

        for rep in range(num_reps_per_condition):
            num_steps = 0
            simulation_results["trials"][rep] = collections.defaultdict(list)
            init_P_G_GIVEN_PHM()  # init p(g) for full model for every repeat trial
            init_P_G_GIVEN_PHM_NOISE_FREE()  # init p(g) for partial model for every repeat trial

            # sample a random goal for the trial. We assume that for a given trial, the goal is fixed but unknown to the autonomy
            random_goal_id = np.random.randint(NUM_GOALS)
            random_goal = mdp_env_params["all_goals"][random_goal_id]  # actual pose of the goal. (x, y, t)

            # log the random goal id and random goal
            simulation_results["trials"][rep]["random_goal_id"] = random_goal_id
            simulation_results["trials"][rep]["random_goal"] = random_goal
            print ("RANDOM GOAL for rep ", random_goal, rep)
            mdp = mdp_list[random_goal_id]  # mdp corresponding to the true sampled goal for the trial

            # pick a random state from the mdp that has the random goal to start the trial.
            s = mdp.get_random_valid_state(is_not_goal=True)
            # s is state, a = task_level action phi_i = true interface level action, phi_m = corrupted interface level action
            assert not mdp._check_if_state_coord_is_goal_state(s)  # ensure that the start state is NOT the goal state

            print ("current_state", s)
            while not mdp._check_if_state_coord_is_goal_state(s):
                print ("   ")
                # sampled task level action from policy, noisy. a_sampled are strings. Could be None depending on sparsity_factor
                a_sampled = mdp.get_optimal_action(s, return_optimal=False)
                # true optimal task level actions. Won't be None
                a_optimal = mdp.get_optimal_action(s, return_optimal=True)
                is_a_sampled_optimal = a_sampled == a_optimal  # to see if the sampled one was same or not
                simulation_results["trials"][rep]["a_sampled"].append(a_sampled)
                simulation_results["trials"][rep]["a_optimal"].append(a_optimal)
                simulation_results["trials"][rep]["is_a_sampled_optimal"].append(is_a_sampled_optimal)
                if a_sampled != "None":
                    # sampled corrupted interface level action corresponding to task-level action, could be None
                    phi = sample_phi_given_a(a_sampled)
                    # corrupted interface level action, could be None
                    phm = sample_phm_given_phi(phi)
                    # print('Before, ', P_G_GIVEN_PHM)
                    simulation_results["trials"][rep]["phi_sampled"].append(phi)
                    simulation_results["trials"][rep]["phm_sampled"].append(phm)
                    simulation_results["trials"][rep]["phi_match_ph_optimal"].append(
                        (phi == TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[a_optimal])
                    )
                    simulation_results["trials"][rep]["phi_match_ph_sampled"].append(
                        (phi == TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[a_sampled])
                    )
                    simulation_results["trials"][rep]["phm_match_ph_optimal"].append(
                        (phm == TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[a_optimal])
                    )
                    simulation_results["trials"][rep]["phm_match_ph_sampled"].append(
                        (phm == TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[a_sampled])
                    )
                    simulation_results["trials"][rep]["goal_belief_before"].append(list(P_G_GIVEN_PHM.values()))
                    simulation_results["trials"][rep]["goal_belief_noise_free_before"].append(
                        list(P_G_GIVEN_UM_NOISE_FREE.values())
                    )
                    if ASSISTANCE_TYPE != "no":
                        # perform inference of g from phm. Note that phm could be None depending on what was sampled.
                        compute_p_g_given_phm(phm, mdp_list, s)  # update belief using full model
                        # update belief using partial model
                        compute_p_g_given_phm_noise_free(phm, mdp_list, s)
                        # infer goal using full model
                        g_inferred, a_inferred, ph_inferred, p_g_given_um_vector = compute_g_a_ph_inferred(mdp_list, s)
                        # infer goal using partial model
                        g_inferred_nf, a_inferred_nf, ph_inferred_nf, p_g_given_um_vector_nf = compute_g_a_ph_inferred(
                            mdp_list, s, use_full_model=False
                        )
                        # apply assistance using full model inference
                        (
                            ph_modified,
                            is_ph_modified_equals_phm,
                            ph_inferred,
                            g_inferred,
                            normalized_h_of_p_g_given_um,
                            is_normalized_entropy_less_than_threshold,
                            is_ph_inferred_equals_phm,
                            did_assistance_intervene,
                        ) = apply_assistance(g_inferred, a_inferred, ph_inferred, p_g_given_um_vector, phm)

                        # print('After, ', P_G_GIVEN_PHM)
                        simulation_results["trials"][rep]["goal_belief_after"].append(list(P_G_GIVEN_PHM.values()))
                        # no update happens
                        simulation_results["trials"][rep]["goal_belief_noise_free_after"].append(
                            list(P_G_GIVEN_UM_NOISE_FREE.values())
                        )
                        ph_inferred_match_with_ph_optimal = (
                            ph_inferred == TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[a_optimal]
                        )
                        ph_inferred_match_with_ph_sampled = (
                            ph_inferred == TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[a_sampled]
                        )
                        simulation_results["trials"][rep]["ph_inferred_match_with_ph_optimal"].append(
                            ph_inferred_match_with_ph_optimal
                        )
                        simulation_results["trials"][rep]["ph_inferred_match_with_ph_sampled"].append(
                            ph_inferred_match_with_ph_sampled
                        )

                        ph_modified_match_with_ph_optimal = (
                            ph_modified == TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[a_optimal]
                        )
                        ph_modified_match_with_ph_sampled = (
                            ph_modified == TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[a_sampled]
                        )
                        simulation_results["trials"][rep]["ph_modified_match_with_ph_optimal"].append(
                            ph_modified_match_with_ph_optimal
                        )
                        simulation_results["trials"][rep]["ph_modified_match_with_ph_sampled"].append(
                            ph_modified_match_with_ph_sampled
                        )

                        simulation_results["trials"][rep]["is_phm_none"].append(phm == "None")

                        # log full model inference results
                        simulation_results["trials"][rep]["ph_inferred"].append(ph_inferred)
                        simulation_results["trials"][rep]["g_inferred"].append(g_inferred)
                        simulation_results["trials"][rep]["g_inferred_match_with_random_goal_id"].append(
                            (g_inferred == random_goal_id)
                        )

                        # log inference results for partial model
                        simulation_results["trials"][rep]["ph_inferred_nf"].append(ph_inferred_nf)
                        simulation_results["trials"][rep]["g_inferred_nf"].append(g_inferred_nf)
                        simulation_results["trials"][rep]["g_inferred_nf_match_with_random_goal_id"].append(
                            (g_inferred_nf == random_goal_id)
                        )
                        simulation_results["trials"][rep]["did_assistance_intervene"].append(did_assistance_intervene)
                        simulation_results["trials"][rep]["normalized_h_of_p_g_given_um"].append(
                            normalized_h_of_p_g_given_um
                        )
                        simulation_results["trials"][rep]["is_normalized_entropy_less_than_threshold"].append(
                            is_normalized_entropy_less_than_threshold
                        )
                        simulation_results["trials"][rep]["is_ph_modified_equals_um"].append(is_ph_modified_equals_phm)
                        simulation_results["trials"][rep]["is_ph_inferred_equals_um"].append(is_ph_inferred_equals_phm)
                        simulation_results["trials"][rep]["ph_applied"].append(ph_modified)

                        if ph_modified != "None":
                            applied_a = TRUE_INTERFACE_ACTION_TO_TASK_ACTION_MAP[ph_modified]
                        else:
                            applied_a = "None"
                        sp = mdp.get_next_state_from_state_action(s, applied_a)
                        simulation_results["trials"][rep]["applied_a"].append(applied_a)
                        simulation_results["trials"][rep]["sp"].append(sp)
                        print ("end of modified assistance")
                    else:
                        # only update belief using full model. Don't do any assistance
                        compute_p_g_given_phm(phm, mdp_list, s)
                        # update belief using partial model
                        compute_p_g_given_phm_noise_free(phm, mdp_list, s)

                        # infer goal using full model
                        g_inferred, a_inferred, ph_inferred, p_g_given_um_vector = compute_g_a_ph_inferred(mdp_list, s)
                        # infer goal using partial model
                        g_inferred_nf, a_inferred_nf, ph_inferred_nf, p_g_given_um_vector_nf = compute_g_a_ph_inferred(
                            mdp_list, s, use_full_model=False
                        )

                        # print('After, ', P_G_GIVEN_PHM)
                        simulation_results["trials"][rep]["goal_belief_after"].append(list(P_G_GIVEN_PHM.values()))
                        simulation_results["trials"][rep]["goal_belief_noise_free_after"].append(
                            list(P_G_GIVEN_UM_NOISE_FREE.values())
                        )
                        simulation_results["trials"][rep]["ph_inferred_match_with_ph_optimal"].append(None)
                        simulation_results["trials"][rep]["ph_inferred_match_with_ph_sampled"].append(None)
                        simulation_results["trials"][rep]["ph_modified_match_with_ph_optimal"].append(None)
                        simulation_results["trials"][rep]["ph_modified_match_with_ph_sampled"].append(None)

                        # simulation_results['trials'][rep]['ph_inferred'].append(None)
                        # log inference results for full model
                        simulation_results["trials"][rep]["is_phm_none"].append(phm == "None")
                        simulation_results["trials"][rep]["ph_inferred"].append(ph_inferred)
                        simulation_results["trials"][rep]["g_inferred"].append(g_inferred)
                        simulation_results["trials"][rep]["g_inferred_match_with_random_goal_id"].append(
                            (g_inferred == random_goal_id)
                        )
                        # log inference results for partial model
                        simulation_results["trials"][rep]["ph_inferred_nf"].append(ph_inferred_nf)
                        simulation_results["trials"][rep]["g_inferred_nf"].append(g_inferred_nf)
                        simulation_results["trials"][rep]["g_inferred_nf_match_with_random_goal_id"].append(
                            (g_inferred_nf == random_goal_id)
                        )

                        simulation_results["trials"][rep]["did_assistance_intervene"].append(None)
                        simulation_results["trials"][rep]["normalized_h_of_p_g_given_um"].append(None)
                        simulation_results["trials"][rep]["is_normalized_entropy_less_than_threshold"].append(None)
                        simulation_results["trials"][rep]["is_ph_modified_equals_um"].append(None)
                        simulation_results["trials"][rep]["is_ph_inferred_equals_um"].append(None)
                        simulation_results["trials"][rep]["ph_applied"].append(phm)
                        if phm != "None":
                            applied_a = TRUE_INTERFACE_ACTION_TO_TASK_ACTION_MAP[phm]
                        else:
                            applied_a = "None"

                        sp = mdp.get_next_state_from_state_action(s, applied_a)
                        simulation_results["trials"][rep]["applied_a"].append(applied_a)
                        simulation_results["trials"][rep]["sp"].append(sp)
                        print ("end of no assistance")

                else:
                    # sampled task level action, a_ sampledis None
                    print ("a_sampled is None")
                    sp = s
                    phi = "None"
                    phm = "None"
                    applied_a = "None"

                    simulation_results["trials"][rep]["phi_sampled"].append("None")
                    simulation_results["trials"][rep]["phm_sampled"].append("None")
                    simulation_results["trials"][rep]["phi_match_ph_optimal"].append("None")
                    simulation_results["trials"][rep]["phi_match_ph_sampled"].append("None")
                    simulation_results["trials"][rep]["phm_match_ph_optimal"].append("None")
                    simulation_results["trials"][rep]["phm_match_ph_sampled"].append("None")
                    # full model inference
                    simulation_results["trials"][rep]["goal_belief_before"].append(list(P_G_GIVEN_PHM.values()))
                    compute_p_g_given_phm(phm, mdp_list, s)  # only update belief. Don't do any assistance
                    g_inferred, a_inferred, ph_inferred, p_g_given_um_vector = compute_g_a_ph_inferred(mdp_list, s)
                    simulation_results["trials"][rep]["goal_belief_after"].append(list(P_G_GIVEN_PHM.values()))

                    # partial model inference
                    simulation_results["trials"][rep]["goal_belief_noise_free_before"].append(
                        list(P_G_GIVEN_UM_NOISE_FREE.values())
                    )
                    # only belief update using partial model. Don't do any assistance
                    compute_p_g_given_phm_noise_free(phm, mdp_list, s)
                    g_inferred_nf, a_inferred_nf, ph_inferred_nf, p_g_given_um_vector_nf = compute_g_a_ph_inferred(
                        mdp_list, s, use_full_model=False
                    )
                    simulation_results["trials"][rep]["goal_belief_noise_free_after"].append(
                        list(P_G_GIVEN_UM_NOISE_FREE.values())
                    )

                    simulation_results["trials"][rep]["ph_inferred_match_with_ph_optimal"].append(None)
                    simulation_results["trials"][rep]["ph_inferred_match_with_ph_sampled"].append(None)
                    simulation_results["trials"][rep]["ph_modified_match_with_ph_optimal"].append(None)
                    simulation_results["trials"][rep]["ph_modified_match_with_ph_sampled"].append(None)

                    simulation_results["trials"][rep]["is_phm_none"].append(phm == "None")

                    # log inference results for full model
                    simulation_results["trials"][rep]["ph_inferred"].append(ph_inferred)
                    simulation_results["trials"][rep]["g_inferred"].append(g_inferred)
                    simulation_results["trials"][rep]["g_inferred_match_with_random_goal_id"].append(
                        (g_inferred == random_goal_id)
                    )

                    # log inference results for partial model
                    simulation_results["trials"][rep]["ph_inferred_nf"].append(ph_inferred_nf)
                    simulation_results["trials"][rep]["g_inferred_nf"].append(g_inferred_nf)
                    simulation_results["trials"][rep]["g_inferred_nf_match_with_random_goal_id"].append(
                        (g_inferred_nf == random_goal_id)
                    )

                    simulation_results["trials"][rep]["did_assistance_intervene"].append(None)
                    simulation_results["trials"][rep]["normalized_h_of_p_g_given_um"].append(None)
                    simulation_results["trials"][rep]["is_normalized_entropy_less_than_threshold"].append(None)
                    simulation_results["trials"][rep]["is_ph_modified_equals_um"].append(None)
                    simulation_results["trials"][rep]["is_ph_inferred_equals_um"].append(None)
                    simulation_results["trials"][rep]["ph_applied"].append(phm)
                    simulation_results["trials"][rep]["applied_a"].append(applied_a)
                    simulation_results["trials"][rep]["sp"].append(sp)

                s = sp
                num_steps += 1
                if num_steps > MAX_SIM_STEPS:
                    print ("Reached max steps")
                    break

            simulation_results["trials"][rep]["total_steps"] = num_steps

            ll = []
            for k, v in simulation_results["trials"][rep].items():
                if type(v) == list:
                    ll.append(len(v))
            ll = set(ll)
            assert len(ll) == 1

        for k, v in simulation_results["trials"].items():
            individual_sim_result = collections.OrderedDict()
            individual_sim_result["combination_dict"] = combination_dict
            individual_sim_result["data"] = v
            individual_sim_result["index"] = k + index * (num_reps_per_condition)
            print (individual_sim_result["index"])
            simulation_result_file_path = os.path.join(
                simulation_results_dir, "sim_" + str(k + index * num_reps_per_condition) + ".pkl"
            )
            with open(simulation_result_file_path, "wb") as fp:
                pickle.dump(individual_sim_result, fp)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--simulation_trial_dir",
        dest="simulation_trial_dir",
        default=os.path.join(os.path.dirname(os.getcwd()), "simulated_human_experiments", "se2_simulation_trial_dir"),
        help="The directory where trials will be stored are",
    )
    parser.add_argument(
        "--num_reps_per_condition",
        action="store",
        type=int,
        default=3,
        help="number of repetetions for single combination of conditions ",
    )
    parser.add_argument(
        "--simulation_results_dir",
        dest="simulation_results_dir",
        default=os.path.join(os.path.dirname(os.getcwd()), "simulated_human_experiments", "se2_simulation_results"),
        help="The directory where the simulation trials will be stored",
    )

    args = parser.parse_args()
    simulate_snp_interaction(args)
