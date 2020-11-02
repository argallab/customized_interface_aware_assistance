# Code developed by Deepak Gopinath*, Mahdieh Nejati Javaremi* in February 2020. Copyright (c) 2020. Deepak Gopinath, Mahdieh Nejati Javaremi, Argallab. (*) Equal contribution

import pickle
import sys
import collections
import rospkg
sys.path.append(os.path.join(rospkg.RosPack().get_path('simulators'), 'scripts'))
import os
import numpy as np
import argparse
from corrective_mode_switch_utils import StartDirection, AssistanceType, PI
import matplotlib.pyplot as plt
import itertools

class DataParser(object):
    """docstring forDataParser."""
    def __init__(self, simulation_results_dir):
        super(DataParser, self).__init__()
        self.simulation_results_dir = simulation_results_dir
        self.simulation_files = os.listdir(simulation_results_dir)
        self.simulation_files = [os.path.join(self.simulation_results_dir, f) for f in self.simulation_files]

    def check_if_sim_file_satisfies_criteria(self, criteria, combination_dict):
        sim_file_satisfies_criteria = True
        for key, value in criteria.items():
            if value != 'all':
                sim_file_satisfies_criteria = sim_file_satisfies_criteria and (combination_dict[key] in criteria[key])

        return sim_file_satisfies_criteria

    def return_all_sim_files(self):
        return self.simulation_files

    def parse_sim_files_for_specific_criteria(self, criteria):
        '''
        criteria is a dict with same keys as combination_dict
        output_keys should be a subset of the keys in trial['data']
        '''
        sim_files_that_satisfies_criteria = []
        for i, f in enumerate(self.simulation_files):
            print('Processing file num', i)
            with open(f, 'rb') as fp:
                simulation_result = pickle.load(fp)
            combination_dict = simulation_result['combination_dict']
            sim_file_satisfies_criteria = self.check_if_sim_file_satisfies_criteria(criteria, combination_dict)
            if sim_file_satisfies_criteria:
                sim_files_that_satisfies_criteria.append(f)
            else:
                continue

        return sim_files_that_satisfies_criteria

class SimulationAnalysis(object):
    """docstring forSimulationAnalysis."""
    def __init__(self, args):
        super(SimulationAnalysis, self).__init__()
        self.simulation_results_dir = args.simulation_results_dir
        self.data_parser = DataParser(self.simulation_results_dir)
        self.combination_dict_keys = ['r_to_g_config', 'num_turns', 'start_direction', 'assistance_type', 'start_mode', 'ui_given_a_noise', 'um_given_ui_noise', 'entropy_threshold', 'target_orientation']
        #experiment params
        self.RG_CONFIGS = ['tr', 'tl', 'bl', 'br']
        self.NUM_TURNS = [1, 2, 3]
        self.START_DIRECTION = [StartDirection.X, StartDirection.Y]
        self.ASSISTANCE_TYPE = [AssistanceType.Filter, AssistanceType.Corrective, AssistanceType.No_Assistance]
        self.START_MODE = [-1, 1]
        self.UI_GIVEN_A_NOISE = [i/10.0 for i in range(1, 9, 2)]
        self.UM_GIVEN_UI_NOISE = [i/10.0 for i in range(1, 9, 2)]
        self.ENTROPY_THRESHOLD = [i/10.0 for i in range(5, 10)]
        self.TARGET_ORIENTATIONS = [-PI/2, PI/2]

    def perform_analysis(self, force_compute_list=[False, False, True]):
        self._check_accuracy_of_corrected_um(force_compute_list[0])
        self._compare_mode_switches_between_assistance_conditions(force_compute_list[1])
        self._compute_percentage_of_intervened(force_compute_list[2])

    def _compute_percentage_of_intervened(self, force_compute=False):
        print("COMPUTE PERCENTAGE INTERVENED")
        criteria = collections.OrderedDict()
        for key in self.combination_dict_keys:
            criteria[key] = 'all'

        subset_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'subsets') #mayebe mvoe this to main function
        if not os.path.exists(subset_path):
            os.makedirs(subset_path)
        compare_mode_switches_between_assistance_conditions_pklnames = os.path.join(subset_path,'is_intervened_pklnames.pkl')
        if os.path.exists(compare_mode_switches_between_assistance_conditions_pklnames):
            with open(compare_mode_switches_between_assistance_conditions_pklnames, 'rb') as fp:
                sim_files_that_satisfies_criteria = pickle.load(fp)
        else:
            sim_files_that_satisfies_criteria = sorted(self.data_parser.return_all_sim_files())


        results_dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')
        if not os.path.exists(results_dir_path):
            os.makedirs(results_dir_path)

        ui_a_vs_um_ui_matrix_for_u_intended_not_equals_um_filename = os.path.join(results_dir_path, 'ui_a_vs_um_ui_matrix_for_u_intended_not_equals_um.pkl')
        ui_a_vs_um_ui_matrix_for_normalized_entropy_filename = os.path.join(results_dir_path, 'ui_a_vs_um_ui_matrix_for_normalized_entropy.pkl')
        ui_a_vs_um_ui_matrix_for_is_intervened_filename = os.path.join(results_dir_path, 'ui_a_vs_um_ui_matrix_for_is_intervened.pkl')
        ui_a_vs_um_ui_matrix_for_is_intervened_after_u_intended_not_equals_um_filename = os.path.join(results_dir_path, 'ui_a_vs_um_ui_matrix_for_is_intervened_after_u_intended_not_equals_um.pkl')
        ui_a_vs_um_ui_matrix_for_is_normalized_entropy_less_than_threshold_filename = os.path.join(results_dir_path, 'ui_a_vs_um_ui_matrix_for_is_normalized_entropy_less_than_threshold.pkl')

        u_intended_not_equals_um_list_filename = os.path.join(results_dir_path, 'u_intended_not_equals_um_list.pkl')
        normalized_entropy_list_filename = os.path.join(results_dir_path, 'normalized_entropy_list.pkl')
        is_intervened_list_filename = os.path.join(results_dir_path, 'is_intervened_list.pkl')
        is_intervened_after_u_intended_not_equals_um_list_filename = os.path.join(results_dir_path, 'is_intervened_after_u_intended_not_equals_um_list.pkl')
        is_normalized_entropy_less_than_threshold_list_filename = os.path.join(results_dir_path, 'is_normalized_entropy_less_than_threshold_list.pkl')

        if os.path.exists(u_intended_not_equals_um_list_filename) and not force_compute:
            print('LOADING FROM FILE')

            with open(ui_a_vs_um_ui_matrix_for_u_intended_not_equals_um_filename, 'rb') as fp:
                ui_a_vs_um_ui_matrix_for_u_intended_not_equals_um = pickle.load(fp)
            with open(ui_a_vs_um_ui_matrix_for_normalized_entropy_filename, 'rb') as fp:
                ui_a_vs_um_ui_matrix_for_normalized_entropy = pickle.load(fp)
            with open(ui_a_vs_um_ui_matrix_for_is_intervened_filename, 'rb') as fp:
                ui_a_vs_um_ui_matrix_for_is_intervened = pickle.load(fp)
            with open(ui_a_vs_um_ui_matrix_for_is_intervened_after_u_intended_not_equals_um_filename, 'rb') as fp:
                ui_a_vs_um_ui_matrix_for_is_intervened_after_u_intended_not_equals_um = pickle.load(fp)
            with open(ui_a_vs_um_ui_matrix_for_is_normalized_entropy_less_than_threshold_filename, 'rb') as fp:
                ui_a_vs_um_ui_matrix_for_is_normalized_entropy_less_than_threshold = pickle.load(fp)

            with open(u_intended_not_equals_um_list_filename, 'rb') as fp:
                u_intended_not_equals_um_list = pickle.load(fp)
            with open(normalized_entropy_list_filename, 'rb') as fp:
                normalized_entropy_list = pickle.load(fp)
            with open(is_intervened_list_filename, 'rb') as fp:
                is_intervened_list = pickle.load(fp)
            with open(is_intervened_after_u_intended_not_equals_um_list_filename, 'rb') as fp:
                is_intervened_after_u_intended_not_equals_um_list = pickle.load(fp)
            with open(is_normalized_entropy_less_than_threshold_list_filename, 'rb') as fp:
                is_normalized_entropy_less_than_threshold_list = pickle.load(fp)

        else:
            normalized_entropy_list = collections.OrderedDict()
            u_intended_not_equals_um_list = collections.OrderedDict()
            is_intervened_list = collections.OrderedDict()
            is_intervened_after_u_intended_not_equals_um_list = collections.OrderedDict()
            is_normalized_entropy_less_than_threshold_list = collections.OrderedDict()

            for ui_a in self.UI_GIVEN_A_NOISE:
                normalized_entropy_list[ui_a] = collections.defaultdict(list)
                u_intended_not_equals_um_list[ui_a] = collections.defaultdict(list)
                is_intervened_list[ui_a] = collections.defaultdict(list)
                is_intervened_after_u_intended_not_equals_um_list[ui_a] = collections.defaultdict(list)
                is_normalized_entropy_less_than_threshold_list[ui_a] = collections.defaultdict(list)

            for k, sf in enumerate(sim_files_that_satisfies_criteria):
                print(k)
                with open(sf, 'rb') as fp:
                    simulation_result = pickle.load(fp)

                ui_a = simulation_result['combination_dict']['ui_given_a_noise']
                um_ui = simulation_result['combination_dict']['um_given_ui_noise']
                u_intended_equals_um = simulation_result['data']['is_u_intended_equals_um']
                is_normalized_entropy_less_than_threshold = simulation_result['data']['is_normalized_entropy_less_than_threshold']

                normalized_entropy_list[ui_a][um_ui].extend(simulation_result['data']['normalized_h_of_p_ui_given_um'])
                is_intervened_list[ui_a][um_ui].extend([a and not b for a, b in zip(is_normalized_entropy_less_than_threshold, u_intended_equals_um)]) #overall percentage
                u_intended_not_equals_um_list[ui_a][um_ui].extend([not a for a in u_intended_equals_um])
                is_normalized_entropy_less_than_threshold_list[ui_a][um_ui].extend(is_normalized_entropy_less_than_threshold)
                temp_list = []
                for ui_equals_um in u_intended_equals_um:
                    if not ui_equals_um:
                        temp_list.append(is_normalized_entropy_less_than_threshold)

                is_intervened_after_u_intended_not_equals_um_list[ui_a][um_ui].extend(temp_list)

            with open(normalized_entropy_list_filename, 'wb') as fp:
                pickle.dump(normalized_entropy_list, fp)
            with open(u_intended_not_equals_um_list_filename, 'wb') as fp:
                pickle.dump(u_intended_not_equals_um_list, fp)
            with open(is_intervened_list_filename, 'wb') as fp:
                pickle.dump(is_intervened_list, fp)
            with open(is_intervened_after_u_intended_not_equals_um_list_filename, 'wb') as fp:
                pickle.dump(is_intervened_after_u_intended_not_equals_um_list, fp)
            with open(is_normalized_entropy_less_than_threshold_list_filename, 'wb') as fp:
                pickle.dump(is_normalized_entropy_less_than_threshold_list, fp)

            flatten = itertools.chain.from_iterable

            ui_a_vs_um_ui_matrix_for_u_intended_not_equals_um = np.zeros((len(self.UI_GIVEN_A_NOISE), len(self.UM_GIVEN_UI_NOISE)))
            ui_a_vs_um_ui_matrix_for_normalized_entropy = np.zeros((len(self.UI_GIVEN_A_NOISE), len(self.UM_GIVEN_UI_NOISE)))
            ui_a_vs_um_ui_matrix_for_is_intervened = np.zeros((len(self.UI_GIVEN_A_NOISE), len(self.UM_GIVEN_UI_NOISE)))
            ui_a_vs_um_ui_matrix_for_is_intervened_after_u_intended_not_equals_um = np.zeros((len(self.UI_GIVEN_A_NOISE), len(self.UM_GIVEN_UI_NOISE)))
            ui_a_vs_um_ui_matrix_for_is_normalized_entropy_less_than_threshold = np.zeros((len(self.UI_GIVEN_A_NOISE), len(self.UM_GIVEN_UI_NOISE)))

            for i, ui_a in enumerate(self.UI_GIVEN_A_NOISE):
                for j, um_ui in enumerate(self.UM_GIVEN_UI_NOISE):
                    print(ui_a, um_ui)
                    average_normalized_entropy = float(sum(normalized_entropy_list[ui_a][um_ui]))/len(normalized_entropy_list[ui_a][um_ui])
                    percentage_normalized_entropy_less_than_threshold = float(sum(is_normalized_entropy_less_than_threshold_list[ui_a][um_ui]))/len(is_normalized_entropy_less_than_threshold_list[ui_a][um_ui])
                    percentage_is_intervened = float(sum(is_intervened_list[ui_a][um_ui]))/len(is_intervened_list[ui_a][um_ui])
                    percentage_u_intended_not_equals_um = float(sum(u_intended_not_equals_um_list[ui_a][um_ui]))/len(u_intended_not_equals_um_list[ui_a][um_ui])
                    temp_list = list(flatten(is_intervened_after_u_intended_not_equals_um_list[ui_a][um_ui]))
                    percentage_is_intervened_after_u_intended_not_equals_um = float(sum(temp_list))/len(temp_list)
                    ui_a_vs_um_ui_matrix_for_normalized_entropy[i,j] = average_normalized_entropy
                    ui_a_vs_um_ui_matrix_for_u_intended_not_equals_um[i,j] = percentage_u_intended_not_equals_um
                    ui_a_vs_um_ui_matrix_for_is_intervened[i,j] = percentage_is_intervened
                    ui_a_vs_um_ui_matrix_for_is_intervened_after_u_intended_not_equals_um[i,j] = percentage_is_intervened_after_u_intended_not_equals_um
                    ui_a_vs_um_ui_matrix_for_is_normalized_entropy_less_than_threshold[i, j] = percentage_normalized_entropy_less_than_threshold


            with open(ui_a_vs_um_ui_matrix_for_u_intended_not_equals_um_filename, 'wb') as fp:
                pickle.dump(ui_a_vs_um_ui_matrix_for_u_intended_not_equals_um, fp)
            with open(ui_a_vs_um_ui_matrix_for_normalized_entropy_filename, 'wb') as fp:
                pickle.dump(ui_a_vs_um_ui_matrix_for_normalized_entropy, fp)
            with open(ui_a_vs_um_ui_matrix_for_is_intervened_filename, 'wb') as fp:
                pickle.dump(ui_a_vs_um_ui_matrix_for_is_intervened, fp)
            with open(ui_a_vs_um_ui_matrix_for_is_intervened_after_u_intended_not_equals_um_filename, 'wb') as fp:
                pickle.dump(ui_a_vs_um_ui_matrix_for_is_intervened_after_u_intended_not_equals_um, fp)
            with open(ui_a_vs_um_ui_matrix_for_is_normalized_entropy_less_than_threshold_filename, 'wb') as fp:
                pickle.dump(ui_a_vs_um_ui_matrix_for_is_normalized_entropy_less_than_threshold, fp)

            print("ui_a_vs_um_ui_matrix_for_normalized_entropy",ui_a_vs_um_ui_matrix_for_normalized_entropy)
            print("ui_a_vs_um_ui_matrix_for_u_intended_not_equals_um", ui_a_vs_um_ui_matrix_for_u_intended_not_equals_um)
            print("ui_a_vs_um_ui_matrix_for_is_intervened", ui_a_vs_um_ui_matrix_for_is_intervened)
            print("ui_a_vs_um_ui_matrix_for_is_intervened_after_u_intended_not_equals_um", ui_a_vs_um_ui_matrix_for_is_intervened_after_u_intended_not_equals_um)
            print("ui_a_vs_um_ui_matrix_for_is_normalized_entropy_less_than_threshold", ui_a_vs_um_ui_matrix_for_is_normalized_entropy_less_than_threshold)

    def _compare_mode_switches_between_assistance_conditions(self, force_compute=False):
        print("COMPARE MODE SWITCHES")
        criteria = collections.OrderedDict()
        for key in self.combination_dict_keys:
            criteria[key] = 'all'

        subset_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'subsets') #mayebe mvoe this to main function
        if not os.path.exists(subset_path):
            os.makedirs(subset_path)
        compare_mode_switches_between_assistance_conditions_pklnames = os.path.join(subset_path,'compare_mode_switches_between_assistance_conditions_pklnames.pkl')
        if os.path.exists(compare_mode_switches_between_assistance_conditions_pklnames):
            with open(compare_mode_switches_between_assistance_conditions_pklnames, 'rb') as fp:
                sim_files_that_satisfies_criteria = pickle.load(fp)
        else:
            sim_files_that_satisfies_criteria = self.data_parser.return_all_sim_files()

        results_dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')
        if not os.path.exists(results_dir_path):
            os.makedirs(results_dir_path)

        #for each ui/a and uim_ui collect the num of modes switches for for each condition

        ui_a_vs_um_ui_matrix_for_total_steps_filename = os.path.join(results_dir_path, 'ui_a_vs_um_ui_matrix_for_total_steps.pkl')
        if os.path.exists(ui_a_vs_um_ui_matrix_for_total_steps_filename) and not force_compute:
            with open(ui_a_vs_um_ui_matrix_for_total_steps_filename, 'rb') as fp:
                ui_a_vs_um_ui_matrix_for_total_steps = pickle.load(fp)
        else:
            ui_a_vs_um_ui_matrix_for_total_steps = collections.OrderedDict()
            for i, ui_a in enumerate(self.UI_GIVEN_A_NOISE):
                ui_a_vs_um_ui_matrix_for_total_steps[ui_a] = collections.OrderedDict()
                for j, um_ui in enumerate(self.UM_GIVEN_UI_NOISE):
                    print("UI_A and UM_UI", ui_a, um_ui)
                    ui_a_vs_um_ui_matrix_for_total_steps[ui_a][um_ui] = collections.defaultdict(list)
                    for k, sf in enumerate(sim_files_that_satisfies_criteria):
                        with open(sf, 'rb') as fp:
                            simulation_result = pickle.load(fp)
                        if simulation_result['combination_dict']['ui_given_a_noise'] == ui_a and simulation_result['combination_dict']['um_given_ui_noise'] == um_ui:
                            ui_a_vs_um_ui_matrix_for_total_steps[ui_a][um_ui][simulation_result['combination_dict']['assistance_type']].append(simulation_result['data']['total_steps'])

            with open(ui_a_vs_um_ui_matrix_for_total_steps_filename, 'wb') as fp:
                pickle.dump(ui_a_vs_um_ui_matrix_for_total_steps, fp)


        for i, ui_a in enumerate(self.UI_GIVEN_A_NOISE):
            filter_list_means = []
            corrective_list_means = []
            no_assistance_list_means = []
            for j, um_ui in enumerate(self.UM_GIVEN_UI_NOISE):
                corrective_list_means.append(np.mean(ui_a_vs_um_ui_matrix_for_total_steps[ui_a][um_ui][AssistanceType.Corrective]))
                filter_list_means.append(np.mean(ui_a_vs_um_ui_matrix_for_total_steps[ui_a][um_ui][AssistanceType.Filter]))
                no_assistance_list_means.append(np.mean(ui_a_vs_um_ui_matrix_for_total_steps[ui_a][um_ui][AssistanceType.No_Assistance]))
            fig, ax = plt.subplots()
            N = 4
            ind = np.arange(N)*3
            width = 0.8
            p1 = ax.bar(ind, no_assistance_list_means, bottom=0, color='#7f6d5f')
            p2 = ax.bar(ind+width, filter_list_means, bottom=0,color='#557f2d')
            p3 = ax.bar(ind+2*width, corrective_list_means, bottom=0, color='#2d7f5e')

            ax.set_title('Internal Model Noise Level = {}'.format(str(ui_a)), fontsize=24)
            ax.set_xticks(ind+1.5*width)
            ax.set_xticklabels(('0.1', '0.3', '0.5', '0.7'))
            ax.set_ylabel('Total Number of Steps', fontsize=20)
            ax.set_xlabel('User Input Distortion Level', fontsize=20)
            ax.set_ylim((0, 40))

            ax.legend((p1[0], p2[0], p3[0]), ('No_Assistance', 'Filter', 'Corrective'), loc='upper left')
            ax.autoscale_view()
            plt.show()

    def _check_accuracy_of_corrected_um(self, force_compute=False):
        criteria = collections.OrderedDict()
        #init all criteria keys to be all.
        for key in self.combination_dict_keys:
            criteria[key] = 'all'

        #for those keys that need to be filtered. replace 'all' with a list containing the allowed values for the key
        criteria['assistance_type'] = [AssistanceType.Corrective]

        subset_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'subsets')
        if not os.path.exists(subset_path):
            os.makedirs(subset_path)

        results_dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'results')
        if not os.path.exists(results_dir_path):
            os.makedirs(results_dir_path)
        only_corrective_subset_pkl_name = os.path.join(subset_path,'only_corrective_subset.pkl')
        if os.path.exists(only_corrective_subset_pkl_name):
            with open(only_corrective_subset_pkl_name, 'rb') as fp:
                sim_files_that_satisfies_criteria = pickle.load(fp)
        else:
            sim_files_that_satisfies_criteria = self.data_parser.parse_sim_files_for_specific_criteria(criteria)
            with open(only_corrective_subset_pkl_name, 'wb') as fp:
                pickle.dump(sim_files_that_satisfies_criteria, fp)
                #save this list and use it for later.

        ui_a_vs_um_ui_matrix_filename = os.path.join(results_dir_path, 'ui_a_vs_um_ui_matrix_for_assistance_match_with_ground_truth.pkl')
        assistance_match_with_ground_truth_list_filename = os.path.join(results_dir_path, 'assistance_match_with_ground_truth_list.pkl')
        if os.path.exists(ui_a_vs_um_ui_matrix_filename) and not force_compute: #assumes that if one file exists the other does too
            with open(assistance_match_with_ground_truth_list_filename, 'rb') as fp:
                assistance_match_with_ground_truth_list = pickle.load(fp)

            with open(ui_a_vs_um_ui_matrix_filename, 'rb') as fp:
                ui_a_vs_um_ui_matrix = pickle.load(fp)
        else:

            assistance_match_with_ground_truth_list = collections.OrderedDict()
            for ui_a in self.UI_GIVEN_A_NOISE:
                assistance_match_with_ground_truth_list[ui_a] = collections.defaultdict(list)

            for k, sf in enumerate(sim_files_that_satisfies_criteria):
                with open(sf, 'rb') as fp:
                    simulation_result = pickle.load(fp)

                ui_a = simulation_result['combination_dict']['ui_given_a_noise']
                um_ui = simulation_result['combination_dict']['um_given_ui_noise']
                assistance_match_with_ground_truth_list[ui_a][um_ui].extend(simulation_result['data']['assistance_match_with_ground_truth'])


            with open(assistance_match_with_ground_truth_list_filename, 'wb') as fp:
                pickle.dump(assistance_match_with_ground_truth_list, fp)

            ui_a_vs_um_ui_matrix = np.zeros((len(self.UI_GIVEN_A_NOISE), len(self.UM_GIVEN_UI_NOISE)))

            for i, ui_a in enumerate(self.UI_GIVEN_A_NOISE):
                for j, um_ui in enumerate(self.UM_GIVEN_UI_NOISE):
                    percentage_of_correct_assistance = float(sum(assistance_match_with_ground_truth_list[ui_a][um_ui]))/len(assistance_match_with_ground_truth_list[ui_a][um_ui])
                    ui_a_vs_um_ui_matrix[i, j] = percentage_of_correct_assistance

            with open(ui_a_vs_um_ui_matrix_filename, 'wb') as fp:
                pickle.dump(ui_a_vs_um_ui_matrix, fp)

        print('UI_A vs UM_UI for ASSISTANCE MATCH WITH GROUND TRUTH', ui_a_vs_um_ui_matrix)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--simulation_results_dir', dest='simulation_results_dir',default=os.path.join(os.path.dirname(os.getcwd()), 'simulation_scripts', 'simulation_results_with_entropy'), help="The directory where the simulation trials will be stored")
    args = parser.parse_args()
    force_compute_list = [False, False, False]
    sa = SimulationAnalysis(args)
    sa.perform_analysis(force_compute_list)
