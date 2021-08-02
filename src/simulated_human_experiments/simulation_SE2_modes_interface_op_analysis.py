import pickle
import sys
import os
import copy
import argparse
import collections
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from enum import Enum

# Enum defintions
class ModelType(Enum):
    FULL_MODEL = 0
    PARTIAL_MODEL = 1


class DataParser(object):
    def __init__(self, simulation_results_dir):
        super(DataParser, self).__init__()
        self.simulation_results_dir = simulation_results_dir
        self.simulation_files = os.listdir(self.simulation_results_dir)
        self.simulation_files = [os.path.join(self.simulation_results_dir, f) for f in self.simulation_files]

    def return_all_sim_files(self):
        return self.simulation_files

    def check_if_sim_file_satisfies_criteria(self, criteria, combination_dict):
        sim_file_satisfies_criteria = True
        for key, value in criteria.items():
            if value != "all":
                sim_file_satisfies_criteria = sim_file_satisfies_criteria and (combination_dict[key] in criteria[key])

        return sim_file_satisfies_criteria

    def parse_sim_files_for_specific_criteria(self, criteria):
        """
        criteria is a dict with same keys as combination_dict
        output_keys should be a subset of the keys in trial['data']
        """
        sim_files_that_satisfies_criteria = []
        for i, f in enumerate(self.simulation_files):
            print("Processing file num", i)
            with open(f, "rb") as fp:
                simulation_result = pickle.load(fp)
            combination_dict = simulation_result["combination_dict"]
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
        self.combination_dict_keys = [
            "num_goals",
            "occupancy_level",
            "rand_direction_factor",
            "sparsity_factor",
            "phi_sparse_level",
            "phm_sparse_level",
            "phi_give_a_noise",
            "phm_given_phi_noise",
            "entropy_threshold",
            "assistance_type",
        ]
        self.NUM_GOALS_LEVELS = [3]  # [2,3,4]
        self.OCCUPANCY_LEVELS = [0.1]  # [0.05, 0.1]
        self.RAND_DIRECTION_LEVELS = [0.2]  # [0.0, 0.2, 0.4, 0.8]
        self.SPARSITY_FACTORS = [0.2]
        self.PHI_SPARSE_LEVELS = [0.1]
        self.PHM_SPARSE_LEVELS = [0.1]
        self.PHI_GIVEN_A_NOISE = [i / 10.0 for i in range(4, 9, 2)]  # (1,9,2)
        self.PHM_GIVEN_PHI_NOISE = [i / 10.0 for i in range(4, 9, 2)]
        self.ENTROPY_THRESHOLD = [i / 10.0 for i in range(8, 10)]  # (5,10)
        self.ASSISTANCE_TYPES = ["no"]

    def perform_analysis(self, force_compute_list=[False, False, False]):
        self._check_goal_inference_accuracy_for_full_and_partial_model(force_compute_list[0])

    def _check_goal_inference_accuracy_for_full_and_partial_model(self, force_compute=False):
        print("COMPUTE GOAL INFERENCE ACCURACY FOR FULL AND PARTIAL MODEL")

        criteria = collections.OrderedDict()
        for key in self.combination_dict_keys:
            criteria[key] = "all"  # all levels for each parameter

        # path where the subset of data is going to stored.
        subset_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "subsets")
        if not os.path.exists(subset_path):
            os.makedirs(subset_path)

        results_dir_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
        if not os.path.exists(results_dir_path):
            os.makedirs(results_dir_path)

        only_no_assistance_subset_pkl_name = os.path.join(subset_path, "only_no_assistance_subset.pkl")
        if os.path.exists(only_no_assistance_subset_pkl_name):
            with open(only_no_assistance_subset_pkl_name, "rb") as fp:
                sim_files_that_satisfies_criteria = pickle.load(fp)
        else:
            sim_files_that_satisfies_criteria = self.data_parser.parse_sim_files_for_specific_criteria(criteria)
            with open(only_no_assistance_subset_pkl_name, "wb") as fp:
                pickle.dump(sim_files_that_satisfies_criteria, fp)
                # save this list and use it for later

        phi_a_vs_phm_phi_matrix_filename = os.path.join(
            results_dir_path, "phi_a_vs_phm_phi_matrix_for_goal_inference_for_full_and_partial_model.pkl"
        )
        # assumes that if one file exists the other does too
        if os.path.exists(phi_a_vs_phm_phi_matrix_filename) and not force_compute:
            with open(phi_a_vs_phm_phi_matrix_filename, "rb") as fp:
                phi_a_vs_phm_phi_matrix = pickle.load(fp)
        else:
            goal_inference_comparison_list = collections.OrderedDict()
            for phi_a in self.PHI_GIVEN_A_NOISE:
                goal_inference_comparison_list[phi_a] = collections.defaultdict(list)

            print("TOTAL NUMBER OF SIM FILES SATISFYING CRITERIA", len(sim_files_that_satisfies_criteria))
            for k, sf in enumerate(sim_files_that_satisfies_criteria):
                if k % 10 == 0:
                    print("Extracting info from file num ", k)
                with open(sf, "rb") as fp:
                    simulation_result = pickle.load(fp)

                phi_a = simulation_result["combination_dict"]["phi_given_a_noise"]
                phm_phi = simulation_result["combination_dict"]["phm_given_phi_noise"]

                full_model_goal_inference_match_percentage = float(
                    sum(simulation_result["data"]["g_inferred_match_with_random_goal_id"])
                ) / len(simulation_result["data"]["g_inferred_match_with_random_goal_id"])

                partial_model_goal_inference_match_percentage = float(
                    sum(simulation_result["data"]["g_inferred_nf_match_with_random_goal_id"])
                ) / len(simulation_result["data"]["g_inferred_nf_match_with_random_goal_id"])

                goal_inference_comparison_list[phi_a][phm_phi].append(
                    (full_model_goal_inference_match_percentage, partial_model_goal_inference_match_percentage)
                )

            phi_a_vs_phm_phi_matrix = np.zeros((len(self.PHI_GIVEN_A_NOISE), len(self.PHM_GIVEN_PHI_NOISE), 2))
            for i, phi_a in enumerate(self.PHI_GIVEN_A_NOISE):
                for j, phm_phi in enumerate(self.PHM_GIVEN_PHI_NOISE):
                    # first element of tuple is goal inference accuracy for full model
                    avg_goal_inference_accuracy_full_model = np.mean(
                        [gi[ModelType.FULL_MODEL.value] for gi in goal_inference_comparison_list[phi_a][phm_phi]]
                    )
                    avg_goal_inference_accuracy_partial_model = np.mean(
                        [gi[ModelType.PARTIAL_MODEL.value] for gi in goal_inference_comparison_list[phi_a][phm_phi]]
                    )
                    phi_a_vs_phm_phi_matrix[i, j, ModelType.FULL_MODEL.value] = avg_goal_inference_accuracy_full_model
                    phi_a_vs_phm_phi_matrix[
                        i, j, ModelType.PARTIAL_MODEL.value
                    ] = avg_goal_inference_accuracy_partial_model

            with open(phi_a_vs_phm_phi_matrix_filename, "wb") as fp:
                pickle.dump(phi_a_vs_phm_phi_matrix, fp)

        print("PHI_A vs PHM_PHM for GOAL INFERENCE FULL and overall avg")
        print(
            phi_a_vs_phm_phi_matrix[:, :, ModelType.FULL_MODEL.value],
            np.mean(phi_a_vs_phm_phi_matrix[:, :, ModelType.FULL_MODEL.value]),
        )
        print("PHI_A vs PHM_PHM for GOAL INFERENCE PARTIAL and overall avg")
        print(
            phi_a_vs_phm_phi_matrix[:, :, ModelType.PARTIAL_MODEL.value],
            np.mean(phi_a_vs_phm_phi_matrix[:, :, ModelType.PARTIAL_MODEL.value]),
        )
        import IPython

        IPython.embed(banner1="check")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--simulation_results_dir",
        dest="simulation_results_dir",
        default=os.path.join(os.path.dirname(os.getcwd()), "simulated_human_experiments", "se2_simulation_results"),
        help="The directory where the se2 simulation results will be stored",
    )
    args = parser.parse_args()
    force_compute_list = [True, True, True]
    sa = SimulationAnalysis(args)
    sa.perform_analysis(force_compute_list)
