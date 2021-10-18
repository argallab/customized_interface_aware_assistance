#!/usr/bin/env python
# Code developed by Deepak Gopinath*, Mahdieh Nejati Javaremi* in February 2020. Copyright (c) 2020. Deepak Gopinath, Mahdieh Nejati Javaremi, Argallab. (*) Equal contribution

import rospy
import rospkg
import pickle
import os
import numpy as np
from envs.srv import PASAllG, PASAllGRequest, PASAllGResponse
from inference_and_correction.msg import InferCorrectInfo
from teleop_nodes.srv import InferCorrect, InferCorrectRequest, InferCorrectResponse
from teleop_nodes.srv import GoalInferModify, GoalInferModifyRequest, GoalInferModifyResponse
from inference_and_correction.msg import GoalInferModifyInfo
import collections
import math
import sys

sys.path.append(os.path.join(rospkg.RosPack().get_path("simulators"), "scripts"))
from simulators.srv import InitBelief, InitBeliefRequest, InitBeliefResponse
from corrective_mode_switch_utils import TRUE_ACTION_TO_COMMAND, LOW_LEVEL_COMMANDS
from corrective_mode_switch_utils import (
    AssistanceType,
    TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP,
    INTERFACE_LEVEL_ACTIONS,
    TASK_LEVEL_ACTIONS,
)


class GoalInferenceAndCorrection(object):
    def __init__(self, subject_id):
        rospy.init_node("goal_inference and correction")
        rospy.Service(
            "/goal_inference_and_correction/handle_inference_and_unintended_actions",
            GoalInferModify,
            self.handle_inference_and_unintended_actions,
        )
        self.P_G_GIVEN_PHM = collections.OrderedDict()
        rospy.Service("/goal_inference_and_correction/init_belief", InitBelief, self.init_P_G_GIVEN_PHM)
        rospy.loginfo("Waiting for sim_env node ")
        rospy.wait_for_service("/sim_env/get_prob_a_s_all_g")
        rospy.loginfo("sim_env node found!")
        self.get_prob_a_s_all_g = rospy.ServiceProxy("/sim_env/get_prob_a_s_all_g", PASAllG)
        # register for service to grab

        self.subject_id = subject_id
        self.distribution_directory_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), "se2_personalized_distributions"
        )
        self.goal_infer_modify_info_pub = rospy.Publisher("/goal_infer_modify_info", GoalInferModifyInfo, queue_size=1)
        self.goal_infer_modify_info_msg = GoalInferModifyInfo()

        self.P_PHI_GIVEN_A = None
        self.P_PHM_GIVEN_PHI = None
        self.DEFAULT_PHI_GIVEN_A_NOISE = 0.1
        self.DEFAULT_PHM_GIVEN_PHI_NOISE = 0.1

        self.ASSISTANCE_TYPE = rospy.get_param("assistance_type", 2)

        self.P_A_S_ALL_G_DICT = collections.OrderedDict()
        self.OPTIMAL_ACTION_FOR_S_G = []
        self.decay_counter = 0
        self.decay_counter_max_value = 1000
        self.decay_scale_factor = 0.01  # lower this value to slow down the decay

        if self.ASSISTANCE_TYPE == 0:
            self.ASSISTANCE_TYPE = AssistanceType.Filter
        elif self.ASSISTANCE_TYPE == 1:
            self.ASSISTANCE_TYPE = AssistanceType.Corrective
        elif self.ASSISTANCE_TYPE == 2:
            self.ASSISTANCE_TYPE = AssistanceType.No_Assistance

        self.ENTROPY_THRESHOLD = rospy.get_param("entropy_threshold", 0.1)
        self.LIKELIHOOD_THRESHOLD = rospy.get_param("likelihood_threshold", 0.1)

        # init all distributions from file
        if os.path.exists(os.path.join(self.distribution_directory_path, str(self.subject_id) + "_p_phi_given_a.pkl")):
            print("LOADING PERSONALIZED P_PHI_GIVEN_A")
            with open(
                os.path.join(self.distribution_directory_path, str(self.subject_id) + "_p_phi_given_a.pkl"), "rb"
            ) as fp:
                self.P_PHI_GIVEN_A = pickle.load(fp)
        else:
            self.P_PHI_GIVEN_A = collections.OrderedDict()
            self.init_P_PHI_GIVEN_A()

        if os.path.exists(
            os.path.join(self.distribution_directory_path, str(self.subject_id) + "_p_phm_given_phi.pkl")
        ):
            print("LOADING PERSONALIZED P_PHM_GIVEN_PHI")
            with open(
                os.path.join(self.distribution_directory_path, str(self.subject_id) + "_p_phm_given_phi.pkl"), "rb"
            ) as fp:
                self.P_PHM_GIVEN_PHI = pickle.load(fp)
        else:
            self.P_PHM_GIVEN_PHI = collections.OrderedDict()
            self.init_P_PHM_GIVEN_PHI()

    def handle_inference_and_unintended_actions(self, req):
        # print("In infer and correct")
        phm = req.phm
        response = GoalInferModifyResponse()
        p_a_s_all_g_response = self.get_prob_a_s_all_g()
        # since TASK_LEVEL_ACTIONS is OrderedDict this is list will always be in the same order
        if p_a_s_all_g_response.status:
            p_a_s_all_g = p_a_s_all_g_response.p_a_s_all_g
            # current_s = p_a_s_all_g_response.current_s 
            current_s = 0
            # update p_a_s_all_g dict
            for g in range(len(p_a_s_all_g)):  # number of goals
                self.P_A_S_ALL_G_DICT[g] = collections.OrderedDict()
                for i, task_level_action in enumerate(TASK_LEVEL_ACTIONS):
                    self.P_A_S_ALL_G_DICT[g][task_level_action] = p_a_s_all_g[g].p_a_s_g[i]

            # get optimal action for all goals for current state as a list ordered by goal index
            self.OPTIMAL_ACTION_FOR_S_G = p_a_s_all_g_response.optimal_action_s_g
            assert len(self.OPTIMAL_ACTION_FOR_S_G) == self.NUM_GOALS

            # do Bayesian inference and update belief over goals.
            self._compute_p_g_given_phm(phm)
            # get inferred goal and optimal task level and interface level action corresponding to max.
            g_inferred, a_inferred, ph_inferred, p_g_given_um_vector = self._compute_g_a_ph_inferred()

            # compute etropy
            normalized_h_of_p_g_given_phm = self._compute_entropy_of_p_g_given_phm()
            # apply assistance by checking entropy (get phm_modified)
            # ph_modified, is_corrected_or_filtered, is_ph_inferred_equals_phm = self._modify_or_pass_phm(
            #     phm, ph_inferred, normalized_h_of_p_g_given_phm
            # )

            ph_modified, is_corrected_or_filtered, is_ph_inferred_equals_phm = self._modify_filter_or_pass_phm(
                phm, ph_inferred, normalized_h_of_p_g_given_phm, current_s
            )

            response.ph_modified = ph_modified
            response.is_corrected_or_filtered = is_corrected_or_filtered
            response.status = True

            # populate response
            self.goal_infer_modify_info_msg.a_inferred = a_inferred  # string
            self.goal_infer_modify_info_msg.ph_inferred = ph_inferred  # string
            self.goal_infer_modify_info_msg.g_inferred = g_inferred
            self.goal_infer_modify_info_msg.normalized_h = normalized_h_of_p_g_given_phm  # float

            self.goal_infer_modify_info_msg.ph_modified = ph_modified
            self.goal_infer_modify_info_msg.is_corrected_or_filtered = is_corrected_or_filtered
            self.goal_infer_modify_info_msg.is_ph_inferred_equals_phm = is_ph_inferred_equals_phm
            self.goal_infer_modify_info_msg.p_g_given_phm = list(self.P_G_GIVEN_PHM.values())
            self.goal_infer_modify_info_pub.publish(self.goal_infer_modify_info_msg)
        else:
            response.ph_modified = "None"
            response.is_corrected_or_filtered = False
            response.status = True

        return response

    def update_assistance_type(self):
        self.ASSISTANCE_TYPE = rospy.get_param("assistance_type")
        if self.ASSISTANCE_TYPE == 0:
            self.ASSISTANCE_TYPE = AssistanceType.Filter
        elif self.ASSISTANCE_TYPE == 1:
            self.ASSISTANCE_TYPE = AssistanceType.Corrective
        elif self.ASSISTANCE_TYPE == 2:
            self.ASSISTANCE_TYPE = AssistanceType.No_Assistance

    def _modify_or_pass_phm(self, phm, ph_inferred, normalized_h_of_p_g_given_phm):
        self.update_assistance_type()
        # print("Normalized Entropy ", normalized_h_of_p_g_given_phm)
        if ph_inferred != phm:
            if normalized_h_of_p_g_given_phm <= self.ENTROPY_THRESHOLD:
                if self.ASSISTANCE_TYPE == AssistanceType.Filter:
                    # need to be interprted properly in the teleop node. None --> Zero Band for SNP
                    ph_modified = "None"
                elif self.ASSISTANCE_TYPE == AssistanceType.Corrective:
                    ph_modified = ph_inferred
                elif self.ASSISTANCE_TYPE == AssistanceType.No_Assistance:
                    return phm, False, False
            else:
                return phm, False, False
        else:
            return phm, False, True

        return ph_modified, True, False  # only triggered when Filter or Corrective mode

    def _modify_filter_or_pass_phm(self, phm, ph_inferred, normalized_h_of_p_g_given_phm, current_s):
        # PHASE 1. check if anything needs to be filtered.
        # Filtering happens when the action generated has low likelihood FOR all goals.

        # Filter phase is triggered only when phm is non None
        # if phm != "None":
        #     # likelihood dict for phm for each goal
        #     likelihood_g = collections.OrderedDict()
        #     for g in self.P_G_GIVEN_PHM.keys():
        #         likelihood = 0.0
        #         for a in self.P_PHI_GIVEN_A.keys():
        #             for phi in self.P_PHM_GIVEN_PHI.keys():
        #                 likelihood += (
        #                     self.P_PHM_GIVEN_PHI[phi][phm]
        #                     * self.P_PHI_GIVEN_A[a][phi]
        #                     * ((self.P_A_S_ALL_G_DICT[g][a]) ** (1.0))
        #                 )
        #         likelihood_g[g] = likelihood

        #     # algo for determining if the action is of low likelihood for all goals, if yes block
        #     filter_assist = True
        #     for g in likelihood_g.keys():
        #         if likelihood_g[g] > self.LIKELIHOOD_THRESHOLD:
        #             # if likelihood for ANY g is higher than likelihood dont filter.
        #             filter_assist = False
        #             break

        #     if filter_assist:
        #         print("In FILTER LIKELIHOOD DICT", likelihood_g)
        #         ph_modified = "None"
        #         return ph_modified, True, False

        # intermediate
        if ph_inferred != phm:
            if normalized_h_of_p_g_given_phm <= self.ENTROPY_THRESHOLD:
                # standard corrective mechanism
                print("In Corrective ")
                ph_modified = ph_inferred
                return ph_modified, True, False
            else:
                # majority vote stuff
                # TO DO: threshold on eqaulity (instead of exact)
                if phm != "None":
                    # for each goal, combine the weights of each optimal action
                    voting_info_dict = collections.defaultdict(list)
                    for g in self.P_G_GIVEN_PHM.keys():
                        weight_g = self.P_G_GIVEN_PHM[g]  # weight associated with each goal
                        # optimal action for g in current state
                        optimal_a_for_g_in_current_state = self.OPTIMAL_ACTION_FOR_S_G[g]
                        # populate dictionary according to action.
                        voting_info_dict[optimal_a_for_g_in_current_state].append(weight_g)

                    # weighted majority voting
                    cumulative_action_weight_dict = collections.OrderedDict()
                    for a in TASK_LEVEL_ACTIONS:
                        cumulative_action_weight_dict[a] = 0.0
                        if a in voting_info_dict.keys():
                            # task level action is present in the dictionary sum up the weights.
                            cumulative_action_weight_dict[a] = sum(voting_info_dict[a])

                    # get task_level_action corresponding to max weight
                    print("In VOTING", cumulative_action_weight_dict)
                    cumulative_action_weight_vector = np.array(cumulative_action_weight_dict.values())
                    a_inferred = cumulative_action_weight_dict.keys()[np.argmax(cumulative_action_weight_vector)]
                    # get interface level action corresponding to a_inferred
                    ph_modified = TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[a_inferred]

                    return ph_modified, True, False
                else:
                    # print("PH_INFERRED NOT EQUALS PHM and PHM == None")
                    return phm, False, True

        else:
            print("PHM equals PH_INFERRED")
            return phm, False, True

    def _compute_entropy_of_p_g_given_phm(self):
        p_g_given_phm_vector = np.array(self.P_G_GIVEN_PHM.values())
        p_g_given_phm_vector = p_g_given_phm_vector + np.finfo(p_g_given_phm_vector.dtype).tiny
        uniform_distribution = np.array([1.0 / p_g_given_phm_vector.size] * p_g_given_phm_vector.size)
        max_entropy = -np.dot(uniform_distribution, np.log2(uniform_distribution))
        normalized_h_of_p_g_given_phm = -np.dot(p_g_given_phm_vector, np.log2(p_g_given_phm_vector)) / max_entropy
        return normalized_h_of_p_g_given_phm

    def _compute_g_a_ph_inferred(self):
        p_g_given_um_vector = np.array(self.P_G_GIVEN_PHM.values())
        # need to add realmin to avoid nan issues with entropy calculation is p_ui_given_um_vector is delta distribution'
        p_g_given_um_vector = p_g_given_um_vector + np.finfo(p_g_given_um_vector.dtype).tiny
        g_inferred = self.P_G_GIVEN_PHM.keys()[np.argmax(p_g_given_um_vector)]  # argmax computation for g_inferred
        # retreive optimal task level action corresponding to inferred goal optimal action will always be not None
        a_inferred = self.OPTIMAL_ACTION_FOR_S_G[g_inferred]  # string (always not None)
        # interface level action corresponding to a inferred
        ph_inferred = TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[a_inferred]
        return g_inferred, a_inferred, ph_inferred, p_g_given_um_vector

    def _compute_p_g_given_phm(self, phm):
        # print("PHM", phm)
        if phm != "None":
            self.decay_counter = 0
            for g in self.P_G_GIVEN_PHM.keys():  # already initialized
                likelihood = 0.0  # likelihood
                for a in self.P_PHI_GIVEN_A.keys():
                    for phi in self.P_PHM_GIVEN_PHI.keys():
                        likelihood += (
                            self.P_PHM_GIVEN_PHI[phi][phm]
                            * self.P_PHI_GIVEN_A[a][phi]
                            * ((self.P_A_S_ALL_G_DICT[g][a]) ** (0.3))
                        )

                self.P_G_GIVEN_PHM[g] = self.P_G_GIVEN_PHM[g] * likelihood  # multiply with prior
            normalization_constant = sum(self.P_G_GIVEN_PHM.values())
            for g in self.P_G_GIVEN_PHM.keys():  # NORMALIZE POSTERIOR
                self.P_G_GIVEN_PHM[g] = self.P_G_GIVEN_PHM[g] / normalization_constant

            # print("Current Belief ", self.P_G_GIVEN_PHM)

        else:
            prob_blend_factor = 1 - np.exp(
                -self.decay_scale_factor * min(self.decay_counter, self.decay_counter_max_value)
            )
            uniform_dist = (1.0 / self.NUM_GOALS) * np.ones(self.NUM_GOALS)
            blended_dist = (1 - prob_blend_factor) * np.array(
                list(self.P_G_GIVEN_PHM.values())
            ) + prob_blend_factor * uniform_dist
            for idx, g in enumerate(self.P_G_GIVEN_PHM.keys()):
                self.P_G_GIVEN_PHM[g] = blended_dist[idx]
            # print("PHM NONE, therefore no belief update", prob_blend_factor)
            self.decay_counter += 1
        # print("Current Belief ", self.P_G_GIVEN_PHM)

    def init_P_G_GIVEN_PHM(self, req):
        """
        Initializes the p(g | phm) dict to uniform dictionary at the beginning of each trial
        """
        # service to be called at the beginning of each trial to reinit the distribution.
        # number of goals could be different for different goals.
        print("In Init Belief Service")
        self.NUM_GOALS = req.num_goals
        self.P_G_GIVEN_PHM = collections.OrderedDict()

        for g in range(self.NUM_GOALS):
            self.P_G_GIVEN_PHM[g] = 1.0 / self.NUM_GOALS

        normalization_constant = sum(self.P_G_GIVEN_PHM.values())
        for g in self.P_G_GIVEN_PHM.keys():  # NORMALIZE POSTERIOR
            self.P_G_GIVEN_PHM[g] = self.P_G_GIVEN_PHM[g] / normalization_constant

        print("Initial Belief ", self.P_G_GIVEN_PHM)
        response = InitBeliefResponse()
        response.status = True
        return response

    def init_P_PHI_GIVEN_A(self):
        # only to be done at the beginning of a session for a subject. No updating between trials
        self.P_PHI_GIVEN_A = collections.OrderedDict()
        for k in TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP.keys():  # task level action
            self.P_PHI_GIVEN_A[k] = collections.OrderedDict()
            for u in INTERFACE_LEVEL_ACTIONS:
                if u == TRUE_TASK_ACTION_TO_INTERFACE_ACTION_MAP[k]:
                    # try to weight the true command more for realistic purposes. Can be offset by using a high PHI_GIVEN_A_NOISE
                    self.P_PHI_GIVEN_A[k][u] = 1.0
                else:
                    self.P_PHI_GIVEN_A[k][u] = 0.0

            delta_dist = np.array(list(self.P_PHI_GIVEN_A[k].values()))
            uniform_dist = (1.0 / len(INTERFACE_LEVEL_ACTIONS)) * np.ones(len(INTERFACE_LEVEL_ACTIONS))
            blended_dist = (
                1 - self.DEFAULT_PHI_GIVEN_A_NOISE
            ) * delta_dist + self.DEFAULT_PHI_GIVEN_A_NOISE * uniform_dist  # np.array
            for index, u in enumerate(INTERFACE_LEVEL_ACTIONS):
                self.P_PHI_GIVEN_A[k][u] = blended_dist[index]

    def init_P_PHM_GIVEN_PHI(self):
        """
        Generates a random p(um|ui). key = ui, subkey = um
        """
        self.P_PHM_GIVEN_PHI = collections.OrderedDict()
        for i in INTERFACE_LEVEL_ACTIONS:  # ui
            self.P_PHM_GIVEN_PHI[i] = collections.OrderedDict()
            for j in INTERFACE_LEVEL_ACTIONS:  # um
                if i == j:
                    # try to weight the true command more for realistic purposes. Can be offset by using a high UM_GIVEN_UI_NOISE
                    self.P_PHM_GIVEN_PHI[i][j] = 1.0
                else:
                    # P_PHM_GIVEN_PHI[i][j] = np.random.random()*UM_GIVEN_UI_NOISE#IF UM_GIVEN_UI_NOISE is 0, then the p(um|ui) is a deterministic mapping
                    self.P_PHM_GIVEN_PHI[i][j] = 0.0

            delta_dist = np.array(list(self.P_PHM_GIVEN_PHI[i].values()))
            uniform_dist = (1.0 / len(INTERFACE_LEVEL_ACTIONS)) * np.ones(len(INTERFACE_LEVEL_ACTIONS))
            blended_dist = (
                1 - self.DEFAULT_PHM_GIVEN_PHI_NOISE
            ) * delta_dist + self.DEFAULT_PHM_GIVEN_PHI_NOISE * uniform_dist  # np.array
            for index, j in enumerate(INTERFACE_LEVEL_ACTIONS):
                self.P_PHM_GIVEN_PHI[i][j] = blended_dist[index]


if __name__ == "__main__":
    subject_id = sys.argv[1]
    GoalInferenceAndCorrection(subject_id)
    rospy.spin()
