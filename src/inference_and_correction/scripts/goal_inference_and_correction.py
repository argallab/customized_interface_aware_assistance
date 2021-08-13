#!/usr/bin/env python
# Code developed by Deepak Gopinath*, Mahdieh Nejati Javaremi* in February 2020. Copyright (c) 2020. Deepak Gopinath, Mahdieh Nejati Javaremi, Argallab. (*) Equal contribution

import rospy
import rospkg
import pickle
import os
import numpy as np
from envs.srv import OptimalAction, OptimalActionRequest, OptimalActionResponse
from inference_and_correction.msg import InferCorrectInfo
from teleop_nodes.srv import InferCorrect, InferCorrectRequest, InferCorrectResponse
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
)


class GoalInferenceAndCorrection(object):
    def __init__(self, subject_id):
        rospy.init_node("goal_inference and correction")
        self.subject_id = subject_id
        self.distribution_directory_path = os.path.join(
            os.path.dirname(os.path.dirname(__file__)), "se2_personalized_distributions"
        )
        self.infer_correct_info_pub = rospy.Publisher("/infer_correct_info", InferCorrectInfo, queue_size=1)
        self.infer_correct_info_msg = InferCorrectInfo()

        self.P_PHI_GIVEN_A = None
        self.P_PHM_GIVEN_PHI = None
        self.DEFAULT_PHI_GIVEN_A_NOISE = 0.01
        self.DEFAULT_PHM_GIVEN_PHI_NOISE = 0.3

        self.P_G_GIVEN_PHM = None
        self.ASSISTANCE_TYPE = rospy.get_param("assistance_type", 2)

        if self.ASSISTANCE_TYPE == 0:
            self.ASSISTANCE_TYPE = AssistanceType.Filter
        elif self.ASSISTANCE_TYPE == 1:
            self.ASSISTANCE_TYPE = AssistanceType.Corrective
        elif self.ASSISTANCE_TYPE == 2:
            self.ASSISTANCE_TYPE = AssistanceType.No_Assistance

        self.ENTROPY_THRESHOLD = rospy.get_param("entropy_threshold", 0.9)

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

        rospy.Service(
            "/goal_inference_and_correction/handle_inference_and_unintended_actions",
            InferCorrect,
            self.handle_inference_and_unintended_actions,
        )
        rospy.Service("/goal_inference_and_correction/init_belief", InitBelief, self.init_P_G_GIVEN_PHM)

        # register for service
        rospy.loginfo("Waiting for sim_env node ")
        rospy.wait_for_service("/sim_env/get_prob_a_s_all_g")
        rospy.loginfo("sim_env node found!")
        self.get_prob_a_s_all_g = rospy.ServiceProxy("/sim_env/get_prob_a_s_all_g", OptimalAction)

    def handle_inference_and_unintended_actions(self, req):
        phm = req.phm
        response = InferCorrectResponse()
        p_a_s_all_g_response = self.get_prob_a_s_all_g()

        if p_a_s_all_g_response.status:
            pass
            # update dictionary of p_a_s_g
            # get current state, mode from p_a_s_all_g_response
            # do Bayesian inference and update belief over goals.
            # get goal corresponding to max.
            # get optimal task level action for that goal. (need mdp access again)
            # get interface level action for that goal. (simple lookup)
            # apply assistance by checking entropy (get phm_modified)
            # populate response
        else:
            response.phm_modified = "Zero Band"
            response.status = True

        return response

    def init_P_G_GIVEN_PHM(self, req):
        """
        Initializes the p(g | phm) dict to uniform dictionary
        """
        # service to be called at the beginning of each trial to reinit the distribution.
        # number of goals could be different for different goals.
        self.NUM_GOALS = req.num_goals
        self.P_G_GIVEN_PHM = collections.OrderedDict()

        for g in range(self.NUM_GOALS):
            self.P_G_GIVEN_PHM[g] = 1.0 / self.NUM_GOALS

        normalization_constant = sum(self.P_G_GIVEN_PHM.values())
        for g in self.P_G_GIVEN_PHM.keys():  # NORMALIZE POSTERIOR
            self.P_G_GIVEN_PHM[g] = self.P_G_GIVEN_PHM[g] / normalization_constant

        print("Initial belief ", self.P_G_GIVEN_PHM)
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
