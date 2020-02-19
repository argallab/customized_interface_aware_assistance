#!/usr/bin/env python

import rospy
import rospkg
import pickle
import os
import numpy as np
from envs.srv import OptimalAction, OptimalActionRequest, OptimalActionResponse
from inference_and_correction.msg import InferCorrectInfo
from teleop_nodes.srv import InferCorrect, InferCorrectRequest, InferCorrectResponse
import collections
from IPython import embed
import math
import sys
sys.path.append(os.path.join(rospkg.RosPack().get_path('simulators'), 'scripts'))
from utils import TRUE_ACTION_TO_COMMAND, LOW_LEVEL_COMMANDS
from utils import AssistanceType


class ModeSwitchInferenceAndCorrection(object):
    def __init__(self, subject_id):
        rospy.init_node('mode_switch_inference_and_correction', anonymous=True)
        self.subject_id = subject_id
        self.distribution_directory_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'personalized_distributions')

        self.infer_correct_info_pub = rospy.Publisher('/infer_correct_info', InferCorrectInfo, queue_size=1)

        self.infer_correct_info_msg = InferCorrectInfo()
        self.P_UI_GIVEN_A = None
        self.P_UM_GIVEN_UI = None
        self.DEFAULT_UI_GIVEN_A_NOISE = 0.01
        self.DEFAULT_UM_GIVEN_UI_NOISE = 0.35
        self.P_UI_GIVEN_UM = collections.OrderedDict()
        self.ASSISTANCE_TYPE = rospy.get_param('assistance_type', 2)
        if self.ASSISTANCE_TYPE == 0:
            self.ASSISTANCE_TYPE = AssistanceType.Filter
        elif self.ASSISTANCE_TYPE == 1:
            self.ASSISTANCE_TYPE = AssistanceType.Corrective
        elif self.ASSISTANCE_TYPE == 2:
            self.ASSISTANCE_TYPE = AssistanceType.No_Assistance

        self.ENTROPY_THRESHOLD = rospy.get_param('entropy_threshold', 0.4)

        for u in LOW_LEVEL_COMMANDS:
            self.P_UI_GIVEN_UM[u] = 1.0/len(LOW_LEVEL_COMMANDS)

        if os.path.exists(os.path.join(self.distribution_directory_path, str(self.subject_id)+'_p_ui_given_a.pkl')):
            print('LOADING PERSONALIZED P_UI_GIVEN_A')
            with open(os.path.join(self.distribution_directory_path, str(self.subject_id)+'_p_ui_given_a.pkl'), 'rb') as fp:
                self.P_UI_GIVEN_A = pickle.load(fp)#assumes that the conditional probability distribution is stored as a collections.OrderedDict conditioned on the mode
        else:
            self.P_UI_GIVEN_A = collections.OrderedDict()
            self._init_p_ui_given_a()

        if os.path.exists(os.path.join(self.distribution_directory_path, str(self.subject_id)+'_p_um_given_ui.pkl')):
            print('LOADING PERSONALIZED P_UM_GIVEN_UI')
            with open(os.path.join(self.distribution_directory_path, str(self.subject_id)+'_p_um_given_ui.pkl'), 'rb') as fp:
                self.P_UM_GIVEN_UI = pickle.load(fp) #assumes that the conditional probability distribution is stored as a collections.OrderedDict
        else:
            self.P_UM_GIVEN_UI = collections.OrderedDict()
            self._init_p_um_given_ui()


        rospy.Service('/mode_switch_inference_and_correction/handle_unintended_commands', InferCorrect, self.handle_unintended_commands)

        rospy.loginfo("Waiting for mode_inference_env node ")
        rospy.wait_for_service("/mode_inference_env/get_optimal_action")
        rospy.loginfo("mode_inference_env node found!")
        self.get_optimal_action = rospy.ServiceProxy('/mode_inference_env/get_optimal_action', OptimalAction)

    def handle_unintended_commands(self, req):
        um = req.um
        response = InferCorrectResponse()
        optimal_action_response = self.get_optimal_action()
        # print optimal_action_response.optimal_high_level_action
        if optimal_action_response.status: #not at the last position
            optimal_a = optimal_action_response.optimal_high_level_action #mode_r, model, move_p, move_n
            current_mode = optimal_action_response.current_mode
            # print("OPTIMAL A", optimal_a)
            # print("CURRENT_MODE", current_mode)
            self.compute_p_ui_given_um(optimal_a, current_mode, um)
            u_intended = self.compute_u_intended() #argmax computation for u_intended
            # print ("U_INTENDED", u_intended)
            normalized_h_of_p_ui_given_um = self.compute_entropy_of_p_ui_given_um()
            # print("ENTROPY", normalized_h_of_p_ui_given_um)
            u_corrected, is_corrected_or_filtered, is_u_intended_equals_um = self.correct_or_pass_um(um, u_intended, normalized_h_of_p_ui_given_um)
            # print ("U_CORRECTED", u_corrected, is_corrected_or_filtered)
            response.u_corrected = u_corrected
            response.is_corrected_or_filtered = is_corrected_or_filtered
            response.status = True

            self.infer_correct_info_msg.optimal_a = optimal_a #string
            self.infer_correct_info_msg.u_intended = u_intended #string
            self.infer_correct_info_msg.normalized_h = normalized_h_of_p_ui_given_um #float
            self.infer_correct_info_msg.u_corrected = u_corrected
            self.infer_correct_info_msg.is_corrected_or_filtered = is_corrected_or_filtered
            self.infer_correct_info_msg.is_u_intended_equals_um = is_u_intended_equals_um
            self.infer_correct_info_pub.publish(self.infer_correct_info_msg)
        else:
            response.u_corrected = 'Zero Band'
            response.status = True

        return response

    def compute_entropy_of_p_ui_given_um(self):
        p_ui_given_um_vector = np.array(self.P_UI_GIVEN_UM.values())
        p_ui_given_um_vector = p_ui_given_um_vector + np.finfo(p_ui_given_um_vector.dtype).tiny
        uniform_distribution = np.array([1.0/p_ui_given_um_vector.size]*p_ui_given_um_vector.size)
        max_entropy = -np.dot(uniform_distribution, np.log2(uniform_distribution))
        normalized_h_of_p_ui_given_um = -np.dot(p_ui_given_um_vector, np.log2(p_ui_given_um_vector))/max_entropy
        return normalized_h_of_p_ui_given_um

    def compute_u_intended(self):
        p_ui_given_um_vector = np.array(self.P_UI_GIVEN_UM.values())
        u_intended = self.P_UI_GIVEN_UM.keys()[np.argmax(p_ui_given_um_vector)]
        return u_intended

    def update_assistance_type(self):
        #TODO maybe replace with service
        self.ASSISTANCE_TYPE =  rospy.get_param('assistance_type')
        if self.ASSISTANCE_TYPE == 0:
            self.ASSISTANCE_TYPE = AssistanceType.Filter
        elif self.ASSISTANCE_TYPE == 1:
            self.ASSISTANCE_TYPE = AssistanceType.Corrective
        elif self.ASSISTANCE_TYPE == 2:
            self.ASSISTANCE_TYPE = AssistanceType.No_Assistance

    def correct_or_pass_um(self, um, u_intended, normalized_h_of_p_ui_given_um):
        self.update_assistance_type()
        # print self.ASSISTANCE_TYPE
        if u_intended != um:
            if normalized_h_of_p_ui_given_um <= self.ENTROPY_THRESHOLD:
                if self.ASSISTANCE_TYPE == AssistanceType.Filter:
                    u_corrected = 'Zero Band' #proxy for zero. Maybe have zero-band
                elif self.ASSISTANCE_TYPE == AssistanceType.Corrective:
                    u_corrected = u_intended
            else:
                return um, False, False
        else:
            return um, False, True

        return u_corrected, True, False

    def compute_p_ui_given_um(self, optimal_a, current_mode, um):
        for ui in LOW_LEVEL_COMMANDS:
            self.P_UI_GIVEN_UM[ui] = self.P_UM_GIVEN_UI[ui][um] * self.P_UI_GIVEN_A[current_mode][optimal_a][ui]

        normalization_constant = sum(self.P_UI_GIVEN_UM.values())
        for u in self.P_UI_GIVEN_UM.keys():
            self.P_UI_GIVEN_UM[u] = self.P_UI_GIVEN_UM[u]/normalization_constant

    def _init_p_ui_given_a(self):
        for mode in TRUE_ACTION_TO_COMMAND.keys():
            self.P_UI_GIVEN_A[mode] = collections.OrderedDict()
            for action in TRUE_ACTION_TO_COMMAND[mode].keys():
                self.P_UI_GIVEN_A[mode][action] = collections.OrderedDict()
                for u in LOW_LEVEL_COMMANDS:
                    if u == TRUE_ACTION_TO_COMMAND[mode][action]:
                        self.P_UI_GIVEN_A[mode][action][u] = 1.0
                    else:
                        self.P_UI_GIVEN_A[mode][action][u] = np.random.random()*self.DEFAULT_UI_GIVEN_A_NOISE
                normalization_constant = sum(self.P_UI_GIVEN_A[mode][action].values())
                self.P_UI_GIVEN_A[mode][action] = collections.OrderedDict({u:(v/normalization_constant) for u, v in self.P_UI_GIVEN_A[mode][action].items()})

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
    subject_id = sys.argv[1]
    s = ModeSwitchInferenceAndCorrection(subject_id)
    rospy.spin()
