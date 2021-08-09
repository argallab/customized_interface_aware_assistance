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
sys.path.append(os.path.join(rospkg.RosPack().get_path('simulators'), 'scripts'))
from corrective_mode_switch_utils import TRUE_ACTION_TO_COMMAND, LOW_LEVEL_COMMANDS
from corrective_mode_switch_utils import AssistanceType

class GoalInferenceAndCorrection(object):
    def __init__(self, subject_id, optim):
        rospy.init_node('goal_inference and correction')
        self.subject_id = subject_id

        self.P_PHI_GIVEN_A = None
        self.P_PHM_GIVEN_PHI = None
        self.DEFAULT_PHI_GIVEN_A_NOISE = 0.01
        self.DEFAULT_PHM_GIVEN_PHI_NOISE = 0.3
        
        self.P_G_GIVEN_PHM = None
        self.ASSISTANCE_TYPE = rospy.get_param('assistance_type', 2)

        if self.ASSISTANCE_TYPE == 0:
            self.ASSISTANCE_TYPE = AssistanceType.Filter
        elif self.ASSISTANCE_TYPE == 1:
            self.ASSISTANCE_TYPE = AssistanceType.Corrective
        elif self.ASSISTANCE_TYPE == 2:
            self.ASSISTANCE_TYPE = AssistanceType.No_Assistance
        