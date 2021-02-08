#!/usr/bin/env python
# Code developed by Deepak Gopinath*, Mahdieh Nejati Javaremi* in February 2020. Copyright (c) 2020. Deepak Gopinath, Mahdieh Nejati Javaremi, Argallab. (*) Equal contribution

import rospy
import rospkg
from teleop_nodes.msg import CartVelCmd
from simulators.msg import State4D
import numpy as np

class PfController(object):
    def __init__(self):
        super(PfController, self).__init__()
        rospy.init_node('stabilization_pf_control')
        rospy.Subscriber('/robot_state', State4D, self.robot_state_cb)
        self.stabilizer_control_pub = rospy.Publisher('/stabilizer_control_action', CartVelCmd, queue_size=1)
    
    def robot_state_cb(self, msg):
        print(msg.robot_continuous_position)

if __name__ == '__main__':
    s = PfController()
    rospy.spin()