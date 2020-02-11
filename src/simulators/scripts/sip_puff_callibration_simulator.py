#!/usr/bin/env python

# This is a python script for teaching hard/soft sip/puff commands to novice sip_and_puff user
# 
# This was written for the paper titled "Do You Really Want To Do that?Customized Handling of Unintended Actions InAssistive Robots" to
# train the user before running the command_following_simulation task 

import rospy
from sensor_msgs.msg import Joy
from envs.sip_puff_callibration_env import SipPuffCallibrationEnv
from utils import LOW_LEVEL_COMMANDS, EXPERIMENT_START_COUNTDOWN


class SipPuffCallibration(object):
    def __init__(self):

        # initialization
        rospy.init_node("sip_puff_callibration")
        rospy.on_shutdown(self.shutdown_hook)
        self.initialize_subscribers()

        env_params = dict()
        env_params['command'] = ''

        self.env = SipPuffCallibrationEnv(env_params)
        # self.call_render(env_params['command'])

        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            self.env.reset()
            self.env.render()
            r.sleep()

    def initialize_subscribers(self):
        rospy.Subscriber('/joy_sip_puff', Joy, self.joy_callback)

    def joy_callback(self, msg): 
        # self.call_render(msg.header.frame_id)
        self.env.env_params['command'] = msg.header.frame_id

    def shutdown_hook(self):
        pass


if __name__ == '__main__':
    SipPuffCallibration()
    rospy.spin()
