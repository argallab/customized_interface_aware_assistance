#!/usr/bin/env python

# This is a python script for teaching hard/soft sip/puff commands to novice sip_and_puff user
# 
# This was written for the paper titled "Do You Really Want To Do that?Customized Handling of Unintended Actions InAssistive Robots" to
# train the user before running the command_following_simulation task 

import rospy
from sensor_msgs.msg import Joy
from envs.sip_puff_callibration import SipPuffCallibrationEnv
from utils import LOW_LEVEL_COMMANDS, EXPERIMENT_START_COUNTDOWN


class SipPuffCallibration(object):
    def __init__(self):

        # initialization
        rospy.init_node("sip_puff_callibration")
        # self.initialize_subscribers()
        # self.initialize_publishers()

        env_params = dict()
        env_params['command'] = 'hello'

        self.env = SipPuffCallibrationEnv(env_params)
        self.env.reset()

        self.call_render(env_params['command'])


    def initialize_subscribers(self):
        rospy.Subscriber('/joy_sip_puff', Joy, self.joy_callback)

    def joy_callback(self, msg): 
        print msg.header.frame_id
        self.call_render(msg.header.frame_id)


    # set new text message and render            
    def call_render(self, msg): 
        self.env.env_params['command'] = msg
        self.env.reset()
        self.env.render()
        rospy.sleep(0.1)

    def shutdown_hook(self):
        pass


if __name__ == '__main__':
    SipPuffCallibration()
    rospy.spin()
