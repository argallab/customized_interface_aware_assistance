#!/usr/bin/env python

# This is a python script for teaching hard/soft sip/puff commands to novice sip_and_puff user
# 
# This was written for the paper titled "Do You Really Want To Do that?Customized Handling of Unintended Actions InAssistive Robots" to
# train the user before running the command_following_simulation task 

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from envs.sip_puff_training_env import SipPuffTrainingEnv
from utils import LOW_LEVEL_COMMANDS, EXPERIMENT_START_COUNTDOWN
from dynamic_reconfigure.server import Server
from simulators.cfg import SipPuffTrainingParadigmConfig
import sys
from random import randrange

class SipPuffTraining(object):
    def __init__(self, iterations=1):

        # initialization
        rospy.init_node("sip_puff_training")
        rospy.on_shutdown(self.shutdown_hook)
        self.initialize_subscribers()
        self.initialize_services()
        self.iterations = int(iterations) # number of iterations each command is to be displayed on screen 
        self.train_prompted = False
        self.countdown_duration = 1.0

        env_params = dict()
        env_params['command'] = ''

        self.env = SipPuffTrainingEnv(env_params)

        r = rospy.Rate(100)

        while not rospy.is_shutdown():
            # self.env.reset()
            self.env.step(self.env.env_params['command'] )
            self.env.render()
            r.sleep()

    def initialize_subscribers(self):
        rospy.Subscriber('/joy_sip_puff', Joy, self.joy_callback)
        rospy.Subscriber('/keyboard_entry', String, self.keyboard_callback)

    def initialize_services(self):
        training_paradigm_srv = Server(SipPuffTrainingParadigmConfig, self.reconfigure_cb)

    def joy_callback(self, msg): 
        # self.call_render(msg.header.frame_id)
        # self.input = msg.header.frame_id
        self.env.env_params['command'] = msg.header.frame_id

    def keyboard_callback(self, msg): 
        # if in prompted training, start
        if self.train_prompted:  
            print 'starting'
            if msg.data == 's':    
                self.env.env_params['start_prompt'] = True        
                self.env.reset()

    # randomize commands iterations
    def generate_command_list(self):
        for i in range(self.iterations): 
            commands = LOW_LEVEL_COMMANDS[:]
            for j in range(len(commands)): 
                rand_index = randrange(len(commands))
                self.command_list.append(commands[rand_index]) 
                commands.pop(rand_index)
        self.env.env_params['prompt_commands'] = self.command_list[:]

    def training_task(self): 
        for i in range(len(self.command_list)): 
            print i
            if self.send_command: 
                self.lock.acquire()
                self.publish_command(self.command_list[i])
                self.call_render(self.command_list[i], self.duration) 
                self.send_command = False
                self.call_render('', self.duration)                   
                self.lock.release()
            else: 
                self.event.wait()  # blocks until flag becomes true
                self.event.clear() # clear flag for next loop
        self.call_render('ALL DONE! :D', self.duration)

    # Training condition switching rosparam: exploratory vs. prompted
    def reconfigure_cb(self, config, level):
        self.train_prompted = config.prompted_training
        if self.train_prompted: 
            print 'starting'
            self.command_list = []
            self.generate_command_list()
        return config


    def shutdown_hook(self):
        pass


if __name__ == '__main__':
    SipPuffTraining(sys.argv[1])
    rospy.spin()