#!/usr/bin/env python

# This is a python script for measuring stochastic deviations of input commands from intended commands
# This was written for the paper titled "Do You Really Want To Do that?Customized Handling of Unintended Actions InAssistive Robots" to
# model personalized distributions for p(u_i|u_m) from user data

import rospy
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from simulators.msg import Command
from envs.text_window_env import TextWindowEnv
from utils import LOW_LEVEL_COMMANDS, EXPERIMENT_START_COUNTDOWN
import pyglet
import sys
from random import randrange
import threading


class CommandFollowing(object):
    def __init__(self, duration=1.0, iterations=1):

        # initialization
        rospy.init_node("command_following")
        self.initialize_subscribers()
        self.initialize_publishers()

        self.duration = float(duration) # duration command text is displayed on screen
        self.iterations = int(iterations) # number of iterations each command is to be displayed on screen 
        self.countdown_duration = 1.0
        self.send_command = True
        self.command_list = []

        self.command_msg = Command()

        env_params = dict()
        env_params['text'] = ''

        self.env = TextWindowEnv(env_params)
        self.env.reset()

        self.generate_command_list() # generate random order of commands 

        self.event = threading.Event()
        self.lock = threading.Lock()


    def initialize_subscribers(self):
        rospy.Subscriber('/keyboard_entry', String, self.keyboard_callback)
        rospy.Subscriber('/joy_sip_puff', Joy, self.joy_callback)

    def initialize_publishers(self):
        # for ros bag purposes (not being used for any code logic)
        self.command_pub = rospy.Publisher('command_prompt', Command, queue_size=1)

    def publish_command(self, msg):
        self.command_msg.header.stamp = rospy.Time.now()
        self.command_msg.command = msg
        self.command_pub.publish(self.command_msg)

    # start experiment
    def keyboard_callback(self, msg): 
        # Start experiment 
        if msg.data == 's':            
            for i in range(len(EXPERIMENT_START_COUNTDOWN)): 
                self.call_render(EXPERIMENT_START_COUNTDOWN[i], self.countdown_duration)
            self.command_following_task()

    def joy_callback(self, msg): 
        if  not self.send_command: 
            if msg.header.frame_id == "input stopped":  
                print 'reset'
                self.send_command = True
                self.event.set()

    # randomize commands iterations
    def generate_command_list(self):
        for i in range(self.iterations): 
            commands = LOW_LEVEL_CONTROL_COMMANDS[:]
            for j in range(len(commands)): 
                rand_index = randrange(len(commands))
                self.command_list.append(commands[rand_index]) 
                commands.pop(rand_index)

    # display commands for desired duration and (wait for user to stop input before sending next command)
    def command_following_task(self): 
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
        self.env.viewer.close()

    # set new text message and render            
    def call_render(self, msg, duration): 
            self.env.env_params['text'] = msg
            self.env.reset()
            self.env.render()
            rospy.sleep(duration)

    def shutdown_hook(self):
        pass


if __name__ == '__main__':
    CommandFollowing(sys.argv[1], sys.argv[2])
    rospy.spin()