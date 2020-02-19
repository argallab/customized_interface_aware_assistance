#!/usr/bin/env python

import os
from backends.rendering import Viewer
from utils import SCALE, VIEWPORT_W, VIEWPORT_H
from utils import COMMAND_TEXT_COLOR, COMMAND_DISPLAY_POSITION, COMMAND_DISPLAY_FONTSIZE 
from utils import TIMER_DISPLAY_POSITION, TIMER_DISPLAY_FONTSIZE, TIMER_COLOR_NEUTRAL, TIMER_DISPLAY_TEXT_Y_ANCHOR
from utils import EXPERIMENT_START_COUNTDOWN    
from simulators.msg import Command
import pyglet
import collections 
import rospy
import time
from pyglet.window import key
import random 
import threading
from IPython import embed

class ActionEnv(object):

    def __init__(self, env_params):

        self.viewer = None
        self.action_msg = Command()

        self.env_params = env_params
        assert self.env_params is not None
        assert 'file_dir' in self.env_params
        assert 'blocks' in self.env_params

        self.file_dir = self.env_params['file_dir']
        self.img_prompt = self.env_params['img_prompt']

        self.ts = time.time()
        self.prompt_ind = 0
        self.msg_prompt = ''
        self.start_prompt = False
        self.clear_for_next_prompt = False
        self.bold = True
        self.action_timing_bound = 4 #seconds
        self.text_timing_bound = 2

        self.period = rospy.Duration(1.0)
        self.timer_thread = threading.Thread(target=self._render_timer, args=(self.period,))
        self.lock = threading.Lock()
        self.current_time = 0
        self.start_timer = False
        self.start_training = False 
        self.next = False 
        self.back = False 
        self.current_block = 0
        self.display_timer = False  

      

    def initialize_viewer(self): 
        if self.viewer is None:
            self.viewer = Viewer(VIEWPORT_W*2, VIEWPORT_H*2)
            self.viewer.set_bounds(0, (VIEWPORT_W*2)/SCALE, 0, (VIEWPORT_H*2)/SCALE)
            self.viewer.window.set_location(650, 300)
            self.timer_thread.start()
        

    def initialize_publishers(self, rostopic):
        # for ros bag purposes (not being used for any code logic)
        self.action_pub = rospy.Publisher(rostopic, Command, queue_size=1)

    def publish_action(self, msg):
        self.action_msg.header.stamp = rospy.Time.now()
        self.action_msg.command = msg
        self.action_pub.publish(self.action_msg)

    def _set_image_path(self):         
        self.file_path = os.path.abspath(os.path.join(self.file_dir, self.img_prompt+'.png'))

    def _render_sprite(self):
        self.viewer.draw_sprite(self.file_path, x=VIEWPORT_W/2, y=VIEWPORT_H/2, scale=.5)

    def _render_text(self):
        self.viewer.draw_text(self.msg_prompt, x=COMMAND_DISPLAY_POSITION[0], y=COMMAND_DISPLAY_POSITION[1], font_size=COMMAND_DISPLAY_FONTSIZE, color=COMMAND_TEXT_COLOR, bold=self.bold)

    def _render_timer_text(self):
        self.viewer.draw_text(str(self.current_time), x=COMMAND_DISPLAY_POSITION[0], y=COMMAND_DISPLAY_POSITION[1], font_size=TIMER_DISPLAY_FONTSIZE, color=TIMER_COLOR_NEUTRAL, anchor_y=TIMER_DISPLAY_TEXT_Y_ANCHOR, bold=True)

    def _render_timer(self, period):
        while not rospy.is_shutdown():
            start = rospy.get_rostime()
            self.lock.acquire()
            if self.start_timer:
                if self.current_time > 0: 
                    self.current_time -= 1
            else:
                pass
            self.lock.release()
            end = rospy.get_rostime()
            if end-start < period:
                rospy.sleep(period - (end-start))
            else:
                rospy.loginfo('took more time')

    def render(self):   
        self.viewer.window.clear()

        if self.img_prompt != '': 
            self._render_sprite()
        if self.display_timer: 
            self._render_timer_text()

        self._render_text()

        return self.viewer.render(False)

    def training_refresher(self): 
        self.img_prompt = self.training_prompts[self.msg_ind]
        self.publish_action(self.img_prompt)
        self._set_image_path()

    def step(self): 

        if self.start_prompt: 
            if self.display_start_countdown: 
                self.display_timer = False 
                if self.start_msg_ind < len(EXPERIMENT_START_COUNTDOWN):
                    if self.ready_for_new_prompt: 
                        self.msg_prompt = EXPERIMENT_START_COUNTDOWN[self.start_msg_ind]
                        self.start_msg_ind += 1
                        self.ts = time.time()
                        self.ready_for_new_prompt = False 
                    if (time.time() - self.ts) >= self.text_timing_bound: 
                        self.ready_for_new_prompt = True 
                else: 
                    self.msg_prompt = ''
                    self.display_start_countdown = False
                    self.clear_for_next_prompt = False
                    self.ready_for_new_prompt = True
                    self.display_timer = True 

            else: 

                if self.ready_for_new_prompt: 
                    self.img_prompt = self.action_prompts[self.prompt_ind]
                    self.publish_action(self.img_prompt)
                    self._set_image_path()
                    self.ready_for_new_prompt = False
                    self.ts = time.time()
                    self.current_time = self.action_timing_bound

                if (time.time() - self.ts) >= self.action_timing_bound: 
                    print 'clear'
                    self.clear_for_next_prompt = True

                if self.clear_for_next_prompt: 
                    self.prompt_ind += 1
                    self.ready_for_new_prompt = True
                    self.clear_for_next_prompt = False 
                    if self.prompt_ind >= len(self.action_prompts): 
                        self.img_prompt = ''
                        self.start_prompt = False                     
                        if int(self.current_block) < int(self.blocks)-1: 
                            self.start_training = True 
                        else: 
                            self.msg_prompt = 'End of Test'

        if self.start_training: 
            self.msg_prompt = ''
            self.display_timer = False 
            self.start_timer = False 
            self.training_refresher() 


    def reset(self): 
        if 'action_prompts' in self.env_params.keys(): 
            self.action_prompts = self.env_params['action_prompts']

        if 'start_prompt' in self.env_params.keys(): 
            if self.start_prompt == False: 
                self.start_prompt = self.env_params['start_prompt']
                self.display_start_countdown = True 
                self.ready_for_new_prompt = True
                self.clear_for_next_prompt = False
                self.start_training = False 
                self.prompt_ind = 0
                self.start_msg_ind = 0
                self.start_timer = True
                random.shuffle(self.action_prompts)

        if 'training_prompts' in self.env_params.keys(): 
            self.training_prompts = self.env_params['training_prompts']
            self.msg_ind = 0 

        if 'blocks' in self.env_params.keys(): 
            self.blocks = self.env_params['blocks']

    def _get_user_input(self): 

        if 'next_prompt' in self.env_params.keys(): 
            self.clear_for_next_prompt = self.env_params['next_prompt']
            self.env_params['next_prompt'] = False # reset 

        if 'next' in self.env_params.keys(): 
            if self.start_training: 
                self.next = self.env_params['next']
                if self.next:                 
                    if self.msg_ind < len(self.training_prompts)-1: 
                        self.msg_ind += 1
                    
                    elif self.msg_ind == len(self.training_prompts)-1: 
                        self.env_params['start_prompt'] = True
                        self.current_block += 1 
                        self.start_training = False 
                        self.img_prompt = ''
                        self.msg_prompt = ''
                        self.reset()

                    self.env_params['next'] = False 

        if 'back' in self.env_params.keys(): 
            if self.start_training: 
                self.back = self.env_params['back']
                if self.back: 
                    if self.msg_ind > 0 : 
                        self.msg_ind -= 1 
                    self.env_params['back'] = False 
    


if __name__ == '__main__':
    ActionEnv()

