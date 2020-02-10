#!/usr/bin/env python

from backends.rendering import Viewer
from utils import SCALE, VIEWPORT_W, VIEWPORT_H
from utils import COMMAND_TEXT_COLOR, COMMAND_DISPLAY_POSITION, COMMAND_DISPLAY_FONTSIZE 
from utils import TIMER_DISPLAY_POSITION, TIMER_DISPLAY_FONTSIZE, TIMER_COLOR_NEUTRAL

import pyglet
import time 
from pyglet import clock

class TextWindowEnv(object):

    def __init__(self, env_params):
        self.viewer = None

        self.env_params = env_params
        assert self.env_params is not None

        assert 'text' in self.env_params
 
        self.text = '' 
        self.bold = True

        # self.timer = Timer()
        self.timer_sec = 0
        self.timer_msec = 60


    def _render_text(self):
        self.viewer.draw_text(self.text, x=COMMAND_DISPLAY_POSITION[0], y=COMMAND_DISPLAY_POSITION[1], font_size=COMMAND_DISPLAY_FONTSIZE, color=COMMAND_TEXT_COLOR, bold=self.bold)

    def render(self):
        if self.viewer is None:
            self.viewer = Viewer(VIEWPORT_W, VIEWPORT_H)
            self.viewer.set_bounds(0, VIEWPORT_W/SCALE, 0, VIEWPORT_H/SCALE)
            self.viewer.window.set_location(650, 300)

        self._render_text()
        self.start_counter()
        return self.viewer.render(False)

    def reset(self): 
        # (mahdieh to do) Definitely better ways of doing this...
        if 'x' in self.env_params.keys(): 
            self.x = self.env_params['x']
        if 'y' in self.env_params.keys(): 
            self.y = self.env_params['y']
        if 'text' in self.env_params.keys(): 
            self.text = self.env_params['text'] 
        if 'font_name' in self.env_params.keys(): 
            self.font_name = self.env_params['font_name']
        if 'font_size' in self.env_params.keys(): 
            self.font_size = self.env_params['font_size']
        if 'anchor_x' in self.env_params.keys(): 
            self.anchor_x = self.env_params['anchor_x']
        if 'anchor_y' in self.env_params.keys(): 
            self.anchor_y = self.env_params['anchor_y']
        if 'color' in self.env_params.keys(): 
            self.color = self.env_params['color']
        if 'bold' in self.env_params.keys(): 
            self.bold = self.env_params['bold']

# class Timer(object): 
#     def __init__(self):
        
        # self.timer_sec = 0
        # self.timer_msec = 60

    def start_counter(self): 
        while timer != self.timer_sec: 
            self.timer_text = str(self.timer_sec)+':'+str(self.timer_msec)
            self.viewer.draw_text(self.timer_text, x=TIMER_DISPLAY_POSITION[0], y=TIMER_DISPLAY_POSITION[1], font_size=TIMER_DISPLAY_FONTSIZE, color=TIMER_COLOR_NEUTRAL, bold=True)
            time.sleep(1)
            timer += 1
            self.viewer.render(False)

    def start_countdown(self): 
        while self.timer_msec != 0: 
            self.timer_text = str(self.timer_sec)+':'+str(self.timer_msec)
            self.viewer.draw_text(self.timer_text, x=TIMER_DISPLAY_POSITION[0], y=TIMER_DISPLAY_POSITION[1], font_size=TIMER_DISPLAY_FONTSIZE, color=TIMER_COLOR_NEUTRAL, bold=True)
            time.sleep(1)
            self.timer_msec -= 1
            self.viewer.render(False)