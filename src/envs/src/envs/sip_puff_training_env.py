#!/usr/bin/env python

from backends.rendering import Viewer, Transform
from utils import SCALE, VIEWPORT_W, VIEWPORT_H, VIEWPORT_WS, VIEWPORT_HS, ROBOT_RADIUS
from utils import COMMAND_TEXT_COLOR, COMMAND_DISPLAY_POSITION, COMMAND_DISPLAY_FONTSIZE 
from utils import LOW_LEVEL_COMMANDS, MODE_DISPLAY_RADIUS, MODE_DISPLAY_TEXT_FONTSIZE, MODE_DISPLAY_TEXT_COLOR
from utils import ACTIVE_MODE_COLOR, NONACTIVE_MODE_COLOR, ACTIVE_MODE_COLOR_ERROR
from utils import MODE_DISPLAY_TEXT_Y_ANCHOR

import pyglet

class SipPuffTrainingEnv(object):

    def __init__(self, env_params):

        self.viewer = None

        self.env_params = env_params
        assert self.env_params is not None

        assert 'command' in self.env_params

        self.env_params['command'] = ''
        self.env_params['active_color'] = ACTIVE_MODE_COLOR

        self.prompt = ''
        self.start_prompt = False 
        self.correct_count_threshold = 30
 
        self.current_command = '' 
        self.bold = True

        self.DIMENSIONS = LOW_LEVEL_COMMANDS

    def _render_command_display(self):
        for i, d in enumerate(self.DIMENSIONS):
            t = Transform(translation=(VIEWPORT_WS/5 + i*VIEWPORT_WS/5, VIEWPORT_HS/2))            
            if d == self.current_command:
                self.viewer.draw_circle(MODE_DISPLAY_RADIUS/SCALE, 30, True, color=self.env_params['active_color']).add_attr(t)
            else:
                self.viewer.draw_circle(MODE_DISPLAY_RADIUS/SCALE, 30, True, color=NONACTIVE_MODE_COLOR).add_attr(t)

    def _render_command_display_text(self):
        for i, d in enumerate(self.DIMENSIONS): 
            self.viewer.draw_text(d, x=VIEWPORT_W/5+i*VIEWPORT_W/5, y=VIEWPORT_H/2-(2*ROBOT_RADIUS), 
                font_size=MODE_DISPLAY_TEXT_FONTSIZE, 
                color=MODE_DISPLAY_TEXT_COLOR, 
                anchor_y=MODE_DISPLAY_TEXT_Y_ANCHOR)

    def _render_command_text(self):
        self.viewer.draw_text(self.prompt, x=COMMAND_DISPLAY_POSITION[0], y=COMMAND_DISPLAY_POSITION[1]/2, font_size=MODE_DISPLAY_TEXT_FONTSIZE, color=COMMAND_TEXT_COLOR, bold=self.bold)

    def render(self):
        if self.viewer is None:
            self.viewer = Viewer(VIEWPORT_W, VIEWPORT_H)
            self.viewer.set_bounds(0, VIEWPORT_W/SCALE, 0, VIEWPORT_H/SCALE)
            self.viewer.window.set_location(650, 300)

        self._render_command_display()
        self._render_command_display_text()
        self._render_command_text()

        return self.viewer.render(False)

    def step(self, input): 
        self.current_command = self.env_params['command']

        if self.start_prompt:             
            if self.ready_for_new_prompt: 
                self.correct_count = 0
                self.prompt = self.prompt_commands[0]
                self.ready_for_new_prompt = False
                self.env_params['active_color'] = ACTIVE_MODE_COLOR_ERROR
            if self.current_command == self.prompt: 
                self.correct_count += 1
                self.env_params['active_color'] = ACTIVE_MODE_COLOR
                if self.correct_count == self.correct_count_threshold: 
                    self.prompt_commands.pop(0)
                    self.ready_for_new_prompt = True
                    if self.prompt_commands == []: 
                        self.start_prompt = False
                        self.prompt = 'End of Prompted Training'


    def reset(self):        
        if 'start_prompt' in self.env_params.keys(): 
            self.start_prompt = self.env_params['start_prompt']
            self.ready_for_new_prompt = True
        if 'prompt_commands' in self.env_params.keys(): 
            self.prompt_commands = self.env_params['prompt_commands']