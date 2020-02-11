#!/usr/bin/env python

from backends.rendering import Viewer, Transform
from utils import SCALE, VIEWPORT_W, VIEWPORT_H
from utils import COMMAND_TEXT_COLOR, COMMAND_DISPLAY_POSITION, COMMAND_DISPLAY_FONTSIZE 
from utils import LOW_LEVEL_COMMANDS, MODE_DISPLAY_RADIUS, MODE_DISPLAY_TEXT_FONTSIZE, MODE_DISPLAY_TEXT_COLOR, MODE_DISPLAY_TEXT_X_OFFSET, MODE_DISPLAY_TEXT_START_POSITION
from utils import MODE_DISPLAY_CIRCLE_START_POSITION_S, MODE_DISPLAY_CIRCLE_X_OFFSET_S, ACTIVE_MODE_COLOR, NONACTIVE_MODE_COLOR, MODE_DISPLAY_TEXT_Y_ANCHOR

import pyglet

class SipPuffCallibrationEnv(object):

    def __init__(self, env_params):

        self.viewer = None

        self.env_params = env_params
        assert self.env_params is not None

        assert 'command' in self.env_params

        self.env_params['command'] = ''
 
        self.text = '' 
        self.bold = True

        self.DIMENSIONS = LOW_LEVEL_COMMANDS


    def _render_command_display(self):
        for i, d in enumerate(self.DIMENSIONS):
            t = Transform(translation=(MODE_DISPLAY_CIRCLE_START_POSITION_S[0]+VIEWPORT_W/2 + i*MODE_DISPLAY_CIRCLE_X_OFFSET_S, MODE_DISPLAY_CIRCLE_START_POSITION_S[1]-VIEWPORT_H/2))
            if d == self.current_command:
                self.viewer.draw_circle(MODE_DISPLAY_RADIUS/SCALE, 30, True, color=ACTIVE_MODE_COLOR).add_attr(t)
            else:
                self.viewer.draw_circle(MODE_DISPLAY_RADIUS/SCALE, 30, True, color=NONACTIVE_MODE_COLOR).add_attr(t)

    def _render_command_display_text(self):
        for i, d in enumerate(self.DIMENSIONS): 
            self.viewer.draw_text(d, x=MODE_DISPLAY_TEXT_START_POSITION[0]+i*MODE_DISPLAY_TEXT_X_OFFSET, y=MODE_DISPLAY_TEXT_START_POSITION[1], font_size=MODE_DISPLAY_TEXT_FONTSIZE, color=MODE_DISPLAY_TEXT_COLOR, anchor_y=MODE_DISPLAY_TEXT_Y_ANCHOR)

    def render(self):
        if self.viewer is None:
            self.viewer = Viewer(VIEWPORT_W, VIEWPORT_H)
            self.viewer.set_bounds(0, VIEWPORT_W/SCALE, 0, VIEWPORT_H/SCALE)
            self.viewer.window.set_location(650, 300)

        self._render_command_display()
        self._render_command_display_text()

    def reset(self): 
        self.current_command = self.env_params['command']
