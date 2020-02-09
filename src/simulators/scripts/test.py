#!/usr/bin/env python

import pyglet
import sys
from backends.rendering import Viewer, SimpleImageViewer
from utils import SCALE, VIEWPORT_W, VIEWPORT_H


class TextWindowEnv(pyglet.window.Window):

    # def __init__(self, env_params, *args,**kwargs):
    #     pyglet.window.Window.__init__(self,*args,**kwargs)

    def __init__(self, env_params):
        self.viewer = None
        self.SimpleImageViewer


        self.env_params = env_params
        assert self.env_params is not None

        assert 'text' in self.env_params

        self.x = self.width//2
        self.y = self.height//2 
        self.text = None 
        self.font_name = 'Arial'
        self.font_size = 36
        self.anchor_x = 'center'
        self.anchor_y = 'center'
        self.color = (250, 100, 100, 250)
        self.bold = False

    def render_text(self):
        self.label = pyglet.text.Label(self.text,
                                    font_name=self.font_name,
                                    font_size=self.font_size,
                                    x=self.x,
                                    y=self.y,
                                    anchor_x=self.anchor_x, 
                                    anchor_y=self.anchor_y,
                                    color=self.color,
                                    bold=self.bold)        

    # def render(self): 
    #     if self.viewer is None: 
    #         self.viewer = Viewer(VIEWPORT_W, VIEWPORT_H)
    #         self.viewer.set_bounds(0, VIEWPORT_W/SCALE, 0, VIEWPORT_H/SCALE)
    #         self.viewer.window.set_location(650, 300)

    #     self.render_text()

    #     return self.viewer.render(False)


    def on_draw(self):
        if self.viewer is None: 
            self.viewer = Viewer(VIEWPORT_W, VIEWPORT_H)
            self.viewer.set_bounds(0, VIEWPORT_W/SCALE, 0, VIEWPORT_H/SCALE)
            self.viewer.window.set_location(650, 300)
        # print('The window was drawn!')
        # print('We are also going to draw some text, just because we can.')
        # sys.stdout.flush()        
        self.render_text()
        self.label.draw()
    
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

class Hud(object):

    def __init__(self, win):
        helv = font.load('Helvetica', win.width / 15.0)
        self.text = font.Text(
            helv,
            'Hello, World!',
            x=win.width / 2,
            y=win.height / 2,
            halign=font.Text.CENTER,
            valign=font.Text.CENTER,
            color=(1, 1, 1, 0.5),
        )
        self.fps = clock.ClockDisplay()

    def draw(self):
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        self.text.draw()
        self.fps.draw()