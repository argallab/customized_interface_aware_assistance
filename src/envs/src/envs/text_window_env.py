#!/usr/bin/env python
from backends.rendering import Viewer
from utils import SCALE, VIEWPORT_W, VIEWPORT_H
from utils import COMMAND_TEXT_COLOR, COMMAND_DISPLAY_POSITION, COMMAND_DISPLAY_FONTSIZE 

import pyglet

class TextWindowEnv(object):

    def __init__(self, env_params):
        self.viewer = None

        self.env_params = env_params
        assert self.env_params is not None

        assert 'text' in self.env_params
 
#         self.x = self.width//2
#         self.y = self.height//2 
        self.text = None 
#         self.font_name = 'Arial'
#         self.font_size = 36
#         self.anchor_x = 'center'
#         self.anchor_y = 'center'
#         self.color = (250, 100, 100, 250)
        self.bold = True


    def _render_text(self):
        '''
        Note that the coordinates of the text should in real pixels. Which is why here there is a multiplicative factor of SCALE.
        '''
        self.viewer.draw_text(self.text, x=COMMAND_DISPLAY_POSITION[0], y=COMMAND_DISPLAY_POSITION[1], font_size=COMMAND_DISPLAY_FONTSIZE, color=COMMAND_TEXT_COLOR, bold=self.bold)

    def render(self):
        if self.viewer is None:
            self.viewer = Viewer(VIEWPORT_W, VIEWPORT_H)
            self.viewer.set_bounds(0, VIEWPORT_W/SCALE, 0, VIEWPORT_H/SCALE)
            self.viewer.window.set_location(650, 300)

        self._render_text()

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



# #!/usr/bin/env python

# import pyglet
# import sys
# from backends.rendering import Viewer, SimpleImageViewer
# from utils import SCALE, VIEWPORT_W, VIEWPORT_H


# class TextWindowEnv(pyglet.window.Window):

#     def __init__(self, env_params, *args,**kwargs):
#         pyglet.window.Window.__init__(self,*args,**kwargs)


#         self.env_params = env_params
#         assert self.env_params is not None

#         assert 'text' in self.env_params

#         self.x = self.width//2
#         self.y = self.height//2 
#         self.text = None 
#         self.font_name = 'Arial'
#         self.font_size = 36
#         self.anchor_x = 'center'
#         self.anchor_y = 'center'
#         self.color = (250, 100, 100, 250)
#         self.bold = False

#     def render_text(self):
#         self.label = pyglet.text.Label(self.text,
#                                     font_name=self.font_name,
#                                     font_size=self.font_size,
#                                     x=self.x,
#                                     y=self.y,
#                                     anchor_x=self.anchor_x, 
#                                     anchor_y=self.anchor_y,
#                                     color=self.color,
#                                     bold=self.bold)        

#     # def render(self): 
#     #     if self.viewer is None: 
#     #         self.viewer = Viewer(VIEWPORT_W, VIEWPORT_H)
#     #         self.viewer.set_bounds(0, VIEWPORT_W/SCALE, 0, VIEWPORT_H/SCALE)
#     #         self.viewer.window.set_location(650, 300)

#     #     self.render_text()

#     #     return self.viewer.render(False)


#     def on_draw(self):
#         # if self.viewer is None: 
#         #     self.viewer = Viewer(VIEWPORT_W, VIEWPORT_H)
#         #     self.viewer.set_bounds(0, VIEWPORT_W/SCALE, 0, VIEWPORT_H/SCALE)
#         #     self.viewer.window.set_location(650, 300)
#         # print('The window was drawn!')
#         # print('We are also going to draw some text, just because we can.')
#         # sys.stdout.flush()        
#         self.render_text()
#         self.label.draw()
    
#     def reset(self): 
#         # (mahdieh to do) Definitely better ways of doing this...
#         if 'x' in self.env_params.keys(): 
#             self.x = self.env_params['x']
#         if 'y' in self.env_params.keys(): 
#             self.y = self.env_params['y']
#         if 'text' in self.env_params.keys(): 
#             self.text = self.env_params['text'] 
#         if 'font_name' in self.env_params.keys(): 
#             self.font_name = self.env_params['font_name']
#         if 'font_size' in self.env_params.keys(): 
#             self.font_size = self.env_params['font_size']
#         if 'anchor_x' in self.env_params.keys(): 
#             self.anchor_x = self.env_params['anchor_x']
#         if 'anchor_y' in self.env_params.keys(): 
#             self.anchor_y = self.env_params['anchor_y']
#         if 'color' in self.env_params.keys(): 
#             self.color = self.env_params['color']
#         if 'bold' in self.env_params.keys(): 
#             self.bold = self.env_params['bold']

