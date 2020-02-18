#!/usr/bin/env python

# import os
# import pyglet
# from IPython import embed


# batch = pyglet.graphics.Batch()


# file_dir = '/home/corrective_mode_switch_assistance/src/envs/sprites'
# file_path = os.path.abspath(os.path.join(file_dir,'clockwise.png'))

# image = pyglet.image.load(file_path)

# ball = pyglet.sprite.Sprite(image)

# cw = [] # no x= or y=
# cw.append(pyglet.sprite.Sprite(image, 50, 50, batch=batch))

# # file_path = os.path.abspath(os.path.join(file_dir,'down.png'))
# # image = pyglet.image.load(file_path)
# # cw.append(pyglet.sprite.Sprite(image, 0, 0, batch=batch))

# window = pyglet.window.Window()

# @window.event
# def on_draw():
#     # cw.draw()
#     batch.draw()
# pyglet.app.run()



# label = pyglet.text.Label('Hello, world',
#                           font_name='Times New Roman',
#                           font_size=36,
#                           x=window.width//2, y=window.height//2,
#                           anchor_x='center', anchor_y='center')
import os
from backends.rendering import Viewer
from utils import SCALE, VIEWPORT_W, VIEWPORT_H
from utils import COMMAND_TEXT_COLOR, COMMAND_DISPLAY_POSITION, COMMAND_DISPLAY_FONTSIZE 
from IPython import embed
import pyglet

class ActionEnv(object):

    def __init__(self):

        self.viewer = None

        file_dir = '/home/corrective_mode_switch_assistance/src/envs/sprites'
        self.file_path = os.path.abspath(os.path.join(file_dir,'clockwise.png'))
        self.render()

    def _render_sprite(self):
        self.viewer.draw_sprite(self.file_path, x=COMMAND_DISPLAY_POSITION[0], y=COMMAND_DISPLAY_POSITION[1])

    def render(self):

        if self.viewer is None:
            self.viewer = Viewer(VIEWPORT_W, VIEWPORT_H)
            self.viewer.set_bounds(0, VIEWPORT_W/SCALE, 0, VIEWPORT_H/SCALE)
            self.viewer.window.set_location(650, 300)
        self._render_sprite()
        return self.viewer.render(False)



if __name__ == '__main__':
    ActionEnv()

