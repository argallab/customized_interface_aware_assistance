#!/usr/bin/env python

import pyglet
from pyglet.window import mouse

window = pyglet.window.Window()
label = pyglet.text.Label('Hello, world',
                          font_name='Times New Roman',
                          font_size=36,
                          x=window.width//2, y=window.height//2,
                          anchor_x='center', anchor_y='center', color=(0, 123, 255, 255))

@window.event
def on_draw():
    # window.clear()
    label.draw()

# @window.event
# def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
#     pyglet.graphics.draw(4, pyglet.gl.GL_QUADS, ('v2f', [x, y, x-dx, y, x-dx, y-dy, x, y-dy]))
#     # print x, y, dx, y, dx, dy, x, dy

pyglet.app.run()
