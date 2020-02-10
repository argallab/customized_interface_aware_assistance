#!/usr/bin/env python
 
# '''Test that the clock returns a reasonable average FPS calculation when
# stimulated at 5 Hz.
# '''
 
# __docformat__ = 'restructuredtext'
# __version__ = '$Id: FPS.py 2529 2009-10-12 17:39:44Z benjamin.coder.smith@gmail.com $'
 
# import time
# import unittest
 
# from pyglet import clock
 
# __noninteractive = True
 
# class FPS(unittest.TestCase):
#     def test_fps(self):
#         clock.set_default(clock.Clock())
#         self.assertTrue(clock.get_fps() == 0)
#         clock.tick()
#         for i in range(10):
#             time.sleep(0.2)
#             clock.tick()
#         result = clock.get_fps()
#         self.assertTrue(abs(result - 5.0) < 0.05)
 
# if __name__ == '__main__':
#     unittest.main()


'''Test that a scheduled function gets called every tick with the correct
time delta.
'''
 
# __docformat__ = 'restructuredtext'
# __version__ = '$Id: TICK.py 310 2006-12-23 15:56:35Z Alex.Holkner $'
 
import time
import unittest
 
from pyglet import clock
 
__noninteractive = True
 
class SCHEDULE(unittest.TestCase):
    callback_dt = None
    callback_count = 0
    def callback(self, dt):
        self.callback_dt = dt
        self.callback_count += 1
 
    def test_schedule(self):
        clock.set_default(clock.Clock())
        clock.schedule(self.callback)
 
        result = clock.tick()
        self.assertTrue(result == self.callback_dt)
        self.callback_dt = None
        time.sleep(1)
 
        result = clock.tick()
        self.assertTrue(result == self.callback_dt)
        self.callback_dt = None
        time.sleep(1)
 
        result = clock.tick()
        self.assertTrue(result == self.callback_dt)
 
    def test_unschedule(self):
        clock.set_default(clock.Clock())
        clock.schedule(self.callback)
 
        result = clock.tick()
        self.assertTrue(result == self.callback_dt)
        self.callback_dt = None
        time.sleep(1)
        clock.unschedule(self.callback)
 
        result = clock.tick()
        self.assertTrue(self.callback_dt == None)
 
    def test_schedule_multiple(self):
        clock.set_default(clock.Clock())
        clock.schedule(self.callback)
        clock.schedule(self.callback)
        self.callback_count = 0
 
        clock.tick()
        self.assertTrue(self.callback_count == 2)
 
        clock.tick()
        self.assertTrue(self.callback_count == 4)
 
    def test_schedule_multiple(self):
        clock.set_default(clock.Clock())
        clock.schedule(self.callback)
        clock.schedule(self.callback)
        self.callback_count = 0
 
        clock.tick()
        self.assertTrue(self.callback_count == 2)
        clock.unschedule(self.callback)
 
        clock.tick()
        self.assertTrue(self.callback_count == 2)
 
if __name__ == '__main__':
    unittest.main()