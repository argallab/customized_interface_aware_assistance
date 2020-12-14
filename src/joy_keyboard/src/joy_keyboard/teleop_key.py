#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
import sys, select, termios, tty
import signal
from IPython import embed
moveBindings = {
		'u':(1,0,0),
		'o':(-1,0,0),
		'j':(0,1,0),
		'l':(0,-1,0),
        'm':(0,0,1),
		'.':(0,0,-1),
			}

modeBindings = {
		'1':(0),
		'2':(1),
		'3':(2),
		}

class KeyTeleop(object):
	"""docstring for KeyTeleop"""
	def __init__(self, frame_id):
		rospy.init_node('teleop_key')
		signal.signal(signal.SIGINT, self.shutdown)
		self.joy_msg = Joy()
		self.frame_id = frame_id
		self.joy_pub = rospy.Publisher('joy_keyboard', Joy, queue_size=1)
		self.settings = termios.tcgetattr(sys.stdin)
		while not rospy.is_shutdown():
			key = self.getKey()
			self.joy_msg.header.stamp = rospy.Time.now()
			self.joy_msg.header.frame_id = self.frame_id
			# self.joy_msg.axes = [0,0,0]
			# self.joy_msg.buttons = [0]*3
			if key in moveBindings.keys():
				self.joy_msg.axes = list(moveBindings[key])
			elif key in modeBindings.keys():
				# embed()
				self.joy_msg.buttons[modeBindings[key]] = 1
			else:
				self.joy_msg.axes = [0,0,0]
				self.joy_msg.buttons = [0]*3
			self.joy_pub.publish(self.joy_msg)


	def getKey(self):
		tty.setraw(sys.stdin.fileno())
		rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
		if rlist:
			key = sys.stdin.read(1)
		else:
			key = ''
	
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
		return key

	def shutdown(self, signum, frame):
		rospy.loginfo("At Last Gracefully Exiting...")
		rospy.sleep(3.0)


if __name__ == '__main__':
	kt = KeyTeleop('keyboard')