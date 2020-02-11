#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
import time
import collections
from dynamic_reconfigure.server import Server
from teleop_nodes.srv import InferCorrect, InferCorrectRequest, InferCorrectResponse
npa = np.array

'''Reads raw sip_and_puff joy data, does thresholding and buffering
 and returns data as hard puff, soft puff, soft sip, hard sip.
 Input and output are both /sensor_msgs/joy'''

class SNPMapping(object):

  ''' data is a Float32Array message '''
  def __init__(self):

    # Initialize
    rospy.init_node('sip_puff_mapping', anonymous=True)
    # Sip/Puff Control Thresholds
    # sip values are positive, puff values are negative in the axes
    # Note that for puff the actual values are negative. The "lower" and "higher" make sense only if absolute values are considered
    # creating limits prevents unintended soft sips and puffs, and creates deadzone between upper and latch limit
    self.lower_sip_limit = rospy.get_param("/sip_and_puff_thresholds/lower_sip_limit")
    self.lower_puff_limit = rospy.get_param("/sip_and_puff_thresholds/lower_puff_limit")
    self.soft_sip_max_limit = rospy.get_param("/sip_and_puff_thresholds/soft_sip_max_limit")
    self.soft_puff_max_limit = rospy.get_param("/sip_and_puff_thresholds/soft_puff_max_limit")
    self.hard_sip_min_limit = rospy.get_param("/sip_and_puff_thresholds/hard_sip_min_limit")
    self.hard_puff_min_limit = rospy.get_param("/sip_and_puff_thresholds/hard_puff_min_limit")
    self.hard_puff_max_limit = rospy.get_param("/sip_and_puff_thresholds/hard_puff_max_limit")
    self.hard_sip_max_limit = rospy.get_param("/sip_and_puff_thresholds/hard_sip_max_limit")

    # latch limit governs the threshold for direction and main mode switches
    self._lock_input = False
    self._ignore_input_counter = 0
    self._num_inputs_to_ignore = 10
    self._button_latch_time = 0.8
    self._old_time = rospy.get_time()
    self.old_msg = Joy()
    self.old_msg.axes = np.zeros(1)
    self.command_to_button_index_map = collections.OrderedDict({'Hard Puff':0, 'Soft Puff': 1, 'Soft Sip': 2, 'Hard Sip': 3})

    # Initialize publisher and subscribers
    rospy.Subscriber('/joy', Joy, self.joy_callback)
    self.pub = rospy.Publisher('joy_sip_puff', Joy, queue_size = 1)

    # Published velocity message
    self.send_msg = Joy()
    self.send_msg.header.stamp = rospy.Time.now()
    self.send_msg.header.frame_id = "Zero Band"
    self.send_msg.axes = np.zeros(1)  # pressure ([-1, 1])
    self.send_msg.buttons = np.zeros(4) # hard puff, soft puff, soft sip, hard sip

    self.is_assistance = rospy.get_param('is_assistance', True)
    rospy.loginfo("Waiting for mode_inference_and_correction node ")
    rospy.wait_for_service("/mode_switch_inference_and_correction/handle_unintended_commands")
    rospy.loginfo("Found mode_inference_and_correction_node")

    self.infer_and_correct_service = rospy.ServiceProxy('/mode_switch_inference_and_correction/handle_unintended_commands', InferCorrect)


  # #######################################################################################
  #                           Check Sip and Puff Limits                                   #
  #######################################################################################
  # checks whether within limits, otherwise air velocity in dead zone (essentailly zero)
  # written this way to make debugging easier if needed
  # labels hard and soft sips and puffs, buffers out
  def checkLimits(self, airVelocity):
    if (self.lower_puff_limit < airVelocity < self.lower_sip_limit):
      self.send_msg.header.frame_id = "Zero Band"
      self.send_msg.buttons = np.zeros(4)
    elif (self.lower_sip_limit < airVelocity < self.soft_sip_max_limit): # register as soft sip
      self.send_msg.header.frame_id = "Soft Sip"
      self.send_msg.buttons[2] = 1
    elif (self.soft_puff_max_limit < airVelocity < self.lower_puff_limit): # register as soft puff
      self.send_msg.header.frame_id = "Soft Puff"
      self.send_msg.buttons[1] = 1
    elif (self.hard_puff_max_limit < airVelocity < self.hard_puff_min_limit): # register as hard puff
      self.send_msg.header.frame_id = "Hard Puff"
      self.send_msg.buttons[0] = 1
      self._lock_input = True
    elif (self.hard_sip_min_limit < airVelocity < self.hard_sip_max_limit): # register as hard sip
      self.send_msg.header.frame_id = "Hard Sip"
      self.send_msg.buttons[3] = 1
      self._lock_input = True
    else:
      self.send_msg.header.frame_id = "Soft-Hard Deadband"
      self.send_msg.buttons = np.zeros(4)
    print("      ")
    rospy.loginfo("Before")
    rospy.loginfo(self.send_msg.header.frame_id)

    if self.is_assistance and self.send_msg.header.frame_id != "Zero Band" and self.send_msg.header.frame_id != "Soft-Hard Deadband" and self.send_msg.header.frame_id != "Input Stopped":
      request = InferCorrectRequest()
      request.um = self.send_msg.header.frame_id
      response = self.infer_and_correct_service(request)
      self.send_msg.header.frame_id = response.u_corrected
      self.send_msg.buttons = np.zeros(4)
      if self.send_msg.header.frame_id != "Zero Band":
        self.send_msg.buttons[self.command_to_button_index_map[self.send_msg.header.frame_id]] = 1
        if self.send_msg.header.frame_id == "Hard Puff" or self.send_msg.header.frame_id == "Hard Sip":
          self._lock_input = True

      rospy.loginfo("After")
      rospy.loginfo(self.send_msg.header.frame_id)

    return 0

  #######################################################################################
  #                                Raw Joy Callback                                     #
  #######################################################################################
  # recieves raw input, checks for buildup and

  def joy_callback(self, msg):
    # prevent robot arm moving after done blowing, zero out velocities
    if msg.buttons[0] is 0 and msg.buttons[1] is 0: # the last input in each blow is 0 for buttons
      self._ignore_input_counter = 0 # the constraints get
      self.send_msg.header.frame_id = "Input Stopped"
      self.send_msg.buttons = np.zeros(4)
      if self._lock_input is True:
        self._lock_input = False

    # Ignore the leadup to powerful blow that leads to mode switch (ONLY FOR SIP-PUFF SYSTEM, otherwise delete)
    if self._ignore_input_counter < self._num_inputs_to_ignore: # seems like thread issue if the number to ignore is too high
        self._ignore_input_counter +=1

    if self._lock_input is False:
      self.send_msg.header.stamp = rospy.Time.now()
      self.send_msg.axes[0] = msg.axes[1]
    #  # If ramp up to hard puff or sip (i.e. steep angle)
    #   if abs((msg.axes[0] - self.old_msg.axes[0])/(msg.header.stamp - self.old_msg.header.stamp).to_sec()) >= 5 and (msg.header.stamp - self.old_msg.header.stamp).to_sec()<0.002:
    #     self.send_msg.header.frame_id = "soft-hard ramp"
    #     self.send_msg.buttons = np.zeros(4)
      # else:
      self.checkLimits(msg.axes[1])
      self.pub.publish(self.send_msg)

    self.old_msg = msg

if __name__ == '__main__':

  SNPMapping()
  rospy.spin()
