#!/usr/bin/env python
import rospy
import sys
import numpy as np
from control_input import ControlInput
from sensor_msgs.msg import Joy
from std_msgs.msg import MultiArrayDimension
from teleop_nodes.msg import CartVelCmd
from teleop_nodes.msg import ModeSwitch
from std_msgs.msg import Int16
from std_srvs.srv import SetBool
from teleop_nodes.cfg import SipPuffModeSwitchParadigmConfig
from teleop_nodes.srv import SetMode, SetModeRequest, SetModeResponse
npa = np.array

class KeyboardTeleopInput(ControlInput):
  '''data is a Cartvel message'''
  def __init__(self, robot_dim=6, finger_dim=3):

    #Initialize
    ControlInput.__init__(self, robot_dim, finger_dim)

    self.initialize_subscribers()
    self.initialize_publishers()
    self.initialize_services()

    self.modeswitch_msg = ModeSwitch()
    self.mode_msg = Int16()
    self._mode = 0
    self._mode_switch_count = 0
    self._lock_input = True # to prevent constant mode switching
    self.mode_switch_paradigm = 2 #two-way mode switching
    self.interface_dim = 1
    self._mode_key = self.get_mode_switch_paradigm(self.interface_dim)

    # Set up velocity command and load velocity limits to be sent to hand nodes
    if self.finger_dim > 0:
        self.effective_finger_dim = 1 #a single dimension's velocity is mapped to ALL the fingers of the robot
    else:
        self.effective_finger_dim = 0
    
    self.gripper_vel_limit = rospy.get_param("/jaco_velocity_limits/gripper_vel_limit")
    self._vel_multiplier = np.ones(self.robot_dim + self.finger_dim)*1
    ## TODO (deepak). Get different multipler for rotation and assigned indices for rotation dimensions:
    self._vel_multiplier[2] = 0.4*self._vel_multiplier[2]
    self._max_cart_vel = np.ones(self.robot_dim + self.finger_dim)*rospy.get_param("/jaco_velocity_limits/max_cart_vel_mulitplier")
    if self.finger_dim > 0:
        for j in range(finger_dim):
            self._max_cart_vel[self.robot_dim + j] = self.gripper_vel_limit
    if not len(self._max_cart_vel) == self.robot_dim + self.finger_dim:
        rospy.logerr('Length of max_cart_vel does not equal number of joints!')
    self._cart_vel = np.zeros(self.robot_dim + self.finger_dim)

    # initialize output velocity message
    self.send_msg = CartVelCmd()
    _dim = [MultiArrayDimension()]
    _dim[0].label = 'cartesian_velocity'
    _dim[0].size = self.robot_dim + self.finger_dim
    _dim[0].stride = self.robot_dim + self.finger_dim
    self.send_msg.velocity.layout.dim = _dim
    self.send_msg.velocity.data = np.zeros_like(self._cart_vel)
    self.send_msg.header.stamp = rospy.Time.now()
    self.send_msg.header.frame_id = 'kb'

    self.waiting_for_release = False
    self.lock.acquire()

    self.publish_mode()

  # handles threading for parallel programming used in receive function
  def handle_threading(self):
    self.lock.acquire()
    try:
      self.data = self.send_msg
    finally:
      self.lock.release()

  def initialize_subscribers(self):
    rospy.Subscriber('joy_keyboard', Joy, self.receive)

  def initialize_publishers(self):
    self.startSend('/user_vel')
    self.mode_switch_pub = rospy.Publisher('/mode_switches', ModeSwitch, queue_size=1)
    self.modepub = rospy.Publisher("/mi/current_mode", Int16, queue_size=1)

  def initialize_services(self):
    rospy.Service('/teleop_node/set_mode', SetMode, self.set_mode)
  
  def publish_mode(self):
    self.modepub.publish(self._mode+1) # +1 for arduino

  # publishes mode and updates parameter server
  def publish_modeswitch(self):
    self._mode_switch_count = self._mode_switch_count+1
    self.modeswitch_msg.header.stamp = rospy.Time.now()
    self.modeswitch_msg.mode = self._mode
    self.modeswitch_msg.num_switches = self._mode_switch_count
    self.mode_switch_pub.publish(self.modeswitch_msg)
    rospy.set_param('mode',self._mode) #update the param server so that the update function in the env can read the current mode. Needed to determine what dimensions need to be zeroed out
    print "Num of mode switches %d" % self._mode_switch_count

  def switchmode(self, msg):
    switch = False
    # Paradigm 1: One-way mode switching
    ####################################
    if self.mode_switch_paradigm == 1:
      if msg.buttons[0]:
        self._mode = (self._mode + 1) % self.robot_dim + self.effective_finger_dim
        switch = True

    # Paradigm 2: Two-way mode switching
    ####################################
    elif self.mode_switch_paradigm == 2:
      if msg.buttons[1]: #press 2 on the keyboard, shift mode forwards
        self._mode = (self._mode + 1) % self.robot_dim + self.effective_finger_dim
        switch = True
      elif msg.buttons[0]: #press 1 on the keybaord shift mode backwards
        self._mode = (self._mode - 1) % self.robot_dim + self.effective_finger_dim
        switch = True

    if switch:
      self._lock_input = True
      print "************MODE IS NOW ", self._mode, " *******************"
      self.modeswitch_msg.header.frame_id = 'user'
      self.publish_modeswitch()
      self.publish_mode()

  def set_mode(self, set_mode):
    self._mode = setmode.mode_index
    #add assert to check that the self._mode doesn't exceed the max allowed mode val
    print "Current mode is ", self._mode
    status = SetModeResponse()
    self.publish_mode()
    self.modeswitch_msg.header.frame_id = 'service'
    self.publish_modeswitch()
    status = True
    return status
      
  def zero_vel(self):
    for i in range(0,(len(self._cart_vel))):
        self._cart_vel[i] = 0

  def receive(self, msg):
    if not self.waiting_for_release:
      if any(msg.buttons):
        self.switchmode(msg)
      self.waiting_for_release = True
    else:
      if not msg.buttons[0] and not msg.buttons[1]:
        self.waiting_for_release = False
    
    self.zero_vel()
    self.handle_velocities(msg)
  
  def handle_velocities(self, msg):
    _axes = np.array(msg.axes)
    # Separate individual axes
    _modes = [int(x) for x in str(self._mode_key[str(self._mode)])]
    # based on mode, find what you want to put _cart_vel
    for i in range(len(_modes)):
        if _modes[i] == 1: # X axis
            self._cart_vel[0] = _axes[i]*self._max_cart_vel[0] * 0.5
        elif _modes[i] == 2: # Y axis, forward y direction is -y, that +1 from joystick should move the robot in -y direction
            self._cart_vel[1] = -_axes[i]*self._max_cart_vel[1] * 0.5
        elif _modes[i] == 3: # Z axis,
            self._cart_vel[2] = _axes[i]*self._max_cart_vel[2] * 0.5
        elif _modes[i] == 4: # Yaw
            self._cart_vel[4] = _axes[i]*self._max_cart_vel[4] * 0.4
        elif _modes[i] == 5: # Pitch
            self._cart_vel[3] = _axes[i]*self._max_cart_vel[3] * 0.4
        elif _modes[i] == 6: # Roll
            self._cart_vel[5] = -_axes[i]*self._max_cart_vel[5] * 0.4
        elif _modes[i] == 7: # gripper mode
          self._cart_vel[6] = _axes[0]*self._max_cart_vel[6]
          self._cart_vel[7] = _axes[0]*self._max_cart_vel[7]
          self._cart_vel[8] = _axes[0]*self._max_cart_vel[8]

    self.send_msg.velocity.data = self._cart_vel
    self.send_msg.header.stamp = rospy.Time.now()
    self.lock.acquire()
    try:
      self.data = self.send_msg
    finally:
      self.lock.release()
  
  def get_mode_switch_paradigm(self, interface_dim):
      switcher = {
        1: {"0":1, "1":2, "2":3, "3":4, "4":5, "5":6, "6":7},
        2: {"0":12, "1":13, "2":45, "3":6, "4":7},
        3: {"0":123, "1":456, "2":7},
      }
      return switcher.get(interface_dim, "nothing")
  

  def getDefaultData(self):
    # since this sends velocity, the default will be all zeros.
    self.lock.acquire()
    try:
      self.data = CartVelCmd() # np.zeros_like(self._cart_vel)
    finally:
      self.lock.release()


if __name__ == "__main__":
  rospy.init_node('keyboard_teleop_node', anonymous=True)
  robot_dim = int(sys.argv[1])
  finger_dim = int(sys.argv[2])
  kb = KeyboardTeleopInput(robot_dim=robot_dim, finger_dim=finger_dim) #this robot dim has to be a parameter. Need to be changed for the 4D robot
  rospy.spin()    