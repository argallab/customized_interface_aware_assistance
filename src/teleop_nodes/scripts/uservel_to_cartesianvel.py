#!/usr/bin/env python
import rospy
import numpy as np
from teleop_nodes.msg import CartVelCmd
from std_msgs.msg import MultiArrayDimension

class UserToControlInput():
    def __init__(self):

        self.robot_type = rospy.get_param('robot_type')
        self.is_joint_velocity = False
        if rospy.has_param('is_joint'):
            self.is_joint_velocity = rospy.get_param('is_joint')

        if self.is_joint_velocity:
            self.control_input_pub = rospy.Publisher('/'+self.robot_type+'_driver/in/joint_velocity_finger', CartVelCmd, queue_size = 1)
            self.command_length = int(self.robot_type[3])
            label = 'joint_velocity'
        else:
            self.control_input_pub = rospy.Publisher('/'+self.robot_type+'_driver/in/cartesian_velocity_finger', CartVelCmd, queue_size = 1)
            self.command_length = 6 + int(self.robot_type[5])
            label = 'cartesian_velocity'

        self.new_vel = CartVelCmd()
        _dim = [MultiArrayDimension()]
        _dim[0].label = label
        _dim[0].size = self.command_length
        _dim[0].stride = self.command_length
        self.new_vel.velocity.layout.dim = _dim
        self.new_vel.velocity.data = np.zeros_like(np.zeros(self.command_length))
        self.new_vel.header.stamp = rospy.Time.now()
        self.new_vel.header.frame_id = 'full_teleop'

        self.t_last_pub = -1  # time of last event publish

    def user_vel_CB(self, msg):
        self.new_vel.velocity = msg.velocity
        self.new_vel.header.stamp = rospy.Time.now()
        # TO DO:
        # Control Input Frame ID should reflect how/where input is coming from
        self.new_vel.header.frame_id = 'full_teleop'
        self.control_input_pub.publish(self.new_vel)
        self.t_last_pub = rospy.get_time()

    def publish_vel_topic(self, event=None):
        if rospy.get_time()-self.t_last_pub>0.01:
            self.new_vel.velocity.data = np.zeros_like(np.zeros(self.command_length))
            self.new_vel.header.stamp = rospy.Time.now()
            self.new_vel.header.frame_id = ''
            self.control_input_pub.publish(self.new_vel)

if __name__ == '__main__':

    rospy.init_node('uservel_to_cartesianvel', anonymous = True)
    control_input = UserToControlInput()
    user_vel_sub = rospy.Subscriber('/user_vel', CartVelCmd, control_input.user_vel_CB)
    rospy.Timer(rospy.Duration(1.0/100.0), control_input.publish_vel_topic) # publish data at 100 hz (required by kinova API)
    rospy.spin()
