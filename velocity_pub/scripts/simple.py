#! /usr/bin/env python 
import rospy
from geometry_msgs.msg import Twist
import tf
import math

class Simple:
  def __init__(self):
    rospy.init_node('simple')
    self.vel_pub = rospy.Publisher('/icart_mini/cmd_vel', Twist, queue_size=10)
    self.world_frame = 'odom'
    self.robot_frame = 'base_link'
    self.tf_listener = tf.TransformListener()
    self.run_time = 10
    self.start_time = rospy.get_time()
    self.now_time = rospy.get_time()

  def time_out(self):
    self.now_time = rospy.get_time()
    if (self.now_time - self.start_time) > self.run_time:
      return 1
    else:
      return 0
    

  def spin(self):
    vel = Twist()
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
      if self.time_out():
        vel.linear.x = 0
        vel.angular.z = 0
      else:
        vel.linear.x = 0.5
        vel.angular.z = math.pi / 4

      self.vel_pub.publish(vel)
      print(vel.linear.x,vel.angular.z)
      rate.sleep()


if __name__== '__main__':
    simple = Simple()
    simple.spin()
