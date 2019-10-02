#! /usr/bin/env python 
import rospy
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf
import math
import numpy as np

class Simple:
  def __init__(self):
    rospy.init_node('simple')
    self.vel_pub = rospy.Publisher('/icart_mini/cmd_vel', Twist, queue_size=10)
    self.best_path_pub = rospy.Publisher('/dwa_best_path', Path, queue_size=10)
    self.world_frame = 'odom'
    self.robot_frame = 'base_link'
    self.tf_listener = tf.TransformListener()
    self.acc_lim_x = 3.0 #m/sen2
    self.acc_lim_th = 6.0 #rad/sec2
    self.max_vel_x = 0.7
    self.min_vel_x = 0
    self.max_rot_vel = 90 * math.pi / 180.0
    self.v_reso = 0.1 #m/s
    self.yawrate_reso = 1.0 * math.pi / 180.0 #rad/s 
    self.sim_time = 1.2
    self.controller_frequency = 10.0
    self.dt = 1.0 / self.controller_frequency
    self.to_goal_cost_gain = 1.0
    self.speed_cost_gain = 1.5
    self.obstacle_cost_gain = 0.8
    self.pose_cost_gain = 0.03
    self.robot_radius = 0.4
    self.path_pub = rospy.Publisher("/dwa_path" , Path, queue_size=10)
    self.goal = [8,2]
 
  def get_robot_pose(self):
    trans = [0.0, 0.0, 0.0]
    yaw = [0.0, 0.0, 0.0]
    try:
      (trans, rot) = self.tf_listener.lookupTransform(self.world_frame, self.robot_frame, rospy.Time(0))
      yaw = tf.transformations.euler_from_quaternion(rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
      print("not get pose")
      pass

    return trans, yaw
    

  def motion(self,x,u,dt):
    x[2] += u[1] * dt
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[3] = u[0]
    x[4] = u[1]

    return x

  def calc_dynamic_window(self,x):
    Vs = [ self.min_vel_x, self.max_vel_x,
            -self.max_rot_vel, self.max_rot_vel]

    Vd = [ x[3] - self.acc_lim_x * self.dt,
           x[3] + self.acc_lim_x * self.dt,
           x[4] - self.acc_lim_th * self.dt,
           x[4] + self.acc_lim_th * self.dt ]

    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw

  def calc_trajectory(self, xinit, v, y):
    
      x = np.array(xinit)
      traj = np.array(x)
      time = 0
      while time <= self.sim_time:
        x = self.motion(x, [v, y], self.dt)
        traj = np.vstack((traj, x))
        time += self.dt

      return traj

  def calc_final_input(self, x, u, dw, goal, ob):
    xinit = x[:]
    min_cost = 10000.0
    min_u = u
    min_u[0] = 0.0
    best_traj = np.array([x])
    count = 0
    
    for v in np.arange(dw[0], dw[1], self.v_reso):
      for y in np.arange(dw[2], dw[3], self.yawrate_reso):
        count += 1
        traj = self.calc_trajectory(xinit, v, y)

        to_goal_cost = self.calc_to_goal_cost(traj, goal)
        speed_cost = self.speed_cost_gain * (self.max_vel_x - traj[-1, 3])
        ob_cost = self.calc_obstacle_cost(traj, ob)
        pose_cost = self.calc_pose_cost(traj, goal)
        final_cost = to_goal_cost + speed_cost + ob_cost + pose_cost

        path = Path()
        path.header.frame_id = self.world_frame
        for i in traj:
          pose = PoseStamped()
          pose.pose.position.x = i[0]
          pose.pose.position.y = i[1]
          pose.pose.position.z = 1 / final_cost
          q = tf.transformations.quaternion_from_euler(0, 0, i[2])
          pose.pose.orientation.x = q[0]
          pose.pose.orientation.y = q[1]
          pose.pose.orientation.z = q[2]
          pose.pose.orientation.w = q[3]
          path.poses.append(pose)
        self.path_pub.publish(path)

        print(round(traj[-1, 0],1), round(traj[-1, 1], 1), " goal:", round(to_goal_cost, 2)," sp:", round(speed_cost, 2)," ob:", round(ob_cost, 2)," pose:", round(pose_cost, 2), " sum:", round(final_cost, 2))
        #print(round(traj[-1, 0],1), round(traj[-1, 1], 1), round(final_cost, 1))

       
        if min_cost >= final_cost:
          min_cost = final_cost
          min_u = [v, y]
          best_traj = traj
    
    print()
    path = Path()
    path.header.frame_id = self.world_frame
    for i in best_traj:
      pose = PoseStamped()
      pose.pose.position.x = i[0]
      pose.pose.position.y = i[1]
      q = tf.transformations.quaternion_from_euler(0, 0, i[2])
      pose.pose.orientation.x = q[0]
      pose.pose.orientation.y = q[1]
      pose.pose.orientation.z = q[2]
      pose.pose.orientation.w = q[3]
      path.poses.append(pose)
    self.best_path_pub.publish(path)
    return min_u, best_traj

  def calc_obstacle_cost(self, traj, ob):
    
    skip_n = 2
    min_r = float("Inf")
    
    for ii in range(len(traj)):
      for i in range(len(ob)):
        ox = ob[i][0]
        oy = ob[i][1]
        dx = traj[ii][0] - ox
        dy = traj[ii][1] - oy

        r = math.sqrt(dx**2 + dy**2)
        if r <= self.robot_radius:
          return float("Inf")

        if min_r >= r:
          min_r = r
    #print(1.0 / min_r)
    return 1.0 / min_r * self.obstacle_cost_gain
    


  def calc_to_goal_cost(self, traj, goal):
    goal_magnitude = math.sqrt(goal[0]**2 + goal[1]**2)
    traj_magnitude = math.sqrt(traj[-1, 0]**2 + traj[-1, 1]**2)
    dot_product = (goal[0]*traj[-1,0]) + (goal[1]*traj[-1, 1])
    error = dot_product / (goal_magnitude*traj_magnitude)
    #print(goal, traj)
    error_angle = math.acos(error)
    #print(dot_product, goal_magnitude, traj_magnitude, error, error_angle)
    cost = self.to_goal_cost_gain * error_angle
    return cost

  def calc_pose_cost(self, traj, goal):
    if len(traj) is 1:
      return float("Inf")
    traj_pose = [traj[-2, 0], traj[-2, 1], traj[-1, 0], traj[-1, 1]]
    goal_pose = [0,0,self.goal[0],self.goal[1]]
    traj_ang = math.atan2(traj_pose[3] - traj_pose[1], traj_pose[2] - traj_pose[0])
    goal_ang = math.atan2(goal_pose[3] - goal_pose[1], goal_pose[2] - goal_pose[0])
    return abs(traj_ang - goal_ang) * self.pose_cost_gain


    
  def spin(self):
    vel = Twist()
    rate = rospy.Rate(self.controller_frequency)
    ob = [[2.5,0],[5.0,0.0],[4,1.5]]
    v = 0.0
    w = 0.0
    while not rospy.is_shutdown():
      trans, e = self.get_robot_pose()
      x = [trans[0], trans[1], e[2], v, w]
      #x = [0, 0, 0, 0, 0]
      dw = self.calc_dynamic_window(x)
      u, traj = self.calc_final_input(x, [v, w], dw, self.goal, ob)
      v = vel.linear.x = u[0]
      w = vel.angular.z = u[1]
      self.vel_pub.publish(vel)
      #print(vel.linear.x, vel.angular.z)
      rate.sleep()

      if math.sqrt((trans[0] - self.goal[0])**2 + (trans[1] - self.goal[1])**2) < 1:
        print("goal")
        break


if __name__== '__main__':
    simple = Simple()
    simple.spin()
