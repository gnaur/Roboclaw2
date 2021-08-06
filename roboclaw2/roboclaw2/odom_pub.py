# -*- coding: utf-8 -*-
"""
Created on Mon Jan 11 01:43:00 2016

@author: ubuntu
"""

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion,TransformStamped
from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat
from sensor_msgs.msg import JointState

import math
import numpy as np
class OdomPub(object):

    def __init__(self,nh,base_width,odom_frame,base_frame,pub_tf,pub_joint_states,left_joint,right_joint):

      self.node = nh
      self.pub_joint_states=pub_joint_states
      self.left_joint=left_joint
      self.right_joint=right_joint

      self.odom_pub = nh.create_publisher(Odometry,"odom",50)

      if pub_tf:
         self.tf_pub = TransformBroadcaster(nh)
      else:
         self.tf_pub = None

      if pub_joint_states:
         self.joint_state_pub = nh.create_publisher(JointState,"joint_state",10)
      else:
         self.joint_state_pub = None


      self.base_width = base_width
      self.odom_frame = odom_frame
      self.base_frame = base_frame

      self.x = 0.0
      self.y = 0.0
      self.th = 0.0

      self.left_th=0.0
      self.right_th=0.0

      self.pub_tf = pub_tf

      self.last_time = self.node.get_clock().now()

    def publishOdom(self,vl,vr,al,ar):

        current_time = self.node.get_clock().now()

        dt = (current_time - self.last_time).nanoseconds/1.0e9

        vx = (vl+vr)/2.0 #d_x / dt
        vth = (vr-vl)/self.base_width

        d_th = vth * dt

        if(vth==0):
            self.x += vx * dt * math.cos(self.th)
            self.y += vx * dt * math.sin(self.th)


        else:
            R = (self.base_width/2) * (vl+vr)/(vr-vl)
            ICC = np.matrix([[self.x - R * math.sin(self.th)], [self.y + R * math.cos(self.th)]])

            Rm = np.matrix([[math.cos(d_th),-math.sin(d_th)],[math.sin(d_th),math.cos(d_th)]])

            P = Rm * np.matrix([[self.x-ICC[0,0]],[self.y-ICC[1,0]]]) + ICC

            self.x = P[0,0]
            self.y = P[1,0]


        self.th += d_th

        
        vy = 0.0 #d_y / dt

        #since all odometry is 6DOF we'll need a quaternion created from yaw
        q=euler2quat(0.0,0.0,self.th)
        odom_quat = Quaternion()
        odom_quat.x=q[1]
        odom_quat.y=q[2]
        odom_quat.z=q[3]
        odom_quat.w=q[0]

        if self.pub_tf:
            #first, we'll publish the transform over tf
            odom_trans=TransformStamped()

            odom_trans.header.stamp = current_time.to_msg()
            odom_trans.header.frame_id =self.odom_frame
            odom_trans.child_frame_id =self.base_frame

            odom_trans.transform.translation.x = self.x
            odom_trans.transform.translation.y = self.y
            odom_trans.transform.translation.z = 0.0
            odom_trans.transform.rotation = odom_quat

            #send the transform
            self.tf_pub.sendTransform(odom_trans)

        if self.pub_joint_states:

            joint_state = JointState()
            joint_state.header.stamp = current_time.to_msg()
            joint_state.header.frame_id = ""

            joint_state.name = [self.left_joint,self.right_joint]
            joint_state.position = [al,ar]
            joint_state.velocity = [vl,vr]

            self.joint_state_pub.publish(joint_state)
           

        #next, we'll publish the odometry message over ROS

        pcv = [0.0 for x in range(36) ]
        pcv[0]  = 0.01
        pcv[7]  = 0.01
        pcv[14] = 99999.0
        pcv[21] = 99999.0
        pcv[28] = 99999.0
        pcv[35] = 0.01

        tcv = pcv
 
        tcv[0]  = 0.01
        tcv[7]  = 0.01
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id =self.odom_frame

        #set the position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat
        odom.pose.covariance = pcv
        
        #set the velocity
        odom.child_frame_id =self.base_frame
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = tcv

        #publish the message
        self.odom_pub.publish(odom)

        self.last_time = current_time


