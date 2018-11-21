#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time


class moveit_test_poses:

  def __init__(self):
    rospy.loginfo('starting init')
    rospy.init_node('moveit_test_target', anonymous = True)
    moveit_commander.roscpp_initialize(sys.argv)
    self.group = moveit_commander.MoveGroupCommander("scara_cpe_group")

#TODO define A,B,C and D
    self.A = (0,0)
    self.B = (0,0)
    self.C = (0,0)
    self.D = (0,0)

  def set_coordonnee(self,x,y):
    self.group.clear_pose_targets()
    self.group_variable_values = [x,y]
    self.group.set_joint_value_target(self.group_variable_values)
    plan = self.group.plan()
    self.group.execute(plan)
    rospy.sleep(1)


if __name__ == '__main__':
    scara = moveit_test_poses()
    scara.set_coordonnee(0.5,0.5)
    scara.set_coordonnee(0,0)
