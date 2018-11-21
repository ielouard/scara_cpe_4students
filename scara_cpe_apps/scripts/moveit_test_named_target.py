#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time


class moveit_test_named_target:

  named_targets=["straight","right","left"]

  def __init__(self):
    rospy.loginfo('starting init')
    rospy.init_node('moveit_test_target', anonymous = True)
    moveit_commander.roscpp_initialize(sys.argv)
    self.group = moveit_commander.MoveGroupCommander("scara_cpe_group")

  def move(self,position):
      self.group.set_named_target(position)
      plan = self.group.plan()
      execute=self.group.execute(plan)



if __name__ == '__main__':

  scara = moveit_test_named_target()

  rospy.loginfo("Starting test named targets")

  for target in scara.named_targets:
      scara.move(target)
      rospy.sleep(2)

  rospy.spin()
