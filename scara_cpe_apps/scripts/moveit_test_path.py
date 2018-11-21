#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


class moveit_test_path:

  def __init__(self):
    rospy.loginfo('starting init')
    rospy.init_node('moveit_catesien', anonymous = True)
    moveit_commander.roscpp_initialize(sys.argv)
    self.group = moveit_commander.MoveGroupCommander("scara_cpe_group")


  def set_cartesien(self):
     pose_target_B = geometry_msgs.msg.Pose()
     pose_target_B.position.x = 0.035
     pose_target_B.position.y = 0.14
     pose_target_B.position.z = -0.025

     pose_target_D = geometry_msgs.msg.Pose()
     pose_target_D.position.x = -0.035
     pose_target_D.position.y = 0.08
     pose_target_D.position.z = -0.025

     pose_target_A = geometry_msgs.msg.Pose()
     pose_target_A.position.x = -0.035
     pose_target_A.position.y = -0.09
     pose_target_A.position.z = -0.025

     self.group.set_named_target('left')
     self.group.go(wait=True)
     rospy.sleep(1)

     waypoints = []

     point_initial = self.group.get_current_pose().pose
     point_initial.position.z = -0.025
     waypoints.append(copy.deepcopy(point_initial))

     waypoints.append(copy.deepcopy(pose_target_D))
#     waypoints.append(copy.deepcopy(pose_target_D))
#     waypoints.append(copy.deepcopy(pose_target_A))

     plan = self.group.plan()
     (plan, fraction) = self.group.compute_cartesian_path(waypoints, 0.01, 0.0)
     self.group.execute(plan)
     rospy.sleep(5)

if __name__ == '__main__':
  scara = moveit_test_path()
  scara.set_cartesien()
