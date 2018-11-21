#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Bool
import time

class Move_Coins:
    #TODO define A AND B
    A = [-0.73,0.08]
    B = [0.035,0.14]
    grip_pub=rospy.Publisher('grip', Bool, queue_size=1)

    def __init__(self):
        rospy.loginfo('starting init')
        rospy.init_node('move_coins', anonymous = True)
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.Subscriber('detect_metal', Bool, self.callback)
        self.group = moveit_commander.MoveGroupCommander("scara_cpe_group")

    def set_coordonnee(self,coord):
        self.group.clear_pose_targets()
        self.group_variable_values = coord
        self.group.set_joint_value_target(self.group_variable_values)
        plan = self.group.plan()
        self.group.execute(plan)
        rospy.sleep(1)

    def callback(self,req):
        pass

if __name__ == '__main__':
    scara = Move_Coins()
    release=Bool()
    release.data=True
    scara.set_coordonnee(scara.A)
    scara.grip_pub.publish(release)
    time.sleep(3)
    scara.set_coordonnee(scara.B)
    release.data=False
    scara.grip_pub.publish(release)
    time.sleep(2)
