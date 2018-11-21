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
    A = [-0.73,0.08]
    B = [0.035,0.14]
    grip_pub=rospy.Publisher('grip', Bool, queue_size=1)
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()

    def __init__(self):
        rospy.loginfo('starting init')
        rospy.init_node('move_coins', anonymous = True)
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.Subscriber('detect_metal', Bool, self.callback)
        self.group = moveit_commander.MoveGroupCommander("scara_cpe_group")

    def init_box(self,box_name,x,y,s):
        self.scene.remove_world_object(box_name)
        time.sleep(2)
        rospy.loginfo("init_box")
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id= "end_link"
        box_pose.pose.orientation.w= 1.0
        box_pose.pose.position.x= x
        box_pose.pose.position.y= y
        box_pose.pose.position.z= 0.025
        self.scene.add_box(box_name, box_pose, size=s)
        rospy.loginfo("end of init box")

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
    scara.init_box("box", 0.051, 0.007, (0.04,0.04,0.04))
    scara.init_box("coin", 0.0, 0.035, (0.02,0.02,0.02))
    scara.scene.attach_box(scara.group.get_end_effector_link(),"coin")
    release=Bool()
    release.data=True
    scara.set_coordonnee(scara.A)
    scara.grip_pub.publish(release)
    time.sleep(3)
    scara.set_coordonnee(scara.B)
    release.data=False
    scara.grip_pub.publish(release)
    time.sleep(2)
