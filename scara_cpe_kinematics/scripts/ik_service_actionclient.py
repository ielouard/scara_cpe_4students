#!/usr/bin/env python
import roslib
roslib.load_manifest('scara_cpe_kinematics')

import rospy
import actionlib
import math
from scara_cpe_kinematics.srv import GoToXY
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal





class Joint:
        def __init__(self):
            self.shoulder_1_joint=0
            self.shoulder_2_joint=0
            self.Link1_LENGTH = 0.08
            self.Link2_LENGTH = 0.047
            self.jta = actionlib.SimpleActionClient('/scara_cpe/scara_cpe_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')
            rospy.Service('gotoxy', GoToXY, self.handle_gotoxy)

        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = ['shoulder_1_joint', 'shoulder_2_joint']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(3)
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)

        def lawOfCosines(self,a,b,c):
            return math.acos((a*a + b*b - c*c) / (2 * a * b))

        def distance(self,x, y):
            return math.sqrt(x*x + y*y)

        def angles(self,x, y):
            dist = self.distance(x,y)
            D1 = math.atan2(y, x)
            D2 = self.lawOfCosines(dist, self.Link1_LENGTH, self.Link2_LENGTH)
            A1 = D1 + D2
            A2 = self.lawOfCosines(self.Link1_LENGTH, self.Link2_LENGTH, dist)
            return [A1, A2]

        def handle_gotoxy(self, req):
            print "Go to :\n " , req
            self.move_joint(self.angles(req.x,req.y))
            return True

def main():
    arm = Joint()
    angles = arm.angles(-0.073,0.08)
    print angles
    arm.move_joint(angles)
    rospy.spin()




if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
