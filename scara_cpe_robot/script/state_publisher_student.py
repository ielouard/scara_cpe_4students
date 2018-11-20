#!/usr/bin/env python

import rospy

from sensor_msgs.msg import JointState as JointStateMoveIt
from dynamixel_msgs.msg import JointState as JointStateDynamixel
from std_msgs.msg import Int16, Header

SHOULDER_1_INDEX = 0
SHOULDER_2_INDEX = 1

class JointStatePublisher():

    def __init__(self):
        rospy.init_node('state_publisher')

        rate = 15 # Hz
        r = rospy.Rate(rate)
        self.header=Header()
        self.name = ['','']
        self.current_pos = [0.0, 0.0]
        self.velocity = [0.0, 0.0]
        self.load = [0.0, 0.0]



        # Start controller state subscribers
        rospy.Subscriber('/shoulder_1_controller/state', JointStateDynamixel, self.shoulder1_state_handler)
        rospy.Subscriber('/shoulder_2_controller/state', JointStateDynamixel, self.shoulder2_state_handler)

        # Start publisher
        self.joint_states_pub = rospy.Publisher('/joint_states', JointStateMoveIt, queue_size=10)

        while self.name[0]=='' or self.name[1]=='' :
            r.sleep()

        #r.sleep()

        rospy.loginfo("Publishing joint_state at " + str(rate) + "Hz")

        while not rospy.is_shutdown():
            self.publish_joint_states()
            r.sleep()

    def shoulder1_state_handler(self, msg):
        # rospy.loginfo("shoulder1: ",msg)

        self.name[SHOULDER_1_INDEX] 		= msg.name
        self.header                  		= msg.header
        self.current_pos[SHOULDER_1_INDEX] 	= msg.current_pos
        self.velocity[SHOULDER_1_INDEX] 	= msg.velocity
        self.load[SHOULDER_1_INDEX] 		= msg.load
        #
        # To be completed ...
        #

    def shoulder2_state_handler(self, msg):
        self.name[SHOULDER_2_INDEX] 		= msg.name
        self.header	                    	= msg.header
        self.current_pos[SHOULDER_2_INDEX] 	= msg.current_pos
        self.velocity[SHOULDER_2_INDEX] 	= msg.velocity
        self.load[SHOULDER_2_INDEX] 		= msg.load

        # rospy.loginfo("shoulder2: ",msg)

        #
        # To be completed ...
        #



    def publish_joint_states(self):
        joint_state=JointStateMoveIt()
        joint_state.header=self.header
        joint_state.name=self.name
        joint_state.position=self.current_pos
        joint_state.velocity=self.velocity
        joint_state.effort=self.load
        print self.header
        # print joint_state
        self.joint_states_pub.publish(joint_state)
        # Construct message & publish joint states
        #
        # To be completed ...
        #


if __name__ == '__main__':
    try:
        JointStatePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        exit()
        pass
