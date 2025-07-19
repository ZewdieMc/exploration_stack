#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D

rospy.init_node('goal_sender')
pub = rospy.Publisher('/goal_pose', Pose2D, queue_size=10)
rospy.sleep(1)

goal = Pose2D(x=1.0, y=4, theta=1.57)  # Pose in meters and radians
pub.publish(goal)
