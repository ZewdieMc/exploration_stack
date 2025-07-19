#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math

class CircularMotionWithMarker:
    def __init__(self):
        rospy.init_node("circular_motion_with_marker")

        # Publishers
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.marker_pub = rospy.Publisher("/circle_marker", Marker, queue_size=1)

        # Parameters
        self.v = rospy.get_param("~v", 0.1)       # Linear velocity
        self.w = rospy.get_param("~w", 0.1)       # Angular velocity
        self.radius = self.v / self.w             # Circle radius

        # Set the center of the circle (offset)
        self.center_x = rospy.get_param("~center_x", 0.0)
        self.center_y = rospy.get_param("~center_y", 1.0)
        self.num_points = 100

        self.rate = rospy.Rate(10)
        rospy.sleep(1.0)  # Ensure connections are up
        self.publish_circle_marker()

    def publish_circle_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "circle"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0  # identity rotation

        for i in range(self.num_points + 1):  # close the loop
            angle = 2 * math.pi * i / self.num_points
            x = self.center_x + self.radius * math.cos(angle)
            y = self.center_y + self.radius * math.sin(angle)
            marker.points.append(Point(x=x, y=y, z=0))

        self.marker_pub.publish(marker)
        rospy.loginfo(f"Published circular path at center=({self.center_x}, {self.center_y})")

    def run(self):
        twist = Twist()
        twist.linear.x = self.v
        twist.angular.z = self.w
        while not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        node = CircularMotionWithMarker()
        node.run()
    except rospy.ROSInterruptException:
        pass
