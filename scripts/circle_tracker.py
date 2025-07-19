#!/usr/bin/env python
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class CircularTrajectory:
    def __init__(self):
        rospy.init_node('circular_trajectory_publisher')
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('/circle_markers', Marker, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)

        self.initial_pose = None
        self.current_pose = None
        rospy.loginfo("Waiting for initial pose...")

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pose = (x, y, yaw)
        if self.initial_pose is None:
            self.initial_pose = self.current_pose
            rospy.loginfo(f"Got initial pose: {self.initial_pose}")

    def wait_until_reached(self, goal_x, goal_y, threshold=0.15):
        rospy.loginfo("Waiting to reach goal...")
        while not rospy.is_shutdown():
            if self.current_pose is None:
                continue
            x, y, _ = self.current_pose
            dx = goal_x - x
            dy = goal_y - y
            dist = math.hypot(dx, dy)
            if dist < threshold:
                rospy.loginfo("Goal reached.")
                break
            self.rate.sleep()

    def generate_and_publish_circle(self):
        while not rospy.is_shutdown() and self.initial_pose is None:
            self.rate.sleep()

        x0, y0, theta0 = self.initial_pose
        radius = 1.0
        num_points = 36
        cx = x0 + radius * math.cos(theta0)
        cy = y0 + radius * math.sin(theta0)

        points = []
        poses = []

        # First generate all points and poses
        for i in range(num_points + 1):
            angle = 2 * math.pi * i / num_points
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            yaw = angle + math.pi / 2

            # Convert yaw to quaternion
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)

            # Create goal pose
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "odom"
            goal.pose.position.x = x
            goal.pose.position.y = y
            goal.pose.orientation.x = q[0]
            goal.pose.orientation.y = q[1]
            goal.pose.orientation.z = q[2]
            goal.pose.orientation.w = q[3]
            poses.append(goal)

            # Add marker point
            pt = Point()
            pt.x = x
            pt.y = y
            pt.z = 0.05
            points.append(pt)

        # ✅ Publish marker first
        self.publish_marker(points)

        # 🔁 Now send one goal at a time
        for i, goal in enumerate(poses):
            self.goal_pub.publish(goal)
            rospy.loginfo(f"Published goal {i+1}/{len(poses)}")
            self.wait_until_reached(goal.pose.position.x, goal.pose.position.y)

    def publish_marker(self, points):
        marker = Marker()
        marker.header.frame_id = "odom"   # Must match RViz fixed frame
        marker.header.stamp = rospy.Time.now()
        marker.ns = "circle"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Marker appearance
        marker.scale.x = 0.03             # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Add points
        marker.points = points

        # Set lifetime to 0 (forever)
        marker.lifetime = rospy.Duration(0)

        # Publish
        for _ in range(10):  # Publish multiple times to ensure RViz gets it
            self.marker_pub.publish(marker)
            rospy.sleep(0.1)
        rospy.loginfo("Published circular trajectory marker.")

if __name__ == '__main__':
    try:
        node = CircularTrajectory()
        node.generate_and_publish_circle()
    except rospy.ROSInterruptException:
        pass
