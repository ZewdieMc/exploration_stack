#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf
import math
from visualization_msgs.msg import Marker

class SequentialGoalPublisher:
    def __init__(self):
        rospy.init_node('sequential_goal_publisher')
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(2)

        # Define five 2D goals: (x, y, theta)
        self.goals = [
            (1.0, 0.0, 0.0),
            (1.0, 1.0, math.pi/2),
            (0.0, 1.0, math.pi),
            (-1.0, 1.0, -math.pi/2),
            (-1.0, 0.0, 0.0),
            (0.0, -1.0, -math.pi/2)
        ]
        self.current_goal_idx = 0
        self.current_pose = (0, 0, 0)
        self.goal_sent = False
        self.marker_pub = rospy.Publisher('/goal_poses_marker', Marker, queue_size=1)

        rospy.sleep(0.5)  # Ensure publisher is ready
        self.publish_goals_marker()

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_pose = (x, y, yaw)

    def publish_goal(self, x, y, theta):
        goal = PoseStamped()
        goal.header.frame_id = "odom"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0
        quat = tf.transformations.quaternion_from_euler(0, 0, theta)
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]
        self.goal_pub.publish(goal)
        rospy.loginfo("Published goal: (%.2f, %.2f, %.2f)", x, y, theta)

    def reached_goal(self, goal, tol_pos=0.07, tol_theta=0.07):
        x, y, theta = self.current_pose
        gx, gy, gtheta = goal
        dist = math.hypot(gx - x, gy - y)
        dtheta = abs((gtheta - theta + math.pi) % (2 * math.pi) - math.pi)
        return dist < tol_pos and dtheta < tol_theta

    def publish_goals_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_poses"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.points = []
        from geometry_msgs.msg import Point
        for x, y, _ in self.goals:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)
        self.marker_pub.publish(marker)

    def run(self):
        while not rospy.is_shutdown() and self.current_goal_idx < len(self.goals):
            goal = self.goals[self.current_goal_idx]
            if not self.goal_sent:
                self.publish_goal(*goal)
                self.goal_sent = True
            if self.reached_goal(goal):
                rospy.loginfo("Goal %d reached.", self.current_goal_idx + 1)
                self.current_goal_idx += 1
                self.goal_sent = False
                rospy.sleep(1.0)  # Small pause before next goal
            # self.rate.sleep()
        rospy.loginfo("All goals published and reached.")

if __name__ == '__main__':
    try:
        SequentialGoalPublisher().run()
    except rospy.ROSInterruptException:
        pass
