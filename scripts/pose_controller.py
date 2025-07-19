#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from nav_msgs.msg import Odometry
import tf
import math
import numpy as np
from visualization_msgs.msg import Marker

def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )
# def wrap_angle(angle):
#     return (angle + np.pi) % (2 * np.pi) - np.pi


def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    d = ((goal[0] - current[0])**2 + (goal[1] - current[1])**2)**0.5
    psi_d = np.arctan2(goal[1] - current[1], goal[0] - current[0])
    psi = wrap_angle(psi_d - current[2])
    v = 0.0 if abs(psi) > 0.05 else Kv * d
    w = Kw * psi
    return v, w

class PoseController:
    def __init__(self):
        rospy.init_node('pose_controller')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        self.rate = rospy.Rate(10)
        self.pose = Pose2D()
        self.goal = None
        
        self.marker_pub = rospy.Publisher('/path_marker', Marker, queue_size=1)  # Add marker publisher
        self.marker_goal_pub = rospy.Publisher('/goal_marker', Marker, queue_size=1)  # Add marker publisher
        self.goal_reached = False 
               
       # # CONTROLLER PARAMETERS
        # Proportional linear velocity controller gain
        self.Kv = 2.5
        # Proportional angular velocity controller gain                   
        self.Kw = 5
        # Maximum linear velocity control action                   
        self.v_max = 0.15
        # Maximum angular velocity control action               
        self.w_max = 0.9  

    def odom_callback(self, msg):
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.pose.theta = yaw

    def goal_callback(self, msg):
        self.goal_reached = False
        self.goal = Pose2D()
        self.goal.x = msg.pose.position.x
        self.goal.y = msg.pose.position.y
        q = msg.pose.orientation
        _, _, self.goal.theta = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.publish_marker()  
        self.publish_goal_arrow()
        
    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.points = []

        from geometry_msgs.msg import Point
        start = Point()
        start.x = self.pose.x
        start.y = self.pose.y
        start.z = 0.0
        end = Point()
        end.x = self.goal.x
        end.y = self.goal.y
        end.z = 0.0
        marker.points.append(start)
        marker.points.append(end)

        self.marker_pub.publish(marker)
        
    def __send_commnd__(self, v, w):
        cmd = Twist()
        cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub.publish(cmd)
        
    def publish_goal_arrow(self):
        # Publish an arrow marker at the goal pose showing position and heading
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "goal_pose"
        marker.id = 1
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.3  # Arrow shaft length
        marker.scale.y = 0.06  # Arrow shaft diameter
        marker.scale.z = 0.06  # Arrow head diameter
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the arrow
        marker.pose.position.x = self.goal.x
        marker.pose.position.y = self.goal.y
        marker.pose.position.z = 0.0

        import tf.transformations as tft
        quat = tft.quaternion_from_euler(0, 0, self.goal.theta)
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]

        self.marker_goal_pub.publish(marker)
               
    def run(self):
        while not rospy.is_shutdown():
            if self.goal and not self.goal_reached:
                current = [self.pose.x, self.pose.y, self.pose.theta]
                goal = [self.goal.x, self.goal.y, self.goal.theta]
                v, w = move_to_point(current, goal, self.Kv, self.Kw)

                # Stop if close enough to goal
                distance = math.hypot(goal[0] - current[0], goal[1] - current[1])
                heading_error = wrap_angle(goal[2] - current[2])
                if distance < 0.05 and abs(heading_error) < 0.05:
                    print("Goal reached")
                    self.__send_commnd__(0, 0)
                    self.goal_reached = True  # Set flag to prevent further commands
                else:
                    self.__send_commnd__(v, w)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        PoseController().run()
    except rospy.ROSInterruptException:
        pass
