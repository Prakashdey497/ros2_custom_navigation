#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math
from rclpy.qos import ReliabilityPolicy, QoSProfile

class GoalPublisherNode(Node):
    def __init__(self):
        super().__init__('twist_publisher_node')

        # Create a publisher for the Twist message
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.x_ = 0.0
        self.y_ = 0.0
        self.orientation_ = 0.0

        self.list_of_goal = [(0,0,0),(5,0,45),(5,5,90),(0,5,0),(0,0,135)]
        self.total_no_of_goal_location  = len(self.list_of_goal)
        self.increment_goal_list = 0

        sub = self.create_subscription(Odometry, "/bot_controller/odom", self.odom_callback, qos_profile=QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = self.create_timer(0.1, self.go_to_goal)


    def odom_callback(self,msg:Odometry):
        self.x_ = msg.pose.pose.position.x
        self.y_ = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (_,_, self.orientation_) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


    def go_to_goal(self):
        new_vel = Twist()
        if (self.increment_goal_list == self.total_no_of_goal_location):
            new_vel.linear.x = 0.0
            new_vel.angular.z = 0.0
            self.cmd_vel_pub.publish(new_vel)
            self.get_logger().info("Successfully Complete all Goal Location!")
            self.destroy_node()
            rclpy.shutdown()

        else:

            goal_x = self.list_of_goal[self.increment_goal_list][0]
            goal_y = self.list_of_goal[self.increment_goal_list][1]
            goal_theta = self.list_of_goal[self.increment_goal_list][2]

            K_linear = 0.5 
            # Ecludian Distance
            distance_to_goal = math.sqrt( (goal_x - self.x_)**2  + (goal_y - self.y_)**2 )
            linear_speed = distance_to_goal * K_linear

            # Angle to Goal
            angle_to_goal =math.atan2(goal_y - self.y_ , goal_x - self.x_)
            
            K_angular = 4.0
            # Corrected angle difference with proper wrapping
            angle_diff = math.atan2(math.sin(angle_to_goal - self.orientation_), math.cos(angle_to_goal - self.orientation_))
            angular_speed = angle_diff * K_angular

            new_vel.linear.x = linear_speed
            new_vel.angular.z = angular_speed

            self.cmd_vel_pub.publish(new_vel)
            
            
            if (distance_to_goal <0.02):
                
                desired_angle = math.radians(goal_theta)
                goal_angle_diff = math.atan2(math.sin(desired_angle - self.orientation_), math.cos(desired_angle - self.orientation_))

                if abs(goal_angle_diff) > 0.05:
                    # Define angular velocity for rotation
                    K_angular = 4.0
                    angular_speed = goal_angle_diff * K_angular

                    # Set the linear velocity to 0 for rotation
                    new_vel.linear.x = 0.0
                    new_vel.angular.z = angular_speed

                    # Publish Twist message
                    self.cmd_vel_pub.publish(new_vel)
                else:
                    # Stop the rotation and proceed to the next goal
                    self.get_logger().info("Goal Reached!")
                    self.get_logger().info(f"Goal Coordinates : ({goal_x}, {goal_y}, {goal_theta})")
                    new_vel.linear.x = 0.0
                    new_vel.angular.z = 0.0
                    self.cmd_vel_pub.publish(new_vel)
                    self.increment_goal_list += 1




def main(args=None):
    rclpy.init(args=args)

    node = GoalPublisherNode()
    rclpy.spin(node)
    # node.rotate(3)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()