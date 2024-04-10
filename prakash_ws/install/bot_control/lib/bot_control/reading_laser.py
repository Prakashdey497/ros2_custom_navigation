#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ReadLaserScanDataNode(Node):
    def __init__(self):
        super().__init__('laser_filter_node')

        self.get_logger().info("running ........")

         # Subscribe to the laser scan topic
        self.create_subscription(LaserScan,'/scan', self.laserCallback,10)

        # Angle range of interest
        self.angle_range_of_interest = 120

        self.filtered_scan_msg = LaserScan()

        # Publisher for filtered laser scan
        self.filtered_scan_pub = self.create_publisher(LaserScan,'/filtered_scan',10)


    def laserCallback(self,msg:LaserScan):

        # Total number of beams in the specified angle range (0 to 120 degrees)
        total_beams = len(msg.ranges)
        required_beams = int(total_beams * (self.angle_range_of_interest / 180))  # Angle range is from -90 to +90 degrees

        # Create a new LaserScan message with filtered ranges
        filtered_scan_msg = LaserScan()
        filtered_scan_msg.header = msg.header
        filtered_scan_msg.angle_min = msg.angle_min  # Adjust angle_min for 0 degrees
        filtered_scan_msg.angle_max = 0.523599  # Adjust angle_max for 120 degrees
        filtered_scan_msg.angle_increment = msg.angle_increment
        filtered_scan_msg.time_increment = msg.time_increment
        filtered_scan_msg.scan_time = msg.scan_time
        filtered_scan_msg.range_min = msg.range_min
        filtered_scan_msg.range_max = msg.range_max
        filtered_scan_msg.ranges = msg.ranges[:required_beams]

        # Publish filtered laser scan
        self.filtered_scan_pub.publish(filtered_scan_msg)


def main(args=None):
    rclpy.init(args=args)

    node = ReadLaserScanDataNode()
    rclpy.spin(node)
   
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()