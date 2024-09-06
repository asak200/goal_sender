import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class RPLIDARFilter(Node):
    def __init__(self):
        super().__init__('rplidar_filter_node')
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.filtered_scan_pub = self.create_publisher(LaserScan, '/filtered_scan', 10)

    def scan_callback(self, msg: LaserScan):
        # Define the angle range to keep (in radians)
        min_angle = -1.57 * 4/3  # Example: -57 degrees (in radians)
        max_angle = 1.57 *4/3  # Example: 57 degrees (in radians)

        # Copy the original scan message
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max

        # Filter ranges
        filtered_scan.ranges = []
        for i, range_value in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if min_angle <= angle <= max_angle:
                filtered_scan.ranges.append(range_value)
            else:
                filtered_scan.ranges.append(float('inf'))  # Ignore this point

        # Publish filtered scan
        self.filtered_scan_pub.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    node = RPLIDARFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()