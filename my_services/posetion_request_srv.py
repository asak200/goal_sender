#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from pose_int.srv import Xyaz
from scipy.spatial.transform import Rotation as R
from math import atan, sin, cos, pi


class GetCurrentPose(Node):

    def __init__(self):
        super().__init__('my_pose')
        self.map_to_base_x = 0
        self.map_to_base_y = 0
        self.obom_to_base_x = 0
        self.obom_to_base_y = 0
        self.map_to_odom_x = 0
        self.map_to_odom_y = 0
        self.alpha = 0
        self.alpha_rad = 0
        self.beta = 0
        self.gama = 0
        self.angle = 0
        self.skip_every = 0
        self.tf_subscription = self.create_subscription(
            TFMessage,
            'tf',
            self.listener_callback,
            10)
        self.tf_subscription
        self.pose_publisher_srv = self.create_service(
            Xyaz,
            'get_pose_srv',
            self.return_pose
        )
        self.get_logger().info('my_pose initilized')

    
    def listener_callback(self, msg):
        i = msg.transforms[0]
        parent = i.header.frame_id
        if parent != 'odom' and parent != 'map':
            return
        child = i.child_frame_id
        dist_x = i.transform.translation.x
        dist_y = i.transform.translation.y
        quat = [0, 0, i.transform.rotation.z, i.transform.rotation.w] 
        r = R.from_quat(quat)
        euler_angles = r.as_euler('xyz', degrees=True)
        
        if parent == 'odom' and child == 'base_link':
            self.r = (dist_x*dist_x + dist_y*dist_y) ** 0.5
            self.beta = atan(dist_y/dist_x)
            self.obom_to_base_x = self.r * cos(self.alpha_rad + self.beta)
            self.obom_to_base_y = self.r * sin(self.alpha_rad + self.beta)
            self.gama = euler_angles[2]
        elif parent == 'map' and child == 'odom':
            self.map_to_odom_x = dist_x
            self.map_to_odom_y = dist_y
            self.alpha_rad = euler_angles[2] * pi / 180
            self.alpha = euler_angles[2]

        self.map_to_base_x = self.map_to_odom_x + self.obom_to_base_x
        self.map_to_base_y = self.map_to_odom_y + self.obom_to_base_y
        self.angle = self.alpha + self.gama
    
    def return_pose(self, request, response):
        response.x = float(self.map_to_base_x)
        response.y = float(self.map_to_base_y)
        response.az = float(self.angle)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GetCurrentPose()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()