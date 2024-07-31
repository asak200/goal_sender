#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pose_int.srv import Xyaz
from tf2_ros import TransformListener, Buffer
from scipy.spatial.transform import Rotation as R
from math import pi

class GetCurrentPose(Node):

    def __init__(self):
        super().__init__('my_pose')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pose_publisher_srv = self.create_service(
            Xyaz,
            'get_pose_srv',
            self.return_pose
        )
        self.get_logger().info('my_pose initilized')

    
    def return_pose(self, request, response: Xyaz.Response):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            response.x = trans.transform.translation.x
            response.y = trans.transform.translation.y
            response.az = self.get_yaw_from_quaternion(trans.transform.rotation)
            return response
        except Exception as e:
            self.get_logger().error(f'Could not transform: {str(e)}')
            response.x = float('nan')
            response.y = float('nan')
            response.az = float('nan')
        
    
    def get_yaw_from_quaternion(self, q):
        r = R.from_quat([q.x, q.y, q.z, q.w])
        euler = r.as_euler('xyz', degrees=False)
        return euler[2] * 180 / pi


def main(args=None):
    rclpy.init(args=args)
    node = GetCurrentPose()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()