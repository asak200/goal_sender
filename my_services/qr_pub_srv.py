#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from pose_int.srv import Xyaz, ArrayResult
from pose_int.msg import MsgDef
from std_msgs.msg import Empty


class QrDrawerSrv(Node):

    def __init__(self):
        super().__init__('get_qr_pose_srv')
        self.get_qr_pose_srv = self.create_service(ArrayResult, 'qr_poses', self.srv_callback)
        self.get_logger().info('qr_getter open')

        self.pose_cli = self.create_client(Xyaz, 'get_pose_srv')
        while not self.pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for server node...')
        
        self.prev_X = 0
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.joy_subscription
        self.qr_posetions = []
        self.get_logger().info('get_qr_pose_srv initilized')

    def send_request(self):
        req = Xyaz.Request()
        req.a = Empty()
        return self.pose_cli.call_async(req)

    def joy_callback(self, msg):
        self.current_X = msg.buttons[2]
        future = self.send_request()
        future.add_done_callback(self.when_pose_is_sent)
        
    def when_pose_is_sent(self, future):
        msg = future.result()
        self.pose_x = msg.x
        self.pose_y = msg.y
        if self.current_X and self.prev_X:
            pass
        elif self.current_X and not self.prev_X:
            a = MsgDef()
            a.data = [self.pose_x, self.pose_y]
            self.qr_posetions.append(a)
            self.prev_X = 1
        else:
            self.prev_X = 0

    def srv_callback(self, req, res):
        res.result = []
        res.result = self.qr_posetions
        return res

def main(args=None):
    rclpy.init(args=args)
    node = QrDrawerSrv()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()