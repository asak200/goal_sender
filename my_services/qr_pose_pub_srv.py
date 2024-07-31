#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from pose_int.srv import Xyaz, ArrayResult
from pose_int.msg import MsgDef
from std_msgs.msg import Empty
from example_interfaces.msg import String


class QrDrawerSrv(Node):

    def __init__(self):
        super().__init__('get_qr_pose_srv')
        self.get_qr_pose_srv = self.create_service(ArrayResult, 'qr_poses', self.srv_callback)
        self.qr_sub = self.create_subscription(String, 'qr_order', self.when_qr_read, 10)
        self.joy_subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.pose_cli = self.create_client(Xyaz, 'get_pose_srv')
        while not self.pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for get_pose server node...')
        
        self.prev_X = 0
        self.qr_posetions = []
        self.get_logger().info('get_qr_pose_srv initilized')

    def send_pose_request(self):
        """Get the current position of the robot from another service"""
        req = Xyaz.Request()
        req.a = Empty()
        return self.pose_cli.call_async(req)

    def when_qr_read(self, msg):
        """when a qr is read, go get sthe current position"""
        future = self.send_pose_request()
        future.add_done_callback(self.when_pose_is_sent_by_qr)

    def when_pose_is_sent_by_qr(self, future):
        """add the current position into a list of positions"""
        msg = future.result()
        self.pose_x = msg.x
        self.pose_y = msg.y
        a = MsgDef()
        a.data = [self.pose_x, self.pose_y]
        self.qr_posetions.append(a)

    def joy_callback(self, msg):
        self.current_X = msg.buttons[2]
        future = self.send_pose_request()
        future.add_done_callback(self.when_pose_is_sent_by_X)
        
    def when_pose_is_sent_by_X(self, future):
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
        """Return a list containing all qr positions in the map"""
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