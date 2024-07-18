#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pose_int.srv import Xyaz, ArrayResult
from pose_int.msg import MsgDef
from std_msgs.msg import Empty

from scipy.spatial.transform import Rotation as R
import yaml
import numpy as np
import cv2

class FeatureDrawer(Node):

    def __init__(self):
        super().__init__('map_updater')
        self.pose_cli = self.create_client(Xyaz, 'get_pose_srv')
        self.qr_pose_cli = self.create_client(ArrayResult, 'qr_poses')
        while not self.pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for get_pose_srv to initiate...')
        while not self.qr_pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for qr_poses to initiate...')
        
        self.wait_time = 0.25
        self.create_timer(self.wait_time, self.timer_callback)
        self.get_logger().info('map_updater initilized')

    def send_request(self):
        req = Xyaz.Request()
        req.a = Empty()
        return self.pose_cli.call_async(req)
    
    def send_request_qr(self):
        req = ArrayResult.Request()
        req.a = Empty()
        return self.qr_pose_cli.call_async(req)

    def timer_callback(self):
        future = self.send_request()
        future.add_done_callback(self.when_pose_is_sent)
        
    def when_pose_is_sent(self, future):
        msg = future.result()
        # self.get_logger().info(f"\nx: {msg.x}\ny: {msg.y}\nangle: {msg.az}")
        
        try: 
            self.draw_on_map(msg.x, msg.y, msg.az)
        except:
            self.get_logger().info("error")

    def draw_on_map(self, current_x: float, current_y: float, theta: float):
        raw_img_dir = '/home/asak/dev_ws2/the_map.png'
        yaml_dir = '/home/asak/dev_ws2/the_map.yaml'
        
        self.img = cv2.imread(raw_img_dir)
        self.img = cv2.resize(self.img, None, fx=2., fy=2.)

        with open(yaml_dir, 'r') as file:
            origin = yaml.safe_load(file)['origin']
        self.or_x = int(abs(origin[0]) / 0.025)
        self.or_y = self.img.shape[0] - int(abs(origin[1]) / 0.025)
        cv2.circle(self.img, (self.or_x, self.or_y), 10, (150, 200, 100), cv2.FILLED) 

        rob_pose = (self.or_x + 40*current_x, self.or_y - 40*current_y)
        rob_size = (40*0.2, 40*0.8)
        rob_angle = 90 - theta
        self.rect = (rob_pose, rob_size, rob_angle)

        future = self.send_request_qr()
        future.add_done_callback(self.when_qr_is_sent)

    def when_qr_is_sent(self, future):
        out_img_dir = '/home/asak/dev_ws2/the_final_map.png'

        qrs = future.result().result
        for i in qrs:
            # self.get_logger().info('circle on:\n')
            # self.get_logger().info(str(i.data[0]))
            circle_pose = (int(self.or_x + 40*i.data[0]), int(self.or_y - 40*i.data[1]))
            cv2.circle(self.img, circle_pose, 10, (0, 0, 150), cv2.FILLED)
        box = cv2.boxPoints(self.rect)
        box = np.int0(box)
        cv2.drawContours(self.img, [box], 0, (0, 0, 255), -1)
        cv2.imwrite(out_img_dir, self.img)



def main(args=None):
    rclpy.init(args=args)
    node = FeatureDrawer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()