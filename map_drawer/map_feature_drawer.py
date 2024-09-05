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
        
        self.raw_img_dir = '/home/asak/dev_ws2/the_map.png'
        self.yaml_dir = '/home/asak/dev_ws2/the_map.yaml'
        self.out_img_dir = '/home/asak/dev_ws2/the_final_map.png'

        self.path_points = [[0., 0.]]
        self.wait_time = 0.5
        self.create_timer(self.wait_time, self.timer_callback)
        self.get_logger().info('map_updater initilized')

    def get_robot_position(self):
        req = Xyaz.Request()
        req.a = Empty()
        return self.pose_cli.call_async(req)
    
    def get_qr_positions(self):
        req = ArrayResult.Request()
        req.a = Empty()
        return self.qr_pose_cli.call_async(req)

    def timer_callback(self):
        future = self.get_robot_position()
        future.add_done_callback(self.when_pose_is_sent)
        
    def when_pose_is_sent(self, future):
        msg = future.result()
        if type(msg.x) == float:
            self.draw_on_map(msg.x, msg.y, msg.az)
        else:
            self.get_logger().info("error")

    def update_path_points(self, x, y):
        delx = self.path_points[-1][0] - x
        dely = self.path_points[-1][1] - y

        if (delx**2 + dely**2) ** 0.5 > 0.1:
            self.path_points.append([x, y])

    def draw_on_map(self, current_x: float, current_y: float, theta: float):
        # update the path points
        self.update_path_points(current_x, current_y)

        # open the map image
        self.img = cv2.imread(self.raw_img_dir)
        self.img = cv2.resize(self.img, None, fx=2., fy=2.)

        with open(self.yaml_dir, 'r') as file:
            origin = yaml.safe_load(file)['origin']
        self.or_x = int(abs(origin[0]) / 0.025)
        self.or_y = self.img.shape[0] - int(abs(origin[1]) / 0.025)

        # draw a circle at the origin
        cv2.circle(self.img, (self.or_x, self.or_y), 10, (150, 200, 100), cv2.FILLED) 

        # prepare the robot's shape
        rob_pose = (self.or_x + 40*current_x, self.or_y - 40*current_y)
        rob_size = (40*0.2, 40*0.8)
        rob_angle = 90 - theta
        self.rect = (rob_pose, rob_size, rob_angle)

        # get the qr code positions
        future = self.get_qr_positions()
        future.add_done_callback(self.when_qr_is_sent)

    def when_qr_is_sent(self, future):
        # draw path points
        if len(self.path_points) > 0:
            for i in range(len(self.path_points) - 1):
                p1 = (int(self.or_x + 40*self.path_points[i][0]  ), int(self.or_y - 40*self.path_points[i][1]))
                p2 = (int(self.or_x + 40*self.path_points[i+1][0]), int(self.or_y - 40*self.path_points[i+1][1]))

                cv2.line(self.img, p1, p2, (0, 150, 0), 2)

        # draw the qr codes
        qrs = future.result().result
        for i in qrs:
            # self.get_logger().info('circle on:\n')
            # self.get_logger().info(str(i.data[0]))
            circle_pose = (int(self.or_x + 40*i.data[0]), int(self.or_y - 40*i.data[1]))
            cv2.circle(self.img, circle_pose, 10, (0, 0, 150), cv2.FILLED)
        
        # draw the robot
        box = cv2.boxPoints(self.rect)
        box = np.int0(box)
        cv2.drawContours(self.img, [box], 0, (0, 0, 255), -1)

        # save the image
        cv2.imwrite(self.out_img_dir, self.img)


def main(args=None):
    rclpy.init(args=args)
    node = FeatureDrawer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()