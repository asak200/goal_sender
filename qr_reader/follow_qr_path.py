#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pose_int.srv import CmdVelReq

import copy
import sys

class MyNode(Node):

    def __init__(self):
        super().__init__('qr_path_follower')
        self.qr_listener = self.create_subscription(String, 'qr_order', self.qr_lis_callback_test, 10)
        self.send_order = self.create_client(CmdVelReq, 'send_vel_srv')

        self.empty_sen_1 = [21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 30, 29, 28, 27, 26, 25, 24, 23, 22]
        self.empty_sen_2 = [23, 24, 25, 26, 27, 28, 29, 30, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22]
        self.empty_sen_3 = [6, 5, 4, 3, 2, 1, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7]
        self.empty_sen_4 = [8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 1, 2, 3, 4, 5, 6, 7]

        self.empty_sen_1D = ['f', 'f', 'f', 'f', 'f', 'l', 'f', 'l', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'l', 'f', 'l', 'f', 'f', 'f', 'f', 'f', 'f', 's']
        self.empty_sen_3D = copy.deepcopy(self.empty_sen_1D)
        self.empty_sen_2D = ['f', 'f', 'f', 'f', 'f', 'r', 'f', 'r', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'r', 'f', 'r', 'f', 'f', 'f', 'f', 'f', 'f', 's']
        self.empty_sen_4D = copy.deepcopy(self.empty_sen_2D)

        self.empty_sen_1_mod = [16, 14, 1, 29, 22]
        self.empty_sen_1D_mod = ['l', 'l', 'l', 'l', 's']
        self.empty_sen_2_mod = [28, 30, 13, 15, 22]
        self.empty_sen_2D_mod = ['r', 'r', 'r', 'r', 's']
        self.empty_sen_3_mod = [1, 29, 16, 14, 7]
        self.empty_sen_3D_mod = ['l', 'l', 'l', 'l', 's']
        self.empty_sen_4_mod = [13, 15, 28, 30, 7]
        self.empty_sen_4D_mod = ['r', 'r', 'r', 'r', 's']

        # self.wei_sen_1_og = [42, 41, 7, 8, 9, 10, 11, 48, 49, 50, 51, 52, 18, 19, 20, 21, 22, 23, 24, 36, 37, 38, 39, 40, 4, 3, 31, 32, 33, 34, 35, 26, 25, 24, 23, 22, 21, 20, 43, 44, 45, 46, 47, 9, 8, 7, 41, 42, 22]
        # self.wei_sen_2_og = [42, 41, 7, 8, 9, 47, 46, 45, 44, 43, 20, 21, 22, 23, 24, 25, 26, 35, 34, 33, 32, 31, 3, 4, 40, 39, 38, 37, 36, 24, 23, 22, 21, 20, 19, 18, 52, 51, 50, 49, 48, 11, 10, 9, 8, 7, 41, 42, 22]
        # self.wei_sen_3_og = [41, 42, 22, 23, 24, 25, 26, 35, 34, 33, 32, 31, 3, 4, 5, 6, 7, 8, 9, 47, 46, 45, 44, 43, 19, 18, 52, 51, 50, 49, 48, 11, 10, 9, 8, 7, 6, 5, 40, 39, 38, 37, 36, 24, 23, 22, 42, 41, 7]
        # self.wei_sen_4_og = [41, 42, 22, 23, 24, 36, 37, 38, 39, 40, 5, 6, 7, 8, 9, 10, 11, 48, 49, 50, 51, 52, 18, 19, 43, 44, 45, 46, 47, 9, 8, 7, 6, 5, 4, 3, 31, 32, 33, 34, 35, 26, 25, 24, 23, 22, 21, 20, 19, 18, 52, 51, 50, 49, 48, 11, 10, 9, 8, 7]
        self.wei_sen_1 = [42, 41, 8, 9, 10, 11, 48, 49, 50, 51, 52, 18, 19, 20, 21, 22, 23, 24, 36, 37, 38, 39, 40, 4, 3, 31, 32, 33, 34, 35, 26, 25, 24, 23, 22, 21, 20, 43, 44, 45, 46, 47, 9, 8, 41, 42, 22]
        self.wei_sen_2 = [42, 41, 8, 9, 47, 46, 45, 44, 43, 20, 21, 22, 23, 24, 25, 26, 35, 34, 33, 32, 31, 3, 4, 40, 39, 38, 37, 36, 24, 23, 22, 21, 20, 19, 18, 52, 51, 50, 49, 48, 11, 10, 9, 8, 41, 42, 22]
        self.wei_sen_2D= ['f', 'l', 'f', 'r', 'f', 'f', 'u', 'f', 'r', 'f', 'f', 'f', 'f', 'f', 'f', 'r', 'f', 'f', 'd', 'f', 'r', 'f', 'r', 'f', 'f', 'u', 'f', 'l', 'f', 'f', 'f', 'f', 'f', 'f', 'l', 'f', 'f', 'd', 'f', 'l', 'f', 'f', 'f', 'l', 'f', 'f', 's']
        self.wei_sen_3 = [41, 42, 23, 24, 25, 26, 35, 34, 33, 32, 31, 3, 4, 5, 6, 7, 8, 9, 47, 46, 45, 44, 43, 19, 18, 52, 51, 50, 49, 48, 11, 10, 9, 8, 7, 6, 5, 40, 39, 38, 37, 36, 24, 23, 42, 41, 7]
        self.wei_sen_1D= ['f', 'l', 'f', 'f', 'f', 'r', 'f', 'f', 'u', 'f', 'r', 'f', 'f', 'f', 'f', 'f', 'f', 'r', 'f', 'f', 'd', 'f', 'l', 'f', 'l', 'f', 'f', 'u', 'f', 'l', 'f', 'f', 'f', 'f', 'f', 'f', 'l', 'f', 'f', 'd', 'f', 'l', 'f', 'l', 'f', 'f', 's']
        self.wei_sen_3D= ['f', 'l', 'f', 'f', 'f', 'r', 'f', 'f', 'u', 'f', 'r', 'f', 'f', 'f', 'f', 'f', 'f', 'r', 'f', 'f', 'd', 'f', 'l', 'f', 'l', 'f', 'f', 'u', 'f', 'l', 'f', 'f', 'f', 'f', 'f', 'f', 'l', 'f', 'f', 'd', 'f', 'l', 'f', 'l', 'f', 'f', 's']
        self.wei_sen_4 = [41, 42, 23, 24, 36, 37, 38, 39, 40, 5, 6, 7, 8, 9, 10, 11, 48, 49, 50, 51, 52, 18, 19, 43, 44, 45, 46, 47, 9, 8, 7, 6, 5, 4, 3, 31, 32, 33, 34, 35, 26, 25, 24, 23, 22, 21, 20, 19, 18, 52, 51, 50, 49, 48, 11, 10, 9, 8, 7]
        self.wei_sen_4D= ['f', 'l', 'f', 'r', 'f', 'f', 'u', 'f', 'r', 'f', 'f', 'f', 'f', 'f', 'f', 'r', 'f', 'f', 'd', 'f', 'r', 'f', 'r', 'f', 'f', 'u', 'f', 'l', 'f', 'f', 'f', 'f', 'f', 'f', 'l', 'f', 'f', 'd', 'f', 'l', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'f', 'l', 'f', 'f', 'f', 'f', 'l', 'f', 'f', 'f', 'f', 's']

        self.wei_sen_1_mod = [41, 11, 50, 52, 24, 38, 40, 3, 33, 35, 20, 45, 47, 8, 22]
        self.wei_sen_1D_mod = ['l','r','u','r','r','d','l','l','u','l','l','d','l','l','s']
        self.wei_sen_2_mod = [41, 9, 45, 43, 26, 33, 31, 4, 38, 36, 18, 50, 48, 8, 22]
        self.wei_sen_2D_mod = ['l', 'r', 'u', 'r', 'r', 'd', 'r', 'r', 'u', 'l', 'l', 'd', 'l', 'l', 's']
        self.wei_sen_3_mod = [42, 26, 33, 31, 9, 45, 43, 18, 50, 48, 5, 38, 36, 23, 7]
        self.wei_sen_3D_mod = ['l', 'r', 'u', 'r', 'r', 'd', 'l', 'l', 'u', 'l', 'l', 'd', 'l', 'l', 's']
        self.wei_sen_4_mod = [42, 24, 38, 40, 11, 50, 52, 19, 45, 47, 3, 33, 35, 18, 48, 7]
        self.wei_sen_4D_mod = ['l', 'r', 'u', 'r', 'r', 'd', 'r', 'r', 'u', 'l', 'l', 'd', 'l', 'l', 'l', 's']

        self.qr_positions = [[0.,0.]]*52
        
        self.ord = CmdVelReq.Request
        self.get_logger().info("qr_path_follower node initilized")

    def qr_lis_callback_test(self, msg: String):
        msg = msg.data
        qr_d = msg.split(';')
        qn = int(qr_d[0][1:])
        self.get_logger().info(f'{qn}')

        x, y = float(qr_d[1]) /1000, float(qr_d[2]) /1000

        self.qr_positions[qn-1] = [x, y]

        if qn == 52:
            print(self.qr_positions)
        
        # self.ord.speed_request = 'r'
        # self.get_logger().info(f'{self.ord.speed_request}')
        # self.send_order.call_async(self.ord)
    
    
    def qr_lis_callback_v1(self, msg: String):
        # print(555)
        msg = msg.data
        qr = msg.split(';')[0][1:]
        # self.get_logger().info(qr)
        if int(qr) == self.empty_sen_3[0]:
            self.ord.speed_request = 'or: ' + self.empty_sen_3D[0]
            self.get_logger().info(f'{self.ord.speed_request}')
            self.empty_sen_3.remove(self.empty_sen_3[0])
            self.empty_sen_3D.remove(self.empty_sen_3D[0])
            # self.send_order.call_async(self.ord)

    def qr_lis_callback_v2(self, msg: String):
        msg = msg.data
        qr = msg.split(';')[0][1:]
        if int(qr) == self.empty_sen_3[0]:
            self.ord.speed_request = 'or: ' + self.empty_sen_3D[0]
            self.get_logger().info(f'{self.ord.speed_request}')
            self.empty_sen_3.remove(self.empty_sen_3[0])
            self.empty_sen_3D.remove(self.empty_sen_3D[0])
            # self.send_order.call_async(self.ord)
        


def main(args=None):
    rclpy.init(args=args)
    node = MyNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
