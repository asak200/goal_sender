#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from pose_int.msg import MsgDef

class SerialComNode(Node):

    def __init__(self):
        super().__init__('serial_com1')
        self.ser_port = '/dev/ttyACM0'



def main(args=None):
    rclpy.init(args=args)
    node = SerialComNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()