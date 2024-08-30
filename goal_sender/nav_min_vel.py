#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MyNode(Node):

    def __init__(self):
        super().__init__('nav_vel_constrainer')
        self.vel_sub = self.create_subscription(Twist, 'cmd_vel_nav', self.callback, 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel_nav_edited', 10)
        self.get_logger().info("nav_vel_constrainer node started")

    def callback(self, msg_rec: Twist):
        v = msg_rec.linear.x
        a = msg_rec.angular.z

        def constrain(x, mns, mxs):
            def sign(n):
                if n>=0:
                    return 1
                else:
                    return -1
            if abs(x) > mxs:
                x =  mxs * sign(x)
            elif abs(x) < mns:
                x = mns * sign(x)
            return x
        v = constrain(v, 0.1, 0.54)
        a = constrain(a, .5, 1.5)
        
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = a
        self.vel_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = MyNode() 
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()