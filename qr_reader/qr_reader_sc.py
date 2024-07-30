#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
import urllib.request

class MyNode(Node):

    def __init__(self):
        super().__init__('qr_reader_node')
        self.create_publisher(String, 'qr_order', 10)
        self.font = cv2.FONT_HERSHEY_PLAIN
        self.url='http://192.168.33.109/'
        cv2.namedWindow("live transmission", cv2.WINDOW_AUTOSIZE)
        # 192.168.33.109
        # 192.168.145.109

        self.pres = ""
        self.prev = ""
        self.order = ""
        self.i = 0
        self.timeout_duration = 2.
        self.create_timer(0.2, self.take_photo)
        self.get_logger().info("Qr reader initialized")

    def take_photo(self):
        # print(self.i)
        try: 
            img_resp=urllib.request.urlopen(self.url+'640x480.jpg', timeout=self.timeout_duration)
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}.")
            return
        imgnp=np.array(bytearray(img_resp.read()),dtype=np.uint8)
        frame=cv2.imdecode(imgnp,-1)
        self.i += 1
        decodedObjects = pyzbar.decode(frame)
        for obj in decodedObjects:
            self.pres=obj.data
            if self.prev == self.pres:
                pass
            else:
                # print("Type:",obj.type)
                # print("Data: ",obj.data)
                self.order = str(obj.data)[9:-1]
                print(self.order)
                self.prev=self.pres
            cv2.putText(frame, self.order, (50, 50), self.font, 2,
                        (255, 255, 0), 3)
        cv2.putText(frame, str(self.i), (300, 50), self.font, 2,
                    (0, 0, 255), 3)

        cv2.imshow("live transmission", frame)
        cv2.waitKey(1)

    

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
