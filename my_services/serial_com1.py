#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty

import serial
import time
import yaml

class SerialComNode(Node):

    def __init__(self):
        super().__init__('serial_com1')
        self.yaml_path = '/home/asak/dev_ws2/src/goal_sender/my_services/gui_data.yaml'
        self.ser_port = '/dev/ttyUSB0'
        self.ser = serial.Serial(self.ser_port, 115200, timeout=1.0)
        time.sleep(2.)
        self.ser.reset_input_buffer()
        self.get_logger().info("Serial com established")

        self.start()

    def start(self):
        try:
            while True:
                time.sleep(0.02)
                if self.ser.in_waiting > 0: # to receive 
                    line = self.ser.readline().decode('utf-8').rstrip()
                    self.analize_msg(line)
        except KeyboardInterrupt:
            print('close serial')
            self.ser.close()

    def analize_msg(self, line: str):
        if not ': ' in line or len(line.split(': ')) != 2:
            return
        order, content = line.split(': ')
        with open(self.yaml_path, 'r') as file:
            data = yaml.safe_load(file)

        if order == 'd' or order == 'wcm' or order == 'wait': # other updater
            data['other'] = order
        elif order == 'start' or order == 'stop' or order == 'done': # status updater
            data['status'] = order
        # sensor data
        data[order] = content
        
        with open(self.yaml_path, 'w') as file:
            yaml.dump(data, file)



def main(args=None):
    rclpy.init(args=args)
    node = SerialComNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()