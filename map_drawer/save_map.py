#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.node import Node
from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


import os
from PIL import Image

class ImageSaverNode(Node):

    def __init__(self):
        super().__init__('image_saver_node')
        timer_period = 1.
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        # self.save_map()

    
    def timer_callback(self):
        self.i +=1
        self.get_logger().info("start saving %d" % self.i)
        self.save_map()
        self.convert_pgm_to_png()
        self.get_logger().info("done saving")

    def save_map(self):
        # Create a launch service
        launch_service = LaunchService()

        # Include the launch file
        launch_description = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                '/home/asak/dev_ws2/src/goal_sender/map_drawer/nav2_saver.launch.py'
            )]),
        )

        # Add the launch description to the launch service
        launch_service.include_launch_description(launch_description)

        # Run the launch service
        launch_service.run()

    def convert_pgm_to_png(self):
        pgm_image_path = '/home/asak/dev_ws2/the_map.pgm'
        png_image_path = '/home/asak/dev_ws2/the_map.png'
        
        pgm_image = Image.open(pgm_image_path)
        pgm_image.save(png_image_path)

        

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()