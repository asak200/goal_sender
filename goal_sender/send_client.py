#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle, GoalStatus

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from sensor_msgs.msg import Joy
from pose_int.srv import Xyaz
from std_msgs.msg import Empty

from scipy.spatial.transform import Rotation as R
import math


class CreateGoalSender(Node): 

    def __init__(self):
        super().__init__('my_goal_sender')
        self.act_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        self.pose_cli = self.create_client(Xyaz, 'get_pose_srv')
        while not self.pose_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for get_pose server node...')

        self.prev_Y = 0
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10)
        self.joy_subscription

    def send_pose_request(self):
        """Get the current position of the robot from another service"""
        req = Xyaz.Request()
        req.a = Empty()
        return self.pose_cli.call_async(req)

    def joy_callback(self, msg):
        self.current_Y = msg.buttons[3]
        if self.current_Y and self.prev_Y:
            pass
        elif self.current_Y and not self.prev_Y:
            self.prev_Y = 1
            future = self.send_pose_request()
            future.add_done_callback(self.when_pose_is_sent)
        else:
            self.prev_Y = 0

    def when_pose_is_sent(self, future):
        msg = future.result()
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.angle = msg.az
        r = R.from_euler('xyz', [0, 0, self.angle], degrees=True)
        q = r.as_quat()
        self.send_goal(msg.x, msg.y, q[2], q[3])

    def send_goal(self, g_x, g_y, g_az, g_aw):
        # create goal
        goal = NavigateToPose.Goal()

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'

        theta = 2 * math.atan2(g_az, g_aw)
        pose.pose.position = Point(x=g_x + 5 * math.cos(theta), 
                                   y=g_y + 5 * math.sin(theta), 
                                   z=0.)
        pose.pose.orientation = Quaternion(x=0., y=0., z=g_az, w=g_aw)

        goal.pose = pose

        # Wait for server
        self.act_client.wait_for_server()

        # Send the goal
        self.get_logger().info("Sending goal")
        self.act_client. \
            send_goal_async(goal). \
                add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future):
        self.goal_handle_: ClientGoalHandle = future.result()
        if self.goal_handle_.accepted:
            self.get_logger().info("goal got accepted")
            self.goal_handle_.get_result_async().add_done_callback(self.goal_result_callback)
        else:
            self.get_logger().warn("goal got denied")

    def goal_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("succeeded")
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("ABORTED")
        self.get_logger().info("Result: ")
        self.get_logger().info(str(status))


def main(args=None):
    rclpy.init(args=args)
    node = CreateGoalSender()
    # node.send_goal(node.pose_x, node.pose_y, node.pose_az)
    rclpy.spin(node)
    rclpy.shutdown()
    # goal = NavigateToPose.Goal()
    # print(goal.pose.header.get_fields_and_field_types())


if __name__ == '__main__':
    main()