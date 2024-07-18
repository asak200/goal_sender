#!/usr/bin/env python3

import cv2
import yaml
import numpy as np

def main(args=None):
    a = [geometry_msgs.msg.TransformStamped(
            header=std_msgs.msg.Header(
                stamp=builtin_interfaces.msg.Time(sec=1495, nanosec=638000000), 
                frame_id='base_link'), 
            child_frame_id='left_wheel', 
            transform=geometry_msgs.msg.Transform(
                translation=geometry_msgs.msg.Vector3(x=0.0, y=0.27, z=0.0), 
                rotation=geometry_msgs.msg.Quaternion(x=-0.5521165886787552, y=-0.44177740153354844, z=-0.44177740153354855, w=0.5521165886787553))), 
        geometry_msgs.msg.TransformStamped(
            header=std_msgs.msg.Header(
                stamp=builtin_interfaces.msg.Time(sec=1495, nanosec=638000000), 
                frame_id='base_link'), 
                child_frame_id='right_wheel', 
                transform=geometry_msgs.msg.Transform(
                    translation=geometry_msgs.msg.Vector3(x=0.0, y=-0.27, z=0.0), 
                    rotation=geometry_msgs.msg.Quaternion(x=0.49395783272423827, y=-0.505970018371023, z=0.5059700183710231, w=0.4939578327242384)))]



if __name__ == '__main__':
    main()

