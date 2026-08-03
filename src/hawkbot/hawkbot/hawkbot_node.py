#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(script_dir)
import HBSDK

def main():
    # Arguments: robot_ip, node_type, cam_quality, robot_param [, port_offset]
    robot_ip = sys.argv[1]
    node_type = sys.argv[2]
    cam_quality = sys.argv[3]
    robot_param = sys.argv[4]

    # Optional 5th argument: port offset for multi-robot support.
    # Each robot instance needs unique local (source) UDP ports to avoid
    # bind() conflicts. The offset is added to control_data_spt and
    # lidar_data_spt BEFORE calling run().
    # Default ports: control_data_spt=29083, lidar_data_spt=18902
    port_offset = int(sys.argv[5]) if len(sys.argv) > 5 else 0

    if port_offset != 0:
        HBSDK.control_data_spt = HBSDK.control_data_spt + port_offset
        HBSDK.lidar_data_spt = HBSDK.lidar_data_spt + port_offset
        print(f"[hawkbot_node] Port offset={port_offset} applied: "
              f"control_spt={HBSDK.control_data_spt}, "
              f"lidar_spt={HBSDK.lidar_data_spt}")

    HBSDK.run(robot_ip, node_type, cam_quality, robot_param)

if __name__ == '__main__':
    main()