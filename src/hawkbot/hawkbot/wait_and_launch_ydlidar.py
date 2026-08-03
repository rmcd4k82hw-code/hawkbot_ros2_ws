#!/usr/bin/python3
# -*- coding: utf-8 -*-

import sys
import os
import time
import json

def main():
    # ROS2 node args will be passed as:
    # wait_and_launch_ydlidar <robot_ip> [ROS arguments...]
    if len(sys.argv) < 2:
        print("[wait_and_launch_ydlidar] Error: Missing robot IP argument", file=sys.stderr)
        sys.exit(1)
        
    robot_ip = sys.argv[1]
    ros_args = sys.argv[2:]
    
    tmp_file = f"/tmp/{robot_ip}"
    
    # 1. Clean up the stale file from previous runs
    initial_mtime = 0
    if os.path.exists(tmp_file):
        try:
            initial_mtime = os.path.getmtime(tmp_file)
            os.remove(tmp_file)
            print(f"[wait_and_launch_ydlidar] Removed stale file {tmp_file}", flush=True)
            initial_mtime = 0
        except Exception as e:
            print(f"[wait_and_launch_ydlidar] Warning: Could not remove {tmp_file}: {e}", flush=True)
            
    print(f"[wait_and_launch_ydlidar] Waiting for HBSDK to connect to {robot_ip} and write {tmp_file}...", flush=True)
    
    # 2. Wait for HBSDK to write the file containing the valid PTS
    pts_path = None
    timeout = 60.0  # 60 seconds timeout
    start_time = time.time()
    
    while time.time() - start_time < timeout:
        if os.path.exists(tmp_file):
            try:
                mtime = os.path.getmtime(tmp_file)
                # Ensure the file is new and not empty
                if mtime > initial_mtime and os.path.getsize(tmp_file) > 0:
                    with open(tmp_file, 'r') as f:
                        data = json.load(f)
                        if "p" in data and data["p"].startswith("/dev/pts/"):
                            candidate_pts = data["p"]
                            # Verify if the PTS device is accessible
                            if os.path.exists(candidate_pts):
                                pts_path = candidate_pts
                                print(f"[wait_and_launch_ydlidar] Found active PTS {pts_path} for IP {robot_ip} after {time.time() - start_time:.1f}s", flush=True)
                                break
            except Exception:
                # File might be partially written, retry in next iteration
                pass
        time.sleep(0.5)
        
    if not pts_path:
        print(f"[wait_and_launch_ydlidar] Error: Timeout ({timeout}s) waiting for PTS file {tmp_file}", file=sys.stderr, flush=True)
        sys.exit(1)
        
    # 3. Execute the actual ydlidar driver node binary
    node_path = os.path.expanduser("~/ROS2_WS/install/ydlidar_ros2_driver/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node")
    if not os.path.exists(node_path):
        import shutil
        found = shutil.which("ydlidar_ros2_driver_node")
        if found:
            node_path = found
        
    if not os.path.exists(node_path):
        print(f"[wait_and_launch_ydlidar] Error: Cannot find ydlidar_ros2_driver_node binary!", file=sys.stderr, flush=True)
        sys.exit(1)
        
    print(f"[wait_and_launch_ydlidar] Replacing process with {node_path}...", flush=True)
    sys.stdout.flush()
    sys.stderr.flush()
    
    # Replace the python process with the C++ binary
    os.execv(node_path, [node_path] + ros_args)

if __name__ == '__main__':
    main()
