#ros lib
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool,UInt16,String
#commom lib
import os
import sys
import math
import numpy as np
import time
from time import sleep
from hawkbotcar_laser.common import *
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

RAD2DEG = 360 / math.pi

class laserWarning(Node):
    def __init__(self,name):
        super().__init__(name)
        #create a sub
        self.sub_laser = self.create_subscription(LaserScan,"/scan",self.registerScan, rclpy.qos.qos_profile_sensor_data)
        #create a pub

        self.pub_Buzzer = self.create_publisher(String,'sound',1)
        

        #declareparam
        self.declare_parameter("LaserAngle",360.0)
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.declare_parameter("ResponseDist",0.3)
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value
        self.declare_parameter("Switch",False)
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        
        self.Right_warning = 0
        self.Left_warning = 0
        self.front_warning = 0
        self.Joy_active = False
        self.ros_ctrl = SinglePID()
        self.ang_pid = SinglePID(3.0, 0.0, 5.0)
        self.Buzzer_state = False
        self.Moving = False

        self.last_publish_time = self.get_clock().now()
        self.publish_interval = rclpy.duration.Duration(seconds=2)
        
        self.timer = self.create_timer(0.01,self.on_timer)
        
    def on_timer(self):
        self.Switch = self.get_parameter('Switch').get_parameter_value().bool_value
        self.LaserAngle = self.get_parameter('LaserAngle').get_parameter_value().double_value
        self.ResponseDist = self.get_parameter('ResponseDist').get_parameter_value().double_value



    def registerScan(self, scan_data):
        if not isinstance(scan_data, LaserScan): return
        if self.Switch == True:
            return

        ranges = np.array(scan_data.ranges)
        minDistList = []
        minDistIDList = []
        
        for i in range(len(ranges)):
            angle = (scan_data.angle_min + scan_data.angle_increment * i) * RAD2DEG
            if angle > 180: angle = angle - 360
            if  abs(angle) < self.LaserAngle and ranges[i] > 0:
            	minDistList.append(ranges[i])
            	minDistIDList.append(angle)
        if len(minDistList) != 0: 
        	minDist = min(minDistList)
        	minDistID = minDistIDList[minDistList.index(minDist)]
        else:
        	return

        print("minDist: ", minDist)
        current_time = self.get_clock().now()
        if minDist <= self.ResponseDist:
            if (current_time - self.last_publish_time) > self.publish_interval:
                print("---------------")
                msg = String()
                msg.data = "2000:800"
                self.pub_Buzzer.publish(msg)
                self.last_publish_time = current_time

        else:
        	print("no obstacles@")
        	
        

def main():
    rclpy.init()
    laser_warn = laserWarning("laser_Warnning")
    print ("start it")
    try:
        rclpy.spin(laser_warn)
    except KeyboardInterrupt:
        pass
    finally:
        laser_warn.destroy_node()
        rclpy.shutdown()
