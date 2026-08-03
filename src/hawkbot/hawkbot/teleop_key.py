#!/usr/bin/env python3
# encoding: utf-8

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

msg = """
======================
   u    i    o  
   j    k    l  
   m    ,    .  
======================
前: i       后: ,
左: j       右: l
左转弯：u   右转弯：o
左后退：m   右后退：.
增加/减少线速度: w/x
增加/减少角速度: e/c
退出: CTRL-C
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
}

speedBindings = {
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return f"当前速度:\tspeed {speed:.2f}\tturn {turn:.2f}"

class TeleopNode(Node):
    def __init__(self, robot_id='null'):
        super().__init__('teleop_twist_keyboard')
        topic = 'cmd_vel' if robot_id == 'null' else f'/{robot_id}/cmd_vel'
        self.publisher = self.create_publisher(Twist, topic, 10)
        self.speed = self.declare_parameter('speed', 0.11).get_parameter_value().double_value
        self.turn = self.declare_parameter('turn', 0.75).get_parameter_value().double_value
        self.x = self.y = self.z = self.th = 0.0
        self.status = 0

        print(msg)
        print(vels(self.speed, self.turn))
        self.settings = termios.tcgetattr(sys.stdin)
        self.run()

    def run(self):
        try:
            while True:
                key = getKey(self.settings)
                if key in moveBindings:
                    self.x, self.y, self.z, self.th = moveBindings[key]
                elif key in speedBindings:
                    self.speed *= speedBindings[key][0]
                    self.turn *= speedBindings[key][1]
                    print(vels(self.speed, self.turn))
                    if self.status == 14:
                        print(msg)
                    self.status = (self.status + 1) % 15
                else:
                    self.x = self.y = self.z = self.th = 0.0
                    if key == '\x03':  # Ctrl-C
                        break

                twist = Twist()
                twist.linear.x = self.x * self.speed
                twist.linear.y = self.y * self.speed
                twist.linear.z = self.z * self.speed
                twist.angular.z = self.th * self.turn
                self.publisher.publish(twist)

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            self.publisher.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main():
    rclpy.init()
    robot_id = 'null'
    for arg in sys.argv[1:]:
        if not arg.startswith('-') and not arg.startswith('__'):
            robot_id = arg
            break
    node = TeleopNode(robot_id)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
