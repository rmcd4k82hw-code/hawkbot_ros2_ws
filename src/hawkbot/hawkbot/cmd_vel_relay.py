import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Subscribe to robot1's control commands
        self.subscription = self.create_subscription(
            Twist,
            '/robot1/cmd_vel',
            self.listener_callback,
            10
        )
        
        # Publish to robot2's control commands
        self.publisher = self.create_publisher(
            Twist,
            '/robot2/cmd_vel',
            10
        )
        
        self.get_logger().info('Command Velocity Relay Node Started.')
        self.get_logger().info('Relaying /robot1/cmd_vel to /robot2/cmd_vel...')

    def listener_callback(self, msg):
        # Forward the message directly
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
