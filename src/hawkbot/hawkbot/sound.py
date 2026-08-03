import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# sound_str = "523:600,587:600,659:600," +  "659:400,698:400,784:400," +"784:400,880:400,988:400," +  "659:400,698:400,784:400," +"784:200,880:200,988:400," +"880:400,784:600,659:1000"

#生日快乐
sound_str1="392:400,392:400,440:700,392:700,523:700,493:1000,392:400,392:400,440:700,392:700,587:700,523:1000";
#小星星
sound_str2 = "261:400,261:400,392:400,392:400,440:400,440:400,392:800,349:400,349:400,329:400,329:400,293:400,293:400,261:800,392:400,392:400,349:400,349:400,329:400,329:400,293:800,392:400,392:400,349:400,349:400,329:400,329:400,293:800,261:400,261:400,392:400,392:400,440:400,440:400,392:800,349:400,349:400,329:400,329:400,293:400,293:400,261:800"


class PublisherNode(Node):

    def __init__(self):
        super().__init__('sound_publisher')
        self.publisher = self.create_publisher(String, 'sound', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = sound_str1
        self.publisher.publish(msg)
        time.sleep(5)
        # self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()