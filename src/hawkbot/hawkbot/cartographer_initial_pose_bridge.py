import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from cartographer_ros_msgs.srv import FinishTrajectory, StartTrajectory
from rclpy.callback_groups import ReentrantCallbackGroup

class CartographerInitialPoseBridge(Node):
    def __init__(self):
        super().__init__('cartographer_initial_pose_bridge')
        
        # Declare parameters
        self.declare_parameter('robot_ns', '')
        self.robot_ns = self.get_parameter('robot_ns').get_parameter_value().string_value
        
        self.declare_parameter('configuration_directory', '')
        self.config_dir = self.get_parameter('configuration_directory').get_parameter_value().string_value
        
        self.declare_parameter('configuration_basename', '')
        self.config_basename = self.get_parameter('configuration_basename').get_parameter_value().string_value

        self.get_logger().info(f"Starting Cartographer Initial Pose Bridge for namespace: '{self.robot_ns}'")
        self.get_logger().info(f"Config dir: '{self.config_dir}', Config file: '{self.config_basename}'")

        # Current active trajectory ID for this robot (default started by node is 1)
        self.current_trajectory_id = 1
        
        self.callback_group = ReentrantCallbackGroup()

        # Service clients
        self.finish_trajectory_client = self.create_client(
            FinishTrajectory, 
            'finish_trajectory', # Resolves to namespace/finish_trajectory
            callback_group=self.callback_group
        )
        self.start_trajectory_client = self.create_client(
            StartTrajectory, 
            'start_trajectory', # Resolves to namespace/start_trajectory
            callback_group=self.callback_group
        )

        # Subscriber to 2D Pose Estimate from RViz
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            'initialpose', # Resolves to namespace/initialpose
            self.initial_pose_callback,
            10,
            callback_group=self.callback_group
        )

    async def initial_pose_callback(self, msg):
        self.get_logger().info(f"Received initial pose estimate: x={msg.pose.pose.position.x:.3f}, y={msg.pose.pose.position.y:.3f}")
        
        # 1. Stop the current running trajectory
        self.get_logger().info(f"Stopping current trajectory (ID {self.current_trajectory_id})...")
        if not self.finish_trajectory_client.service_is_ready():
            self.get_logger().error("FinishTrajectory service is not ready!")
            return
            
        finish_req = FinishTrajectory.Request()
        finish_req.trajectory_id = self.current_trajectory_id
        
        try:
            finish_res = await self.finish_trajectory_client.call_async(finish_req)
            self.get_logger().info(f"Trajectory {self.current_trajectory_id} stopped. Status: code={finish_res.status.code}, message='{finish_res.status.message}'")
        except Exception as e:
            self.get_logger().warn(f"FinishTrajectory call produced error/warning: {e}. Proceeding to start new trajectory...")

        # 2. Start a new trajectory with the new initial pose
        self.get_logger().info("Starting new trajectory with initial pose...")
        if not self.start_trajectory_client.service_is_ready():
            self.get_logger().error("StartTrajectory service is not ready!")
            return

        # Increment trajectory ID
        self.current_trajectory_id += 1

        start_req = StartTrajectory.Request()
        start_req.configuration_directory = self.config_dir
        start_req.configuration_basename = self.config_basename
        start_req.use_initial_pose = True
        # We pass the pose directly from the RViz message
        start_req.initial_pose = msg.pose.pose
        start_req.relative_to_trajectory_id = 0 # Relative to the pbstream map trajectory (ID 0)

        try:
            start_res = await self.start_trajectory_client.call_async(start_req)
            self.get_logger().info(f"New trajectory started (ID {start_res.trajectory_id}). Status: code={start_res.status.code}, message='{start_res.status.message}'")
            # Update our tracker to the newly assigned trajectory ID
            self.current_trajectory_id = start_res.trajectory_id
        except Exception as e:
            self.get_logger().error(f"Failed to start new trajectory: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CartographerInitialPoseBridge()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
