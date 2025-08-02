#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan
from std_srvs.srv import SetBool


class DriveNode(Node):
    def __init__(self):
        super().__init__('drive')
        self.get_logger().info(f"Initializing {self.get_name()}...")
        
        # Callback group for parallel execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Publisher: odom
        self.odom_pub = self.create_publisher(
            OccupancyGrid,
            'odom',
            QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST
)
        )

        # Subscriber: cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    deadline=rclpy.time.Duration(seconds=1.0),
    lifespan=rclpy.time.Duration(seconds=10.0),
    liveliness=QoSLivelinessPolicy.AUTOMATIC,
    liveliness_lease_duration=rclpy.time.Duration(seconds=100.0)
),
            callback_group=self.callback_group
        )

        # Service client: get_path
        self.get_path_client = self.create_client(
            GetPlan,
            'get_path',
            callback_group=self.callback_group
        )

        # Service server: lights
        self.lights_srv = self.create_service(
            SetBool,
            'lights',
            self.lights_callback,
            callback_group=self.callback_group
        )

        # Action server: path_feedback
        self.path_feedback_action_server = ActionServer(
            self,
            FollowPath,
            'path_feedback',
            execute_callback=self.execute_path_feedback_callback,
            callback_group=self.callback_group,
            goal_callback=self.handle_path_feedback_goal,
            cancel_callback=self.handle_path_feedback_cancel,
            result_service_qos_profile=QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST
),
            cancel_service_qos_profile=QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST
),
            goal_service_qos_profile=QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST
),
            feedback_pub_qos_profile=QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST
),
        )

        # Action client: path
        self.path_action_client = ActionClient(
            self,
            FollowWaypoints,
            'path',
            callback_group=self.callback_group,
            result_service_qos_profile=QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST
),
            cancel_service_qos_profile=QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST
),
            goal_service_qos_profile=QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST
),
            feedback_sub_qos_profile=QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST
),
        )

        # Timer: timer_1
        self.timer_1_timer = self.create_timer(
            1.0,
            self.timer_1_callback,
            callback_group=self.callback_group
        )

    def cmd_vel_callback(self, msg):
        self.get_logger().info(f'Received message on cmd_vel')
        # Implement cmd_vel subscriber callback

    def lights_callback(self, request, response):
        self.get_logger().info(f'Service lights called')
        # Implement lights service callback
        return response

    def handle_path_feedback_goal(self, goal_request):
        self.get_logger().info('Received goal request for path_feedback')
        return rclpy.action.GoalResponse.ACCEPT

    def handle_path_feedback_cancel(self, goal_handle):
        self.get_logger().info('Received cancel request for path_feedback')
        return rclpy.action.CancelResponse.ACCEPT

    async def execute_path_feedback_callback(self, goal_handle):
        self.get_logger().info('Executing goal for path_feedback')
        # Implement action execution
        result = FollowPath.Result()
        return result

    def timer_1_callback(self):
        self.get_logger().info('timer_1 timer triggered')
        # Implement timer_1 timer callback


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = DriveNode()
        
        # Use multi-threaded executor if specified
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
