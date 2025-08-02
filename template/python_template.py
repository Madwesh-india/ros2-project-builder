def generate_python_node(config):
    # Extract configuration
    node_name_class = config['node_name'].title().replace('_', '') + 'Node'
    node_name = config['node_name']
    publisher_configs = config['publisher_configs']
    subscriber_configs = config['subscriber_configs']
    client_configs = config['client_configs']
    service_configs = config['service_configs']
    action_server_configs = config['action_server_configs']
    action_client_configs = config['action_client_configs']
    timer_configs = config['timer_configs']

    # Generate imports
    imports = [
        "#!/usr/bin/env python3",
        "import rclpy",
        "from rclpy.node import Node",
        "from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy, QoSLivelinessPolicy",
        "from rclpy.action import ActionServer, ActionClient",
        "from rclpy.callback_groups import ReentrantCallbackGroup",
    ]
    
    # Add message imports
    message_types = set()
    for pub in publisher_configs:
        parts = pub['type'].split('/')
        message_types.add(f"from {parts[0]}.msg import {parts[2]}")
    for sub in subscriber_configs:
        parts = sub['type'].split('/')
        message_types.add(f"from {parts[0]}.msg import {parts[2]}")
    for client in client_configs:
        parts = client['type'].split('/')
        message_types.add(f"from {parts[0]}.srv import {parts[2]}")
    for srv in service_configs:
        parts = srv['type'].split('/')
        message_types.add(f"from {parts[0]}.srv import {parts[2]}")
    for action in action_server_configs + action_client_configs:
        parts = action['type'].split('/')
        message_types.add(f"from {parts[0]}.action import {parts[2]}")
    
    imports.extend(sorted(message_types))
    imports_str = '\n'.join(imports)
    
    # Generate class definition
    class_def = f"""
class {node_name_class}(Node):
    def __init__(self):
        super().__init__('{node_name}')
        self.get_logger().info(f"Initializing {{self.get_name()}}...")
        
        # Callback group for parallel execution
        self.callback_group = ReentrantCallbackGroup()
        """
    
    # Generate setup code
    setup_code = []
    
    # Create publishers
    for pub in publisher_configs:
        msg_type = pub['type'].split('/')[-1]
        qos_profile = _build_qos_profile(pub['qos'])
        setup_code.append(f"        # Publisher: {pub['name']}")
        setup_code.append(f"        self.{pub['name']}_pub = self.create_publisher(")
        setup_code.append(f"            {msg_type},")
        setup_code.append(f"            '{pub['name']}',")
        setup_code.append(f"            {qos_profile}")
        setup_code.append(f"        )")
        setup_code.append("")
    
    # Create subscribers
    for sub in subscriber_configs:
        msg_type = sub['type'].split('/')[-1]
        qos_profile = _build_qos_profile(sub['qos'])
        setup_code.append(f"        # Subscriber: {sub['name']}")
        setup_code.append(f"        self.{sub['name']}_sub = self.create_subscription(")
        setup_code.append(f"            {msg_type},")
        setup_code.append(f"            '{sub['name']}',")
        setup_code.append(f"            self.{sub['name']}_callback,")
        setup_code.append(f"            {qos_profile},")
        setup_code.append(f"            callback_group=self.callback_group")
        setup_code.append(f"        )")
        setup_code.append("")
    
    # Create service clients
    for client in client_configs:
        srv_type = client['type'].split('/')[-1]
        setup_code.append(f"        # Service client: {client['name']}")
        setup_code.append(f"        self.{client['name']}_client = self.create_client(")
        setup_code.append(f"            {srv_type},")
        setup_code.append(f"            '{client['name']}',")
        setup_code.append(f"            callback_group=self.callback_group")
        setup_code.append(f"        )")
        setup_code.append("")
    
    # Create service servers
    for srv in service_configs:
        srv_type = srv['type'].split('/')[-1]
        setup_code.append(f"        # Service server: {srv['name']}")
        setup_code.append(f"        self.{srv['name']}_srv = self.create_service(")
        setup_code.append(f"            {srv_type},")
        setup_code.append(f"            '{srv['name']}',")
        setup_code.append(f"            self.{srv['name']}_callback,")
        setup_code.append(f"            callback_group=self.callback_group")
        setup_code.append(f"        )")
        setup_code.append("")

    # Create action servers
    for action in action_server_configs:
        action_type = action['type'].split('/')[-1]
        qos_profile = _build_qos_profile(action['qos'])
        setup_code.append(f"        # Action server: {action['name']}")
        setup_code.append(f"        self.{action['name']}_action_server = ActionServer(")
        setup_code.append(f"            self,")
        setup_code.append(f"            {action_type},")
        setup_code.append(f"            '{action['name']}',")
        setup_code.append(f"            execute_callback=self.execute_{action['name']}_callback,")
        setup_code.append(f"            callback_group=self.callback_group,")
        setup_code.append(f"            goal_callback=self.handle_{action['name']}_goal,")
        setup_code.append(f"            cancel_callback=self.handle_{action['name']}_cancel,")
        setup_code.append(f"            result_service_qos_profile={qos_profile},")
        setup_code.append(f"            cancel_service_qos_profile={qos_profile},")
        setup_code.append(f"            goal_service_qos_profile={qos_profile},")
        setup_code.append(f"            feedback_pub_qos_profile={qos_profile},")
        setup_code.append(f"        )")
        setup_code.append("")
    
    # Create action clients
    for action in action_client_configs:
        action_type = action['type'].split('/')[-1]
        qos_profile = _build_qos_profile(action['qos'])
        setup_code.append(f"        # Action client: {action['name']}")
        setup_code.append(f"        self.{action['name']}_action_client = ActionClient(")
        setup_code.append(f"            self,")
        setup_code.append(f"            {action_type},")
        setup_code.append(f"            '{action['name']}',")
        setup_code.append(f"            callback_group=self.callback_group,")
        setup_code.append(f"            result_service_qos_profile={qos_profile},")
        setup_code.append(f"            cancel_service_qos_profile={qos_profile},")
        setup_code.append(f"            goal_service_qos_profile={qos_profile},")
        setup_code.append(f"            feedback_sub_qos_profile={qos_profile},")
        setup_code.append(f"        )")
        setup_code.append("")
    
    # Create timers
    for timer in timer_configs:
        setup_code.append(f"        # Timer: {timer['name']}")
        setup_code.append(f"        self.{timer['name']}_timer = self.create_timer(")
        setup_code.append(f"            {timer['period']},")
        setup_code.append(f"            self.{timer['name']}_callback,")
        setup_code.append(f"            callback_group=self.callback_group")
        setup_code.append(f"        )")
        setup_code.append("")
    
    setup_str = '\n'.join(setup_code)
    
    # Generate callback stubs
    callbacks = []
    
    # Subscriber callbacks
    for sub in subscriber_configs:
        msg_type = sub['type'].split('/')[-1]
        callbacks.append(f"    def {sub['name']}_callback(self, msg):")
        callbacks.append(f"        self.get_logger().info(f'Received message on {sub['name']}')")
        callbacks.append(f"        # Implement {sub['name']} subscriber callback")
        callbacks.append("")
    
    # Service callbacks
    for srv in service_configs:
        srv_type = srv['type'].split('/')[-1]
        callbacks.append(f"    def {srv['name']}_callback(self, request, response):")
        callbacks.append(f"        self.get_logger().info(f'Service {srv['name']} called')")
        callbacks.append(f"        # Implement {srv['name']} service callback")
        callbacks.append(f"        return response")
        callbacks.append("")
    
    # Action server callbacks
    for action in action_server_configs:
        action_type = action['type'].split('/')[-1]
        
        # Goal callback
        callbacks.append(f"    def handle_{action['name']}_goal(self, goal_request):")
        callbacks.append(f"        self.get_logger().info('Received goal request for {action['name']}')")
        callbacks.append(f"        return rclpy.action.GoalResponse.ACCEPT")
        callbacks.append("")
        
        # Cancel callback
        callbacks.append(f"    def handle_{action['name']}_cancel(self, goal_handle):")
        callbacks.append(f"        self.get_logger().info('Received cancel request for {action['name']}')")
        callbacks.append(f"        return rclpy.action.CancelResponse.ACCEPT")
        callbacks.append("")
        
        # Execute callback
        callbacks.append(f"    async def execute_{action['name']}_callback(self, goal_handle):")
        callbacks.append(f"        self.get_logger().info('Executing goal for {action['name']}')")
        callbacks.append(f"        # Implement action execution")
        callbacks.append(f"        result = {action_type}.Result()")
        callbacks.append(f"        return result")
        callbacks.append("")
    
    # Timer callbacks
    for timer in timer_configs:
        callbacks.append(f"    def {timer['name']}_callback(self):")
        callbacks.append(f"        self.get_logger().info('{timer['name']} timer triggered')")
        callbacks.append(f"        # Implement {timer['name']} timer callback")
        callbacks.append("")
    
    callbacks_str = '\n'.join(callbacks)
    
    # Generate main function
    main_func = f"""
def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = {node_name_class}()
        
        # Use multi-threaded executor if specified
        {"executor = rclpy.executors.MultiThreadedExecutor()" if config.get('executor') == 'multi_threaded' else "executor = None"}
        {"executor.add_node(node)" if config.get('executor') == 'multi_threaded' else ""}
        
        {"executor.spin()" if config.get('executor') == 'multi_threaded' else "rclpy.spin(node)"}
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
    
    return f"{imports_str}\n\n{class_def}\n{setup_str}\n{callbacks_str}\n{main_func}"


def _build_qos_profile(qos_config):
    """Build QoS profile string from configuration"""
    if not qos_config:
        return "10  # Default depth"
    
    # Map string values to QoS policy constants
    reliability_map = {
        "reliable": "QoSReliabilityPolicy.RELIABLE",
        "best_effort": "QoSReliabilityPolicy.BEST_EFFORT"
    }
    
    durability_map = {
        "volatile": "QoSDurabilityPolicy.VOLATILE",
        "transient_local": "QoSDurabilityPolicy.TRANSIENT_LOCAL"
    }
    
    history_map = {
        "keep_last": "QoSHistoryPolicy.KEEP_LAST",
        "keep_all": "QoSHistoryPolicy.KEEP_ALL"
    }
    
    profile = f"QoSProfile(\n"
    profile += f"    depth={qos_config.get('depth', 10)},\n"
    profile += f"    reliability={reliability_map[qos_config['reliability']]},\n"
    profile += f"    durability={durability_map[qos_config['durability']]},\n"
    profile += f"    history={history_map[qos_config['history']]}"
    
    # Add advanced QoS settings if present
    if 'deadline' in qos_config:
        profile += f",\n    deadline=rclpy.time.Duration(seconds={qos_config['deadline']})"
    if 'lifespan' in qos_config:
        profile += f",\n    lifespan=rclpy.time.Duration(seconds={qos_config['lifespan']})"
    if 'liveliness' in qos_config:
        liveliness_map = {
            "automatic": "QoSLivelinessPolicy.AUTOMATIC",
            "manual_by_topic": "QoSLivelinessPolicy.MANUAL_BY_TOPIC"
        }
        profile += f",\n    liveliness={liveliness_map[qos_config['liveliness']]}"
    if 'liveliness_lease' in qos_config:
        profile += f",\n    liveliness_lease_duration=rclpy.time.Duration(seconds={qos_config['liveliness_lease']})"
    
    profile += "\n)"
    return profile
