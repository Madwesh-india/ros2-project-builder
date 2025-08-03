def generate_python_code(config, indent_spaces=4):
    """Generate a Python ROS 2 node from configuration.
    
    Args:
        config: Dictionary containing node configuration
        indent_spaces: Number of spaces per indentation level (default: 4)
    """
    # Track current indentation level
    current_indent = 0
    
    def increase_indent():
        nonlocal current_indent
        current_indent += 1
        return ' ' * (current_indent * indent_spaces)
    
    def decrease_indent():
        nonlocal current_indent
        current_indent -= 1
        return ' ' * (current_indent * indent_spaces)
    
    def get_indent():
        return ' ' * (current_indent * indent_spaces)
    
    # Extract configuration
    node_name_class = config['node_name'].title().replace('_', '') + 'Node'
    node_name = config['node_name']
    publisher_configs = config['publisher_configs'] if config.get('publishers') else []
    subscriber_configs = config['subscriber_configs'] if config.get('subscribers') else []
    client_configs = config['client_configs'] if config.get('clients') else []
    service_configs = config['service_configs'] if config.get('services') else []
    action_server_configs = config['action_server_configs'] if config.get('action_servers') else []
    action_client_configs = config['action_client_configs'] if config.get('action_clients') else []
    timer_configs = config['timer_configs'] if config.get('timers') else []


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
    class_def = [
        f"class {node_name_class}(Node):",
        f"{increase_indent()}def __init__(self):",
        f"{increase_indent()}super().__init__('{node_name}')",
        f"{get_indent()}self.get_logger().info(f\"Initializing {{self.get_name()}}...\")",
        "",
    ]
    
    if config.get('executor') == 'multi_threaded':
        class_def.extend([
            f"{get_indent()}# Callback group for parallel execution",
            f"{get_indent()}self.callback_group = ReentrantCallbackGroup()",
            ""
        ])

    # Generate setup code
    setup_code = []
    
    def build_qos_profile(qos_config):
        """Build QoS profile string with proper indentation"""
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
        
        
        profile = [f"QoSProfile("]
        profile.append(f"{increase_indent()}depth={qos_config.get('depth', 10)},")
        profile.append(f"{get_indent()}reliability={reliability_map[qos_config['reliability']]},")
        profile.append(f"{get_indent()}durability={durability_map[qos_config['durability']]},")
        profile.append(f"{get_indent()}history={history_map[qos_config['history']]},")
        
        # Add advanced QoS settings if present
        if 'deadline' in qos_config:
            profile.append(f"{get_indent()}deadline=rclpy.time.Duration(seconds={qos_config['deadline']}),")
        if 'lifespan' in qos_config:
            profile.append(f"{get_indent()}lifespan=rclpy.time.Duration(seconds={qos_config['lifespan']}),")
        if 'liveliness' in qos_config:
            liveliness_map = {
                "automatic": "QoSLivelinessPolicy.AUTOMATIC",
                "manual_by_topic": "QoSLivelinessPolicy.MANUAL_BY_TOPIC"
            }
            profile.append(f"{get_indent()}liveliness={liveliness_map[qos_config['liveliness']]},")
        if 'liveliness_lease' in qos_config:
            profile.append(f"{get_indent()}liveliness_lease_duration=rclpy.time.Duration(seconds={qos_config['liveliness_lease']}),")
        
        profile.append(f"{decrease_indent()})")
        return '\n'.join(profile)
    
    # Create publishers
    for pub in publisher_configs:
        msg_type = pub['type'].split('/')[-1]
        setup_code.extend([
            f"{get_indent()}# Publisher: {pub['name']}",
            f"{get_indent()}self.{pub['name']}_pub = self.create_publisher(",
            f"{increase_indent()}{msg_type},",
            f"{get_indent()}'{pub['name']}',",
            f"{get_indent()}{build_qos_profile(pub['qos'])}",
            f"{decrease_indent()})",
            ""
        ])
    
    # Create subscribers
    for sub in subscriber_configs:
        msg_type = sub['type'].split('/')[-1]
        setup_code.extend([
            f"{get_indent()}# Subscriber: {sub['name']}",
            f"{get_indent()}self.{sub['name']}_sub = self.create_subscription(",
            f"{increase_indent()}{msg_type},",
            f"{get_indent()}'{sub['name']}',",
            f"{get_indent()}self.{sub['name']}_callback,",
            f"{get_indent()}{build_qos_profile(sub['qos'])},"+(f'\n{get_indent()}callback_group=self.callback_group' if config.get('executor') == 'multi_threaded' else ''),
            f"{decrease_indent()})",
            ""
        ])
    
    # Create service clients
    for client in client_configs:
        srv_type = client['type'].split('/')[-1]
        setup_code.extend([
            f"{get_indent()}# Service client: {client['name']}",
            f"{get_indent()}self.{client['name']}_client = self.create_client(",
            f"{increase_indent()}{srv_type},",
            f"{get_indent()}'{client['name']}',"+(f'\n{get_indent()}callback_group=self.callback_group' if config.get('executor') == 'multi_threaded' else ''),
            f"{decrease_indent()})",
            ""
        ])
    
    # Create service servers
    for srv in service_configs:
        srv_type = srv['type'].split('/')[-1]
        setup_code.extend([
            f"{get_indent()}# Service server: {srv['name']}",
            f"{get_indent()}self.{srv['name']}_srv = self.create_service(",
            f"{increase_indent()}{srv_type},",
            f"{get_indent()}'{srv['name']}',",
            f"{get_indent()}self.{srv['name']}_callback,"+(f'\n{get_indent()}callback_group=self.callback_group' if config.get('executor') == 'multi_threaded' else ''),
            f"{decrease_indent()})",
            ""
        ])

    # Create action servers
    for action in action_server_configs:
        action_type = action['type'].split('/')[-1]
        setup_code.extend([
            f"{get_indent()}# Action server: {action['name']}",
            f"{get_indent()}self.{action['name']}_action_server = ActionServer(",
            f"{increase_indent()}self,",
            f"{get_indent()}{action_type},",
            f"{get_indent()}'{action['name']}',",
            f"{get_indent()}execute_callback=self.execute_{action['name']}_callback,"+(f'\n{get_indent()}callback_group=self.callback_group,' if config.get('executor') == 'multi_threaded' else ''),
            f"{get_indent()}goal_callback=self.handle_{action['name']}_goal,",
            f"{get_indent()}cancel_callback=self.handle_{action['name']}_cancel,",
            # f"{get_indent()}result_service_qos_profile={build_qos_profile(action['qos'])},",
            # f"{get_indent()}cancel_service_qos_profile={build_qos_profile(action['qos'])},",
            # f"{get_indent()}goal_service_qos_profile={build_qos_profile(action['qos'])},",
            # f"{get_indent()}feedback_pub_qos_profile={build_qos_profile(action['qos'])},",
            f"{decrease_indent()})",
            ""
        ])
    
    # Create action clients
    for action in action_client_configs:
        action_type = action['type'].split('/')[-1]
        setup_code.extend([
            f"{get_indent()}# Action client: {action['name']}",
            f"{get_indent()}self.{action['name']}_action_client = ActionClient(",
            f"{increase_indent()}self,",
            f"{get_indent()}{action_type},",
            f"{get_indent()}'{action['name']}',"+(f'\n{get_indent()}callback_group=self.callback_group,' if config.get('executor') == 'multi_threaded' else ''),
            # f"{get_indent()}result_service_qos_profile={build_qos_profile(action['qos'])},",
            # f"{get_indent()}cancel_service_qos_profile={build_qos_profile(action['qos'])},",
            # f"{get_indent()}goal_service_qos_profile={build_qos_profile(action['qos'])},",
            # f"{get_indent()}feedback_sub_qos_profile={build_qos_profile(action['qos'])},",
            f"{decrease_indent()})",
            ""
        ])
    
    # Create timers
    for timer in timer_configs:
        setup_code.extend([
            f"{get_indent()}# Timer: {timer['name']}",
            f"{get_indent()}self.{timer['name']}_timer = self.create_timer(",
            f"{increase_indent()}{timer['period']},",
            f"{get_indent()}self.{timer['name']}_callback,"+(f'\n{get_indent()}callback_group=self.callback_group,' if config.get('executor') == 'multi_threaded' else ''),
            f"{decrease_indent()})",
            ""
        ])
    
    setup_code.append(f"{decrease_indent()}")

    # Generate callback stubs
    callbacks = []
    
    # Subscriber callbacks
    for sub in subscriber_configs:
        callbacks.extend([
            f"{get_indent()}def {sub['name']}_callback(self, msg):",
            f"{increase_indent()}self.get_logger().info(f'Received message on {sub['name']}')",
            f"{get_indent()}# Implement {sub['name']} subscriber callback",
            f"{decrease_indent()}",
            ""
        ])
    
    # Service callbacks
    for srv in service_configs:
        callbacks.extend([
            f"{get_indent()}def {srv['name']}_callback(self, request, response):",
            f"{increase_indent()}self.get_logger().info(f'Service {srv['name']} called')",
            f"{get_indent()}# Implement {srv['name']} service callback",
            f"{get_indent()}return response",
            f"{decrease_indent()}",
            ""
        ])
    
    # Action server callbacks
    for action in action_server_configs:
        callbacks.extend([
            f"{get_indent()}def handle_{action['name']}_goal(self, goal_request):",
            f"{increase_indent()}self.get_logger().info('Received goal request for {action['name']}')",
            f"{get_indent()}return rclpy.action.GoalResponse.ACCEPT",
            f"{decrease_indent()}",
            "",
            f"{get_indent()}def handle_{action['name']}_cancel(self, goal_handle):",
            f"{increase_indent()}self.get_logger().info('Received cancel request for {action['name']}')",
            f"{get_indent()}return rclpy.action.CancelResponse.ACCEPT",
            f"{decrease_indent()}",
            "",
            f"{get_indent()}async def execute_{action['name']}_callback(self, goal_handle):",
            f"{increase_indent()}self.get_logger().info('Executing goal for {action['name']}')",
            f"{get_indent()}# Implement action execution",
            f"{get_indent()}result = {action['type'].split('/')[-1]}.Result()",
            f"{get_indent()}return result",
            f"{decrease_indent()}",
            ""
        ])
    
    # Timer callbacks
    for timer in timer_configs:
        callbacks.extend([
            f"{get_indent()}def {timer['name']}_callback(self):",
            f"{increase_indent()}self.get_logger().info('{timer['name']} timer triggered')",
            f"{get_indent()}# Implement {timer['name']} timer callback",
            f"{decrease_indent()}",
            ""
        ])

    callbacks.append(f"{decrease_indent()}")

    main_func = [
        "def main(args=None):",
        f"{increase_indent()}rclpy.init(args=args)",
        f"{get_indent()}try:",
        f"{increase_indent()}node = {node_name_class}()"
    ]

    # If executor is multi-threaded, insert related lines
    if config.get("executor") == "multi_threaded":
        main_func += [
            f"{get_indent()}# Use multi-threaded executor if specified",
            f"{get_indent()}executor = rclpy.executors.MultiThreadedExecutor()",
            f"{get_indent()}executor.add_node(node)",
            f"{get_indent()}executor.spin()"
        ]
    else:
        main_func.append(f"{get_indent()}rclpy.spin(node)")

    # Continue with the rest of the function
    main_func += [
        f"{decrease_indent()}except KeyboardInterrupt:",
        f"{increase_indent()}pass",
        f"{decrease_indent()}finally:",
        f"{increase_indent()}node.destroy_node()",
        f"{get_indent()}rclpy.shutdown()",
        f"{decrease_indent()}",
        f"{decrease_indent()}",
        "if __name__ == '__main__':",
        f"{increase_indent()}main()"
    ]

    
    # Combine all parts
    result = [
        imports_str,
        "",
        '\n'.join(class_def),
        '\n'.join(setup_code),
        '\n'.join(callbacks),
        '\n'.join(main_func)
    ]
    
    return '\n'.join(result)