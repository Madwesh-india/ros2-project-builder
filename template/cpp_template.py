def generate_cpp_node(config):
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
    imports = ['#include "rclcpp/rclcpp.hpp"', '#include "rclcpp_action/rclcpp_action.hpp"']
    
    # Add message includes
    message_types = set()
    for pub in publisher_configs:
        parts = pub['type'].split('/')
        message_types.add(f'{parts[0]}/msg/{parts[2]}')
    for sub in subscriber_configs:
        parts = sub['type'].split('/')
        message_types.add(f'{parts[0]}/msg/{parts[2]}')
    for client in client_configs:
        parts = client['type'].split('/')
        message_types.add(f'{parts[0]}/srv/{parts[2]}')
    for srv in service_configs:
        parts = srv['type'].split('/')
        message_types.add(f'{parts[0]}/srv/{parts[2]}')
    for action in action_server_configs + action_client_configs:
        parts = action['type'].split('/')
        message_types.add(f'{parts[0]}/action/{parts[2]}')
    
    for msg_type in message_types:
        imports.append(f'#include "{msg_type}.hpp"')
    
    imports_str = '\n'.join(sorted(set(imports)))
    
    # Generate setup code
    setup_code = []
    qos_profiles = {}
    
    # Create QoS profiles
    for i, pub in enumerate(publisher_configs):
        qos_name = f"{pub['name']}_qos"
        qos_profiles[pub['name']] = qos_name
        setup_code.append(f"// QoS profile for {pub['name']} publisher")
        setup_code.append(f"auto {qos_name} = rclcpp::QoS({pub['qos']['depth']})")
        setup_code.append(f"  .reliability(rclcpp::QoSReliabilityPolicy::{pub['qos']['reliability']})")
        setup_code.append(f"  .durability(rclcpp::QoSDurabilityPolicy::{pub['qos']['durability']})")
        setup_code.append(f"  .history(rclcpp::QoSHistoryPolicy::{pub['qos']['history']});")
        
        # Add advanced QoS settings if present
        for key in ['deadline', 'lifespan', 'liveliness_lease']:
            if key in pub['qos']:
                duration = float(pub['qos'][key])
                setup_code.append(f"  {qos_name}.{key}(std::chrono::duration<double>({duration}));")
        if 'liveliness' in pub['qos']:
            setup_code.append(f"  {qos_name}.liveliness(rclcpp::QoSLivelinessPolicy::{pub['qos']['liveliness']});")
        setup_code.append("")
    
    # Create publishers
    for pub in publisher_configs:
        msg_type = pub['type'].replace('/', '::msg::')
        setup_code.append(f"{pub['name']}_pub = this->create_publisher<{msg_type}>(\"{pub['name']}\", {qos_profiles[pub['name']]});")
    
    setup_code.append("")
    
    # Create subscribers
    for sub in subscriber_configs:
        msg_type = sub['type'].replace('/', '::msg::')
        qos_name = f"{sub['name']}_qos"
        qos_profiles[sub['name']] = qos_name
        
        setup_code.append(f"// QoS profile for {sub['name']} subscriber")
        setup_code.append(f"auto {qos_name} = rclcpp::QoS({sub['qos']['depth']})")
        setup_code.append(f"  .reliability(rclcpp::QoSReliabilityPolicy::{sub['qos']['reliability']})")
        setup_code.append(f"  .durability(rclcpp::QoSDurabilityPolicy::{sub['qos']['durability']})")
        setup_code.append(f"  .history(rclcpp::QoSHistoryPolicy::{sub['qos']['history']});")
        
        # Add advanced QoS settings
        for key in ['deadline', 'lifespan', 'liveliness_lease']:
            if key in sub['qos']:
                duration = float(sub['qos'][key])
                setup_code.append(f"  {qos_name}.{key}(std::chrono::duration<double>({duration}));")
        if 'liveliness' in sub['qos']:
            setup_code.append(f"  {qos_name}.liveliness(rclcpp::QoSLivelinessPolicy::{sub['qos']['liveliness']});")
        
        setup_code.append(f"{sub['name']}_sub = this->create_subscription<{msg_type}>(")
        setup_code.append(f"  \"{sub['name']}\", {qos_name},")
        setup_code.append(f"  std::bind(&{node_name_class}::{sub['name']}_callback, this, std::placeholders::_1));")
        setup_code.append("")
    
    # Create service clients
    for client in client_configs:
        srv_type = client['type'].replace('/', '::srv::')
        setup_code.append(f"{client['name']}_client = this->create_client<{srv_type}>(\"{client['name']}\");")
    
    setup_code.append("")
    
    # Create service servers
    for srv in service_configs:
        srv_type = srv['type'].replace('/', '::srv::')
        setup_code.append(f"{srv['name']}_srv = this->create_service<{srv_type}>(")
        setup_code.append(f"  \"{srv['name']}\",")
        setup_code.append(f"  std::bind(&{node_name_class}::{srv['name']}_callback, this,")
        setup_code.append("    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));")
        setup_code.append("")
    
    # Create action servers
    for action in action_server_configs:
        action_type = action['type'].replace('/', '::action::')
        qos_name = f"{action['name']}_qos"
        
        setup_code.append(f"// QoS profile for {action['name']} action server")
        setup_code.append(f"auto {qos_name} = rclcpp::QoS({action['qos']['depth']})")
        setup_code.append(f"  .reliability(rclcpp::QoSReliabilityPolicy::{action['qos']['reliability']})")
        setup_code.append(f"  .durability(rclcpp::QoSDurabilityPolicy::{action['qos']['durability']})")
        setup_code.append(f"  .history(rclcpp::QoSHistoryPolicy::{action['qos']['history']});")
        
        setup_code.append(f"{action['name']}_action_server = rclcpp_action::create_server<{action_type}>(")
        setup_code.append(f"  this, \"{action['name']}\",")
        setup_code.append(f"  std::bind(&{node_name_class}::handle_{action['name']}_goal, this, std::placeholders::_1, std::placeholders::_2),")
        setup_code.append(f"  std::bind(&{node_name_class}::handle_{action['name']}_cancel, this, std::placeholders::_1),")
        setup_code.append(f"  std::bind(&{node_name_class}::handle_{action['name']}_accepted, this, std::placeholders::_1));")
        setup_code.append("")
    
    # Create action clients
    for action in action_client_configs:
        action_type = action['type'].replace('/', '::action::')
        qos_name = f"{action['name']}_qos"
        
        setup_code.append(f"// QoS profile for {action['name']} action client")
        setup_code.append(f"auto {qos_name} = rclcpp::QoS({action['qos']['depth']})")
        setup_code.append(f"  .reliability(rclcpp::QoSReliabilityPolicy::{action['qos']['reliability']})")
        setup_code.append(f"  .durability(rclcpp::QoSDurabilityPolicy::{action['qos']['durability']})")
        setup_code.append(f"  .history(rclcpp::QoSHistoryPolicy::{action['qos']['history']});")
        
        setup_code.append(f"{action['name']}_action_client = rclcpp_action::create_client<{action_type}>(this, \"{action['name']}\");")
        setup_code.append("")
    
    # Create timers
    for timer in timer_configs:
        setup_code.append(f"{timer['name']}_timer = this->create_wall_timer(")
        setup_code.append(f"  std::chrono::duration<double>({timer['period']}),")
        setup_code.append(f"  std::bind(&{node_name_class}::{timer['name']}_callback, this));")
        setup_code.append("")
    
    setup_str = '\n'.join(setup_code)
    
    # Generate variable declarations
    variables = []
    
    # Publishers
    for pub in publisher_configs:
        msg_type = pub['type'].replace('/', '::msg::')
        variables.append(f"rclcpp::Publisher<{msg_type}>::SharedPtr {pub['name']}_pub;")
    
    # Subscribers
    for sub in subscriber_configs:
        msg_type = sub['type'].replace('/', '::msg::')
        variables.append(f"rclcpp::Subscription<{msg_type}>::SharedPtr {sub['name']}_sub;")
    
    # Service clients
    for client in client_configs:
        srv_type = client['type'].replace('/', '::srv::')
        variables.append(f"rclcpp::Client<{srv_type}>::SharedPtr {client['name']}_client;")
    
    # Service servers
    for srv in service_configs:
        srv_type = srv['type'].replace('/', '::srv::')
        variables.append(f"rclcpp::Service<{srv_type}>::SharedPtr {srv['name']}_srv;")
    
    # Action servers
    for action in action_server_configs:
        action_type = action['type'].replace('/', '::action::')
        variables.append(f"rclcpp_action::Server<{action_type}>::SharedPtr {action['name']}_action_server;")
    
    # Action clients
    for action in action_client_configs:
        action_type = action['type'].replace('/', '::action::')
        variables.append(f"rclcpp_action::Client<{action_type}>::SharedPtr {action['name']}_action_client;")
    
    # Timers
    for timer in timer_configs:
        variables.append(f"rclcpp::TimerBase::SharedPtr {timer['name']}_timer;")
    
    variables_str = '\n'.join(variables)
    
    # Generate callback stubs
    callbacks = []
    
    # Subscriber callbacks
    for sub in subscriber_configs:
        msg_type = sub['type'].replace('/', '::msg::')
        callbacks.append(f"void {sub['name']}_callback(const {msg_type}::SharedPtr msg)")
        callbacks.append("{")
        callbacks.append(f"  // Implement {sub['name']} subscriber callback")
        callbacks.append(f"  RCLCPP_INFO(this->get_logger(), \"Received message on {sub['name']}\");")
        callbacks.append("}\n")
    
    # Service callbacks
    for srv in service_configs:
        srv_type = srv['type'].replace('/', '::srv::')
        callbacks.append(f"void {srv['name']}_callback(")
        callbacks.append("  const std::shared_ptr<rmw_request_id_t> request_header,")
        callbacks.append(f"  const std::shared_ptr<{srv_type}::Request> request,")
        callbacks.append(f"  const std::shared_ptr<{srv_type}::Response> response)")
        callbacks.append("{")
        callbacks.append("  (void)request_header;")
        callbacks.append(f"  // Implement {srv['name']} service callback");
        callbacks.append(f"  RCLCPP_INFO(this->get_logger(), \"Service {srv['name']} called\");")
        callbacks.append("}\n")
    
    # Action server callbacks
    for action in action_server_configs:
        action_type = action['type'].replace('/', '::action::')
        
        # Goal callback
        callbacks.append(f"rclcpp_action::GoalResponse handle_{action['name']}_goal(")
        callbacks.append("  const rclcpp_action::GoalUUID & uuid,")
        callbacks.append(f"  std::shared_ptr<const {action_type}::Goal> goal)")
        callbacks.append("{")
        callbacks.append("  (void)uuid;")
        callbacks.append("  (void)goal;")
        callbacks.append(f"  RCLCPP_INFO(this->get_logger(), \"Received goal request for {action['name']}\");")
        callbacks.append("  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;")
        callbacks.append("}\n")
        
        # Cancel callback
        callbacks.append(f"rclcpp_action::CancelResponse handle_{action['name']}_cancel(")
        callbacks.append(f"  const std::shared_ptr<rclcpp_action::ServerGoalHandle<{action_type}>> goal_handle)")
        callbacks.append("{")
        callbacks.append("  (void)goal_handle;")
        callbacks.append(f"  RCLCPP_INFO(this->get_logger(), \"Received cancel request for {action['name']}\");")
        callbacks.append("  return rclcpp_action::CancelResponse::ACCEPT;")
        callbacks.append("}\n")
        
        # Accepted callback
        callbacks.append(f"void handle_{action['name']}_accepted(")
        callbacks.append(f"  const std::shared_ptr<rclcpp_action::ServerGoalHandle<{action_type}>> goal_handle)")
        callbacks.append("{")
        callbacks.append("  (void)goal_handle;")
        callbacks.append(f"  RCLCPP_INFO(this->get_logger(), \"Executing goal for {action['name']}\");")
        callbacks.append("  // Start executing the action in a separate thread if needed");
        callbacks.append("}\n")
    
    # Timer callbacks
    for timer in timer_configs:
        callbacks.append(f"void {timer['name']}_callback()")
        callbacks.append("{")
        callbacks.append(f"  // Implement {timer['name']} timer callback");
        callbacks.append(f"  RCLCPP_INFO(this->get_logger(), \"{timer['name']} timer triggered\");")
        callbacks.append("}\n")
    
    callbacks_str = '\n'.join(callbacks)
    
    # Generate the complete CPP file
    template = f"""
#include "rclcpp/rclcpp.hpp"
{imports_str}

class {node_name_class} : public rclcpp::Node
{{
  public:
    {node_name_class}(): Node("{node_name}")
    {{
      {setup_str}
    }}
  
  private:
    {variables_str}
    
    {callbacks_str}
}};

int main(int argc, char * argv[])
{{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<{node_name_class}>();
  
  // Use multi-threaded executor if specified
  {"rclcpp::executors::MultiThreadedExecutor executor;" if config.get('executor') == 'multi_threaded' else ""}
  {"executor.add_node(node);" if config.get('executor') == 'multi_threaded' else ""}
  {"executor.spin();" if config.get('executor') == 'multi_threaded' else "rclcpp::spin(node);"}
  
  rclcpp::shutdown();
  return 0;
}}
"""
    return template
