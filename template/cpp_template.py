import re
from ros_utility import get_ros_distro

def generate_cpp_code(config, indent_spaces=4):
    """Generate a C++ ROS 2 node from configuration.
    
    Args:
        config: Dictionary containing node configuration
        indent_spaces: Number of spaces per indentation level (default: 4)
    """
    current_indent = 0
    indent_str = ' ' * indent_spaces
    
    def increase_indent():
        nonlocal current_indent
        current_indent += 1
        return indent_str * current_indent
    
    def decrease_indent():
        nonlocal current_indent
        current_indent -= 1
        return indent_str * current_indent
    
    def get_indent():
        return indent_str * current_indent
    
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
    
    # Generate includes
    includes = [
        "#include <memory>",
        "#include <rclcpp/rclcpp.hpp>",
        "#include <rclcpp_action/rclcpp_action.hpp>",
        "#include <rclcpp/callback_group.hpp>",
        "#include <rclcpp/executors/multi_threaded_executor.hpp>"
    ]
    
    def pascal_to_snake(name):
        return re.sub(r'(?<!^)([A-Z])', r'_\1', name).lower()

    # Add message includes
    message_includes = set()
    for pub in publisher_configs:
        parts = pub['type'].split('/')
        message_includes.add(f"#include <{parts[0]}/msg/{pascal_to_snake(parts[2])}.hpp>")
    for sub in subscriber_configs:
        parts = sub['type'].split('/')
        message_includes.add(f"#include <{parts[0]}/msg/{pascal_to_snake(parts[2])}.hpp>")
    for client in client_configs:
        parts = client['type'].split('/')
        message_includes.add(f"#include <{parts[0]}/srv/{pascal_to_snake(parts[2])}.hpp>")
    for srv in service_configs:
        parts = srv['type'].split('/')
        message_includes.add(f"#include <{parts[0]}/srv/{pascal_to_snake(parts[2])}.hpp>")
    for action in action_server_configs + action_client_configs:
        parts = action['type'].split('/')
        message_includes.add(f"#include <{parts[0]}/action/{pascal_to_snake(parts[2])}.hpp>")
    
    includes.extend(sorted(message_includes))
    includes_str = '\n'.join(includes)
    
    # Generate class definition
    class_def = [
        f"class {node_name_class} : public rclcpp::Node {{",
        f"{increase_indent()}public:",
        f"{increase_indent()}{node_name_class}() : Node(\"{node_name}\") {{",
        f"{increase_indent()}RCLCPP_INFO(this->get_logger(), \"Initializing %s...\", this->get_name());",
        ""
    ]
    
    # Setup callback group if needed
    if config.get('executor') == 'multi_threaded':
        class_def.extend([
            f"{get_indent()}// Create callback group for parallel execution",
            f"{get_indent()}callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);",
        ])
        if config.get('subscribers'):
            class_def.append(f"{get_indent()}rclcpp::SubscriptionOptions sub_options;")

        class_def.append("")


    
    # QoS profile generator
    def build_qos_profile(qos_config):
        if not qos_config:
            return "rclcpp::QoS(10)"  # Default depth
        
        depth = qos_config.get('depth', 10)
        profile = [f"rclcpp::QoS({depth})"]
        increase_indent()
        # Reliability policy
        if qos_config.get('reliability') == 'best_effort':
            profile.append(f"{get_indent()}.reliability(rclcpp::ReliabilityPolicy::BestEffort)")
        elif qos_config.get('reliability') == 'reliable':
            profile.append(f"{get_indent()}.reliability(rclcpp::ReliabilityPolicy::Reliable)")
        
        # Durability policy
        if qos_config.get('durability') == 'transient_local':
            profile.append(f"{get_indent()}.durability(rclcpp::DurabilityPolicy::TransientLocal)")
        elif qos_config.get('durability') == 'volatile':
            profile.append(f"{get_indent()}.durability(rclcpp::DurabilityPolicy::Volatile)")
        
        # History policy
        if qos_config.get('history') == 'keep_all':
            profile.append(f"{get_indent()}.history(rclcpp::HistoryPolicy::KeepAll)")
        elif qos_config.get('history') == 'keep_last':
            profile.append(f"{get_indent()}.keep_last({depth})")
        
        # Advanced QoS settings
        if 'deadline' in qos_config:
            profile.append(f"{get_indent()}.deadline(rclcpp::Duration::from_seconds({qos_config['deadline']}))")
        if 'lifespan' in qos_config:
            profile.append(f"{get_indent()}.lifespan(rclcpp::Duration::from_seconds({qos_config['lifespan']}))")
        if 'liveliness' in qos_config:
            if qos_config['liveliness'] == 'automatic':
                profile.append(f"{get_indent()}.liveliness(rclcpp::LivelinessPolicy::Automatic)")
            elif qos_config['liveliness'] == 'manual_by_topic':
                profile.append(f"{get_indent()}.liveliness(rclcpp::LivelinessPolicy::ManualByTopic)")
        if 'liveliness_lease' in qos_config:
            profile.append(f"{get_indent()}.liveliness_lease_duration(rclcpp::Duration::from_seconds({qos_config['liveliness_lease']}))")
        
        qos_str = '\n'.join(profile)
        decrease_indent()
        return qos_str
    
    # Create publishers
    for pub in publisher_configs:
        msg_type = pub['type'].split('/')[-1]
        ns = pub['type'].split('/')[0]
        
        class_def.extend([
            f"{get_indent()}// Publisher: {pub['name']}",
            f"{get_indent()}{pub['name']}_pub_ = this->create_publisher<{ns}::msg::{msg_type}>(",
            f"{increase_indent()}\"{pub['name']}\",",
            f"{get_indent()}{build_qos_profile(pub.get('qos', {}))}",
            f"{decrease_indent()});",
            ""
        ])
    
    class_def.append("\n")

    # Create subscribers
    for sub in subscriber_configs:
        msg_type = sub['type'].split('/')[-1]
        ns = sub['type'].split('/')[0]
        
        class_def.extend([
            f"{get_indent()}// Subscriber: {sub['name']}",
            f"{get_indent()}{sub['name']}_sub_ = this->create_subscription<{ns}::msg::{msg_type}>(",
            f"{increase_indent()}\"{sub['name']}\",",
            f"{get_indent()}{build_qos_profile(sub.get('qos', {}))},",
            f"{get_indent()}[this](const {ns}::msg::{msg_type}::SharedPtr msg) {{",
            f"{increase_indent()}this->{sub['name']}_callback(msg);",
            f"{decrease_indent()}}}" + (f",\n{get_indent()}sub_options" if config.get('executor') == 'multi_threaded' else ""),
            f"{decrease_indent()});",
            ""
        ])
    
    class_def.append("\n")

    # Create service clients
    for client in client_configs:
        srv_type = client['type'].split('/')[-1]
        ns = client['type'].split('/')[0]
        callback_group = f", {'rmw_qos_profile_services_default' if get_ros_distro() == "humble" else build_qos_profile(client.get('qos', {}))}, callback_group_" if config.get('executor') == 'multi_threaded' else ""
        
        class_def.extend([
            f"{get_indent()}// Service client: {client['name']}",
            f"{get_indent()}{client['name']}_client_ = this->create_client<{ns}::srv::{srv_type}>(",
            f"{increase_indent()}\"{client['name']}\"{callback_group}",
            f"{decrease_indent()});",
            ""
        ])

    class_def.append("\n")
    
    # Create service servers
    for srv in service_configs:
        srv_type = srv['type'].split('/')[-1]
        ns = srv['type'].split('/')[0]
        
        class_def.extend([
            f"{get_indent()}// Service server: {srv['name']}",
            f"{get_indent()}{srv['name']}_srv_ = this->create_service<{ns}::srv::{srv_type}>(",
            f"{increase_indent()}\"{srv['name']}\",",
            f"{get_indent()}[this](const std::shared_ptr<{ns}::srv::{srv_type}::Request> request,",
            f"{get_indent()} std::shared_ptr<{ns}::srv::{srv_type}::Response> response) {{",
            f"{increase_indent()}this->{srv['name']}_callback(request, response);",
            f"{decrease_indent()}}},\n{get_indent()}{'rmw_qos_profile_services_default' if get_ros_distro() == "humble" else build_qos_profile(srv.get('qos', {}))}" + (f",\n{get_indent()}callback_group_" if config.get('executor') == 'multi_threaded' else ""),
            f"{decrease_indent()});",
            ""
        ])

    class_def.append("\n")
    
    # Create action servers
    for action in action_server_configs:
        action_type = action['type'].split('/')[-1]
        ns = action['type'].split('/')[0]
        options_var = f"{action['name']}_options"

        class_def.extend([
            f"{get_indent()}// Action server: {action['name']}",
            f"{get_indent()}rcl_action_server_options_t {options_var} = rcl_action_server_get_default_options();",
            "",
            f"{get_indent()}{action['name']}_action_server_ = rclcpp_action::create_server<{ns}::action::{action_type}>(",
            f"{increase_indent()}this,",
            f"{get_indent()}\"{action['name']}\",",
            f"{get_indent()}[this](const rclcpp_action::GoalUUID &, std::shared_ptr<const {ns}::action::{action_type}::Goal>) {{",
            f"{increase_indent()}return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;",
            f"{decrease_indent()}}},",
            f"{get_indent()}[this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<{ns}::action::{action_type}>> handle) {{",
            f"{increase_indent()}return rclcpp_action::CancelResponse::ACCEPT;",
            f"{decrease_indent()}}},",
            f"{get_indent()}[this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<{ns}::action::{action_type}>> handle) {{",
            f"{increase_indent()}this->execute_{action['name']}_callback(handle);",
            f"{decrease_indent()}}},",
            f"{get_indent()}{options_var}" + (f",\n{get_indent()}callback_group_" if config.get('executor') == 'multi_threaded' else ""),
            f"{decrease_indent()});",
            ""
        ])


    class_def.append("\n")
    
    # Create action clients
    for action in action_client_configs:
        action_type = action['type'].split('/')[-1]
        ns = action['type'].split('/')[0]
        
        class_def.extend([
            f"{get_indent()}// Action client: {action['name']}",
            f"{get_indent()}{action['name']}_action_client_ = rclcpp_action::create_client<{ns}::action::{action_type}>(",
            f"{increase_indent()}this, \"{action['name']}\"" + (f", callback_group_" if config.get('executor') == 'multi_threaded' else ""),
            f"{decrease_indent()});",
            ""
        ])

    class_def.append("\n")
    
    # Create timers
    for timer in timer_configs:
        period = timer['period']
        callback_group = ", callback_group_" if config.get('executor') == 'multi_threaded' else ""
        
        class_def.extend([
            f"{get_indent()}// Timer: {timer['name']}",
            f"{get_indent()}{timer['name']}_timer_ = this->create_wall_timer(",
            f"{increase_indent()}std::chrono::milliseconds({int(period*1000)}),",
            f"{get_indent()}[this]() {{ this->{timer['name']}_callback(); }}{callback_group}",
            f"{decrease_indent()});",
            ""
        ])

    class_def.append("\n")
    
    # Close constructor
    class_def.append(f"{decrease_indent()}}}")
    class_def.append("")
    
    # Generate private section with callback declarations
    class_def.append("private:")
    current_indent = 1
    
    # Member variables
    members = []
    for pub in publisher_configs:
        members.append(f"{get_indent()}rclcpp::Publisher<{pub['type'].split('/')[0]}::msg::{pub['type'].split('/')[-1]}>::SharedPtr {pub['name']}_pub_;")
    for sub in subscriber_configs:
        members.append(f"{get_indent()}rclcpp::Subscription<{sub['type'].split('/')[0]}::msg::{sub['type'].split('/')[-1]}>::SharedPtr {sub['name']}_sub_;")
    for client in client_configs:
        members.append(f"{get_indent()}rclcpp::Client<{client['type'].split('/')[0]}::srv::{client['type'].split('/')[-1]}>::SharedPtr {client['name']}_client_;")
    for srv in service_configs:
        members.append(f"{get_indent()}rclcpp::Service<{srv['type'].split('/')[0]}::srv::{srv['type'].split('/')[-1]}>::SharedPtr {srv['name']}_srv_;")
    for action in action_server_configs:
        members.append(f"{get_indent()}rclcpp_action::Server<{action['type'].split('/')[0]}::action::{action['type'].split('/')[-1]}>::SharedPtr {action['name']}_action_server_;")
    for action in action_client_configs:
        members.append(f"{get_indent()}rclcpp_action::Client<{action['type'].split('/')[0]}::action::{action['type'].split('/')[-1]}>::SharedPtr {action['name']}_action_client_;")
    for timer in timer_configs:
        members.append(f"{get_indent()}rclcpp::TimerBase::SharedPtr {timer['name']}_timer_;")
    
    if config.get('executor') == 'multi_threaded':
        members.append(f"{get_indent()}rclcpp::CallbackGroup::SharedPtr callback_group_;")
    
    # Callback declarations
    callbacks = []
    for sub in subscriber_configs:
        msg_type = sub['type'].split('/')[-1]
        ns = sub['type'].split('/')[0]
        callbacks.append(f"{get_indent()}void {sub['name']}_callback(const {ns}::msg::{msg_type}::SharedPtr msg);")
    
    for srv in service_configs:
        srv_type = srv['type'].split('/')[-1]
        ns = srv['type'].split('/')[0]
        callbacks.append(
            f"{get_indent()}void {srv['name']}_callback("
            f"const std::shared_ptr<{ns}::srv::{srv_type}::Request> request,"
            f" std::shared_ptr<{ns}::srv::{srv_type}::Response> response);"
        )
    
    for action in action_server_configs:
        action_type = action['type'].split('/')[-1]
        ns = action['type'].split('/')[0]
        callbacks.append(
            f"{get_indent()}void execute_{action['name']}_callback("
            f"const std::shared_ptr<rclcpp_action::ServerGoalHandle<{ns}::action::{action_type}>> handle);"
        )
    
    for timer in timer_configs:
        callbacks.append(f"{get_indent()}void {timer['name']}_callback();")
    
    # Combine class definition
    class_def.extend(members)
    class_def.append("")
    class_def.extend(callbacks)
    class_def.append(f"}}; // class {node_name_class}")
    class_def.append(f"{decrease_indent()}")
    
    # Generate callback implementations
    impls = []
    for sub in subscriber_configs:
        msg_type = sub['type'].split('/')[-1]
        ns = sub['type'].split('/')[0]
        impls.extend([
            f"void {node_name_class}::{sub['name']}_callback(const {ns}::msg::{msg_type}::SharedPtr msg) {{",
            f"{increase_indent()}RCLCPP_INFO(this->get_logger(), \"Received message on {sub['name']}\");",
            f"{get_indent()}// Implement {sub['name']} subscriber callback",
            f"}}",
            f"{decrease_indent()}"
        ])
    
    for srv in service_configs:
        srv_type = srv['type'].split('/')[-1]
        ns = srv['type'].split('/')[0]
        impls.extend([
            f"void {node_name_class}::{srv['name']}_callback(const std::shared_ptr<{ns}::srv::{srv_type}::Request> request, std::shared_ptr<{ns}::srv::{srv_type}::Response> response) {{",
            f"{increase_indent()}RCLCPP_INFO(this->get_logger(), \"Service {srv['name']} called\");",
            f"{get_indent()}// Implement {srv['name']} service callback",
            f"}}",
            f"{decrease_indent()}"
        ])
    
    for action in action_server_configs:
        action_type = action['type'].split('/')[-1]
        ns = action['type'].split('/')[0]
        impls.extend([
            f"void {node_name_class}::execute_{action['name']}_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<{ns}::action::{action_type}>> handle) {{",
            f"{increase_indent()}RCLCPP_INFO(this->get_logger(), \"Executing goal for {action['name']}\");",
            f"{get_indent()}// Implement action execution",
            f"{get_indent()}auto result = std::make_shared<{ns}::action::{action_type}::Result>();",
            f"{get_indent()}handle->succeed(result);",
            f"}}",
            f"{decrease_indent()}"
        ])
    
    for timer in timer_configs:
        impls.extend([
            f"void {node_name_class}::{timer['name']}_callback() {{",
            f"{increase_indent()}RCLCPP_INFO(this->get_logger(), \"{timer['name']} timer triggered\");",
            f"{get_indent()}// Implement {timer['name']} timer callback",
            f"}}",
            f"{decrease_indent()}"
        ])
    
    # Generate main function
    main_func = [
        "int main(int argc, char** argv) {",
        f"{increase_indent()}rclcpp::init(argc, argv);",
        f"{get_indent()}try {{",
        f"{increase_indent()}auto node = std::make_shared<{node_name_class}>();"
    ]
    
    if config.get('executor') == 'multi_threaded':
        main_func.extend([
            f"{get_indent()}rclcpp::executors::MultiThreadedExecutor executor;",
            f"{get_indent()}executor.add_node(node);",
            f"{get_indent()}executor.spin();"
        ])
    else:
        main_func.append(f"{get_indent()}rclcpp::spin(node);")
    
    main_func.extend([
        f"{decrease_indent()}}} catch (const std::exception& e) {{",
        f"{increase_indent()}RCLCPP_ERROR(rclcpp::get_logger(\"main\"), \"Exception: %s\", e.what());",
        f"{decrease_indent()}}}",
        f"{get_indent()}rclcpp::shutdown();",
        f"{get_indent()}return 0;",
        "}"
    ])
    
    # Combine all parts
    result = [
        includes_str,
        "",
        "using namespace std::chrono_literals;",
        "",
        '\n'.join(class_def),
        '\n'.join(impls),
        '\n'.join(main_func)
    ]
    
    return '\n'.join(result)