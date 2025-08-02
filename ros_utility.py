import os
import re
import subprocess
import sys
from typing import Dict, List
from parse import parse_ros2_msg, parse_ros2_srv, parse_ros2_action


def get_all_interfaces() -> Dict[str, Dict[str, List[str]]]:
    """
    Retrieve all available ROS 2 interfaces (messages, services, actions) from the system.
    
    Scans the ROS 2 environment using 'ros2 interface list' command and parses the output
    into a structured dictionary format organized by package and interface type.
    
    Returns:
        A nested dictionary structured as:
        {
            "package_name": {
                "msg": ["MessageType1", "MessageType2"],
                "srv": ["ServiceType1"],
                "action": ["ActionType1"]
            },
            ...
        }
        
    Raises:
        SystemExit: If ROS 2 environment is not properly sourced or command fails
    """
    # Command to list all ROS 2 interfaces
    command = ["ros2", "interface", "list"]
    
    try:
        # Execute the command and capture output
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=True
        )
    except subprocess.CalledProcessError as e:
        print(f"Error executing 'ros2 interface list': {e.stderr}", file=sys.stderr)
        print("Please ensure ROS 2 is properly sourced in your environment", file=sys.stderr)
        sys.exit(e.returncode)
    except FileNotFoundError:
        print("Error: 'ros2' command not found. Is ROS 2 installed and sourced?", file=sys.stderr)
        sys.exit(1)

    # Regular expression to parse interface paths
    # Matches patterns like: "package_name/msg/TypeName"
    # or "package_name/srv/TypeName" or "package_name/action/TypeName"
    interface_pattern = re.compile(
        r'^\s*([a-z][a-z0-9_]*)/(msg|srv|action)/([A-Z][a-zA-Z0-9_]*)$',
        re.MULTILINE
    )

    interfaces = {}
    
    # Process each matching interface found in the command output
    for match in interface_pattern.finditer(result.stdout):
        package, iface_type, type_name = match.groups()
        
        # Initialize package entry if not exists
        if package not in interfaces:
            interfaces[package] = {}
            
        # Initialize interface type list if not exists
        if iface_type not in interfaces[package]:
            interfaces[package][iface_type] = []
            
        # Add the interface type to the appropriate list
        interfaces[package][iface_type].append(type_name)
    
    return interfaces

def get_interface_details(interface: str) -> Dict:
    """
    Retrieve and parse the details of a specific ROS 2 interface (message, service, or action).

    Args:
        interface: The full interface path in the format 'package_name/type/InterfaceName'
                  where type is 'msg', 'srv', or 'action'

    Returns:
        A dictionary containing the parsed interface details. The structure varies by type:
        - For messages: {'fields': [list of field definitions]}
        - For services: {'request': [request fields], 'response': [response fields]}
        - For actions: {'goal': [goal fields], 'result': [result fields], 'feedback': [feedback fields]}

    Raises:
        ValueError: If the interface type is not 'msg', 'srv', or 'action'
        SystemExit: If the ROS 2 command fails or interface is not found
    """
    # Validate interface format
    parts = interface.split('/')
    if len(parts) != 3 or parts[1] not in ['msg', 'srv', 'action']:
        raise ValueError(f"Invalid interface format: {interface}. Expected 'package/type/name' where type is msg/srv/action")

    # Command to show interface details without comments
    command = ["ros2", "interface", "show", interface, "--no-comments"]

    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            check=True
        )
    except subprocess.CalledProcessError as e:
        print(f"Error retrieving interface details: {e.stderr}", file=sys.stderr)
        print(f"Interface '{interface}' may not exist", file=sys.stderr)
        sys.exit(e.returncode)
    except FileNotFoundError:
        print("Error: 'ros2' command not found. Is ROS 2 installed and sourced?", file=sys.stderr)
        sys.exit(1)

    # Route to appropriate parser based on interface type
    interface_type = parts[1]
    if interface_type == "msg":
        return parse_ros2_msg(result.stdout)
    elif interface_type == "srv":
        return parse_ros2_srv(result.stdout)
    elif interface_type == "action":
        return parse_ros2_action(result.stdout)
    else:
        # This should never happen due to earlier validation
        raise ValueError(f"Unsupported interface type: {interface_type}")


def create_ros_pkg(path: str, package_name: str, language: str) -> bool:
    """
    Create a ROS 2 package
    
    Args:
        path: Workspace path (must contain 'src' directory)
        package_name: Name for the new package
        language: 'python' or 'cpp'
        
    Returns:
        bool: True if successful, False otherwise
    """
    # Validate inputs
    src_path = os.path.join(path, "src")
    if not os.path.isdir(src_path):
        print(f"Error: Missing 'src' directory in {path}", file=sys.stderr)
        return False

    if language not in ("python", "cpp"):
        print(f"Error: Unsupported language '{language}'", file=sys.stderr)
        return False

    # Configure command based on language
    build_type, deps = ("ament_python", "rclpy") if language == "python" else ("ament_cmake", "rclcpp")
    
    cmd = [
        "ros2", "pkg", "create",
        package_name,
        "--build-type", build_type,
        "--dependencies", deps,
        "--destination-directory", src_path
    ]

    # Execute command
    try:
        print(f"Creating {language} package '{package_name}'...")
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            check=True
        )
        print(f"Success! Package created at: {src_path}")
        return True
        
    except subprocess.CalledProcessError as e:
        print(f"Failed to create package: {e.stderr}", file=sys.stderr)
        return False
        
    except FileNotFoundError:
        print("Error: 'ros2' command not found - is ROS 2 sourced?", file=sys.stderr)
        return False

if __name__ == "__main__":
    print(get_interface_details("std_msgs/msg/Bool"))