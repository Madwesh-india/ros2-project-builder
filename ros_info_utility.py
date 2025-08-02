import re
import subprocess
import sys
from typing import Dict, List


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

