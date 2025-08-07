import os
import re
import subprocess
import sys
from typing import Dict, List
from parse import parse_ros2_msg, parse_ros2_srv, parse_ros2_action
import xml.etree.ElementTree as ET
from xml.dom import minidom


def get_ros_distro():
    return os.environ.get('ROS_DISTRO', 'Unknown')


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
    

def create_ros_pkg(config: Dict) -> bool:
    """
    Create a ROS 2 package
    
    Args:
        config: The full config of the setup
    """

    # Validate inputs
    src_path = os.path.join(config["workspace_path"], "src")
    if not os.path.isdir(src_path):
        print(f"Error: Missing 'src' directory in {config['workspace_path']}", file=sys.stderr)
        return False

    for package_name in config["package_name"]:
        # Check if package already exists
        pkg_path = os.path.join(src_path, package_name)
        if os.path.exists(pkg_path):
            print(f"Package '{package_name}' already exists at {pkg_path}")

        language = config[package_name]["language"]

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
            print(f"Success! Package created at: {pkg_path}")
            
        except subprocess.CalledProcessError as e:
            print(f"Failed to create package: {e.stderr}", file=sys.stderr)


def add_console_script_to_setup(setup_path, new_entry):
    with open(setup_path, 'r') as file:
        content = file.read()

    # Match entry_points console_scripts block
    pattern = r"(entry_points\s*=\s*{\s*['\"]console_scripts['\"]\s*:\s*\[\s*)([^]]*?)(\s*\])"
    match = re.search(pattern, content, re.DOTALL)

    if not match:
        print("Could not find 'console_scripts' entry_points block.")
        return

    prefix, middle, suffix = match.groups()

    # Parse existing entries
    entries = [e.strip().strip('"\'') for e in middle.split(',') if e.strip()]
    if new_entry in entries:
        print(f"'{new_entry}' already exists in console_scripts.")
        return

    entries.append(new_entry)
    new_middle = ',\n        '.join(f"'{e}'" for e in entries)

    new_block = f"{prefix}{new_middle}{suffix}"
    updated_content = re.sub(pattern, new_block, content, flags=re.DOTALL)

    with open(setup_path, 'w') as file:
        file.write(updated_content)

    print(f"Added '{new_entry}' to console_scripts.")

def collect_ros_deps(config):
    """
    Scan your config.json and pull out all unique ROS packages you need to depend on.
    """
    deps = set()

    def pkg_from_type(t):
        # e.g. "nav_msgs/msg/OccupancyGrid" -> "nav_msgs"
        return t.split('/')[0]

    for pub in config.get("publisher_configs", []):
        deps.add(pkg_from_type(pub["type"]))
    for sub in config.get("subscriber_configs", []):
        deps.add(pkg_from_type(sub["type"]))
    for cli in config.get("client_configs", []):
        deps.add(pkg_from_type(cli["type"]))
    for srv in config.get("service_configs", []):
        deps.add(pkg_from_type(srv["type"]))
    for action in config.get("action_server_configs", []):
        deps.add(pkg_from_type(action["type"]))
    for action in config.get("action_client_configs", []):
        deps.add(pkg_from_type(action["type"]))
    return sorted(deps)

def update_cmakelists(cmake_path, pkg_name, executable_name, deps):
    """
    Updates CMakeLists.txt to:
    - Add missing find_package(...) lines after '# find dependencies'
    - Add executable + ament_target_dependencies
    - Ensure install(TARGETS ...) is present
    - Move ament_package() to the end if needed
    """
    with open(cmake_path, 'r') as f:
        text = f.read()

    # Collect existing find_package calls
    existing_packages = set(re.findall(r'find_package\((\w+)\s+REQUIRED\)', text))

    # Generate new find_package lines
    new_fp_lines = [f'find_package({dep} REQUIRED)' for dep in deps if dep not in existing_packages]

    if new_fp_lines:
        insert_pos = text.find("# find dependencies")
        if insert_pos != -1:
            # Insert after the '# find dependencies' line
            insert_pos = text.find('\n', insert_pos) + 1
        else:
            # Fallback: after project(...)
            match = re.search(r'(project\(.*?\))', text)
            if match:
                insert_pos = match.end() + 1
            else:
                insert_pos = 0  # insert at start if nothing found
        text = text[:insert_pos] + '\n'.join(new_fp_lines) + '\n' + text[insert_pos:]

    # Remove ament_package() to reappend at the end
    text = re.sub(r'\n?ament_package\(\)\n?', '', text)

    # Append executable and dependencies
    if f'add_executable({executable_name}_node' not in text:
        text += f'\n\n# Auto-generated node\nadd_executable({executable_name}_node src/{executable_name}_node.cpp)'

    if f'ament_target_dependencies({executable_name}_node' not in text:
        deps_line = ' '.join(deps)
        text += f'\nament_target_dependencies({executable_name}_node {deps_line})'

    # Ensure separate install(TARGETS ...) line for this node
    install_line = f'install(TARGETS {executable_name}_node DESTINATION lib/${{PROJECT_NAME}})'
    if install_line not in text:
        text += f'\n{install_line}'

    # Ensure only one ament_package() at the end
    text = re.sub(r'\n?ament_package\(\)\n?', '', text)
    text += '\nament_package()\n'

    with open(cmake_path, 'w') as f:
        f.write(text)

    print(f"→ Updated {cmake_path}")

    
def update_package_xml(package_xml_path, deps):
    """
    Load package.xml, add <depend>DEP</depend> for each missing dep, and write back (cleaned & prettified).
    """
    tree = ET.parse(package_xml_path)
    root = tree.getroot()

    # find existing <depend> tags
    existing = {d.text for d in root.findall('depend')}

    # Find position to insert new <depend> elements
    insert_at = None
    for idx, el in enumerate(root):
        if el.tag == 'depend':
            insert_at = idx
            break
    if insert_at is None:
        for idx, el in enumerate(root):
            if el.tag in ('description', 'maintainer'):
                insert_at = idx + 1
                break
    if insert_at is None:
        insert_at = len(root)

    # Insert new dependencies
    for d in deps:
        if d not in existing:
            new = ET.Element('depend')
            new.text = d
            root.insert(insert_at, new)
            insert_at += 1

    # Convert and clean up the output
    rough_string = ET.tostring(root, encoding='utf-8')
    reparsed = minidom.parseString(rough_string)
    pretty_xml = reparsed.toprettyxml(indent="  ")

    # Remove excessive blank lines
    clean_lines = [line for line in pretty_xml.split('\n') if line.strip()]
    final_xml = '\n'.join(clean_lines) + '\n'

    with open(package_xml_path, 'w', encoding='utf-8') as f:
        f.write(final_xml)

    print(f"→ Updated {package_xml_path}")

if __name__ == "__main__":
    print(get_interface_details("std_msgs/msg/Bool"))