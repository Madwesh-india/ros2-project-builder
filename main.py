from user_interface import gather_project_config
from ros_utility import (
    get_all_interfaces, create_ros_pkg,
    add_console_script_to_setup, collect_ros_deps, 
    update_cmakelists, update_package_xml
)
from template.cpp_template import generate_cpp_code
from template.python_template import generate_python_code

import json

def main():
    """
    Entry point for scaffolding a ROS 2 package with basic node code

    Steps:
        - Retrieves available ROS interfaces (topics, services, actions)
        - Gathers user-defined project configuration via CLI
        - Creates a ROS 2 package (C++ or Python)
        - Generates a basic node implementation using templates
        - Adds dependencies to CMakeLists.txt and package.xml (C++ case)
        - Registers console script in setup.py (Python case)

    Returns:
        config (dict): Final project configuration collected from the user
    """
    all_interfaces = get_all_interfaces()
    config = gather_project_config(all_interfaces)
    
    # with open("sample.json", "r") as f:
    #     config = json.loads(f.read())
    
    # scaffold the package
    create_ros_pkg(config)

    for package_name in config["package_name"]:
        pkg_path = f"{config['workspace_path']}/src/{package_name}"
        cmake_path = f"{pkg_path}/CMakeLists.txt"
        package_xml = f"{pkg_path}/package.xml"

        for node_name in config[package_name]["node_name"]:
            deps = collect_ros_deps(config[package_name][node_name])

            if config[package_name]["language"] == "python":
                deps.append("rclpy")
                deps.append("rclpy_action")
                python_code = generate_python_code(config, package_name, node_name)
                out = f"{pkg_path}/{package_name}/{config[package_name][node_name]['executable']}_node.py"
                with open(out, 'w+') as f: f.write(python_code)
                print(f"Generated {out}")
                add_console_script_to_setup(f"{pkg_path}/setup.py",
                                            f"{config[package_name][node_name]['executable']}={package_name}.{config[package_name][node_name]['executable']}_node:main")
                
            else:
                deps.append("rclcpp")
                deps.append("rclcpp_action")
                cpp_code = generate_cpp_code(config, package_name, node_name)
                out = f"{pkg_path}/src/{config[package_name][node_name]['executable']}_node.cpp"
                with open(out, 'w+') as f: f.write(cpp_code)
                print(f"Generated {out}")

                # patch CMakeLists.txt & package.xml
                update_cmakelists(cmake_path, package_name, config[package_name][node_name]['executable'], deps)
            
            update_package_xml(package_xml, deps)

    return config

if __name__ == "__main__":
    main()
