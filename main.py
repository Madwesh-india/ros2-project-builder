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
    all_interfaces = get_all_interfaces()
    config = gather_project_config(all_interfaces)

    if not config["executable"]:
        config["executable"] = config["node_name"]

    # scaffold the package
    create_ros_pkg(config["workspace_path"], config["package_name"], config["language"])

    pkg_path = f"{config['workspace_path']}/src/{config['package_name']}"
    cmake_path = f"{pkg_path}/CMakeLists.txt"
    package_xml = f"{pkg_path}/package.xml"

    deps = collect_ros_deps(config)

    if config["language"] == "python":
        python_code = generate_python_code(config)
        out = f"{pkg_path}/{config['package_name']}/{config['executable']}_node.py"
        with open(out, 'w+') as f: f.write(python_code)
        print(f"Generated {out}")
        add_console_script_to_setup(f"{pkg_path}/setup.py",
                                    f"{config['executable']}={config['package_name']}.{config['executable']}_node:main")
    else:
        # C++
        cpp_code = generate_cpp_code(config)
        out = f"{pkg_path}/src/{config['node_name']}_node.cpp"
        with open(out, 'w+') as f: f.write(cpp_code)
        print(f"Generated {out}")

        # patch CMakeLists.txt & package.xml
        update_cmakelists(cmake_path, config["package_name"], config["node_name"], deps)
        update_package_xml(package_xml, deps)

    return config

if __name__ == "__main__":
    main()
