from user_interface import gather_project_config
from ros_utility import get_all_interfaces, create_ros_pkg
from template.cpp_template import generate_cpp_node
from template.python_template import generate_python_node

import json

def main():
    # all_interfaces = get_all_interfaces()
    # config = gather_project_config(all_interfaces)

    import json

    with open("sample_py.json", "r") as f:
        config = json.loads(f.read())

    create_ros_pkg(config["workspace_path"], config["package_name"], config["language"])

    if config["language"] == "python":
        python_code = generate_python_node(config)

        # Save to file
        output_filename = f"./{config['node_name']}_node.py"
        with open(output_filename, 'w') as f:
            f.write(python_code)

        print(f"Successfully generated {output_filename}")

    else:
        # Generate C++ code
        cpp_code = generate_cpp_node(config)

        # Save to file
        output_filename = f"./{config['node_name']}_node.cpp"
        with open(output_filename, 'w') as f:
            f.write(cpp_code)

        print(f"Successfully generated {output_filename}")

    return config

if __name__ == "__main__":
    config = main()


