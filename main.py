import json
from parse import parse_ros2_msg, parse_ros2_srv, parse_ros2_action
from user_interface import gather_project_config
from ros_info_utility import get_all_interfaces


all_interfaces = get_all_interfaces()

print(json.dumps(all_interfaces, indent=2))

config = gather_project_config(all_interfaces)

# command = ["ros2", "interface", "show", f"{pkg}/{com_type}/{inter}", "--no-comments"]

# interface_show = subprocess.run(command, capture_output=True, text=True)

# print(f"{pkg}/{com_type}/{inter}")

# print(interface_show.stdout)

# if com_type == "msg":
#     print(json.dumps(parse_ros2_msg(interface_show.stdout), indent=2))
# elif com_type == "srv":
#     print(json.dumps(parse_ros2_srv(interface_show.stdout), indent=2))
# elif com_type == "action":
#     print(json.dumps(parse_ros2_action(interface_show.stdout), indent=2))
