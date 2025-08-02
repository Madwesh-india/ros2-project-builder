import random
import re
import sys
import subprocess
import json
from parse import parse_ros2_msg, parse_ros2_srv, parse_ros2_action

command = ["ros2", "interface", "list"]

interface_list = subprocess.run(command, capture_output=True, text=True)

if interface_list.returncode != 0:
    print("Error executing command:", interface_list.stderr, file=sys.stderr)
    print("Check if ros2 is sourced")
    sys.exit(interface_list.returncode)

pattern = re.compile(r'^\s*([\w_]+)/(\w+)/([\w_]+)$', re.MULTILINE)

result = {}

for match in pattern.finditer(interface_list.stdout):
    package, iface_type, type_name = match.groups()
    result.setdefault(package, {}).setdefault(iface_type, []).append(type_name)


pkg = random.choice(list(result.keys()))
com_type = random.choice(list(result[pkg].keys()))
inter = random.choice(result[pkg][com_type])

command = ["ros2", "interface", "show", f"{pkg}/{com_type}/{inter}", "--no-comments"]

interface_show = subprocess.run(command, capture_output=True, text=True)

print(f"{pkg}/{com_type}/{inter}")

print(interface_show.stdout)

if com_type == "msg":
    print(json.dumps(parse_ros2_msg(interface_show.stdout), indent=2))
elif com_type == "srv":
    print(json.dumps(parse_ros2_srv(interface_show.stdout), indent=2))
elif com_type == "action":
    print(json.dumps(parse_ros2_action(interface_show.stdout), indent=2))
