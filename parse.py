
from collections import deque
import re


def parse_ros2_msg(desc: str):
    """
    Parse an indented ROS 2 .msg description into a nested dict structure,
    including both fields and constants.
    """
    # regex for constants: e.g. "int32 X=123" or "string FOO='bar'"
    const_re = re.compile(
        r'^(?P<indent>\s*)(?P<type>[\w\/]+)\s+'
        r'(?P<name>[A-Z][A-Z0-9_]*)=(?P<value>.+)$'
    )
    # regex for normal fields: e.g. "float64[36] covariance"
    field_re = re.compile(
        r'^(?P<indent>\s*)(?P<type>[\w\/\[\]\<\>\=]+)\s+'
        r'(?P<name>[\w_]+)\s*$'
    )

    root = {}
    stack = deque([(-1, root)])  # (indent_level, current_dict)

    for raw in desc.splitlines():
        # try constant first
        m_const = const_re.match(raw)
        if m_const:
            indent = len(m_const.group('indent').replace('\t','    '))
            base_type = m_const.group('type')
            name = m_const.group('name')
            value = m_const.group('value').strip()
            # pop until correct parent
            while stack and indent <= stack[-1][0]:
                stack.pop()
            parent = stack[-1][1]
            parent[name] = {
                'type': base_type,
                'constant': True,
                'value': value
            }
            continue

        # then normal field
        m = field_re.match(raw)
        if not m:
            continue

        indent = len(m.group('indent').replace('\t','    '))
        full_type = m.group('type')
        name = m.group('name')

        # detect array
        array = bool(re.search(r'\[.*\]', full_type))
        # extract base type and size/bound if any
        arr_match = re.match(r'^(?P<base>[\w\/]+)(?:\[(?P<size>[^\]]*)\])?$', full_type)
        base_type = arr_match.group('base')
        size = arr_match.group('size') if arr_match and arr_match.group('size') != '' else None

        while stack and indent <= stack[-1][0]:
            stack.pop()
        parent = stack[-1][1]

        parent[name] = {
            'type': base_type,
            'array': array,
            'size': size,
            'children': {}
        }
        stack.append((indent, parent[name]['children']))

    return root


def parse_ros2_srv(desc: str):
    """
    Parse a .srv description string into:
      { 'request': {...}, 'response': {...} }
    splitting on the first '---'.
    """
    parts = desc.split('---', 1)
    req_desc = parts[0].strip()
    resp_desc = parts[1].strip() if len(parts) > 1 else ''
    return {
        'request': parse_ros2_msg(req_desc),
        'response': parse_ros2_msg(resp_desc)
    }

def parse_ros2_action(desc: str):
    """
    Parse a .action description string into:
      { 'goal': {...}, 'result': {...}, 'feedback': {...} }
    splitting on the two '---' delimiters.
    """
    parts = [p.strip() for p in desc.split('---')]
    # ROS 2 action files are: goal---result---feedback
    keys = ['goal','result','feedback']
    result = {}
    for key, part in zip(keys, parts):
        result[key] = parse_ros2_msg(part)
    return result