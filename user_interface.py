"""
ROS 2 Project Configuration Wizard
---------------------------------
Interactive command-line interface for configuring ROS 2 projects. Collects node specifications,
QoS settings, and workspace configuration through a guided terminal interface. Now with full support
for action servers and clients and proper name validation.
"""

import questionary
from prompt_toolkit import prompt
from prompt_toolkit.completion import Completer, Completion
import os
import re

class CleanTildePathCompleter(Completer):
    def get_completions(self, document, complete_event):
        text = document.text_before_cursor.strip()
        
        # Don't show completions if input is just "~"
        if text == "~":
            return

        expanded = os.path.expanduser(text)
        dirname = os.path.dirname(expanded) or "."
        prefix = os.path.basename(expanded)

        try:
            entries = os.listdir(dirname)
        except Exception:
            return

        for entry in entries:
            full_path = os.path.join(dirname, entry)

            if not os.path.isdir(full_path):
                continue
            if entry.startswith('.'):
                continue
            if not entry.startswith(prefix):
                continue

            display = entry
            yield Completion(display, start_position=-len(prefix))

def _get_workspace_path(use_existing: bool) -> str:
    """
    Prompt for and validate workspace path
    Args:
        use_existing: Whether to validate existing workspace
    Returns:
        Validated absolute path to workspace
    """
    while True:
        path = prompt("Enter workspace path (absolute or relative): ",
                     completer=CleanTildePathCompleter(),
                     complete_while_typing=True)
        path = os.path.expanduser(path)
        src_path = os.path.join(path, "src")

        if use_existing:
            if os.path.isdir(src_path):
                print(f"[SUCCESS] Valid workspace found at: {path}")
                return path
            print("[ERROR] Invalid workspace - missing 'src' directory")
        else:
            try:
                os.makedirs(src_path, exist_ok=True)
                print(f"[INFO] Created new workspace at: {path}")
                return path
            except OSError as e:
                print(f"[ERROR] Workspace creation failed: {str(e)}")


def _validate_name(name: str, name_type: str = "base") -> bool:
    """
    Validate a name according to ROS 2 naming conventions
    Args:
        name: Name to validate
        name_type: Type of name ("base", "package", "executable")
    Returns:
        bool: True if valid, False otherwise
    """
    patterns = {
        "base": r"^[a-zA-Z_][a-zA-Z0-9_]*$",          # Node names, component names
        "package": r"^[a-z][a-z0-9_]*$",               # Package names (lowercase only)
    }
    
    if name_type not in patterns:
        raise ValueError(f"Invalid name_type: {name_type}")
    
    return re.match(patterns[name_type], name) is not None


def _get_validated_name(prompt_text: str, default: str = "", name_type: str = "base") -> str:
    """
    Prompt for and validate a name according to ROS 2 naming conventions
    Args:
        prompt_text: Description of the value being requested
        default: Default value to suggest
        name_type: Type of name to validate ("base", "package", "executable")
    Returns:
        Validated name string
    """
    error_messages = {
        "base": "must start with a letter/underscore and contain only alphanumerics/underscores",
        "package": "must be lowercase, start with a letter, and contain only alphanumerics/underscores",
    }

    while True:
        value = questionary.text(prompt_text, default=default).ask()

        if name_type == "executable" and value == "":
            return value
        
        if name_type == "executable":
            name_type = "base"
            
        if _validate_name(value, name_type):
            return value
            
        print(f"Invalid {name_type} name: {error_messages[name_type]}")


def _configure_qos_profile(component_type: str) -> dict:
    """
    Configure Quality of Service settings
    Args:
        component_type: Type of component (Publisher/Subscriber/Action)
    Returns:
        Dictionary of QoS configuration
    """
    print(f"\nQoS Configuration for {component_type}")
    print("----------------------------------")
    
    qos = {
        "reliability": questionary.select(
            "Reliability policy (message delivery guarantee):",
            choices=[
                "reliable",
                "best_effort"
            ],
            default="reliable"
        ).ask(),
        "durability": questionary.select(
            "Durability policy (message persistence):",
            choices=[
                "volatile",
                "transient_local"
            ],
            default="volatile"
        ).ask(),
        "history": questionary.select(
            "History policy (message queue handling):",
            choices=[
                "keep_last",
                "keep_all"
            ],
            default="keep_last"
        ).ask()
    }

    # Configure queue depth if using keep_last policy
    if qos["history"] == "keep_last":
        while True:
            depth = questionary.text("Queue depth (number of messages to store):", default="10").ask()
            if depth.isdigit() and int(depth) > 0:
                qos["depth"] = int(depth)
                break
            print("Invalid input - please enter a positive integer")

    # Optional advanced QoS settings
    if questionary.confirm("Configure advanced QoS settings?").ask():
        print("\nAdvanced QoS Settings")
        print("--------------------")
        qos.update({
            "deadline": _get_optional_float("Deadline duration in seconds (optional):"),
            "lifespan": _get_optional_float("Lifespan duration in seconds (optional):"),
            "liveliness": questionary.select(
                "Liveliness policy (node availability detection):",
                choices=["automatic", "manual_by_topic"],
                default="automatic"
            ).ask(),
            "liveliness_lease": _get_optional_float("Liveliness lease duration in seconds (optional):")
        })
    
    return qos


def _get_optional_float(prompt_text: str) -> float:
    """
    Prompt for optional floating point value
    Args:
        prompt_text: Description of the value being requested
    Returns:
        Float value or None if not provided
    """
    while True:
        value = questionary.text(prompt_text).ask()
        if not value:
            return None
        try:
            return float(value)
        except ValueError:
            print("Invalid input - please enter a number or leave blank")


def _get_positive_int(prompt_text: str, default: str) -> int:
    """
    Validate positive integer input
    Args:
        prompt_text: Description of the value needed
        default: Default value to suggest
    Returns:
        Validated positive integer
    """
    while True:
        value = questionary.text(prompt_text, default=default).ask()
        if value.isdigit() and int(value) >= 0:
            return int(value)
        print("Invalid input - please enter a non-negative integer")


def _select_interface(interface_type: str, interfaces: dict) -> str:
    """
    Select ROS interface with navigation support
    Args:
        interface_type: Type of interface (Publisher/Subscriber/Service/Client/Action)
        interfaces: Available interfaces dictionary
    Returns:
        Selected interface path (pkg/type/name)
    """
    # Determine interface category based on type
    if interface_type in ["Publisher", "Subscriber"]:
        category = "msg"
    elif interface_type in ["Service", "Client"]:
        category = "srv"
    elif interface_type in ["ActionServer", "ActionClient"]:
        category = "action"
    else:
        raise ValueError(f"Unsupported interface type: {interface_type}")
    
    while True:
        # Filter packages that contain the required interface type
        valid_pkgs = [p for p, v in interfaces.items() if category in v]
        if not valid_pkgs:
            print(f"No packages found with {category} interfaces")
            return None
            
        pkg = questionary.autocomplete(
            f"Select {interface_type} package:",
            choices=valid_pkgs
        ).ask()

        if pkg not in valid_pkgs:
            print(f"Package '{pkg}' not found or doesn't contain {category} interfaces")
            continue

        # Get available message/service/action types in package
        types = list(interfaces[pkg][category])
        if not types:
            print(f"Package '{pkg}' has no {category} interfaces")
            continue
            
        type_choices = ["< Back to package selection"] + types
        type_selected = questionary.select(
            f"Select {interface_type} type:",
            choices=type_choices
        ).ask()

        if type_selected == "< Back to package selection":
            continue
        return f"{pkg}/{category}/{type_selected}"


def _configure_components(count: int, comp_type: str, interfaces: dict) -> list:
    """
    Configure multiple components of the same type
    Args:
        count: Number of components to configure
        comp_type: Type of component (Publisher/Subscriber/Service/Client/Action)
        interfaces: Available ROS interfaces
    Returns:
        List of component configuration dictionaries
    """
    components = []
    for i in range(1, count + 1):
        print(f"\n{comp_type} Configuration ({i} of {count})")
        print("------------------------------")
        
        name = _get_validated_name(
            f"Enter {comp_type.lower()} name:",
            name_type="base"
        )
        interface = _select_interface(comp_type, interfaces)
        if not interface:
            continue
            
        # QoS needed for publishers, subscribers, and actions
        if comp_type in ["Publisher", "Subscriber", "ActionServer", "ActionClient"]:
            qos = _configure_qos_profile(comp_type)
        else:
            qos = None
        
        components.append({
            "name": name,
            "type": interface,
            "qos": qos
        })
    return components


def _configure_timers(count: int) -> list:
    """
    Configure timer components
    Args:
        count: Number of timers to configure
    Returns:
        List of timer configuration dictionaries
    """
    timers = []
    for i in range(1, count + 1):
        print(f"\nTimer Configuration ({i} of {count})")
        print("--------------------------")
        
        name = _get_validated_name(
            "Enter timer name:",
            default=f"timer_{i}",
            name_type="base"
        )
        while True:
            period = questionary.text("Enter timer period in seconds:", default="1.0").ask()
            try:
                period = float(period)
                if period > 0:
                    break
            except ValueError:
                pass
            print("Invalid input - please enter a positive number")
        
        timers.append({"name": name, "period": period})
    return timers


def gather_project_config(interfaces: dict) -> dict:
    """
    Main configuration workflow
    Args:
        interfaces: Dictionary of available ROS interfaces
    Returns:
        Complete project configuration dictionary
    """
    print("\n" + "="*60)
    print(" ROS 2 Project Configuration Wizard ".center(60, "-"))
    print("="*60 + "\n")

    # Workspace configuration
    use_existing = questionary.confirm("Use existing ROS workspace?").ask()
    ws_path = _get_workspace_path(use_existing)

    # Core project settings with name validation
    config = {
        "workspace_path": ws_path,
        "package_name": _get_validated_name(
            "Enter package name:",
            name_type="package"
        ),
        "language": questionary.select(
            "Select implementation language:",
            choices=[
                "python",
                "cpp"
            ],
            default="python"
        ).ask(),
        "node_name": _get_validated_name(
            "Enter node name:",
            name_type="base"
        ),
        "executable": _get_validated_name(
            "Enter executable name (If empty uses node_name):",
            default="",
            name_type="executable"
        ),
    }

    # Component quantities
    print("\nComponent Quantities")
    print("--------------------")
    config.update({
        "publishers": _get_positive_int("Number of publishers:", "0"),
        "subscribers": _get_positive_int("Number of subscribers:", "0"),
        "clients": _get_positive_int("Number of service clients:", "0"),
        "services": _get_positive_int("Number of service servers:", "0"),
        "action_servers": _get_positive_int("Number of action servers:", "0"),
        "action_clients": _get_positive_int("Number of action clients:", "0"),
        "timers": _get_positive_int("Number of timers:", "0")
    })

    # C++ specific configuration
    config["executor"] = questionary.select(
        "Select executor type:",
        choices=[
            "single_threaded",
            "multi_threaded"
        ],
        default="single_threaded"
    ).ask()

    # Detailed component configuration
    if config["publishers"] > 0:
        config["publisher_configs"] = _configure_components(
            config["publishers"], "Publisher", interfaces
        )
    
    if config["subscribers"] > 0:
        config["subscriber_configs"] = _configure_components(
            config["subscribers"], "Subscriber", interfaces
        )
    
    if config["clients"] > 0:
        config["client_configs"] = _configure_components(
            config["clients"], "Client", interfaces
        )
    
    if config["services"] > 0:
        config["service_configs"] = _configure_components(
            config["services"], "Service", interfaces
        )
    
    if config["action_servers"] > 0:
        config["action_server_configs"] = _configure_components(
            config["action_servers"], "ActionServer", interfaces
        )
    
    if config["action_clients"] > 0:
        config["action_client_configs"] = _configure_components(
            config["action_clients"], "ActionClient", interfaces
        )
    
    if config["timers"] > 0:
        config["timer_configs"] = _configure_timers(config["timers"])
    
    print("\n" + "="*60)
    print(" Configuration Complete ".center(60, "-"))
    print("="*60 + "\n")
    return config


if __name__ == "__main__":
    # Example interface structure for testing:
    interfaces = {
        "std_msgs": {"msg": ["String", "Int32"]},
        "example_interfaces": {"srv": ["AddTwoInts"]},
        "action_tutorials_interfaces": {"action": ["Fibonacci"]}
    }
    config = gather_project_config(interfaces)
    
    import json
    print(json.dumps(config, indent=2))