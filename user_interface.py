import questionary
from prompt_toolkit import prompt
from prompt_toolkit.completion import PathCompleter
import os


def _get_workspace_path_prompt() -> str:
    return prompt("📁 Select or enter workspace path: ", completer=PathCompleter())


def _get_validated_workspace_path(use_existing_ws:bool):
    while True:
        ws_path = _get_workspace_path_prompt()
        src_path = os.path.join(ws_path, "src")

        if use_existing_ws:
            if os.path.isdir(ws_path) and os.path.isdir(src_path):
                print(f"✅ Found existing workspace at: {ws_path}")
                return ws_path
            else:
                print("❌ Workspace path invalid or missing 'src'. Please try again.")
        else:
            try:
                os.makedirs(src_path, exist_ok=True)
                print(f"📁 Created new workspace at: {ws_path}")
                return ws_path
            except Exception as e:
                print(f"❌ Failed to create workspace: {e}")
                continue


def _ask_positive_int(label:str, default="0"):
    while True:
        value = questionary.text(label, default=default).ask()
        try:
            int_val = int(value)
            if int_val < 0:
                raise ValueError
            return int_val
        except ValueError:
            print("Please enter a non-negative integer.")

def _ask_type_with_back(label, interfaces):
    key = "msg" if ("Publisher" in label or "Subscriber" in label) else "srv"

    while True:
        choice = []

        for x in interfaces.keys():
            if key in interfaces[x]:
                choice.append(x)

        pkg = questionary.autocomplete(
            f"{label} Package: ",
            choices=choice
        ).ask()

        if pkg not in interfaces:
            print(f"'{pkg}' not found in available interface packages. Please choose a valid package.")
            continue

        if key not in interfaces[pkg]:
            print(f"'{pkg}' does not contain any '{key.upper()}' interfaces.")
            continue

        type_choices = list(interfaces[pkg][key])
        type_choices.insert(0, "<< Back")

        type_selected = questionary.select(
            f"{label} Type: ", choices=type_choices
        ).ask()

        if type_selected == "<< Back":
            print("Going back to package selection...\n")
            continue  
        else:
            return f"{pkg}/{key}/{type_selected}"

def _ask_component_details(count, label, interfaces: dict):
    components = []
    for i in range(count):
        print(f"\n📌 {label}: {i+1}/{count}")
        entry = {}
        entry["name"] = questionary.text(f"{label} Name: ").ask()
        entry["type"] = _ask_type_with_back(label, interfaces)

        components.append(entry)
    return components

def _ask_publishers(count, interfaces: dict):
    return _ask_component_details(count, "Publisher", interfaces)

def _ask_subscribers(count, interfaces: dict):
    return _ask_component_details(count, "Subscriber", interfaces)

def _ask_clients(count, interfaces: dict):
    return _ask_component_details(count, "Client", interfaces)

def _ask_services(count, interfaces: dict):
    return _ask_component_details(count, "Service", interfaces)

def _ask_timers(count):
    timers = []
    for i in range(count):
        print(f"\nTimer {i+1}/{count}")

        name = questionary.text(f"Timer name", default=f"timer_{i+1}").ask()
        while not name:
            print("Timer name cannot be empty.")
            name = questionary.text(f"Timer name", default=f"timer_{i+1}").ask()

        period = questionary.text("Timer period (seconds)", default="1.0").ask()
        try:
            period = float(period)
            if period <= 0:
                raise ValueError()
        except ValueError:
            print("Please enter a valid positive number for the period.")
            period = float(questionary.text("Timer period (seconds)", default="1.0").ask())

        timers.append({
            "name": name,
            "period": period
        })

    return timers



def get_info(interfaces: dict) -> dict:
    print("🔧 ROS 2 Project Setup")

    use_existing_ws = questionary.confirm("Use existing workspace?").ask()

    ws_path = _get_validated_workspace_path(use_existing_ws)

    pkg_name = questionary.text("📦 Package name").ask()
    language = questionary.select("💻 Language", choices=["python", "cpp"]).ask()

    node_name = questionary.text("🧠 Node name").ask()
    executable_name = questionary.text("⚙️ Executable name", default=node_name).ask()
    namespace = questionary.text("📛 Namespace", default="/").ask()

    pubs = _ask_positive_int("📤 Number of publishers", default="0")
    subs = _ask_positive_int("📥 Number of subscribers", default="0")
    clients = _ask_positive_int("🧾 Number of clients", default="0")
    servers = _ask_positive_int("🛠️ Number of services", default="0")
    timers = _ask_positive_int("⏲️ Number of timers", default="0")

    executor = None
    if language == "cpp":
        executor = questionary.select("🏃 Executor type (C++)", choices=["single", "multi"]).ask()

    config = {
        "use_existing_ws": use_existing_ws,
        "ws_path": ws_path,
        "package_name": pkg_name,
        "language": language,
        "node_name": node_name,
        "executable_name": executable_name,
        "namespace": namespace,
        "num_publishers": int(pubs),
        "num_subscribers": int(subs),
        "num_clients": int(clients),
        "num_services": int(servers),
        "num_timers": int(timers),
        "executor": executor
    }

    print("/n")
    config["publishers"] = _ask_publishers(pubs, interfaces)

    print("/n")
    config["subscribers"] = _ask_subscribers(pubs, interfaces)

    print("/n")
    config["clients"] = _ask_clients(pubs, interfaces)

    print("/n")
    config["services"] = _ask_services(pubs, interfaces)

    print("/n")
    config["services"] = _ask_timers(pubs)

    return config


if __name__ == "__main__":
    get_info({})
