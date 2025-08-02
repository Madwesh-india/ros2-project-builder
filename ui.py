import questionary
from prompt_toolkit import prompt
from prompt_toolkit.completion import PathCompleter
import os


def get_workspace_path_prompt():
    return prompt("📁 Select or enter workspace path: ", completer=PathCompleter())


def get_validated_workspace_path(use_existing_ws):
    while True:
        ws_path = get_workspace_path_prompt()
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


def ask_positive_int(label, default="0"):
    while True:
        value = questionary.text(label, default=default).ask()
        try:
            int_val = int(value)
            if int_val < 0:
                raise ValueError
            return int_val
        except ValueError:
            print("❌ Please enter a non-negative integer.")



def main():
    print("🔧 ROS 2 Project Setup")

    use_existing_ws = questionary.confirm("Use existing workspace?").ask()

    ws_path = get_validated_workspace_path(use_existing_ws)

    pkg_name = questionary.text("📦 Package name").ask()
    language = questionary.select("💻 Language", choices=["python", "cpp"]).ask()

    node_name = questionary.text("🧠 Node name").ask()
    executable_name = questionary.text("⚙️ Executable name", default=node_name).ask()
    namespace = questionary.text("📛 Namespace", default="/").ask()

    pubs = ask_positive_int("📤 Number of publishers", default="0")
    subs = ask_positive_int("📥 Number of subscribers", default="0")
    clients = ask_positive_int("🧾 Number of clients", default="0")
    servers = ask_positive_int("🛠️ Number of services", default="0")
    timers = ask_positive_int("⏲️ Number of timers", default="0")

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
        "num_servers": int(servers),
        "num_timers": int(timers),
        "executor": executor
    }

    print("\n📦 Collected Config:")
    for key, value in config.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
