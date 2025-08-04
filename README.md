# ROS 2 Project Builder

ROS 2 Project Builder is a development automation tool designed to streamline the creation and configuration of ROS 2 packages and nodes. It significantly reduces boilerplate by auto-generating publishers, subscribers, services, clients, action servers/clients, and more—with support for both Python and C++.

## ✅ Features (Implemented)

- 📦 Automatic detection of available ROS 2 interfaces (messages/services/actions)
- 🧠 Interactive terminal-based UI with autocomplete and validation
- 🔄 Auto-generated ROS 2 nodes in **Python** and **C++** with callbacks
- 🧭 QoS configuration support for publishers and subscribers
- ⚙️ Generation of:
  - Publishers and subscribers 
  - Service servers and clients
  - Action servers and clients

## 🚧 Features (Planned / In Progress)

- 🛠 Code generation for:
  - ROS 2 parameters
  - Launch files
- 🧭 QoS configuration for:
  - Service servers/clients
  - Action servers/clients
- 📚 Example usage:
  - Demonstrating variable access in callbacks and node logic

---

## 📦 Installation

Clone the repository and install the dependencies:

```bash
git clone https://github.com/Madwesh-india/ros2-project-builder.git
cd ros2-project-builder
pip install -r requirements.txt
````

> **Note**: Requires Python 3.8+ and ROS 2 (tested with Humble/Jazzy).

---

## 🚀 How to Run

Run the main script to start the project builder interface:

```bash
python3 main.py
```

You'll be guided through a series of prompts to configure your ROS 2 package, including selecting message types, node components (pub/sub/service/etc.), and QoS profiles.

---

## 🗂 Project Structure

```
ros2-project-builder/
├── LICENSE
├── main.py                # Entry point for the CLI tool
├── parse.py               # Interface parsing logic
├── README.md
├── requirements.txt
├── ros_utility.py         # Helper functions for ROS 2-specific logic
├── template/              # Code templates for Python and C++
│   ├── cpp_template.py
│   ├── __init__.py
│   └── python_template.py
└── user_interface.py      # Interactive UI and input validation
```

---

## 💡 Example Use Case

Want to generate a ROS 2 node with:

* 1 Publisher (`std_msgs/String`)
* 1 Subscriber (`sensor_msgs/Image`)
* A Service Server
* QoS settings set to `reliable`

Simply run:

```bash
python3 main.py
```

...and select the appropriate options. Your package with the fully scaffolded node will be ready to build and run.

---

## 📌 Contributing

Contributions, suggestions, and bug reports are welcome. Please open an issue or submit a pull request.

---

## 📄 License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details.
