# Getting started with Pixi

Pixi makes cross-platform ROS 2 development easy. You can build and run both capture and playback nodes on **Linux and Windows**—with no system-wide ROS install.

---
## Node details
For more details about the node see: [ROS2 Lowwi Wakeword Detection Node page in implementation tab](/lowwi/about/implementation_ros/)

## 📦 Install Pixi

**Linux**:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

**Windows** (PowerShell):

```powershell
powershell -ExecutionPolicy ByPass -c "irm -useb https://pixi.sh/install.ps1 | iex"
```

---

## 🚀 Clone & Build Project

```bash
git clone https://github.com/CLFML/lowwi.git
cd lowwi
pixi install
pixi run build
```

Or launch VSCode with the environment:

```bash
pixi run vscode
```

> ✅ **Note (Windows):** Always build in **Release** or **RelWithDebInfo**, not Debug!  
> *(Ctrl+Shift+P → "CMake: Select Variant")*

---

## ⚡ Using as a Pixi Dependency

Want to use `custom_pkg` from another Pixi-based project?

### 1. Init a new project

```bash
mkdir my_project && cd my_project
pixi init
```

### 2. Edit `pixi.toml`

Add these:

```toml
[project]
channels = [
  "https://fast.prefix.dev/conda-forge",
  "https://prefix.dev/robostack-jazzy",
  "https://clfml.github.io/conda_ros2_jazzy_channel/"
]

[dependencies]
ros-jazzy-ros-base = "*"
ros-jazzy-audio-tools = "*"
ros-jazzy-lowwi = "*"
colcon-common-extensions = "*"
rosdep = "*"
```

### 🧠 Optional: VSCode Support

Add to your `pixi.toml`:

```toml
[target.linux-64.dependencies]
python-devtools = "*"
pybind11 = "*"
numpy = "*"

[target.win-64.dependencies]
python-devtools = "*"

[target.linux-64.tasks]
vscode = 'env -u LD_LIBRARY_PATH code .'

[target.win-64.tasks]
vscode = "code ."
```

---

### 3. Run the node

```bash
pixi install
# Copy the models dir from Lowwi Git repo to your own project folder
pixi shell
# run this command to launch lowwi node with default models:
ros2 run lowwi lowwi_node \
  --ros-args \
  -p wakeword.phrases:="['Hey Mycroft', 'Hey Jarvis']" \
  -p wakeword.models:="['models/example_wakewords/hey_mycroft.onnx', 'models/example_wakewords/hey_jarvis.onnx']" \
  -p wakeword.min_activations:="[2, 2]"
```

### 3.5 Run the node with launch-file

For automation purposes you can use a launch-file ("lowwi_launch.py") like this:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lowwi',
            executable='lowwi_node',
            name='lowwi_node',
            output='screen',
            parameters=['params.yaml']  # Path to your params file
        )
    ])
```

with `params.yaml` file:
```yaml
lowwi_node:
  ros__parameters:
    wakeword.phrases: ["Hey Mycroft", "Hey Jarvis"]
    wakeword.models: ["models/example_wakewords/hey_mycroft.onnx", "models/example_wakewords/hey_jarvis.onnx"]
    wakeword.min_activations: [2,2]
```
In `pixi shell` run:

```bash
ros2 launch lowwi_launch.py
```

