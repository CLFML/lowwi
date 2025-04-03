# 🐧 Getting Started with Native ROS

Prefer a traditional system-wide install? Use the prebuilt `.deb` package for **Ubuntu Noble / ROS 2 Jazzy**.

---

## Node details
For more details about the node see: [ROS2 Lowwi Wakeword Detection Node page in implementation tab](/lowwi/about/implementation_ros/)

## 📦 Install package via `.deb`

Install the latest `.deb` package directly from [Releases](https://github.com/CLFML/ROS2_Audio_Tools/releases):

```bash
curl -s https://api.github.com/repos/CLFML/lowwi/releases/latest \
  | grep "browser_download_url.*deb" \
  | cut -d : -f 2,3 \
  | tr -d \" \
  | wget -qi -
sudo dpkg -i ./ros-jazzy-*.deb
```

---

## ✅ Run the Nodes

Make sure ROS is sourced:

```bash
source /opt/ros/jazzy/setup.sh
```
**Then copy the models dir from Lowwi Git repo to your own project folder**

When model dir present run the node:

```bash
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
In `sourced shell` run:

```bash
ros2 launch lowwi_launch.py
```
