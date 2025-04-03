# **ROS2 Lowwi Wakeword Detection Node**

A ROS 2 node that listens to audio input and detects custom wakewords using the Lowwi library. Triggers a callback when a wakeword is detected.

---

### ✅ Topics

| Topic             | Type                                  | Description                                |
|------------------|---------------------------------------|--------------------------------------------|
| `/audio_stamped` | `audio_tools/msg/AudioDataStamped`    | Raw audio input in S16LE format            |

---

### ⚙️ Parameters

| Parameter                      | Type               | Description                               |
|-------------------------------|--------------------|-------------------------------------------|
| `wakeword.phrases`            | `string[]`         | List of phrases to detect (e.g. "Hey Jarvis") |
| `wakeword.models`             | `string[]`         | Path to corresponding ONNX model files    |
| `wakeword.min_activations`    | `int[]`            | Minimum activations needed to trigger     |

---

### 🧩 Implementation Notes

- Written in **C++** using `rclcpp`.
- Subscribes to incoming audio and converts S16LE at 16KHz to `float32`.
- Uses **CLFML::LOWWI** to manage wakeword detection.
- Wakewords are loaded at runtime from ROS parameters.

---

### 🏁 Run Example

```bash
ros2 run lowwi_pkg lowwi_node \
  --ros-args \
  -p wakeword.phrases:="['Hey Mycroft', 'Hey Jarvis']" \
  -p wakeword.models:="['models/example_wakewords/hey_mycroft.onnx', 'models/example_wakewords/hey_jarvis.onnx']" \
  -p wakeword.min_activations:="[2, 2]"
```