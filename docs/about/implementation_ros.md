# **ROS 2 Lowwi Wakeword Detection Node**

A ROS 2 node that listens to audio input and detects custom wakewords using the Lowwi library. Triggers a callback and publishes detection results when a wakeword is detected.

---

### ✅ Topics

| Topic             | Type                                  | Description                                |
|------------------|---------------------------------------|--------------------------------------------|
| `/audio_stamped` (default) | `audio_tools/msg/AudioDataStamped` | Raw audio input in S16LE format            |
| `/lowwi_ww` (default)       | `lowwi/msg/WakeWord`              | Published when a wakeword is detected      |

---

### ⚙️ Parameters

| Parameter                      | Type               | Description                                                            |
|-------------------------------|--------------------|------------------------------------------------------------------------|
| `audio_topic`                 | `string`           | Topic to subscribe to for audio input (default: `/audio_stamped`)     |
| `output_topic`               | `string`           | Topic to publish wakeword detection messages (default: `/lowwi_ww`)   |
| `wakeword.phrases`            | `string[]`         | List of phrases to detect (e.g. `["Hey Jarvis"]`)                      |
| `wakeword.models`             | `string[]`         | Path to corresponding ONNX model files                                |
| `wakeword.min_activations`    | `int[]` *(optional)* | Minimum activations to trigger wakeword (default: `5` if omitted)     |
| `wakeword.refractory`         | `int[]` *(optional)* | Refractory period (cooldown) per wakeword (default: `20`)             |
| `wakeword.threshold`          | `float[]` *(optional)* | Confidence threshold per wakeword (default: `0.5`)                  |

---

### 🧩 Implementation Notes

- Written in **C++** using `rclcpp`.
- Subscribes to incoming audio with **S16LE at 16kHz**
- Uses **CLFML::LOWWI** to manage wakeword detection.
- Wakewords and model settings are loaded at runtime via parameters.
- Gracefully handles missing optional parameters by applying sensible defaults.

---

### 📤 WakeWord Message Structure

Published to `/lowwi_ww` (or a user-defined topic), of type `lowwi/msg/WakeWord`:

| Field               | Type             | Description                                      |
|--------------------|------------------|--------------------------------------------------|
| `header.stamp`     | `builtin_interfaces/Time` | Timestamp of the triggering audio sample        |
| `header.frame_id`  | `string`         | Frame from which the audio was captured         |
| `wakeword_detected`| `bool`           | Always `true` when message is published         |
| `wakeword_name`    | `string`         | Name/phrase of the detected wakeword            |
| `wakeword_confidence` | `float32`     | Detection confidence (0.0 – 1.0)                 |

---

### 🏁 Run Example

```bash
ros2 run lowwi_pkg lowwi_node \
  --ros-args \
  -p wakeword.phrases:="['Hey Mycroft', 'Hey Jarvis']" \
  -p wakeword.models:="['models/hey_mycroft.onnx', 'models/hey_jarvis.onnx']" \
  -p wakeword.min_activations:="[2, 3]" \
  -p wakeword.refractory:="[25, 30]" \
  -p wakeword.threshold:="[0.6, 0.65]" \
  -p audio_topic:="/my_custom_audio" \
  -p output_topic:="/custom_wakeword_output"
```