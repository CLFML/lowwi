# Lowwi - Lightweight openwakeword implementation written in C++.
 Wakeword detector based on OpenWakeWord-cpp but faster, cleaner and more lightweight.

- Plain C/C++ implementation with minimal dependencies (ONNXRuntime)
- Runs on ARM as well (Tested on RPI 3,4 and 5)
- Written with performance in mind, < 3% CPU utilization on x86/64 CPUs

## API Features
This library offers support for:
- Single-model wakeword detection
- Multi-model wakeword detection

### Single-model wakeword detection

This is some example code for wakeword detection:

```cpp
...
 /* Create new Lowwi runtime */
CLFML::LOWWI::Lowwi ww_runtime;

/* Create new wakeword */
CLFML::LOWWI::Lowwi_word_t ww;
ww.cbfunc = wakeword_callback;
ww.model_path = "models/example_wakewords/hey_mycroft.onnx";
ww.phrase = "Hey Mycroft";
    
/* Add wakeword to ww-runtime */
ww_runtime.add_wakeword(ww);

std::vector<float> audio_samples;
...

while(1) {
    ww_runtime.run(audio_samples);
}

```


### Multi-model wakeword detection

Just create two or more Lowwi_word_t's:

```cpp
...
/* Create new wakeword */
CLFML::LOWWI::Lowwi_word_t ww;
ww.cbfunc = wakeword_callback;
ww.model_path = "models/example_wakewords/hey_mycroft.onnx";
ww.phrase = "Hey Mycroft";
    
/* Add wakeword to ww-runtime */
ww_runtime.add_wakeword(ww);

/* Create new wakeword */
CLFML::LOWWI::Lowwi_word_t ww2;
ww2.cbfunc = wakeword_callback;
ww2.model_path = "models/example_wakewords/hey_jarvis.onnx";
ww2.phrase = "Hey Jarvis";
    
/* Add wakeword to ww-runtime */
ww_runtime.add_wakeword(ww2);
```

### Example code
For a full example showcasing the API functions see the example code in [example/LOWWI_demo_mic/demo.cpp](example/LOWWI_demo_mic/demo_mic.cpp).

## Building with CMake
Before using this library you will need the following packages installed:

- Working C++ compiler (GCC, Clang, MSVC (2017 or Higher))
- CMake
- Ninja (**Optional**, but preferred)
- SDL (when using Lowwi mic example)

### Running the examples (CPU)
1. Clone this repo
2. Run:
```bash
cmake . -B build -G Ninja
```
3. Let CMake generate and run:
```bash
cd build && ninja
```
4. After building you can run (linux & mac):
```bash
./LOWWI_demo
```
or (if using windows)
```bat
LOWWI_demo.exe
```

### Usage with ROS Pixi (Highly recommeneded with ROS2)
**See the [wiki page!](https://clfml.github.io/lowwi/usage/build_environment/ros2_pixi_build_linux_windows/)**

## Usage with ROS prebuilt .deb package
**See the [wiki page!](https://clfml.github.io/lowwi/usage/build_environment/usage_with_native_linux/)**


### Using it in your project as library
Add this to your top-level CMakeLists file:
```cmake
include(FetchContent)

FetchContent_Declare(
    Lowwi
    GIT_REPOSITORY https://github.com/CLFML/lowwi
    GIT_TAG main
    SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/Lowwi
)
FetchContent_MakeAvailable(Lowwi)
...
target_link_libraries(YOUR_EXECUTABLE CLFML::Lowwi)
```
Or manually clone this repo and add the library to your project using:
```cmake
add_subdirectory(lowwi)
...
target_link_libraries(YOUR_EXECUTABLE CLFML::Lowwi)
```


## Aditional documentation
See our [wiki](https://clfml.github.io/lowwi/)...

## Todo
- Add language bindings for Python, C# and Java
- Add support for MakeFiles, Bazel and Conan
- Add Unit-tests 
- Add TPU support
- Add automatic wakeword training
- Merge melspectrogram & embedding model?

## License
This work is licensed under the Apache 2.0 license.