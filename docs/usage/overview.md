# Overview

The library offers a relatively simple abstraction for use with self-trained openwakeword models.

## API
The library API consists of the following functions:
```cpp
namespace CLFML::LOWWI {

/**
 * @brief The user-provided struct which provides the classifier model settings
 * @param phrase The model identifier which get's passed into the callback function when triggered.
 * @param model_path The classifier model path
 * @param cbfunc Function pointer to a callback function that get's called when wakeword is triggered
 * @param cb_arg Additional function argument that get's passed in the callback when wakeword is triggered
 *               (void pointer)
 * @param refractory The negative feedback on activation, when activated this factor makes the debouncing work :)
 *                   Increasing it gives a higher negative bounty, thus dampening any further activations.
 *                   (Default = 20)
 *
 * @param threshold The threshold determines whether model confidence is worth acting on (default = 0.5f)
 * @param min_activations Number of activations the model should have to be considered detected
 *                       (Default = 5, but depends on how well the model is trained and how easy to detect)
 *                       (It's like a debouncing system)
 */
struct Lowwi_word_t
{
    std::string phrase = "";
    std::filesystem::path model_path = std::filesystem::path("");
    std::function<void(Lowwi_ctx_t, std::shared_ptr<void>)> cbfunc = nullptr;
    std::shared_ptr<void> cb_arg = nullptr;
    int refractory = 20;
    float threshold = 0.5f;
    uint8_t min_activations = 5;
    uint8_t debug = false;
};

/**
* @brief Add new wakeword to detection runtime
* @param lowwi_word Struct with the properties 
*                   of the to be added wakeword
*/
void Lowwi::add_wakeword(const Lowwi_word_t& lowwi_word);

/**
* @brief Remove wakeword from detection runtime
* @param model_path Model path of the to be removed wakeword
*/
void remove_wakeword(std::filesystem::path model_path);

/**
* @brief Runs wakeword detection runtime on audio samples
* @param audio_samples Audio samples to parse
*/
void Lowwi::run(const std::vector<float> &audio_samples);
}
```
The comments above the functions describe fairly well what each function does. Here some additional notes;

## CMake integration
This project uses CMake for generating the build files. The CMakeLists is configured as follows:

### Target(s)
The main target defined in the CMakeLists is the `Lowwi` target. **As this will not be the only library released under the CLFML organisation, we chose to namespace it and call it `CLFML::Lowwi`**. 

Other targets which are defined in the CMake files of this project are the Unit tests.


### Configuration options

Some of the configuration options which can be used to generate the CMake project are:

- `CLFML_FACE_DETECTOR_BUILD_EXAMPLE_PROJECTS`; Build example projects (fragment & mic demo) (Default=ON, *only when project is not part of other project)


### Integrating it into your own project
Here are some CMake snippets which indicate how this project might be included into your own CMake project.

!!! example "Automatically fetching from GitHub"
    CPU only:
    ```cmake
    include(FetchContent)

    FetchContent_Declare(
     Lowwi
     GIT_REPOSITORY https://github.com/CLFML/lowwi.git
     GIT_TAG        main
    )
    FetchContent_MakeAvailable(Lowwi)

    ...

    target_link_libraries(YOUR_MAIN_EXECUTABLE_NAME CLFML::Lowwi)
    ```

!!! example "Manually using add_subdirectory"
    First make sure that this library is cloned into the project directory!
        CPU only:
    ```cmake
    add_subdirectory(lowwi)
    ...

    target_link_libraries(YOUR_MAIN_EXECUTABLE_NAME CLFML::Lowwi)
    ```
