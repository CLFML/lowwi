cmake_minimum_required(VERSION 3.1...3.14)

project(Lowwi VERSION 3.11.5 LANGUAGES CXX)

# Optionally build the examples
option(CLFML_LOWWI_BUILD_EXAMPLE_PROJECTS "Build example projects" ON)


# Internal flag used to determine whether or not to build examples automatically
set(CLFML_LOWWI_MAIN_PROJECT OFF)

if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(CMAKE_CXX_STANDARD 17)
    set(CLFML_LOWWI_MAIN_PROJECT ON)
else()
    set(CLFML_LOWWI_BUILD_EXAMPLE_PROJECTS OFF)
endif()

if (POLICY CMP0077)
    # Allow CMake 3.13+ to override options when using FetchContent / add_subdirectory.
    cmake_policy(SET CMP0077 NEW)
endif()

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
    cmake_policy(SET CMP0135 NEW)
endif()

# Create the Lowi Library
add_library(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/src/lowwi.cpp
                            ${CMAKE_CURRENT_LIST_DIR}/src/lowwi_melspectrogram.cpp
                            ${CMAKE_CURRENT_LIST_DIR}/src/lowwi_embedding.cpp
                            ${CMAKE_CURRENT_LIST_DIR}/src/lowwi_wakeword.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE "-Wall")
# Create alias CLFML::Lowi
add_library(CLFML::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src)


# Check if onnxruntime target exists?
if (TARGET onnxruntime)
    target_link_libraries(${PROJECT_NAME} onnxruntime)
else()
   # Fetch onnx manually
    include(FetchContent)
 
    set(ONNXRUNTIME_ROOTDIR ${CMAKE_CURRENT_LIST_DIR}/lib/x64)
    # Check if we are compiling with MSVC for Windows
    if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        # Get onnx release for windows x64 from Github page
        FetchContent_Declare(
        onnxruntime
        URL https://github.com/microsoft/onnxruntime/releases/download/v1.20.1/onnxruntime-win-x64-1.20.1.zip
        SOURCE_DIR ${ONNXRUNTIME_ROOTDIR}
        )
        FetchContent_MakeAvailable(onnxruntime)

        # Add ONNX includes to include path
        target_include_directories(${PROJECT_NAME} PUBLIC ${ONNXRUNTIME_ROOTDIR}/include)

        # Link ONNX directories, which contain the .lib files for ONNX
        link_directories(${PROJECT_NAME} ${ONNXRUNTIME_ROOTDIR}/lib)

        # Link the specific .lib file libraries to LOWWI
        target_link_libraries(${PROJECT_NAME} onnxruntime onnxruntime_providers_shared)

        # Turn off some compile warnings which are windows specific (but not fixable, due to it being an ONNX problem)
        target_compile_options(${PROJECT_NAME} PRIVATE /wd4820  # Suppresses warning C4820
                                                       /wd4514  # Suppresses warning C4514
                                                       /wd4710  # Suppresses warning C4710
                                                       /Qspectre # Enable spectre vulnerability mitigations
                                                       )

        # Get list of all dll files in the onnxruntime lib directory
        file(GLOB onnx_dll_files ${ONNXRUNTIME_ROOTDIR}/lib/*.dll)

        # Copy the DLL files to the executable dir.
        add_custom_target(copy_onnxruntime_dll ALL
                          COMMAND ${CMAKE_COMMAND} -E copy ${onnx_dll_files} $<TARGET_FILE_DIR:${PROJECT_NAME}>
                          COMMENT "Copying ONNX dll files to build folder"
                            )
    else()
        # Get Linux x64 onnx release from GitHub
        FetchContent_Declare(
            onnxruntime
            URL https://github.com/microsoft/onnxruntime/releases/download/v1.18.0/onnxruntime-linux-x64-1.18.0.tgz
            SOURCE_DIR ${ONNXRUNTIME_ROOTDIR}
        )
        FetchContent_MakeAvailable(onnxruntime)
        # Add ONNX library include files to include path
        target_include_directories(${PROJECT_NAME} PUBLIC ${ONNXRUNTIME_ROOTDIR}/include)
        # Link the directories to project
        link_directories(${PROJECT_NAME} ${ONNXRUNTIME_ROOTDIR}/lib)
        # Link the onnxruntime library to project
        target_link_libraries(${PROJECT_NAME} onnxruntime)
    endif()

endif()

# Models have to be copied to executable directory
set(MODELS_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/models)

add_custom_target(copy_models ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${MODELS_SOURCE_DIR} $<TARGET_FILE_DIR:${PROJECT_NAME}>/models
    COMMENT "Copying models directory to build folder"
)

# Build examples?
if(CLFML_LOWWI_BUILD_EXAMPLE_PROJECTS)
    # Microphone demo has dependency on SDL2
    # Windows dependencies are annoying (as always has been)
    # So we include it manually into build
    if(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        include(FetchContent)

        # Fetch latest windows release of SDL2
        FetchContent_Declare(
            SDL2
            URL https://github.com/libsdl-org/SDL/releases/download/release-2.30.10/SDL2-devel-2.30.10-VC.zip
            SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/SDL2
        )
        FetchContent_MakeAvailable(SDL2)

        # include the SDL2-config manually, as adding to modules did not work for me
        include("${CMAKE_CURRENT_LIST_DIR}/lib/SDL2/cmake/sdl2-config.cmake")

        # Get all dll files in SDL2 library folder
        file(GLOB sdl2_dll_files ${CMAKE_CURRENT_LIST_DIR}/lib/SDL2/lib/x64/*.dll)

        # Copy all dll_files of SDL2 to executable dir
        add_custom_target(copy_sdl2_dll ALL
                          COMMAND ${CMAKE_COMMAND} -E copy ${sdl2_dll_files} $<TARGET_FILE_DIR:${PROJECT_NAME}>
                          COMMENT "Copying dll file to build folder"
                          )
    else()
        # On linux & mac, we want to use system/brew provided packages
        find_package(SDL2 REQUIRED)
    endif()

    # Example fragments for demo_fragments should be copied to executable dir too
    add_custom_target(copy_example_fragments ALL
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_LIST_DIR}/example/LOWWI_demo_fragment/example_fragments $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMENT "Copying example fragments to build folder"
    )
    
    # Add the demo_fragments demo
    add_executable(LOWWI_demo ${CMAKE_CURRENT_LIST_DIR}/example/LOWWI_demo_fragment/demo.cpp)
    
    # Add the mic demo
    add_executable(LOWWI_demo_mic ${CMAKE_CURRENT_LIST_DIR}/example/LOWWI_demo_mic/demo_mic.cpp
                                  ${CMAKE_CURRENT_LIST_DIR}/example/LOWWI_demo_mic/audio_async/audio_async.cpp)
    
    # Set the example fragments path
    set(LOWWI_EXAMPLE_FRAGMENTS_PATH $<TARGET_FILE_DIR:${PROJECT_NAME}>)
    target_compile_definitions(LOWWI_demo PUBLIC -DCLFML_LOWWI_EXAMPLE_FRAGMENTS="${LOWWI_EXAMPLE_FRAGMENTS_PATH}")

    # Link Lowwi demo's with Lowwi
    target_link_libraries(LOWWI_demo PRIVATE Lowwi)
    target_include_directories(LOWWI_demo_mic PRIVATE ${SDL2_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/example/LOWWI_demo_mic/audio_async)
    target_link_libraries(LOWWI_demo_mic PUBLIC Lowwi ${SDL2_LIBRARIES})
endif()

