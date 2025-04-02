cmake_minimum_required(VERSION 3.13)  # Raise minimum since CMP0077 needs 3.13+

project(lowwi VERSION 3.11.5 LANGUAGES C CXX)

# === Options ===
option(CLFML_LOWWI_BUILD_EXAMPLE_PROJECTS "Build example projects" ON)
option(CLFML_LOWWI_ROS2_PACKAGE_BUILD "Build a ROS2 package (requires ROS2 jazzy)" OFF)
option(CLFML_LOWWI_CONDA_PACKAGING "Build as conda package" OFF)
option(CLFML_LOWWI_NATIVE_DEB_PACKAGING "Build native deb package (sets paths to /opt/ros/jazzy/lib)" OFF)

# === C++ standard and policies ===
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(POLICY CMP0077)
    cmake_policy(SET CMP0077 NEW)
endif()

if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
    cmake_policy(SET CMP0135 NEW)
endif()

# === Main project check ===
set(CLFML_LOWWI_MAIN_PROJECT OFF)
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR AND NOT CLFML_LOWWI_ROS2_PACKAGE_BUILD)
    set(CLFML_LOWWI_MAIN_PROJECT ON)
else()
    set(CLFML_LOWWI_BUILD_EXAMPLE_PROJECTS OFF)
endif()

# === Lowwi Library ===
add_library(lowwi_lib
    src/lowwi.cpp
    src/lowwi_melspectrogram.cpp
    src/lowwi_embedding.cpp
    src/lowwi_wakeword.cpp
)
add_library(CLFML::lowwi ALIAS lowwi_lib)
target_include_directories(lowwi_lib PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src)

if(MSVC)
    target_compile_options(lowwi_lib PRIVATE /wd4625 /wd4626)
endif()

# === ONNX Runtime Integration ===
include(FetchContent)
set(ONNXRUNTIME_ROOTDIR ${CMAKE_CURRENT_LIST_DIR}/lib/x64)

if(CLFML_LOWWI_NATIVE_DEB_PACKAGING AND NOT EXISTS "${ONNXRUNTIME_ROOTDIR}/include")
    unset(FETCHCONTENT_FULLY_DISCONNECTED CACHE)
    message(STATUS "ONNX Runtime source directory missing — enabling network fetch")
endif()

if(TARGET onnxruntime)
    target_link_libraries(lowwi_lib onnxruntime)
else()
    if(MSVC)
        FetchContent_Declare(
            onnxruntime
            URL https://github.com/microsoft/onnxruntime/releases/download/v1.17.1/onnxruntime-win-x64-1.17.1.zip
            SOURCE_DIR ${ONNXRUNTIME_ROOTDIR}
        )
        FetchContent_MakeAvailable(onnxruntime)

        target_include_directories(lowwi_lib PUBLIC ${ONNXRUNTIME_ROOTDIR}/include)
        target_link_libraries(lowwi_lib
            "${ONNXRUNTIME_ROOTDIR}/lib/onnxruntime.lib"
            "${ONNXRUNTIME_ROOTDIR}/lib/onnxruntime_providers_shared.lib"
        )

        target_compile_options(lowwi_lib PRIVATE
            /wd4820 /wd4514 /wd4710 /wd4625 /wd4626 /wd5045 /wd4365
        )

        file(GLOB onnx_dll_files ${ONNXRUNTIME_ROOTDIR}/lib/*.dll)
        add_custom_target(copy_onnxruntime_dll ALL
            COMMAND ${CMAKE_COMMAND} -E copy ${onnx_dll_files} $<TARGET_FILE_DIR:lowwi_lib>
            COMMENT "Copying ONNX DLLs to build folder"
        )
    else()
        FetchContent_Declare(
            onnxruntime
            URL https://github.com/microsoft/onnxruntime/releases/download/v1.18.1/onnxruntime-linux-x64-1.18.1.tgz
            SOURCE_DIR ${ONNXRUNTIME_ROOTDIR}
        )
        FetchContent_MakeAvailable(onnxruntime)

        target_include_directories(lowwi_lib PUBLIC ${ONNXRUNTIME_ROOTDIR}/include)
        target_link_libraries(lowwi_lib ${ONNXRUNTIME_ROOTDIR}/lib/libonnxruntime.so)
    endif()
endif()

# === Copy Models ===
set(MODELS_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/models)
add_custom_target(copy_models ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${MODELS_SOURCE_DIR} $<TARGET_FILE_DIR:lowwi_lib>/models
    COMMENT "Copying models directory to build folder"
)

# === ROS2 Package ===
if(CLFML_LOWWI_ROS2_PACKAGE_BUILD)
    if(CLFML_LOWWI_CONDA_PACKAGING)
        message(STATUS "Conda packaging enabled")
        add_compile_definitions(-DCLFML_LOWWI_CONDA_PACKAGING)
    endif()
    if(CLFML_LOWWI_NATIVE_DEB_PACKAGING)
        message(STATUS "Native deb packaging enabled")
        add_compile_definitions(-DCLFML_LOWWI_NATIVE_DEB_PACKAGING)
        file(GLOB onnx_so_files ${CMAKE_CURRENT_LIST_DIR}/lib/x64/lib/*.so*)
	install(FILES ${onnx_so_files} DESTINATION lib/)
    endif()

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(std_srvs REQUIRED)
    find_package(SDL2)
    find_package(audio_tools)

    target_link_libraries(lowwi_lib ${cv_bridge_LIBRARIES})

    add_executable(lowwi_node bindings/ros2/lowwi_node.cpp)

    rosidl_generate_interfaces(${PROJECT_NAME}
        bindings/ros2/msg/WakeWord.msg
        DEPENDENCIES std_msgs
    )

    ament_export_dependencies(rosidl_default_runtime)
    ament_target_dependencies(lowwi_node
        rclcpp std_msgs std_srvs audio_tools
    )

    rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
    target_link_libraries(lowwi_node lowwi_lib "${cpp_typesupport_target}" ${audio_tools_LIBRARIES})
    target_include_directories(lowwi_node PUBLIC ${audio_tools_INCLUDE_DIRS})

    if(MSVC)
        file(GLOB onnx_dlls ${ONNXRUNTIME_ROOTDIR}/lib/*.dll)
        file(GLOB onnx_libs ${ONNXRUNTIME_ROOTDIR}/lib/*.lib)
        install(FILES ${onnx_dlls} ${onnx_libs} DESTINATION lib/)
    else()
        file(GLOB onnx_so_files lib/*.so*)
        install(FILES ${onnx_so_files} DESTINATION lib/)
    endif()

    set(LOWWI_EXAMPLE_FRAGMENTS_PATH ${CMAKE_CURRENT_LIST_DIR}/example/LOWWI_demo_fragment/example_fragments)
    target_compile_definitions(lowwi_lib PUBLIC -DCLFML_LOWWI_EXAMPLE_FRAGMENTS="${LOWWI_EXAMPLE_FRAGMENTS_PATH}")

    install(TARGETS lowwi_lib DESTINATION lib/)
    install(TARGETS lowwi_node DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY ${MODELS_SOURCE_DIR} DESTINATION lib/${PROJECT_NAME})
    install(FILES bindings/ros2/set_ld_path.sh DESTINATION lib/${PROJECT_NAME})

    ament_package()
endif()

# === Examples ===
if(CLFML_LOWWI_BUILD_EXAMPLE_PROJECTS)
    if(MSVC)
        FetchContent_Declare(
            SDL2
            URL https://github.com/libsdl-org/SDL/releases/download/release-2.30.10/SDL2-devel-2.30.10-VC.zip
            SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/lib/SDL2
        )
        FetchContent_MakeAvailable(SDL2)
        include("${CMAKE_CURRENT_LIST_DIR}/lib/SDL2/cmake/sdl2-config.cmake")

        file(GLOB sdl2_dlls ${CMAKE_CURRENT_LIST_DIR}/lib/SDL2/lib/x64/*.dll)
        add_custom_target(copy_sdl2_dll ALL
            COMMAND ${CMAKE_COMMAND} -E copy ${sdl2_dlls} $<TARGET_FILE_DIR:lowwi_lib>
            COMMENT "Copying SDL2 DLLs to build folder"
        )
    else()
        find_package(SDL2 REQUIRED)
    endif()

    add_custom_target(copy_example_fragments ALL
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_CURRENT_LIST_DIR}/example/LOWWI_demo_fragment/example_fragments
        $<TARGET_FILE_DIR:lowwi_lib>
        COMMENT "Copying example fragments to build folder"
    )

    add_executable(LOWWI_demo example/LOWWI_demo_fragment/demo.cpp)
    add_executable(LOWWI_demo_mic
        example/LOWWI_demo_mic/demo_mic.cpp
        example/LOWWI_demo_mic/audio_async/audio_async.cpp
    )

    set(LOWWI_EXAMPLE_FRAGMENTS_PATH $<TARGET_FILE_DIR:lowwi_lib>)
    target_compile_definitions(LOWWI_demo PUBLIC -DCLFML_LOWWI_EXAMPLE_FRAGMENTS="${LOWWI_EXAMPLE_FRAGMENTS_PATH}")

    target_link_libraries(LOWWI_demo PRIVATE lowwi_lib)
    target_include_directories(LOWWI_demo_mic PRIVATE
        ${SDL2_INCLUDE_DIRS}
        ${CMAKE_CURRENT_LIST_DIR}/example/LOWWI_demo_mic/audio_async
    )
    target_link_libraries(LOWWI_demo_mic PUBLIC lowwi_lib ${SDL2_LIBRARIES})
endif()
