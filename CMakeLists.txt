cmake_minimum_required(VERSION 3.1...3.14)

project(Lowwi VERSION 3.11.5 LANGUAGES CXX)

set(MAIN_PROJECT OFF)


# Optionally build the examples
option(CLFML_LOWWI_BUILD_EXAMPLE_PROJECTS "Build example projects" ON)

if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(CMAKE_CXX_STANDARD 17)
    set(MAIN_PROJECT ON)
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

add_library(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/src/lowwi.cpp
                            ${CMAKE_CURRENT_LIST_DIR}/src/lowwi_melspectrogram.cpp
                            ${CMAKE_CURRENT_LIST_DIR}/src/lowwi_embedding.cpp
                            ${CMAKE_CURRENT_LIST_DIR}/src/lowwi_wakeword.cpp)
add_library(CLFML::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src)

target_link_libraries(${PROJECT_NAME})

if (TARGET onnxruntime)
    target_link_libraries(${PROJECT_NAME} onnxruntime)
else()
    set(ONNX_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/x64)
    include(FetchContent)
    FetchContent_Declare(
    onnxruntime
    URL https://github.com/microsoft/onnxruntime/releases/download/v1.18.0/onnxruntime-linux-x64-1.18.0.tgz
    SOURCE_DIR ${ONNX_PATH}
    )
    FetchContent_MakeAvailable(onnxruntime)
    add_library(onnxruntime SHARED IMPORTED )
    # You can define two import-locations: one for debug and one for release.
    set_target_properties(onnxruntime PROPERTIES IMPORTED_LOCATION ${ONNX_PATH}/lib/libonnxruntime.so)
    target_include_directories(onnxruntime INTERFACE ${ONNX_PATH}/include)
    target_link_libraries(${PROJECT_NAME} onnxruntime)
endif()

set(LOWWI_MODEL_DIR ${CMAKE_CURRENT_LIST_DIR}/models)
target_compile_definitions(${PROJECT_NAME} PUBLIC -DCLFML_LOWWI_MODEL_DIR="${LOWWI_MODEL_DIR}")

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

set(MODELS_SOURCE_DIR ${CMAKE_CURRENT_LIST_DIR}/models)
set(MODELS_DEST_DIR ${CMAKE_BINARY_DIR}/models)

add_custom_target(copy_models ALL
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${MODELS_SOURCE_DIR} ${MODELS_DEST_DIR}
    COMMENT "Copying models directory to build folder"
)

if(CLFML_LOWWI_BUILD_EXAMPLE_PROJECTS)
    find_package(SDL2 REQUIRED)
    set(LOWWI_EXAMPLE_FRAGMENTS_PATH ${CMAKE_BINARY_DIR}/example_fragments/)
    add_custom_target(copy_example_fragments ALL
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_LIST_DIR}/example/LOWWI_demo_fragment/example_fragments ${LOWWI_EXAMPLE_FRAGMENTS_PATH}
        COMMENT "Copying example fragments to build folder"
    )
    add_executable(LOWWI_demo ${CMAKE_CURRENT_LIST_DIR}/example/LOWWI_demo_fragment/demo.cpp)
    add_executable(LOWWI_demo_mic ${CMAKE_CURRENT_LIST_DIR}/example/LOWWI_demo_mic/demo_mic.cpp
                                  ${CMAKE_CURRENT_LIST_DIR}/example/LOWWI_demo_mic/audio_async/audio_async.cpp)
    target_compile_definitions(LOWWI_demo PUBLIC -DCLFML_LOWWI_EXAMPLE_FRAGMENTS="${LOWWI_EXAMPLE_FRAGMENTS_PATH}")
    target_link_libraries(LOWWI_demo PRIVATE Lowwi)
    target_include_directories(LOWWI_demo_mic PRIVATE ${SDL2_INCLUDE_DIRS} ${CMAKE_CURRENT_LIST_DIR}/example/LOWWI_demo_mic/audio_async)
    target_link_libraries(LOWWI_demo_mic PUBLIC Lowwi ${SDL2_LIBRARIES})
endif()

