cmake_minimum_required(VERSION 3.16)
project(TrafficSim LANGUAGES CXX)

if(NOT MINGW)
    message(FATAL_ERROR "TrafficSim uses the bundled MinGW GLFW library. Configure with -DCMAKE_CXX_COMPILER=g++.")
endif()

set(glfw_root "${CMAKE_CURRENT_SOURCE_DIR}/../../Dependencies")
set(glfw_library "${glfw_root}/lib/libglfw3.a")
if(NOT EXISTS "${glfw_root}/include/GLFW/glfw3.h" OR NOT EXISTS "${glfw_library}")
    set(glfw_root "${CMAKE_CURRENT_SOURCE_DIR}/../glfw-3.5.1.bin.WIN64/glfw-3.5.1.bin.WIN64")
    set(glfw_library "${glfw_root}/lib-mingw-w64/libglfw3.a")
endif()
if(NOT EXISTS "${glfw_root}/include/GLFW/glfw3.h" OR NOT EXISTS "${glfw_library}")
    message(FATAL_ERROR "GLFW headers or MinGW library missing. See README.md for the expected directories.")
endif()

add_executable(TrafficSim
    src/main.cpp
    src/Simulation.cpp
    src/Geometry.cpp
    src/BikeModel.cpp
    src/BikeBody.cpp
    src/BikeRunningGear.cpp
    src/Renderer.cpp
    src/Font8x8.cpp
)
set_target_properties(TrafficSim PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
target_compile_options(TrafficSim PRIVATE -Wall -Wextra)
target_include_directories(TrafficSim PRIVATE "${glfw_root}/include")
target_link_options(TrafficSim PRIVATE -static -static-libgcc -static-libstdc++)
target_link_libraries(TrafficSim PRIVATE "${glfw_library}" opengl32 gdi32)

enable_testing()
add_test(NAME bike_models COMMAND TrafficSim --model-test)
add_test(NAME simulation COMMAND TrafficSim --self-test)
add_test(NAME opengl_smoke COMMAND TrafficSim --smoke)
