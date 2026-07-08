cmake_minimum_required(VERSION 3.10)
project(tactile_sensor_sdk CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 静态库
add_library(tactile_sensor STATIC
    src/serial_port.cpp
    src/tactile_sensor.cpp)
target_link_libraries(tactile_sensor PUBLIC Threads::Threads)

# 示例
add_executable(tactile_example examples/example_main.cpp)
target_link_libraries(tactile_example PRIVATE tactile_sensor)

# 测试
enable_testing()
add_executable(test_protocol test/test_protocol.cpp)
add_test(NAME protocol_test COMMAND test_protocol)