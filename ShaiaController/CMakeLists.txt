cmake_minimum_required(VERSION 3.22)
project(ShaiaController)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

add_compile_definitions(SPDLOG_ACTIVE_LEVEL=SPDLOG_LEVEL_DEBUG)

set(ARM_CONTROLLER_LIB ArmController)
set(HEAD_CONTROLLER_LIB HeadController)
set(SHAIA_LIB ShaiaLib)
set(OSC_LIB oscpack)
set(SERIAL_LIB serialib)
set(DXL_LIB dynamixel)
set(YAML_LIB yaml-cpp)

include_directories(3rdparty/spdlog/include)
include_directories(Include)

add_subdirectory(3rdparty)
add_subdirectory(Shaia)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC Shaia/Include)
target_link_libraries(${PROJECT_NAME} PUBLIC ${SHAIA_LIB})
