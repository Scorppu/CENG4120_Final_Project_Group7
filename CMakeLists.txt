cmake_minimum_required(VERSION 3.29.2)

project(FPGA_Router)

# Source files
set(SOURCE_FILES src/main.cpp src/Reader/Reader.cpp src/Router/Router.cpp)
set(CMAKE_CXX_STANDARD 17)

# Executable name
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
