cmake_minimum_required(VERSION 3.29.2)

project(FPGA_Router)

# Source files
set(SOURCE_FILES src/main.cpp src/Reader/STReader.cpp src/Router/router.cpp src/Writer/output.cpp)

# Executable name
add_executable(${PROJECT_NAME} ${SOURCE_FILES})
