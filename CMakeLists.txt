cmake_minimum_required(VERSION 3.29.2)

project(FPGA_Router)

# Source files
set(SOURCE_FILES src/main.cpp src/Reader/Reader.cpp src/Router/Router.cpp src/Writer/Writer.cpp src/PathfindingAlgorithms/AStarSearch.cpp)
set(CMAKE_CXX_STANDARD 11)

# Executable name
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Add the -O3 optimization flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
