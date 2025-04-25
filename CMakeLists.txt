cmake_minimum_required(VERSION 3.10)

project(FPGA_Router)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)

# Include directories
include_directories(src)

# Source files for the main FPGA Router
set(ROUTER_SOURCES 
    src/main.cpp 
    src/Reader/Reader.cpp 
    src/Router/Router.cpp 
    src/PathfindingAlgorithms/AStarSearch.cpp 
    src/PathfindingAlgorithms/SteinerArborescence.cpp 
    src/PathfindingAlgorithms/SteinerSALT.cpp
)

# Steiner SALT Test executable
set(SALT_TEST_SOURCES
    src/PathfindingAlgorithms/SteinerSALTTest.cpp
    src/PathfindingAlgorithms/SteinerSALT.cpp
)

# Build the main router executable
add_executable(${PROJECT_NAME} ${ROUTER_SOURCES})

# Build the SteinerSALT test executable
add_executable(SteinerSALTTest ${SALT_TEST_SOURCES})
