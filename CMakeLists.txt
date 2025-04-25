cmake_minimum_required(VERSION 3.10.2)

project(FPGA_Router LANGUAGES CXX)

# Specify include directories if needed
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Source files - verify these paths are correct
set(SOURCE_FILES 
    src/main.cpp 
    src/Reader.cpp 
    src/router.cpp 
    src/Writer.cpp 
    src/AStarSearch.cpp
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Executable name
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Explicitly set linker language
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Add the -O3 optimization flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
