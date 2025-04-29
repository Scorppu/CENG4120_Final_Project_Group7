cmake_minimum_required(VERSION 3.10.2)

project(FPGA_Router LANGUAGES CXX)

# Specify include directories if needed
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Source files - verify these paths are correct
set(SOURCE_FILES 
    src/main.cpp 
    src/Readers/Reader.cpp 
    src/Routers/Router.cpp 
    src/Writers/Writer.cpp 
    src/PathfindingAlgorithms/AStarSearch.cpp
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Executable name
add_executable(${PROJECT_NAME} ${SOURCE_FILES})

# Explicitly set linker language
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# Add the -O3 optimization flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

add_custom_target(design1
    COMMAND $<TARGET_FILE:FPGA_Router> "../data/xcvu3p/xcvu3p.device" "../data/benchmarks/design1.netlist" "../data/results/design1.result"
    DEPENDS FPGA_Router
    USES_TERMINAL
    COMMENT "Running FPGA_Router with design 1"
)

add_custom_target(design2
    COMMAND $<TARGET_FILE:FPGA_Router> "../data/xcvu3p/xcvu3p.device" "../data/benchmarks/design2.netlist" "../data/results/design2.result"
    DEPENDS FPGA_Router
    USES_TERMINAL
    COMMENT "Running FPGA_Router with design 2"
)

add_custom_target(design3
    COMMAND $<TARGET_FILE:FPGA_Router> "../data/xcvu3p/xcvu3p.device" "../data/benchmarks/design3.netlist" "../data/results/design3.result"
    DEPENDS FPGA_Router
    USES_TERMINAL
    COMMENT "Running FPGA_Router with design 3"
)

add_custom_target(design4
    COMMAND $<TARGET_FILE:FPGA_Router> "../data/xcvu3p/xcvu3p.device" "../data/benchmarks/design4.netlist" "../data/results/design4.result"
    DEPENDS FPGA_Router
    USES_TERMINAL
    COMMENT "Running FPGA_Router with design 4"
)

add_custom_target(design5
    COMMAND $<TARGET_FILE:FPGA_Router> "../data/xcvu3p/xcvu3p.device" "../data/benchmarks/design5.netlist" "../data/results/design5.result"
    DEPENDS FPGA_Router
    USES_TERMINAL
    COMMENT "Running FPGA_Router with design 5"
)

# enable_testing()

# # Set path to the eval file and data directories
# set(EVAL_FILE ${CMAKE_SOURCE_DIR}/data/eval)
# set(DEVICE_DIR ${CMAKE_SOURCE_DIR}/data/xcvu3p)
# set(BENCHMARK_DIR ${CMAKE_SOURCE_DIR}/data/benchmarks)
# set(RESULT_DIR ${CMAKE_SOURCE_DIR}/data/results)

# # Make sure the result directory exists
# file(MAKE_DIRECTORY ${RESULT_DIR})

# # Make the eval file executable (important!)
# file(CHMOD ${EVAL_FILE} 
#      PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)


# if (WIN32)
#     add_test(
#         NAME eval_test1
#         COMMAND wsl ${EVAL_FILE} "${DEVICE_DIR}/xcvu3p.device" "${BENCHMARK_DIR}/design1.netlist" "${RESULT_DIR}/design1.result"
#         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#     )

#     add_test(
#         NAME eval_test2
#         COMMAND wsl ${EVAL_FILE} "${DEVICE_DIR}/xcvu3p.device" "${BENCHMARK_DIR}/design2.netlist" "${RESULT_DIR}/design2.result"
#         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#     )

#     add_test(
#         NAME eval_test3
#         COMMAND wsl ${EVAL_FILE} "${DEVICE_DIR}/xcvu3p.device" "${BENCHMARK_DIR}/design3.netlist" "${RESULT_DIR}/design3.result"
#         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#     )

#     add_test(
#         NAME eval_test4
#         COMMAND wsl ${EVAL_FILE} "${DEVICE_DIR}/xcvu3p.device" "${BENCHMARK_DIR}/design4.netlist" "${RESULT_DIR}/design4.result"
#         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#     )

#     add_test(
#         NAME eval_test5
#         COMMAND wsl ${EVAL_FILE} "${DEVICE_DIR}/xcvu3p.device" "${BENCHMARK_DIR}/design5.netlist" "${RESULT_DIR}/design5.result"
#         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#     )

# else ()
#     add_test(
#         NAME eval_test1
#         COMMAND ${EVAL_FILE} "${DEVICE_DIR}/xcvu3p.device" "${BENCHMARK_DIR}/design1.netlist" "${RESULT_DIR}/design1.result"
#         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#     )

#     add_test(
#         NAME eval_test2
#         COMMAND ${EVAL_FILE} "${DEVICE_DIR}/xcvu3p.device" "${BENCHMARK_DIR}/design2.netlist" "${RESULT_DIR}/design2.result"
#         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#     )

#     add_test(
#         NAME eval_test3
#         COMMAND ${EVAL_FILE} "${DEVICE_DIR}/xcvu3p.device" "${BENCHMARK_DIR}/design3.netlist" "${RESULT_DIR}/design3.result"
#         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#     )

#     add_test(
#         NAME eval_test4
#         COMMAND ${EVAL_FILE} "${DEVICE_DIR}/xcvu3p.device" "${BENCHMARK_DIR}/design4.netlist" "${RESULT_DIR}/design4.result"
#         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#     )

#     add_test(
#         NAME eval_test5
#         COMMAND ${EVAL_FILE} "${DEVICE_DIR}/xcvu3p.device" "${BENCHMARK_DIR}/design5.netlist" "${RESULT_DIR}/design5.result"
#         WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
#     )
# endif()





