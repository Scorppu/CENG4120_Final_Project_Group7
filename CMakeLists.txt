cmake_minimum_required(VERSION 2.8)

project(FPGA_Router)
include(CheckCXXCompilerFlag)  # Case-sensitive in CMake 2.8


CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)

# Source files - verify these paths are correct
set(SOURCE_FILES 
    src/main.cpp 
    src/Readers/Reader.cpp 
    src/Routers/Router.cpp 
    src/Writers/Writer.cpp 
    src/PathfindingAlgorithms/AStarSearch.cpp
)

CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)

if(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")  # For GCC 4.x
else()
  message(FATAL_ERROR "Compiler lacks C++11 support. Use GCC =4.7 or upgrade CMake.")
endif()

add_executable(${PROJECT_NAME} ${SOURCE_FILES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")


# # if on CSE linux server
# add_custom_target(design1
#     COMMAND $<TARGET_FILE:FPGA_Router> "../../../../../../../research/d1/gds/whlin23/public/xcvu3p.device" "../data/benchmarks/design1.netlist" "../data/results/design1.result"
#     DEPENDS FPGA_Router
#     COMMENT "Running FPGA_Router with design 1\n"
# )

# add_custom_target(design2
#     COMMAND $<TARGET_FILE:FPGA_Router> "../../../../../../../research/d1/gds/whlin23/public/xcvu3p.device" "../data/benchmarks/design2.netlist" "../data/results/design2.result"
#     DEPENDS FPGA_Router
#     COMMENT "Running FPGA_Router with design 2\n"
# )

# add_custom_target(design3
#     COMMAND $<TARGET_FILE:FPGA_Router> "../../../../../../../research/d1/gds/whlin23/public/xcvu3p.device" "../data/benchmarks/design3.netlist" "../data/results/design3.result"
#     DEPENDS FPGA_Router
#     COMMENT "Running FPGA_Router with design 3\n"
# )

# add_custom_target(design4
#     COMMAND $<TARGET_FILE:FPGA_Router> "../../../../../../../research/d1/gds/whlin23/public/xcvu3p.device" "../data/benchmarks/design4.netlist" "../data/results/design4.result"
#     DEPENDS FPGA_Router
#     COMMENT "Running FPGA_Router with design 4\n"
# )

# add_custom_target(design5
#     COMMAND $<TARGET_FILE:FPGA_Router> "../../../../../../../research/d1/gds/whlin23/public/xcvu3p.device" "../data/benchmarks/design5.netlist" "../data/results/design5.result"
#     DEPENDS FPGA_Router
#     COMMENT "Running FPGA_Router with design 5\n"
# )


# if on local machine
if(WIN32 OR APPLE)
  add_custom_target(design1
      COMMAND $<TARGET_FILE:FPGA_Router> "../data/xcvu3p/xcvu3p.device" "../data/benchmarks/design1.netlist" "../data/results/design1.result"
      DEPENDS FPGA_Router
      USES_TERMINAL
      COMMENT "Running FPGA_Router with design 1\n"
  )

  add_custom_target(design2
      COMMAND $<TARGET_FILE:FPGA_Router> "../data/xcvu3p/xcvu3p.device" "../data/benchmarks/design2.netlist" "../data/results/design2.result"
      DEPENDS FPGA_Router
      USES_TERMINAL
      COMMENT "Running FPGA_Router with design 2\n"
  )

  add_custom_target(design3
      COMMAND $<TARGET_FILE:FPGA_Router> "../data/xcvu3p/xcvu3p.device" "../data/benchmarks/design3.netlist" "../data/results/design3.result"
      DEPENDS FPGA_Router
      USES_TERMINAL
      COMMENT "Running FPGA_Router with design 3\n"
  )

  add_custom_target(design4
      COMMAND $<TARGET_FILE:FPGA_Router> "../data/xcvu3p/xcvu3p.device" "../data/benchmarks/design4.netlist" "../data/results/design4.result"
      DEPENDS FPGA_Router
      USES_TERMINAL
      COMMENT "Running FPGA_Router with design 4\n"
  )

  add_custom_target(design5
      COMMAND $<TARGET_FILE:FPGA_Router> "../data/xcvu3p/xcvu3p.device" "../data/benchmarks/design5.netlist" "../data/results/design5.result"
      DEPENDS FPGA_Router
      USES_TERMINAL
      COMMENT "Running FPGA_Router with design 5\n"
  )
elseif(UNIX)
  add_custom_target(design1
      COMMAND $<TARGET_FILE:FPGA_Router> "../../../../../../../research/d1/gds/whlin23/public/xcvu3p.device" "../data/benchmarks/design1.netlist" "../data/results/design1.result"
      DEPENDS FPGA_Router
      COMMENT "Running FPGA_Router with design 1\n"
  )

  add_custom_target(design2
      COMMAND $<TARGET_FILE:FPGA_Router> "../../../../../../../research/d1/gds/whlin23/public/xcvu3p.device" "../data/benchmarks/design2.netlist" "../data/results/design2.result"
      DEPENDS FPGA_Router
      COMMENT "Running FPGA_Router with design 2\n"
  )

  add_custom_target(design3
      COMMAND $<TARGET_FILE:FPGA_Router> "../../../../../../../research/d1/gds/whlin23/public/xcvu3p.device" "../data/benchmarks/design3.netlist" "../data/results/design3.result"
      DEPENDS FPGA_Router
      COMMENT "Running FPGA_Router with design 3\n"
  )

  add_custom_target(design4
      COMMAND $<TARGET_FILE:FPGA_Router> "../../../../../../../research/d1/gds/whlin23/public/xcvu3p.device" "../data/benchmarks/design4.netlist" "../data/results/design4.result"
      DEPENDS FPGA_Router
      COMMENT "Running FPGA_Router with design 4\n"
  )

  add_custom_target(design5
      COMMAND $<TARGET_FILE:FPGA_Router> "../../../../../../../research/d1/gds/whlin23/public/xcvu3p.device" "../data/benchmarks/design5.netlist" "../data/results/design5.result"
      DEPENDS FPGA_Router
      COMMENT "Running FPGA_Router with design 5\n"
  )
endif()