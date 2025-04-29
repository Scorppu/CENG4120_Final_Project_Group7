# FPGA Router

A high-performance FPGA router implementing the PathFinder algorithm with parallel processing and A* search optimizations. Developed as the final project for CENG4120 by Group 7.

## Team Members
- Chan Eugene
- Chan Yik Kuen 
- Lei Hei Tung

## Overview

This router takes FPGA device specifications and netlists as input, and produces valid routing paths that connect all nets without congestion. Key performance metrics include:
1. Congestion elimination (no shared nodes)
2. Net routing success rate
3. Total wirelength minimization

## Algorithm

The implementation uses a negotiation-based PathFinder algorithm:

1. **Initial Routing Phase**: Routes all nets while ignoring congestion, creating an initial solution.
2. **Iterative Negotiation Phase**: Repeatedly rips up and re-routes nets with increasing penalties for congestion.
3. **Cost Function**: Combines base cost, historical congestion, and present congestion.
4. **Path Finding**: Uses A* search with spatial distance heuristic for efficient path discovery.

## Key Features

- **Parallel Processing**: Divides nets among multiple threads for faster routing
- **A* Search**: Uses distance-based heuristics to find optimal paths quickly
- **Adaptive Congestion Penalties**: Incrementally increases costs for congested resources
- **Routing Prioritization**: Handles more complex nets (with many sinks) first
- **Timeout Handling**: Ensures results are produced within time constraints

## Project Structure

```
CENG4120_Final_Project_Group7 - FPGA Router
│
├── data/                                 # Test data
│   ├── xcvu3p/                           # Input device (not included in remote repository)
│   │   └── xcvu3p.device                 # get: https://github.com/ippan-kaishain/CENG4120-2025-Final/releases/download/Released/xcvu3p.tar.bz2
│   │                                     
│   ├── benchmarks/                       # Input netlists
│   │   ├── design1.netlist               # Simple test case
│   │   ├── design2.netlist               # Small test case
│   │   ├── design3.netlist               # Medium test case
│   │   ├── design4.netlist               # Medium test case
│   │   └── design5.netlist               # Large real-world case
│   │
│   └── results/                          # Output directory for results (not included in remote repository)
│       ├── design1.result                
│       ├── design2.result
│       ├── design3.result
│       ├── design4.result
│       └── design5.result
│
├── docs/                                 # Documentation
│   └── CENG4120-Final-FPGA Routing.docx  # Assignment specification
│
├── src/                                  # Source code
│   ├── main.cpp                          # Program entry point
│   ├── DataStructure.hpp                 # Shared data structures
│   │
│   ├── PathfindingAlgorithms/            # Pathfinders
│   │   ├── AStarSearch.cpp               # A* Search implementation
│   │   └── AStarSearch.hpp               # A* Search header
│   │
│   ├── Readers/                          # Input parsers
│   │   ├── Reader.cpp                    # Parses device file and netlist
│   │   └── Reader.hpp                    # Reader header file
│   │
│   ├── Routers/                          # Routers
│   │   ├── Router.cpp                    # Main Router
│   │   └── Router.hpp                    # Router header file
│   │
│   └── Writers/                          # Writers
│   │   ├── Writer.cpp                    # Main Writer
│       └── Writer.hpp                    # Writer header file
│
├── .gitignore
├── CMakeLists.txt                        # CMake file
├── README.md                             # This file
└── PROJECT_STRUCTURE.txt                 # Contains Project file structure
```

## Building

To compile the router:

# For CSE slurm
```bash
cd build
cmake ..
cmake .
make
```

# For Windows (CMAKE 3.5+)
```bash
cd build
cmake -S .. -B .
ninja
```

# For MacOS (CMAKE 3.5+)
```bash
cd build
cmake -S .. -B .
make
```

## Usage

# For CSE slurm (actual submission)

```bash
./FPGA_router <device> <netlist> <result>
```

Example:
```bash
./FPGA_router xcvu3p.device design1.netlist design1.result
```

# For testing (Windows)

```bash
# result is automatically written to design<number>.result
ninja design1 # or design2/3/4/5
```

# For testing (MacOS)

```bash
# result is automatically written to design<number>.result
make design1 # or design2/3/4/5
```

## Benchmarks

The router is tested on 5 designs of increasing complexity:
- Design 1-2: Simple test cases
- Design 3-4: Medium difficulty designs
- Design 5: Large real-world FPGA design

Time limits:
- Designs 1-4: 100 seconds
- Design 5: 250 seconds

## Results Analysis

The router outputs the following metrics:
- Number of successfully routed nets
- Presence of congestion
- Total wirelength
- Execution time

Results are ranked based on routing success and wirelength minimization.
