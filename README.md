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
├── src/                                  # Source code
│   ├── main.cpp                          # Program entry point
│   │
│   ├── Interface/                        # Header files
│   │   ├── Reader.hpp                    # Input handling interface
│   │   ├── Router.hpp                    # PathFinder algorithm interface
│   │   └── Writer.hpp                    # Output generation interface
│   │
│   ├── Reader/                           # Input implementation
│   │   └── STReader.cpp                  # Parses device files and netlists
│   │
│   ├── Router/                           # Routing implementation
│   │   └── router.cpp                    # PathFinder algorithm with A* search
│   │
│   └── Writer/                           # Output implementation
│       └── output.cpp                    # Generates routing results
│
├── data/                                 # Test data
│   ├── benchmarks/                       # Input netlists
│   │   ├── design1.netlist               # Simple test case
│   │   ├── design2.netlist               # Small test case
│   │   ├── design3.netlist               # Medium test case
│   │   ├── design4.netlist               # Medium test case
│   │   └── design5.netlist               # Large real-world case
│   │
│   └── results/                          # Output directory for results
│
├── docs/                                 # Documentation
│   └── CENG4120-Final-FPGA Routing.docx  # Assignment specification
│
├── README.md                             # This file
└── PROJECT_STRUCTURE.txt                 # Contains Project file structure
```

## Building

To compile the router:

```bash
g++ src/AStarSearch.cpp src/main.cpp src/Reader.cpp src/router.cpp src/Writer.cpp -o router
cd build
make
```

## Usage

```bash
./FPGA_router <device> <netlist> <result>
```

Example:
```bash
./FPGA_router xcvu3p.device design1.netlist design1.result
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
