# CENG4120_Final_Project_Group7
Chan Eugene, Chan Yik Kuen, Lei Hei Tung

## FPGA Router Implementation

This project implements an FPGA router using the PathFinder algorithm with optimizations for performance and solution quality.

### Features

- Parallel processing for faster routing
- A* search with distance-based heuristic
- Adaptive congestion handling
- Prioritized net routing based on complexity
- Automatic timeout handling to meet time limits

### Project Structure

The project is organized in a modular way:

- `input.h/cpp`: Data structures and input parsing functions
- `router.h/cpp`: PathFinder algorithm implementation
- `output.h/cpp`: Output generation and statistics
- `main.cpp`: Program entry point and coordination

### Building the Router

To compile the router, simply run:

```
make
```

This will create the `fpga_router` executable.

### Running the Router

To run the router on a design:

```
./fpga_router <device_file> <netlist_file> <output_file>
```

Example:
```
./fpga_router xcvu3p.device design1.netlist design1.route
```

### Algorithm Details

The implemented router uses the PathFinder algorithm with the following key components:

1. Initial routing of all nets while allowing congestion
2. Iterative rip-up and re-route with increasing congestion penalties
3. History-based cost updates to discourage sharing of resources
4. A* search for efficient path finding
5. Parallel routing of nets for performance

The router prioritizes:
1. Eliminating congestion (shared nodes)
2. Maximizing the number of successfully routed nets
3. Minimizing total wirelength
