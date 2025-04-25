# Obstacle-Aware SteinerSALT Algorithm

This project implements an obstacle-aware Steiner Shallow Light Tree (SALT) algorithm for rectilinear routing in VLSI design. The algorithm efficiently finds paths connecting multiple sinks to a source while avoiding obstacles.

## Features

- **Obstacle Avoidance**: Handles rectangular obstacles in the routing area.
- **Rectilinear Steiner Tree**: Creates Steiner trees with rectilinear (Manhattan) connections.
- **Path Optimization**: Includes refinement techniques like:
  - Intersection cancellation
  - L-shape flipping
  - U-shape shifting
- **Congestion Management**: Supports congestion-aware routing.
- **Visibility Graph**: Uses obstacle corners as potential Steiner points.

## Implementation Details

### Obstacle Representation

Obstacles are represented as rectangles with coordinates for the top-left and bottom-right corners. The implementation includes methods to check if a point is inside an obstacle and if a line segment intersects an obstacle.

### Obstacle-Aware Path Finding

When a direct path between two nodes would intersect an obstacle, the algorithm uses A* search with a Manhattan distance heuristic to find the shortest path around the obstacles.

### Refinement Techniques

The implementation includes refinement techniques that are obstacle-aware:

1. **Intersection Cancellation**: When two paths intersect, create a Steiner point at the intersection if it's not inside an obstacle.
2. **L-Shape Flipping**: Replace L-shaped paths with direct connections if they don't intersect obstacles and reduce the total wirelength.
3. **U-Shape Shifting**: Optimize U-shaped structures by repositioning Steiner points while avoiding obstacles.

### Building and Running

#### Windows

```batch
build_test.bat
```

This script will:
1. Create a build directory
2. Generate CMake files
3. Build the SteinerSALTTest executable
4. Run the test if the build was successful

#### Manual Build (CMake)

```bash
mkdir -p build
cd build
cmake ..
cmake --build . --target SteinerSALTTest
./SteinerSALTTest
```

## Algorithm Structure

1. **Initialization**: Set up the graph and obstacles.
2. **Visibility Graph**: Build a visibility graph using obstacle corners as potential Steiner points.
3. **MST Construction**: Build an obstacle-aware minimum spanning tree.
4. **DFS Traversal**: Find breakpoints in the MST.
5. **Steiner SPT**: Construct a light Steiner shortest path tree for the breakpoints.
6. **Refinement**: Apply obstacle-aware refinement techniques.
7. **Path Extraction**: Convert the tree to a path in the original graph.

## Future Improvements

- Support for more complex obstacle shapes
- Multi-threaded implementation for larger problem instances
- Integration with congestion-driven global routing
- Improved heuristics for Steiner point placement

## Requirements

- C++14 or higher
- CMake 3.10 or higher (for building with CMake)
