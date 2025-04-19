#include "AStarSearch.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>

// Structure for A* nodes in the priority queue
struct AStarNode {
    int id;
    double fScore;  // Total estimated cost (g + h)
    
    // Constructor
    AStarNode(int id, double fScore) : id(id), fScore(fScore) {}
    
    // Comparison operator for priority queue (min heap)
    bool operator>(const AStarNode& other) const {
        return fScore > other.fScore;
    }
};

// Constructor
AStarSearch::AStarSearch(const std::vector<std::vector<int>>& edges, 
                        const std::vector<Node>& nodes)
    : edges(edges), nodes(nodes) {
    // Set default functions
    heuristicFunc = [this](int currentId, int targetId) {
        return this->defaultHeuristic(currentId, targetId);
    };
    
    costFunc = [this](int fromId, int toId) {
        return this->defaultCost(fromId, toId);
    };
    
    // Initialize neighbor cache
    buildNeighborCache();
    
    // Initialize node ID to index map
    for (size_t i = 0; i < nodes.size(); ++i) {
        nodeIdToIndex[nodes[i].id] = i;
    }
    
    // Set default timeout to 5 seconds
    timeoutMs = 5000;
}

// Build a cache of neighbors for quick lookup
void AStarSearch::buildNeighborCache() {
    neighborCache.clear();
    
    // Pre-build the neighbor map for quick lookup
    for (const auto& edge : edges) {
        if (!edge.empty()) {
            int parent = edge[0];
            std::vector<int>& neighbors = neighborCache[parent];
            
            // Add all children as neighbors
            for (size_t i = 1; i < edge.size(); ++i) {
                neighbors.push_back(edge[i]);
            }
        }
    }
}

// Default heuristic function - Manhattan distance
double AStarSearch::defaultHeuristic(int currentId, int targetId) {
    // Find the nodes by ID using the index map
    auto currentIt = nodeIdToIndex.find(currentId);
    auto targetIt = nodeIdToIndex.find(targetId);
    
    if (currentIt == nodeIdToIndex.end() || targetIt == nodeIdToIndex.end()) {
        return 0;
    }
    
    const Node& currentNode = nodes[currentIt->second];
    const Node& targetNode = nodes[targetIt->second];
    
    // Calculate Manhattan distance
    return std::abs(currentNode.beginX - targetNode.beginX) +
           std::abs(currentNode.beginY - targetNode.beginY);
}

// Default cost function - uniform cost
double AStarSearch::defaultCost(int fromId, int toId) {
    // Uniform cost of 1 between adjacent nodes
    return 1.0;
}

// Get neighbors of a node
std::vector<int> AStarSearch::getNeighbors(int nodeId) {
    // Use the neighbor cache for O(1) lookup
    auto it = neighborCache.find(nodeId);
    if (it != neighborCache.end()) {
        return it->second;
    }
    
    // Return empty vector if node has no neighbors
    return std::vector<int>();
}

// Set timeout in milliseconds
void AStarSearch::setTimeout(int milliseconds) {
    timeoutMs = milliseconds;
}

// Reconstruct path from the came_from map
std::vector<int> AStarSearch::reconstructPath(
    const std::unordered_map<int, int>& cameFrom,
    int current
) {
    std::vector<int> path;
    path.push_back(current);
    
    // Traverse the came_from map backwards to build the path
    while (cameFrom.find(current) != cameFrom.end()) {
        current = cameFrom.at(current);
        path.push_back(current);
    }
    
    // Reverse to get path from source to target
    std::reverse(path.begin(), path.end());
    return path;
}

// A* Search algorithm
std::vector<int> AStarSearch::findPath(int sourceNodeId, int targetNodeId) {
    // Start timing
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Priority queue for open set
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openSet;
    
    // Maps to store metadata
    std::unordered_map<int, double> gScore; // Cost from start to node
    std::unordered_map<int, double> fScore; // Estimated total cost
    std::unordered_map<int, int> cameFrom;  // Parent pointers
    std::unordered_set<int> closedSet;      // Nodes already evaluated
    
    // Track visited edges to avoid revisiting paths
    // Using pair of integers for better performance
    std::unordered_set<uint64_t> visitedEdges;
    
    // Helper function to create a unique edge identifier using bit shifting
    auto createEdgeId = [](int fromId, int toId) -> uint64_t {
        return (static_cast<uint64_t>(fromId) << 32) | static_cast<uint64_t>(toId);
    };
    
    // Initialize with start node
    openSet.push(AStarNode(sourceNodeId, 0));
    gScore[sourceNodeId] = 0;
    fScore[sourceNodeId] = heuristicFunc(sourceNodeId, targetNodeId);
    
    // Safety counter to prevent infinite loops
    const int MAX_ITERATIONS = 10000; // Reduced from 100000
    int iterations = 0;
    
    // Main A* loop
    while (!openSet.empty() && iterations < MAX_ITERATIONS) {
        // Check for timeout
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - startTime).count();
        
        if (elapsedMs > timeoutMs) {
            std::cerr << "Warning: A* search timeout after " << elapsedMs 
                      << "ms (" << iterations << " iterations)" << std::endl;
            break;
        }
        
        // Get node with lowest fScore
        int current = openSet.top().id;
        openSet.pop();
        iterations++;
        
        // If we've reached the target, reconstruct the path
        if (current == targetNodeId) {
            std::cout << "Path found after " << iterations << " iterations ("
                      << elapsedMs << "ms)" << std::endl;
            return reconstructPath(cameFrom, current);
        }
        
        // Skip if already evaluated
        if (closedSet.find(current) != closedSet.end()) {
            continue;
        }
        
        // Mark as evaluated
        closedSet.insert(current);
        
        // Process all neighbors (now using the cache)
        for (int neighbor : getNeighbors(current)) {
            // Skip if already evaluated
            if (closedSet.find(neighbor) != closedSet.end()) {
                continue;
            }
            
            // Check if this edge has been visited before using the more efficient edge ID
            uint64_t edgeId = createEdgeId(current, neighbor);
            if (visitedEdges.find(edgeId) != visitedEdges.end()) {
                continue;  // Skip previously visited edges
            }
            
            // Mark this edge as visited
            visitedEdges.insert(edgeId);
            
            // Calculate tentative gScore
            double tentativeGScore = gScore[current] + costFunc(current, neighbor);
            
            // If this path is better than any previous one
            if (gScore.find(neighbor) == gScore.end() || tentativeGScore < gScore[neighbor]) {
                // Update metadata
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeGScore;
                fScore[neighbor] = gScore[neighbor] + heuristicFunc(neighbor, targetNodeId);
                
                // Add to open set
                openSet.push(AStarNode(neighbor, fScore[neighbor]));
            }
        }
    }
    
    // Performance statistics
    auto endTime = std::chrono::high_resolution_clock::now();
    auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(
        endTime - startTime).count();
    
    // Check if we hit the iteration limit
    if (iterations >= MAX_ITERATIONS) {
        std::cerr << "Warning: A* search terminated after reaching maximum iterations (" 
                  << MAX_ITERATIONS << ") without finding a path (" << totalTime << "ms)" << std::endl;
    }
    
    // If we get here, no path was found
    return std::vector<int>();
}

// Find paths from a source to multiple targets
std::vector<std::vector<int>> AStarSearch::findPaths(int sourceNodeId, const std::vector<int>& targetNodeIds) {
    std::vector<std::vector<int>> paths;
    paths.reserve(targetNodeIds.size());
    
    for (int targetId : targetNodeIds) {
        paths.push_back(findPath(sourceNodeId, targetId));
    }
    
    return paths;
}

// Set custom heuristic function
void AStarSearch::setHeuristicFunction(std::function<double(int, int)> func) {
    heuristicFunc = func;
}

// Set custom cost function
void AStarSearch::setCostFunction(std::function<double(int, int)> func) {
    costFunc = func;
} 