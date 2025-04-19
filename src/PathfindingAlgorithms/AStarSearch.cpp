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
AStarSearch::AStarSearch(const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) 
    : edges(edges), 
      nodes(nodes), 
      timeoutMs(5000) 
{
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
}

// Build a cache of neighbors for quick lookup
void AStarSearch::buildNeighborCache() {
    neighborCache.clear();
    
    // First pass: collect node IDs and pre-allocate vectors
    std::unordered_set<int> allNodeIds;
    for (const auto& edge : edges) {
        if (!edge.empty()) {
            allNodeIds.insert(edge[0]);
            for (size_t i = 1; i < edge.size(); ++i) {
                allNodeIds.insert(edge[i]);
            }
        }
    }
    
    // Pre-allocate space in neighborCache for all nodes
    for (int nodeId : allNodeIds) {
        neighborCache[nodeId].reserve(8);  // Reserve space for typical number of neighbors
    }
    
    // Build the neighbor map for quick lookup (unidirectional)
    for (const auto& edge : edges) {
        if (!edge.empty()) {
            int parent = edge[0];
            
            // Add all children as neighbors (parent->child only)
            for (size_t i = 1; i < edge.size(); ++i) {
                neighborCache[parent].push_back(edge[i]);
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

// Main pathfinding method
std::vector<int> AStarSearch::findPath(int sourceNodeId, int targetNodeId) {
    // Special case for when source and target are the same
    if (sourceNodeId == targetNodeId) {
        return {sourceNodeId};
    }
    
    // Start timing
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Priority queue for open set
    using SearchNode = std::pair<double, int>;  // <f_score, node_id>
    std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> openSet;
    
    // Maps to store metadata
    std::unordered_map<int, double> gScore; // Cost from start to node
    std::unordered_map<int, double> fScore; // Estimated total cost
    std::unordered_map<int, int> cameFrom;  // Parent pointers
    std::unordered_set<int> closedSet;      // Nodes already evaluated
    
    // Track visited edges to avoid revisiting paths (using bit-shifted integer for efficiency)
    std::unordered_set<uint64_t> visitedEdges;
    
    // Helper function to create a unique edge identifier
    auto createEdgeId = [](int fromId, int toId) -> uint64_t {
        return (static_cast<uint64_t>(fromId) << 32) | static_cast<uint64_t>(toId);
    };
    
    // Initialize with start node
    openSet.push({0, sourceNodeId});  // <f_score, node_id>
    gScore[sourceNodeId] = 0;
    fScore[sourceNodeId] = heuristicFunc(sourceNodeId, targetNodeId);
    
    // Safety counter to prevent infinite loops
    const int MAX_ITERATIONS = 50000;
    int iterations = 0;
    
    // Main A* loop
    while (!openSet.empty() && iterations < MAX_ITERATIONS) {
        // Check for timeout
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - startTime).count();
        
        if (elapsedMs > timeoutMs) {
            // Only log timeouts in verbose mode
            break;
        }
        
        // Get node with lowest fScore
        int current = openSet.top().second;
        openSet.pop();
        iterations++;
        
        // If we've reached the target, reconstruct the path
        if (current == targetNodeId) {
            return reconstructPath(cameFrom, current);
        }
        
        // Skip if already evaluated
        if (closedSet.find(current) != closedSet.end()) {
            continue;
        }
        
        // Mark as evaluated
        closedSet.insert(current);
        
        // Process all neighbors 
        for (int neighbor : getNeighbors(current)) {
            // Skip if already evaluated
            if (closedSet.find(neighbor) != closedSet.end()) {
                continue;
            }
            
            // Check if this edge has been visited before
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
                openSet.push({fScore[neighbor], neighbor});
            }
        }
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