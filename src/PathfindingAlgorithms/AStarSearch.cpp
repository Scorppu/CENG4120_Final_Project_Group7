#include "AStarSearch.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <limits>

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
    nodeIdToIndex.reserve(nodes.size()); // Pre-allocate for efficiency
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

// Get neighbors of a node - OPTIMIZED to return const reference
const std::vector<int>& AStarSearch::getNeighbors(int nodeId) {
    // Use the neighbor cache for O(1) lookup
    static const std::vector<int> emptyVec; // For missing entries
    auto it = neighborCache.find(nodeId);
    return (it != neighborCache.end()) ? it->second : emptyVec;
}

// Set timeout in milliseconds
void AStarSearch::setTimeout(int milliseconds) {
    timeoutMs = milliseconds;
}

// Reconstruct path from the came_from map - OPTIMIZED
std::vector<int> AStarSearch::reconstructPath(
    const std::unordered_map<int, int>& cameFrom,
    int current
) {
    // Estimate path size to reduce reallocations
    std::vector<int> path;
    path.reserve(cameFrom.size() + 1);  // Reserve space for worst case
    
    // Count path length first to avoid reversal
    int pathLength = 1;  // Start with 1 for the target node
    int temp = current;
    auto it = cameFrom.find(temp);
    while (it != cameFrom.end()) {
        pathLength++;
        temp = it->second;
        it = cameFrom.find(temp);
    }
    
    // Pre-allocate the correctly sized vector
    path.resize(pathLength);
    
    // Fill the path in the correct order (from source to target)
    int index = pathLength - 1;
    path[index--] = current;
    
    // Use operator[] instead of at() for better performance
    auto findIt = cameFrom.find(current);
    while (findIt != cameFrom.end()) {
        current = findIt->second;
        path[index--] = current;
        findIt = cameFrom.find(current);
    }
    
    return path;
}

// Main pathfinding method - OPTIMIZED
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
    
    // Maps to store metadata - consider dense arrays if node IDs are dense and consecutive
    std::unordered_map<int, double> gScore; // Cost from start to node
    std::unordered_map<int, double> fScore; // Estimated total cost
    std::unordered_map<int, int> cameFrom;  // Parent pointers
    std::unordered_set<int> closedSet;      // Nodes already evaluated
    
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
        
        // Process all neighbors - OPTIMIZED to avoid edge tracking overhead
        for (int neighbor : getNeighbors(current)) {
            // Skip if already evaluated
            if (closedSet.find(neighbor) != closedSet.end()) {
                continue;
            }
            
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