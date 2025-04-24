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
      timeoutMs(5000),
      congestionPenaltyFactor(1.0)
{
    // Set default functions
    heuristicFunc = [this](int currentId, int targetId) {
        return this->defaultHeuristic(currentId, targetId);
    };
  
    // Initialize neighbor cache
    buildNeighborCache();
    
    // Initialize node ID to index map
    nodeIdToIndex.reserve(nodes.size()); // Pre-allocate for efficiency
    for (size_t i = 0; i < nodes.size(); ++i) {
        nodeIdToIndex[nodes[i].id] = i;
    }
    
    // Initialize congestion map
    resetCongestion();
}

// Build a cache of neighbors for quick lookup
void AStarSearch::buildNeighborCache() {
    neighborCache.clear();
    
    // Estimate neighbor cache size based on edge count
    neighborCache.reserve(nodes.size());
    
    // Build the neighbor map for quick lookup in a single pass
    for (const auto& edge : edges) {
        if (edge.size() < 2) continue; // Skip empty or invalid edges
        
        int parent = edge[0];
        
        // Pre-allocate space for this parent's neighbors if not already done
        auto it = neighborCache.find(parent);
        if (it == neighborCache.end()) {
            // Reserve typical number of neighbors to avoid frequent reallocations
            neighborCache[parent].reserve(8);
        }
        
        // Add all children as neighbors (parent->child only)
        for (size_t i = 1; i < edge.size(); ++i) {
            neighborCache[parent].push_back(edge[i]);
            
            // Ensure the child node has an entry in the cache (even if empty)
            // This avoids having to check for key existence in getNeighbors
            if (neighborCache.find(edge[i]) == neighborCache.end()) {
                neighborCache[edge[i]]; // Insert with empty vector
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

// Congestion aware cost function
double AStarSearch::congestionAwareCost(int fromId, int toId) {
    // Base cost (uniform cost of 1)
    double baseCost = defaultCost(fromId, toId);
    
    // Get congestion at the destination node
    double congestion = getCongestion(toId);
    
    // Calculate congestion penalty
    double congestionPenalty = congestion * congestionPenaltyFactor;
    
    // Return combined cost: base cost + congestion penalty
    return baseCost + congestionPenalty;
}

// Get neighbors of a node - OPTIMIZED to return const reference
const std::vector<int>& AStarSearch::getNeighbors(int nodeId) {
    // Use the neighbor cache for O(1) lookup
    static const std::vector<int> emptyVec; // For missing entries
    
    // Use find instead of operator[] to avoid insertion of new elements
    auto it = neighborCache.find(nodeId);
    
    // Since we ensure all nodes have entries in buildNeighborCache,
    // this should rarely happen, but we keep it for safety
    return (it != neighborCache.end()) ? it->second : emptyVec;
}

// Set timeout in milliseconds
void AStarSearch::setTimeout(int milliseconds) {
    timeoutMs = milliseconds;
}

// Set congestion penalty factor
void AStarSearch::setCongestionPenaltyFactor(double factor) {
    congestionPenaltyFactor = factor;
}

// Update congestion map after routing a path
void AStarSearch::updateCongestion(const std::vector<int>& path, double congestionIncrement) {
    for (int nodeId : path) {
        congestionMap[nodeId] += congestionIncrement;
    }
}

// Reset congestion map
void AStarSearch::resetCongestion() {
    congestionMap.clear();
}

// Get current congestion at a node
double AStarSearch::getCongestion(int nodeId) const {
    auto it = congestionMap.find(nodeId);
    return (it != congestionMap.end()) ? it->second : 0.0;
}

// Reconstruct path from the came_from map - OPTIMIZED
void AStarSearch::reconstructPath(const std::unordered_map<int, int>& cameFrom, int current, std::vector<int>& path) {
    while (cameFrom.find(current) != cameFrom.end()) {
        path.push_back(current);
        current = cameFrom.at(current);
    }
    path.push_back(current); // add the start node
    std::reverse(path.begin(), path.end());
}


// Main pathfinding method - OPTIMIZED
void AStarSearch::findPath(int sourceNodeId, int targetNodeId, std::vector<int>& path) {
    // Special case for when source and target are the same
    if (sourceNodeId == targetNodeId) {
        return;
    }
    
    // Start timing
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Priority queue for open set
    using SearchNode = std::pair<double, int>;  // <f_score, node_id>
    std::priority_queue<SearchNode, std::vector<SearchNode>, std::greater<SearchNode>> openSet;
    
    // Maps to store metadata - consider dense arrays if node IDs are dense and consecutive
    gScore.clear(); // Cost from start to node
    fScore.clear(); // Estimated total cost
    cameFrom.clear();  // Parent pointers
    closedSet.clear();      // Nodes already evaluated
    
    // Initialize with start node
    openSet.push({0, sourceNodeId});  // <f_score, node_id>
    gScore[sourceNodeId] = 0;
    fScore[sourceNodeId] = heuristicFunc(sourceNodeId, targetNodeId);
    
    // Safety counter to prevent infinite loops
    const int MAX_ITERATIONS = 50000;
    int iterations = 0;
    
    // Initialize variables
    int current;
    double hValue, nodeCongestion, totalCost, tentativeGScore;

    std::chrono::high_resolution_clock::time_point currentTime;
    std::chrono::milliseconds elapsedMs;
    
    // Main A* loop
    while (!openSet.empty() && iterations < MAX_ITERATIONS) {
        // Check for timeout - only every 100 iterations to reduce overhead
        if (iterations % 100 == 0) {
            currentTime = std::chrono::high_resolution_clock::now();
            elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime);
            if (elapsedMs.count() > timeoutMs) {
                break;
            }
        }
        
        // Get node with lowest fScore
        current = openSet.top().second;
        openSet.pop();
        iterations++;
        
        // If we've reached the target, reconstruct the path
        if (current == targetNodeId) {
            reconstructPath(cameFrom, current, path);
            return;
        }
        
        // Skip if already evaluated
        if (closedSet.count(current)) {
            continue;
        }
        
        // Mark as evaluated
        closedSet.insert(current);
        
        // Process all neighbors - OPTIMIZED to avoid edge tracking overhead
        for (int neighbor : getNeighbors(current)) {
            // Skip if already evaluated
            if (closedSet.count(neighbor)) {
                continue;
            }
            
            // Calculate tentative gScore using the congestion-aware cost function
            tentativeGScore = gScore[current] + congestionAwareCost(current, neighbor);
            
            // If this path is better than any previous one
            if (gScore.find(neighbor) == gScore.end() || tentativeGScore < gScore[neighbor]) {
                // Update metadata
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeGScore;
                // Use heuristic plus congestion penalty for more informed search
                hValue = heuristicFunc(neighbor, targetNodeId);
                nodeCongestion = getCongestion(neighbor);
                totalCost = gScore[neighbor] + hValue + (nodeCongestion * congestionPenaltyFactor);
                fScore[neighbor] = totalCost;
                
                // Add to open set
                openSet.push({fScore[neighbor], neighbor});
            }
        }
    }
    
    // If we get here, no path was found
    return;
}

// Find paths from a source to multiple targets
// std::vector<std::vector<int>> AStarSearch::findPaths(int sourceNodeId, const std::vector<int>& targetNodeIds) {
//     std::vector<std::vector<int>> paths;
//     paths.reserve(targetNodeIds.size());
    
//     // Create a copy of all target IDs for processing
//     std::vector<int> remainingTargets = targetNodeIds;

//     // Initialize variables
//     int currentTarget;
//     std::vector<int> path;
    
//     // Process targets one by one, updating congestion after each path
//     while (!remainingTargets.empty()) {
//         // Find the nearest target based on heuristic distance
//         auto bestTargetIt = std::min_element(remainingTargets.begin(), remainingTargets.end(),
//             [this, sourceNodeId](int target1, int target2) {
//                 return heuristicFunc(sourceNodeId, target1) < heuristicFunc(sourceNodeId, target2);
//             });
        
//         // Get the selected target
//         currentTarget = *bestTargetIt;
        
//         // Find path to this target
//         findPath(sourceNodeId, currentTarget, path);
        
//         // Add the path to our results
//         paths.push_back(path);
        
//         // Update congestion map with this path
//         if (!path.empty()) {
//             updateCongestion(path, 5.0);
//         }
        
//         // Remove this target from the remaining list
//         remainingTargets.erase(bestTargetIt);
//     }
    
//     return paths;
// }

// Set custom heuristic function
void AStarSearch::setHeuristicFunction(std::function<double(int, int)> func) {
    heuristicFunc = func;
}
