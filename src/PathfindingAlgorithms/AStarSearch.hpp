#ifndef ASTAR_SEARCH_HPP
#define ASTAR_SEARCH_HPP

#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <functional>
#include <memory>
#include "../DataStructure.hpp"

class AStarSearch {
public:
    // Constructor
    AStarSearch(const std::vector<std::vector<int>>& edges, 
                const std::vector<Node>& nodes);
    
    // Main pathfinding method
    // Returns a vector of node IDs representing the path from source to target
    void findPath(int sourceNodeId, int targetNodeId, std::vector<int>& path);
    
    // Modified pathfinding method that also tracks congested nodes
    void findPath(int sourceNodeId, int targetNodeId, std::vector<int>& path, 
                 std::unordered_map<int, std::unordered_set<int>>& congestedNodes, int netId);
    
    // New overload that uses a previous path for backtracking when direct routing fails
    void findPath(int sourceNodeId, int targetNodeId, std::vector<int>& path,
                  std::unordered_map<int, std::unordered_set<int>>& congestedNodes, int netId,
                  const std::vector<int>& prevPath);
    
    // Find paths from a source to multiple targets
    // Returns a vector of paths, one for each target
    std::vector<std::vector<int>> findPaths(int sourceNodeId, const std::vector<int>& targetNodeIds);
    
    // Set custom heuristic function
    void setHeuristicFunction(std::function<double(int, int)> heuristicFunc);
    
    // Set custom cost function for edges
    void setCostFunction(std::function<double(int, int)> costFunc);
    
    // Set timeout for pathfinding (in milliseconds)
    void setTimeout(int milliseconds);

    // Set congestion penalty factor
    void setCongestionPenaltyFactor(double factor);
    
    // Update congestion map after routing a path
    void updateCongestion(const std::vector<int>& path, double congestionIncrement = 1.0);
    
    // Reset congestion map
    void resetCongestion();
    
    // Get current congestion at a node
    double getCongestion(int nodeId) const;
    
    // Get the edges vector
    const std::vector<std::vector<int>>& getEdges() const { return edges; }
    
    // Get the nodes vector
    const std::vector<Node>& getNodes() const { return nodes; }

private:
    // Reference to the graph structure
    const std::vector<std::vector<int>>& edges;
    const std::vector<Node>& nodes;
    
    // Cache of node neighbors for efficient lookup
    std::unordered_map<int, std::vector<int>> neighborCache;
    
    // Map from node ID to index in nodes vector
    std::unordered_map<int, size_t> nodeIdToIndex;
    
    // Congestion map: node ID -> congestion value
    std::unordered_map<int, double> congestionMap;
    
    // Congestion penalty factor
    double congestionPenaltyFactor;
    
    // Timeout in milliseconds
    int timeoutMs;
    
    // Heuristic function (estimates cost from current to goal)
    std::function<double(int, int)> heuristicFunc;
    
    // Cost function (actual cost between adjacent nodes)
    std::function<double(int, int)> costFunc;
    
    // Build the neighbor cache for efficient lookups
    void buildNeighborCache();
    
    // Default heuristic: Manhattan distance
    double defaultHeuristic(int currentId, int targetId);
    
    // Default cost: uniform cost of 1
    double defaultCost(int fromId, int toId);
    
    // Congestion aware cost function
    double congestionAwareCost(int fromId, int toId);
    
    // Reconstruct path from came_from map
    void reconstructPath(
        const std::unordered_map<int, int>& cameFrom,
        int current,
        std::vector<int>& path
    );
    
    // Get neighbors of a node - optimized to return const reference
    const std::vector<int>& getNeighbors(int nodeId);

    std::unordered_map<int, double> gScore;
    std::unordered_map<int, double> fScore;
    std::unordered_map<int, int> cameFrom;  
    std::unordered_set<int> closedSet; 
};

#endif // ASTAR_SEARCH_HPP 