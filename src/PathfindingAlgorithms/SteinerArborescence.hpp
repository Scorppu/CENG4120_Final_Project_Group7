#ifndef STEINER_ARBORESCENCE_HPP
#define STEINER_ARBORESCENCE_HPP

#include <vector>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include <limits>
#include <cmath>
#include "../Datastructure.hpp"

// Class for Steiner Tree node (internal representation)
class SteinerNode {
public:
    int x, y;             // Node coordinates
    int id;               // Node ID from input graph (if sink node, -1 for Steiner points)
    bool isSink;          // Whether this is a sink node
    bool isSteinerPoint;  // Whether this is a Steiner point
    std::vector<std::shared_ptr<SteinerNode>> children; // Children in the Steiner tree

    SteinerNode(int x, int y, int id = -1, bool isSink = false)
        : x(x), y(y), id(id), isSink(isSink), isSteinerPoint(!isSink) {}
};

// Class for a Steiner tree
class SteinerTree {
public:
    std::shared_ptr<SteinerNode> root; // Root of this tree
    
    SteinerTree(std::shared_ptr<SteinerNode> root = nullptr) : root(root) {}
};

// Main class implementing Rectilinear Steiner Arborescence (RSA) algorithm
class SteinerArborescence {
public:
    // Constructor
    SteinerArborescence(const std::vector<std::vector<int>>& edges, 
                        const std::vector<Node>& nodes);
    
    // Find a Steiner arborescence connecting source to multiple sinks
    std::vector<int> findSteinerTree(int sourceNodeId, const std::vector<int>& sinkNodeIds);
    
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

private:
    // Reference to the graph structure
    const std::vector<std::vector<int>>& edges;
    const std::vector<Node>& nodes;
    
    // Map from node ID to index in nodes vector
    std::unordered_map<int, size_t> nodeIdToIndex;
    
    // Map from (x,y) coordinates to node IDs for efficient lookup
    std::unordered_map<int, std::unordered_map<int, int>> coordToNodeId;
    
    // Congestion map: node ID -> congestion value
    std::unordered_map<int, double> congestionMap;
    
    // Congestion penalty factor
    double congestionPenaltyFactor;
    
    // Timeout in milliseconds
    int timeoutMs;
    
    // Internal implementation of the RSA algorithm
    SteinerTree buildRectilinearSteinerArborescence(
        int sourceX, int sourceY, 
        const std::vector<std::pair<int, int>>& sinkCoordinates,
        const std::vector<int>& sinkIds);
    
    // Helper method to merge two Steiner trees
    SteinerTree mergeTrees(const SteinerTree& tree1, const SteinerTree& tree2);
    
    // Helper method to find the node in the graph closest to the given coordinates
    int findClosestNode(int x, int y);
    
    // Convert Steiner tree to a path in the original graph
    std::vector<int> steinerTreeToPath(const SteinerTree& tree, int sourceNodeId);
    
    // Connect two nodes with a rectilinear path (Manhattan distance)
    std::vector<int> connectNodesRectilinear(int fromId, int toId);
    
    // Helper method to get node coordinates by ID
    std::pair<int, int> getNodeCoordinates(int nodeId);
};

#endif // STEINER_ARBORESCENCE_HPP 