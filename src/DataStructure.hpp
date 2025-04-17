#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H

#include <vector>
#include <string>
#include <unordered_map>
#include <queue>
#include <limits>
#include <memory>

struct Node {
    int id;
    std::string type;
    int length;
    int beginX, beginY, endX, endY;
    std::string name;
};

struct Net {
    int id;
    std::string name;
    int sourceNodeId;
    std::vector<int> sinkNodeIds;
};

struct Edge {
    int parentID;
    std::vector<int> childID;
    int length;
    int weight;
};


// Routing resource node with congestion information
struct RouteNode {
    int nodeId;
    double historicalCost = 1.0;         // Accumulated historical congestion cost
    double presentCost = 1.0;            // Current iteration congestion cost
    int usageCount = 0;                  // Number of nets using this node
    
    // Calculate total cost for routing through this node
    double getTotalCost() const {
        return historicalCost * presentCost;
    }
};

// Structure to represent a routing path from source to one sink
struct RoutingPath {
    int netId;
    int sinkNodeId;
    std::vector<int> path;               // Sequence of node IDs forming the path
    double cost;                         // Total path cost
    
    bool operator<(const RoutingPath& other) const {
        return cost > other.cost;        // For priority queue (min cost first)
    }
};

// Represents a complete routing solution for a net (from source to all sinks)
struct NetRouting {
    int netId;
    std::vector<RoutingPath> paths;      // One path for each sink
    double totalCost;                    // Sum of all path costs
    bool isRouted = false;               // Whether this net has been successfully routed
};

// Node for A* search algorithm
struct AStarNode {
    int nodeId;
    double gCost;                        // Cost from source to this node
    double hCost;                        // Heuristic cost from this node to target
    double fCost;                        // Total cost (g + h)
    int parentNodeId;                    // Parent node in the path
    
    bool operator>(const AStarNode& other) const {
        return fCost > other.fCost;      // For priority queue (min cost first)
    }
};

// Used in A* search for priority queue
typedef std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> AStarPriorityQueue;

// Congestion map: tracks congestion for all routing resources
class CongestionMap {
public:
    std::unordered_map<int, RouteNode> routeNodes;
    
    // Initialize a route node if it doesn't exist
    void initRouteNode(int nodeId) {
        if (routeNodes.find(nodeId) == routeNodes.end()) {
            routeNodes[nodeId] = RouteNode{nodeId};
        }
    }
    
    // Update congestion after a successful routing iteration
    void updateCongestion(double historicalFactor) {
        for (auto& pair : routeNodes) {
            RouteNode& node = pair.second;
            if (node.usageCount > 1) {
                // Increase historical cost for congested nodes
                node.historicalCost += historicalFactor * (node.usageCount - 1);
            }
            // Reset present cost and usage for next iteration
            node.presentCost = 1.0;
            node.usageCount = 0;
        }
    }
};

// Structure to track routing statistics
struct RoutingStats {
    int totalNets = 0;
    int successfullyRouted = 0;
    int totalWirelength = 0;
    bool hasCongestion = false;
    double executionTime = 0.0;
};

#endif // DATASTRUCTURE_H

