#ifndef DATASTRUCTURE_H
#define DATASTRUCTURE_H

#include <vector>
#include <string>
#include <unordered_map>
#include <queue>
#include <limits>
#include <memory>
#include <map>

struct Node {
    int id;
    std::string type;
    int length;
    int beginX, beginY, endX, endY; // Bounds: X(0, 107) Y(0, 299)
    std::string name;
};

struct SteinerNode : Node {
    bool isSteiner = true;
};

struct Net {
    int id;
    std::string name;
    std::vector<int> nodeIDs;
    std::pair<std::pair<int, int>, std::pair<int, int>> max_min_xy; // used for bounding box
    std::vector<std::pair<int,int>> rsttEdges; // used to store edges for RST-T
};

// Simpler alternative to RoutingTree
struct NetRoute {
    int netId;                          // ID of the net
    std::string netName;                // Name of the net
    std::vector<std::pair<int, int>> edges;  // Edges in the route (src_node, dest_node)
    bool isRouted;                      // Whether the routing was successful
    
    NetRoute() {}

    NetRoute(int netId, std::string netName) 
        : netId(netId), netName(netName), isRouted(false) {}
    
    // Add an edge to the route
    void addEdge(int fromNode, int toNode) {
        edges.push_back(std::make_pair(fromNode, toNode));
    }
    
    // Add multiple edges at once
    void addEdges(const std::vector<std::pair<int, int>>& newEdges) {
        edges.insert(edges.end(), newEdges.begin(), newEdges.end());
    }
};

#endif // DATASTRUCTURE_H

