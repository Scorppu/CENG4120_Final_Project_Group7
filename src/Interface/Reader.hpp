#ifndef INPUT_H
#define INPUT_H

#include <string>
#include <vector>
#include <unordered_map>
#include <limits>
#include <cmath>

// Node structure to represent FPGA nodes
struct Node {
    int id;
    int type;
    int length;
    int beginX, beginY, endX, endY;
    std::string name;
    std::vector<int> children;
    
    // Distance estimate for A* search
    double getDistanceTo(const Node& other) const {
        double dx1 = endX - beginX;
        double dy1 = endY - beginY;
        double dx2 = other.beginX - endX;
        double dy2 = other.beginY - endY;
        return std::sqrt(dx2*dx2 + dy2*dy2);
    }
};

// Net structure representing connections to be routed
struct Net {
    int id;
    std::string name;
    int sourceNodeId;
    std::vector<int> sinkNodeIds;
    
    // Routing result
    std::vector<std::pair<int, int>> routingResult; // (parent, child) pairs
    bool isRouted = false;
    
    // Calculate bounding box for this net
    void calculateBoundingBox(const std::unordered_map<int, Node>& nodes, 
                              int& minX, int& minY, int& maxX, int& maxY) const;
};

// Global data structures
extern std::unordered_map<int, Node> nodes;  // Node ID -> Node
extern std::vector<Net> nets;                // All nets

// Input parsing functions
bool parseDevice(const std::string& devicePath);
bool parseNetlist(const std::string& netlistPath);

#endif // INPUT_H 