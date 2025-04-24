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
    std::vector<int> nodeIDs;
    std::vector<std::pair<int, int>> edges;
};

// Forward declaration
struct RoutingTree;

// Define RoutingTree with smart pointers for memory management
struct RoutingTree : public std::enable_shared_from_this<RoutingTree> {
    int NetId;
    int currentNodeId;
    std::vector<int> sinkNodeIds;
    std::vector<std::shared_ptr<RoutingTree>> children;
    std::weak_ptr<RoutingTree> parent;  // Weak pointer to avoid reference cycles
    std::vector<std::pair<int, int>> edges;
    bool isRouted;  // Track if all sinks were successfully routed
    
    RoutingTree(int netId, int currentNodeId, std::vector<int> sinkNodeIds) {
        this->NetId = netId;
        this->currentNodeId = currentNodeId;
        this->sinkNodeIds = sinkNodeIds;
        this->isRouted = false;  // Initialize as false
    }
    
    // Add clean destructor to help ensure children are released
    ~RoutingTree() {
        // Clear children explicitly
        children.clear();
    }
    
    // Helper to add a child and set up the parent relationship
    void addChild(std::shared_ptr<RoutingTree> child) {
        children.push_back(child);
        child->parent = shared_from_this();
    }
};

#endif // DATASTRUCTURE_H

