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

struct RoutingTree{
    int NetId;
    int currentNodeId;
    std::vector<int> sinkNodeIds;
    std::vector<RoutingTree*> children;
    RoutingTree* parent;
    std::vector<std::pair<int, int>> edges;
    RoutingTree(int netId, int currentNodeId, std::vector<int> sinkNodeIds) {
        this->NetId = netId;
        this->currentNodeId = currentNodeId;
        this->sinkNodeIds = sinkNodeIds;
        this->parent = nullptr;
    }
};


#endif // DATASTRUCTURE_H

