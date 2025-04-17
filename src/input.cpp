#include "include/input.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

// After input.h include, now include router.h
#include "include/router.h"

// Global data structures implementation
std::unordered_map<int, Node> nodes;
std::vector<Net> nets;

// Implementation of the bounding box calculation
void Net::calculateBoundingBox(const std::unordered_map<int, Node>& nodes,
                               int& minX, int& minY, int& maxX, int& maxY) const {
    minX = std::numeric_limits<int>::max();
    minY = std::numeric_limits<int>::max();
    maxX = std::numeric_limits<int>::min();
    maxY = std::numeric_limits<int>::min();
    
    auto updateBounds = [&](const Node& node) {
        minX = std::min(minX, std::min(node.beginX, node.endX));
        minY = std::min(minY, std::min(node.beginY, node.endY));
        maxX = std::max(maxX, std::max(node.beginX, node.endX));
        maxY = std::max(maxY, std::max(node.beginY, node.endY));
    };
    
    if (nodes.count(sourceNodeId)) {
        updateBounds(nodes.at(sourceNodeId));
    }
    
    for (int sinkId : sinkNodeIds) {
        if (nodes.count(sinkId)) {
            updateBounds(nodes.at(sinkId));
        }
    }
}

// Parse the device file
bool parseDevice(const std::string& devicePath) {
    std::ifstream deviceFile(devicePath);
    if (!deviceFile.is_open()) {
        std::cerr << "Failed to open device file: " << devicePath << std::endl;
        return false;
    }
    
    std::string line;
    // Read the number of nodes
    int numNodes;
    std::getline(deviceFile, line);
    numNodes = std::stoi(line);
    
    // Read node definitions
    for (int i = 0; i < numNodes; i++) {
        std::getline(deviceFile, line);
        std::istringstream iss(line);
        
        Node node;
        iss >> node.id >> node.type >> node.length >> node.beginX >> node.beginY >> node.endX >> node.endY;
        
        // Read the rest of the line as the node name
        std::string nameTemp;
        iss >> nameTemp;  // Get the first part of the name
        std::getline(iss, line);  // Get the rest of the line
        node.name = nameTemp + line;
        
        nodes[node.id] = node;
        
        // Initialize costs
        nodeHistoryCost[node.id] = 0.0;
        nodeUsageCount[node.id] = 0;
    }
    
    // Read adjacency list (edges)
    while (std::getline(deviceFile, line)) {
        std::istringstream iss(line);
        int parentId;
        iss >> parentId;
        
        int childId;
        while (iss >> childId) {
            nodes[parentId].children.push_back(childId);
        }
    }
    
    deviceFile.close();
    return true;
}

// Parse the netlist file
bool parseNetlist(const std::string& netlistPath) {
    std::ifstream netlistFile(netlistPath);
    if (!netlistFile.is_open()) {
        std::cerr << "Failed to open netlist file: " << netlistPath << std::endl;
        return false;
    }
    
    std::string line;
    // Read the number of nets
    int numNets;
    std::getline(netlistFile, line);
    numNets = std::stoi(line);
    
    // Read net definitions
    for (int i = 0; i < numNets; i++) {
        std::getline(netlistFile, line);
        std::istringstream iss(line);
        
        Net net;
        iss >> net.id >> net.name >> net.sourceNodeId;
        
        int sinkId;
        while (iss >> sinkId) {
            net.sinkNodeIds.push_back(sinkId);
        }
        
        nets.push_back(net);
    }
    
    netlistFile.close();
    return true;
} 