#include "SteinerSALT.hpp"
#include "../Datastructure.hpp"
#include <iostream>
#include <vector>

int main() {
    // Create a simple grid graph
    std::vector<std::vector<int>> edges;
    std::vector<Node> nodes;
    
    // Create a 10x10 grid of nodes
    int nodeId = 0;
    for (int y = 0; y < 10; ++y) {
        for (int x = 0; x < 10; ++x) {
            Node node;
            node.id = nodeId;
            node.beginX = x;
            node.beginY = y;
            nodes.push_back(node);
            nodeId++;
        }
    }
    
    // Initialize SteinerSALT algorithm
    SteinerSALT steiner(edges, nodes);
    steiner.setVerbose(true);
    
    // Add an obstacle in the middle of the grid
    Obstacle obstacle(3, 3, 6, 6);
    steiner.addObstacle(obstacle);
    
    std::cout << "Finding Steiner tree with obstacle..." << std::endl;
    
    // Define source and sink nodes
    int sourceId = 0;  // Node at (0,0)
    std::vector<int> sinkIds = {99};  // Node at (9,9)
    
    // Find Steiner tree
    std::vector<int> path = steiner.findSteinerTree(sourceId, sinkIds);
    
    // Print resulting path
    std::cout << "Path from source to sink (with obstacle):" << std::endl;
    for (int nodeId : path) {
        auto coords = steiner.getNodeCoordinates(nodeId);
        std::cout << "Node " << nodeId << " at (" << coords.first << "," << coords.second << ")" << std::endl;
    }
    
    // Now try without the obstacle for comparison
    steiner.clearObstacles();
    
    std::cout << "\nFinding Steiner tree without obstacle..." << std::endl;
    
    // Find Steiner tree again
    std::vector<int> pathNoObstacle = steiner.findSteinerTree(sourceId, sinkIds);
    
    // Print resulting path
    std::cout << "Path from source to sink (without obstacle):" << std::endl;
    for (int nodeId : pathNoObstacle) {
        auto coords = steiner.getNodeCoordinates(nodeId);
        std::cout << "Node " << nodeId << " at (" << coords.first << "," << coords.second << ")" << std::endl;
    }
    
    // Test with multiple sinks
    std::cout << "\nFinding Steiner tree with multiple sinks..." << std::endl;
    
    // Add the obstacle back
    steiner.addObstacle(obstacle);
    
    // Define multiple sinks
    std::vector<int> multipleSinks = {99, 90, 9};  // Nodes at (9,9), (0,9), (9,0)
    
    // Find Steiner tree
    std::vector<int> multiPath = steiner.findSteinerTree(sourceId, multipleSinks);
    
    // Print resulting path
    std::cout << "Path connecting source to multiple sinks (with obstacle):" << std::endl;
    for (int nodeId : multiPath) {
        auto coords = steiner.getNodeCoordinates(nodeId);
        std::cout << "Node " << nodeId << " at (" << coords.first << "," << coords.second << ")" << std::endl;
    }
    
    return 0;
} 