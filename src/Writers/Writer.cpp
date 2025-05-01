#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <unordered_map>
#include "Writer.hpp"

// Constructor
Writer::Writer(std::string outputPath) : outputPath(outputPath), routingResults(nullptr), originalNets(nullptr) {}

// Write output to file
void Writer::writeOutput() {
    if (!routingResults || !originalNets) {
        std::cerr << "Error: No routing results or original nets provided" << std::endl;
        return;
    }
    
    // Create a mapping from net ID to net name for quick lookup
    std::unordered_map<int, std::string> netNames;
    for (const Net& net : *originalNets) {
        netNames[net.id] = net.name;
    }
    
    // Open output file
    std::ofstream outFile(outputPath);
    if (!outFile.is_open()) {
        std::cerr << "Error: Could not open output file " << outputPath << std::endl;
        return;
    }
    
    // Write each routed net to the output file
    bool isFirstNet = true;
    for (const auto& netRoute : *routingResults) {
        // Only include successfully routed nets
        if (!netRoute.isRouted) {
            continue;
        }
        
        // Add an empty line between nets (but not before the first net)
        if (!isFirstNet) {
            outFile << std::endl;
        } else {
            isFirstNet = false;
        }
        
        // Write net header: ID and name
        int netId = netRoute.netId;
        // Use the name from NetRoute directly, fall back to map if needed
        std::string netName = !netRoute.netName.empty() ? netRoute.netName : 
                             (netNames.count(netId) ? netNames[netId] : "UnknownNet");
        outFile << netId << " " << netName << std::endl;
        
        // Write each edge (parent-child node pair)
        for (const auto& edge : netRoute.edges) {
            outFile << edge.first << " " << edge.second << std::endl;
        }
    }
    
    outFile.close();
    std::cout << "Successfully wrote routing results to " << outputPath << std::endl;
}

// Set the routing results to write
void Writer::setRoutingResults(const std::vector<NetRoute>* results) {
    routingResults = results;
}

// Set the original nets for name lookup
void Writer::setOriginalNets(const std::vector<Net>* nets) {
    originalNets = nets;
}

