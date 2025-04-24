#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <vector>
#include "DataStructure.hpp"
#include "Reader/Reader.hpp"
#include "Router/Router.hpp"
#include "Writer/Writer.hpp"

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: <device_file> <netlist_file> <output_file>" << std::endl;
        return 1;
    }

    std::string deviceFile = argv[1];
    std::string netlistFile = argv[2];
    std::string outputFile = argv[3];

    auto totalStart = std::chrono::high_resolution_clock::now();
    
    // Minimized logging to improve performance
    bool verboseLogging = false;
    
    // Create and scope all data structures
    std::vector<Node> nodes;
    std::vector<std::vector<int>> edges;
    std::vector<Net> nets;
    
    // Parse input files
    Reader reader = Reader(deviceFile, netlistFile);

    auto deviceParseStart = std::chrono::high_resolution_clock::now();
    reader.parseDevice(nodes, edges);
    auto deviceParseEnd = std::chrono::high_resolution_clock::now();
    std::cout << "Nodes: " << nodes.size() << ", Edges: " << edges.size() << std::endl;
    
    std::cout << "Device parsing: " << std::chrono::duration_cast<std::chrono::seconds>(
        deviceParseEnd - deviceParseStart).count() << "s" << std::endl;

    auto netlistParseStart = std::chrono::high_resolution_clock::now();
    reader.parseNetlist(nets);
    auto netlistParseEnd = std::chrono::high_resolution_clock::now();
    
    std::cout << "Netlist parsing: " << std::chrono::duration_cast<std::chrono::seconds>(
        netlistParseEnd - netlistParseStart).count() << "s" << std::endl;
    std::cout << "Nets: " << nets.size() << std::endl;
    
    // Estimate memory footprint and pre-allocate if possible
    std::cout << "Estimating memory usage..." << std::endl;
    size_t estimatedNodeMemory = nodes.size() * sizeof(Node);
    size_t estimatedEdgeMemory = 0;
    for (const auto& edge : edges) {
        estimatedEdgeMemory += edge.size() * sizeof(int) + sizeof(std::vector<int>);
    }
    size_t estimatedNetMemory = nets.size() * sizeof(Net);
    std::cout << "Estimated memory usage: " 
              << (estimatedNodeMemory + estimatedEdgeMemory + estimatedNetMemory) / (1024 * 1024) 
              << "MB" << std::endl;
    
    // Create router 
    Router router;
    router.setVerbose(verboseLogging);
    
    // Perform routing
    auto routingStart = std::chrono::high_resolution_clock::now();
    router.routeAllNets(nets, edges, nodes);
    auto routingEnd = std::chrono::high_resolution_clock::now();
    std::cout << "Total routing time: " << std::chrono::duration_cast<std::chrono::seconds>(
        routingEnd - routingStart).count() << "s" << std::endl;
        
    // Print routing results
    router.printRoutingResults();
    
    // Write output to file
    Writer writer(outputFile);
    writer.setRoutingResults(&router.getRoutingResults());
    writer.setOriginalNets(&nets);
    writer.writeOutput();
    
    // Clean up resources
    if (verboseLogging) {
        std::cout << "Cleaning up resources..." << std::endl;
    }
    router.clearAll();
    
    auto totalEnd = std::chrono::high_resolution_clock::now();
    std::cout << "Total program time: " << std::chrono::duration_cast<std::chrono::seconds>(
        totalEnd - totalStart).count() << "s" << std::endl;

    return 0;
}

