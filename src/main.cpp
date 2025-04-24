#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <vector>
#include <string>
#include "DataStructure.hpp"
#include "Reader/Reader.cpp"
#include "Router/Router.cpp"
#include "Writer/Write.cpp"

void printUsage(const char* programName) {
    std::cout << "Usage: " << programName << " <device_file> <netlist_file> <output_file> [options]" << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --verbose                        Enable verbose logging" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 4) {
        printUsage(argv[0]);
        return 1;
    }

    std::string deviceFile = argv[1];
    std::string netlistFile = argv[2];
    std::string outputFile = argv[3];

    // Parse command-line options
    bool verboseLogging = false;             // Default logging level
    
    for (int i = 4; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "--verbose") {
            verboseLogging = true;
        } else {
            std::cerr << "Unknown option: " << arg << std::endl;
            printUsage(argv[0]);
            return 1;
        }
    }

    auto totalStart = std::chrono::high_resolution_clock::now();
    
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
    
    // Print routing configuration
    std::cout << "\n==== Routing Configuration ====\n";
    std::cout << "Strategy: Steiner Arborescence" << std::endl;
    std::cout << "Verbose logging: " << (verboseLogging ? "enabled" : "disabled") << std::endl;
    std::cout << "==============================\n" << std::endl;
    
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

