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
    std::cout << "  --verbose                  Enable verbose logging" << std::endl;
    std::cout << "  --timeout <ms>             Set timeout for pathfinding (default: 5000)" << std::endl;
    std::cout << "  --congestion <val>         Set congestion penalty factor (default: 1.5)" << std::endl;
    std::cout << "  --epsilon <val>            Set epsilon for SALT algorithm (default: 0.5)" << std::endl;
    std::cout << "  --use-salt                 Use SALT algorithm instead of Arborescence (default)" << std::endl;
    std::cout << "  --use-arborescence         Use Arborescence algorithm instead of SALT" << std::endl;
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
    int timeout = 5000;
    double congestionFactor = 1.5;
    double epsilon = 0.5;
    bool useSALT = true;                     // Default to using SALT algorithm
    
    for (int i = 4; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "--verbose") {
            verboseLogging = true;
        } else if (arg == "--timeout" && i + 1 < argc) {
            timeout = std::stoi(argv[++i]);
        } else if (arg == "--congestion" && i + 1 < argc) {
            congestionFactor = std::stod(argv[++i]);
        } else if (arg == "--epsilon" && i + 1 < argc) {
            epsilon = std::stod(argv[++i]);
        } else if (arg == "--use-salt") {
            useSALT = true;
        } else if (arg == "--use-arborescence") {
            useSALT = false;
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
    
    // Display routing configuration
    std::cout << "\n==== Routing Configuration ====" << std::endl;
    std::cout << "Strategy: " << (useSALT ? "SALT" : "Steiner Arborescence") << std::endl;
    std::cout << "Verbose logging: " << (verboseLogging ? "enabled" : "disabled") << std::endl;
    std::cout << "Timeout: " << timeout << "ms" << std::endl;
    std::cout << "Congestion factor: " << congestionFactor << std::endl;
    if (useSALT) {
        std::cout << "Epsilon: " << epsilon << std::endl;
    }
    std::cout << "==============================" << std::endl;
    
    // Create router and route all nets
    Router router;
    
    // Set router configuration based on command line options
    router.setVerbose(verboseLogging);
    router.setTimeout(timeout);
    router.setCongestionFactor(congestionFactor);
    router.setEpsilon(epsilon);
    router.setUseSALT(useSALT);
    
    auto startRouteTime = std::chrono::high_resolution_clock::now();
    router.routeAllNets(nets, edges, nodes);
    
    auto endRouteTime = std::chrono::high_resolution_clock::now();
    auto routeTime = std::chrono::duration_cast<std::chrono::seconds>(endRouteTime - startRouteTime).count();
    std::cout << "Total routing time: " << routeTime << "s" << std::endl;
        
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

