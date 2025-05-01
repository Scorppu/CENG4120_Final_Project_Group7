#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <vector>
#include <regex>
#include <string>
#include "DataStructure.hpp"
#include "Readers/Reader.hpp"
#include "Routers/Router.hpp"
#include "Writers/Writer.hpp"

// Helper function to extract design number from filename
int extractDesignNumber(const std::string& filename) {
    // Use regex to extract design number from patterns like "design1.netlist" or "design_1.netlist"
    std::regex designPattern("design[_]?(\\d+)");
    std::smatch matches;
    
    if (std::regex_search(filename, matches, designPattern) && matches.size() > 1) {
        return std::stoi(matches[1].str());
    }
    
    // Default to design 1 if no match found
    return 1;
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: <device_file> <netlist_file> <output_file>" << std::endl;
        return 1;
    }

    std::string deviceFile = argv[1];
    std::string netlistFile = argv[2];
    std::string outputFile = argv[3];

    auto totalStart = std::chrono::steady_clock::now();
    
    // Minimized logging to improve performance
    bool verboseLogging = false;
    
    // Create and scope all data structures
    std::vector<Node> nodes;
    std::vector<std::vector<int>> edges;
    std::vector<Net> nets;
    std::map<int, std::vector<int>> x_to_ys;

    // Parse input files
    Reader reader = Reader(deviceFile, netlistFile);

    auto deviceParseStart = std::chrono::steady_clock::now();
    reader.parseDevice(nodes, edges);
    auto deviceParseEnd = std::chrono::steady_clock::now();
    std::cout << "Nodes: " << nodes.size() << ", Edges: " << edges.size() << std::endl;
    
    std::cout << "Device parsing: " << std::chrono::duration_cast<std::chrono::seconds>(
        deviceParseEnd - deviceParseStart).count() << "s" << std::endl;

    auto netlistParseStart = std::chrono::steady_clock::now();
    reader.parseNetlist(nets, nodes, x_to_ys);
    auto netlistParseEnd = std::chrono::steady_clock::now();
    
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
    // SteinerTreeRouter stRouter;
    
    // Extract design number from netlist filename and set in router
    int designNumber = extractDesignNumber(netlistFile);
    std::cout << "Detected design number: " << designNumber << std::endl;
    
    // Set design-specific parameters
    router.setDesignNumber(designNumber);
    
    // Pass the program start time to the router for total time calculation
    router.setProgramStartTime(totalStart);
    
    // Set global timeout (default 249 seconds, but can be adjusted)
    if (designNumber == 5) {
        // For design 5, we need to stay under 250s total time
        router.setTimeout(249);
    } else {
        // For other designs, set a more generous timeout
        router.setTimeout(1000);
    }

    // Perform routing
    auto routingStart = std::chrono::steady_clock::now();
    router.routeAllNets(nets, edges, nodes, x_to_ys);
    auto routingEnd = std::chrono::steady_clock::now();
    
    std::cout << "Total routing time: " << std::chrono::duration_cast<std::chrono::seconds>(
        routingEnd - routingStart).count() << "s" << std::endl;
        
    // Print routing results
    router.printRoutingResults();
    // stRouter.printRoutingResults();
    
    // Write output to file
    Writer writer(outputFile);
    writer.setRoutingResults(&router.getRoutingResults());
    // writer.setRoutingResults(&stRouter.getRoutingResults());
    writer.setOriginalNets(&nets);
    writer.writeOutput();
    
    // Clean up resources
    if (verboseLogging) {
        std::cout << "Cleaning up resources..." << std::endl;
    }
    router.clearAll();
    // stRouter.clearAll();
    
    auto totalEnd = std::chrono::steady_clock::now();
    std::cout << "Total program time: " << std::chrono::duration_cast<std::chrono::seconds>(
        totalEnd - totalStart).count() << "s" << std::endl;

    return 0;
}

