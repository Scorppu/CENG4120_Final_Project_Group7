#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <vector>
#include "DataStructure.hpp"
#include "Reader/Reader.cpp"
#include "Router/Router.cpp"

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: <device_file> <netlist_file> <output_file>" << std::endl;
        return 1;
    }

    std::string deviceFile = argv[1];
    std::string netlistFile = argv[2];
    std::string outputFile = argv[3];

    auto start = std::chrono::high_resolution_clock::now();
    // Scope the variables to ensure proper cleanup
    {
        
        std::vector<Node> nodes;
        std::vector<std::vector<int>> edges;
        std::vector<Net> nets;
        
        // Parse input files
        Reader reader = Reader(deviceFile, netlistFile);

        auto deviceParseStart = std::chrono::high_resolution_clock::now();
        reader.parseDevice(nodes, edges);
        auto deviceParseEnd = std::chrono::high_resolution_clock::now();
        std::cout << "Nodes count: " << nodes.size() << std::endl;
        std::cout << "Edges count: " << edges.size() << std::endl;

        // Verify device parsing
        reader.verifyDeviceParsing(nodes, edges);
        
        std::cout << "Device parsing time: " << std::chrono::duration_cast<std::chrono::seconds>(
            deviceParseEnd - deviceParseStart).count() << " seconds" << std::endl;

        auto netlistParseStart = std::chrono::high_resolution_clock::now();
        reader.parseNetlist(nets);
        auto netlistParseEnd = std::chrono::high_resolution_clock::now();
        
        std::cout << "Netlist parsing time: " << std::chrono::duration_cast<std::chrono::seconds>(
            netlistParseEnd - netlistParseStart).count() << " seconds" << std::endl;
        std::cout << "Nets count: " << nets.size() << std::endl;
        
        // Create router in a nested scope for faster cleanup
        {
            Router router;
            
            // Start routing timer
            auto routingStart = std::chrono::high_resolution_clock::now();
            
            // Perform routing
            router.routeAllNets(nets, edges, nodes);
            
            // End routing timer
            auto routingEnd = std::chrono::high_resolution_clock::now();
            std::cout << "Total routing time: " << std::chrono::duration_cast<std::chrono::seconds>(
                routingEnd - routingStart).count() << " seconds" << std::endl;
                
            // Print routing results
            router.printRoutingResults();
            
            // Explicitly clean up router resources
            std::cout << "Cleaning up router resources..." << std::endl;
            router.clearAll();
            std::cout << "Router cleanup complete." << std::endl;
        }
        
        // Explicitly clean up large data structures
        std::cout << "Cleaning up all resources..." << std::endl;
        nodes.clear();
        edges.clear();
        nets.clear();
        std::cout << "Cleanup complete." << std::endl;
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    std::cout << "Program completed successfully." << std::endl;
    std::cout << "Total program time: " << std::chrono::duration_cast<std::chrono::seconds>(end - start).count() << " seconds" << std::endl;
    return 0;
}

