#include "../Interface/Writer.hpp"
#include "../Interface/Reader.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>

// External declarations
extern std::vector<Net> nets;

// Write routing result to output file
bool writeRoutingResult(const std::string& outputPath) {
    std::ofstream outputFile(outputPath);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open output file: " << outputPath << std::endl;
        return false;
    }
    
    for (const auto& net : nets) {
        if (!net.isRouted) continue;
        
        outputFile << net.id << " " << net.name << std::endl;
        for (const auto& edge : net.routingResult) {
            outputFile << edge.first << " " << edge.second << std::endl;
        }
        outputFile << std::endl;
    }
    
    outputFile.close();
    return true;
}

// Print routing statistics
void printRoutingStats(int routedNets, bool congested, int wirelength, double runtime) {
    std::cout << "=============================" << std::endl;
    std::cout << "Routing Results" << std::endl;
    std::cout << "=============================" << std::endl;
    std::cout << "Routed nets: " << routedNets << "/" << nets.size() 
              << " (" << std::fixed << std::setprecision(2) 
              << (100.0 * routedNets / nets.size()) << "%)" << std::endl;
    std::cout << "Congestion: " << (congested ? "Yes" : "No") << std::endl;
    std::cout << "Total wirelength: " << wirelength << std::endl;
    std::cout << "Runtime: " << std::fixed << std::setprecision(3) << runtime << " seconds" << std::endl;
    std::cout << "=============================" << std::endl;
} 