#include "Interface/Reader.hpp"
#include "Interface/Router.hpp"
#include "Interface/Writer.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char* argv[]) {
    // Check command-line arguments
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <device> <netlist> <result>" << std::endl;
        return 1;
    }
    
    std::string devicePath = argv[1];
    std::string netlistPath = argv[2];
    std::string resultPath = argv[3];
    
    // Start timing
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Parse input files
    std::cout << "Parsing device file..." << std::endl;
    if (!parseDevice(devicePath)) {
        return 1;
    }
    
    std::cout << "Parsing netlist file..." << std::endl;
    if (!parseNetlist(netlistPath)) {
        return 1;
    }
    
    // Setup timeout thread
    int timeLimit = (nets.size() > 10000) ? 250 : 100; // Design 5 has more nets
    std::thread timeoutThread([timeLimit]() {
        std::this_thread::sleep_for(std::chrono::seconds(timeLimit - 5)); // Give 5 seconds for cleanup
        timedOut = true;
        std::cout << "Timeout reached! Finishing up..." << std::endl;
    });
    timeoutThread.detach();
    
    // Perform routing
    std::cout << "Routing " << nets.size() << " nets..." << std::endl;
    int numThreads = std::min(8, static_cast<int>(std::thread::hardware_concurrency()));
    std::cout << "Using " << numThreads << " threads" << std::endl;
    
    parallelPathFinder(numThreads);
    
    // Analyze results
    int routedNets = countRoutedNets();
    bool congested = hasCongestion();
    int wirelength = calculateWirelength();
    
    // Write output
    std::cout << "Writing routing result..." << std::endl;
    if (!writeRoutingResult(resultPath)) {
        return 1;
    }
    
    // Calculate runtime
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    double runtimeSeconds = duration.count() / 1000.0;
    
    // Print statistics
    printRoutingStats(routedNets, congested, wirelength, runtimeSeconds);
    
    return 0;
} 