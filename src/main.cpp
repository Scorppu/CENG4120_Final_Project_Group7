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

    std::vector<Node> nodes;
    std::vector<Edge> edges;
    std::vector<Net> nets;
    
    // Parse input files
    Reader reader = Reader(deviceFile, netlistFile);

    auto deviceParseStart = std::chrono::high_resolution_clock::now();
    reader.parseDevice(nodes, edges);
    std::cout << "Device parsing time: " << std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - deviceParseStart).count() << " seconds" << std::endl;
    std::cout << reader.counter << std::endl;



    auto netlistParseStart = std::chrono::high_resolution_clock::now();
    reader.parseNetlist(nets);
    std::cout << "Netlist parsing time: " << std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - netlistParseStart).count() << " seconds" << std::endl;

    return 0;
}

