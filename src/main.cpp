#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
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

    // Parse input files
    Reader reader = Reader(deviceFile, netlistFile);

    auto deviceParseStart = std::chrono::high_resolution_clock::now();
    reader.parseDevice();
    auto deviceParseEnd = std::chrono::high_resolution_clock::now();

    // Verify device parsing
    reader.verifyDeviceParsing();
    std::cout << "Device parsing time: " << std::chrono::duration_cast<std::chrono::seconds>(deviceParseEnd - deviceParseStart).count() << " seconds" << std::endl;

    auto netlistParseStart = std::chrono::high_resolution_clock::now();
    reader.parseNetlist();
    auto netlistParseEnd = std::chrono::high_resolution_clock::now();

    // Verify netlist parsing
    reader.verifyNetlist();
    std::cout << "Netlist parsing time: " << std::chrono::duration_cast<std::chrono::milliseconds>(netlistParseEnd - netlistParseStart).count() << " milliseconds" << std::endl;



    /* ROUTING BEGINS HERE */


    
    return 0;
}

