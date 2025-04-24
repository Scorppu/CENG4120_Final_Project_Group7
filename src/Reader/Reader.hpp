#ifndef READER_HPP
#define READER_HPP

#include <string>
#include <vector>
#include "../Datastructure.hpp"

class Reader {
private:
    std::string devicePath;
    std::string netlistPath;
    
public:
    int counter = 0;

    // Constructor
    Reader(std::string devicePath, std::string netlistPath);

    // Parse the device file and return the number of nodes
    virtual bool parseDevice(std::vector<Node>& nodes, std::vector<std::vector<int>>& edges);

    // Parse the netlist file
    virtual bool parseNetlist(std::vector<Net>& nets);
    
    // Add method to verify and display device parsing results
    void verifyDeviceParsing(std::vector<Node>& nodes, std::vector<std::vector<int>>& edges);
    
    // Print all connections for a node
    void printAllConnections(std::vector<std::vector<int>>& edges);
    
    // Verify netlist parsing results
    void verifyNetlist(std::vector<Net>& nets);
};

#endif // READER_HPP
