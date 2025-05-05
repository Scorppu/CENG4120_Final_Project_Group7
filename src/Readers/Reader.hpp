#ifndef READER_HPP
#define READER_HPP

#include <string>
#include <vector>
#include "../DataStructure.hpp"

class Reader {
private:
    std::string devicePath;
    std::string netlistPath;
    
public:
    int counter = 0;

    // Constructor
    Reader(std::string devicePath, std::string netlistPath);

    // Parse the device file and return the number of nodes
    virtual bool parseDevice(std::vector<Node>& nodes, std::vector<std::vector<int>>& edges, std::vector<std::vector<std::vector<int>>>& coordinateLookup);

    // Parse the netlist file
    virtual bool parseNetlist(std::vector<Net>& nets, std::vector<Node>& nodes, std::map<int, std::vector<int>>& x_to_ys, std::vector<std::vector<std::vector<int>>>& coordinateLookup, std::vector<std::vector<int>>& edges);
    
    // Add method to verify and display device parsing results
    void verifyDeviceParsing(std::vector<Node>& nodes, std::vector<std::vector<int>>& edges);
    
    // Print all connections for a node
    void printAllConnections(std::vector<std::vector<int>>& edges);
    
    // Verify netlist parsing results
    void verifyNetlist(std::vector<Net>& nets);

    // Get RST-T edges for each net
    void getRSTTEdges(Net& net, std::vector<Node>& nodes, std::vector<std::vector<std::vector<int>>>& coordinateLookup, std::vector<std::vector<int>>& edges);
};

#endif // READER_HPP
