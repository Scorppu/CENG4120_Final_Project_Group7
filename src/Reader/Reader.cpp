#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <string>
#include <iterator>
#include "../Datastructure.hpp"

class Reader {
public:
    std::string devicePath;
    std::string netlistPath;

    int counter = 0;

    // Parse the device file and return the number of nodes
    virtual bool parseDevice(std::vector<Node>& nodes, std::vector<Edge>& edges) {
        // Get first line of device file and define the number of nodes

        std::ifstream deviceFile(devicePath);
        if (!deviceFile.is_open()) {
            std::cerr << "Failed to open device file: " << devicePath << std::endl;
            return false;
        }
        
        std::string line;
        // Read the number of nodes
        int numNodes;
        if (!std::getline(deviceFile, line)) {
            std::cerr << "Failed to read number of nodes from device file" << std::endl;
            return false;
        }
        
        try {
            numNodes = std::stoi(line);
        } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid number of nodes in device file: " << line << std::endl;
            return false;
        }
        
        std::cout << "Number of nodes: " << numNodes << std::endl;
        
        // Read node definitions
        nodes.resize(numNodes);
        for (int i = 0; i < numNodes; i++) {
            if (!std::getline(deviceFile, line)) {
                std::cerr << "Failed to read node " << i << " from device file" << std::endl;
                return false;
            }
            
            std::istringstream iss(line);
            
            Node node;
            
            // Parse ID and type string
            if (!(iss >> node.id >> node.type)) {
                std::cerr << "Failed to parse node ID and type: " << line << std::endl;
                return false;
            }
            
            // Parse the rest of the node data
            if (!(iss >> node.length >> node.beginX >> node.beginY >> node.endX >> node.endY)) {
                std::cerr << "Failed to parse node coordinates: " << line << std::endl;
                return false;
            }

            // Get the name (rest of the line)
            std::getline(iss, node.name);
            // Remove leading whitespace, if any
            if (!node.name.empty() && node.name[0] == ' '){
                node.name = node.name.substr(1);
            }
            
            nodes.push_back(node);
            counter++;
        }
        
        // Read adjacency list (edges)
        edges.resize(numNodes);
        int childId;
        while (std::getline(deviceFile, line)) {            
            std::istringstream iss(line);

            Edge edge;
            if (!(iss >> edge.parentID)) {
                std::cerr << "Failed to parse parent ID in adjacency list: '" << line << "'" << std::endl;
                continue;
            }
            
            std::cout << "parentId: " << edge.parentID << " ";
            edge.childID = std::vector<int>(
                std::istream_iterator<int>(iss),
                std::istream_iterator<int>()
            );
            std::cout << std::endl;
            edges.emplace_back(std::move(edge));
        }
        


        deviceFile.close();
        return true;
    }

    virtual void parseNetlist(std::vector<Net>& nets) {
        std::ifstream netlistFile(netlistPath);
        if (!netlistFile.is_open()) {
            std::cerr << "Failed to open netlist file: " << netlistPath << std::endl;
            return;
        }
        
        std::string line;
        while (std::getline(netlistFile, line)) {
            std::istringstream iss(line);
            std::string type;
            iss >> type;
            
            if (type == "NET") {
                Net net;
                iss >> net.id;
                std::getline(iss, net.name);
                // Remove leading whitespace, if any
                if (!net.name.empty() && net.name[0] == ' '){
                    net.name = net.name.substr(1);
                }
                
                // Read the next line (SOURCE)
                if (std::getline(netlistFile, line)) {
                    std::istringstream sourceIss(line);
                    std::string sourceType;
                    sourceIss >> sourceType >> net.sourceNodeId;
                    
                    // Read SINKs
                    while (std::getline(netlistFile, line)) {
                        std::istringstream sinkIss(line);
                        std::string sinkType;
                        sinkIss >> sinkType;
                        
                        if (sinkType == "SINK") {
                            int sinkId;
                            sinkIss >> sinkId;
                            net.sinkNodeIds.push_back(sinkId);
                        } else if (sinkType == "NET" || sinkType == "") {
                            // We've reached the next net or end of file
                            // Put the line back for next iteration
                            netlistFile.seekg(-line.length()-1, std::ios_base::cur);
                            break;
                        }
                    }
                }
                
                nets.push_back(net);
            }
        }
    }

    Reader(std::string devicePath, std::string netlistPath) {
        this->devicePath = devicePath;
        this->netlistPath = netlistPath;
    }
};



