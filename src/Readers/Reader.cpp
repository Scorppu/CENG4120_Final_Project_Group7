#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <string>
#include <iterator>
#include <algorithm>
#include <limits>
#include <cstring>  // For strncmp
#include "Reader.hpp"

// Constructor
Reader::Reader(std::string devicePath, std::string netlistPath) {
    this->devicePath = devicePath;
    this->netlistPath = netlistPath;
}

// Parse the device file and return the number of nodes
bool Reader::parseDevice(std::vector<Node>& nodes, std::vector<std::vector<int>>& edges) {
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
    
    
    // Read node definitions
    nodes.clear();
    nodes.reserve(numNodes);  // Reserve space without creating objects
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
    
    // Ultra-fast adjacency list reading
    edges.clear();
    edges.resize(numNodes);
    
    // Pre-allocate a reasonable size for each vector to avoid reallocations
    for (auto& edge : edges) {
        edge.reserve(8); // Most edges probably have fewer than 8 connections
    }
    
    int edgeIndex = 0;
    while (std::getline(deviceFile, line) && edgeIndex < numNodes) {
        // Skip empty lines
        if (line.empty()) {
            continue;
        }
        
        // Single-pass parsing without nested loops
        const char* str = line.c_str();
        char* end;
        long value;
        
        // Parse the entire line in one pass
        for (;;) {
            // Skip leading whitespace and find first digit
            while (*str && (*str == ' ' || *str == '\t')) str++;
            if (!*str) break;  // End of string
            
            // Convert to number directly from current position
            value = strtol(str, &end, 10);
            if (str == end) break;  // No conversion possible
            
            // Add to current edge vector
            edges[edgeIndex].push_back(static_cast<int>(value));
            
            // Move to next position
            str = end;
        }
        
        if (!edges[edgeIndex].empty()) {
            edgeIndex++;
        }
    }
    
    // Adjust final size if needed
    if (edgeIndex < numNodes) {
        edges.resize(edgeIndex);
    }

    deviceFile.close();
    return true;
}

// std::vector<std::pair<int, int>> Reader::getMSTEdges(Net& net, std::vector<Node>& nodes) {
//     // TODO: Implement Prim's algorithm to get MST edges
//     // X and Y coordinates of a node within the netlist is found in nodes[nodeID].beginX and nodes[nodeID].beginY
//     // required nodeID is found in net.nodeIDs
//     // Use Prim's algorithm to find the minimum spanning tree
//     size_t n = net.nodeIDs.size();
//     if (n == 0) return {};

//     std::vector<bool> in_mst(n, false);
//     std::vector<std::pair<double, int>> min_edge(n, {std::numeric_limits<double>::infinity(), -1});
//     min_edge[0] = {0.0, -1};

//     std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<std::pair<double, int>>> pq;
//     pq.push({0.0, 0});

//     std::vector<std::pair<int, int>> mst_edges;

//     while (!pq.empty()) {
//         double current_weight = pq.top().first;
//         int u = pq.top().second;
//         pq.pop();

//         if (in_mst[u]) continue;
//         in_mst[u] = true;

//         if (min_edge[u].second != -1) {
//             mst_edges.emplace_back(net.nodeIDs[min_edge[u].second], net.nodeIDs[u]);
//         }

//         for (int v = 0; v < n; ++v) {
//             if (!in_mst[v]) {
//                 int id_u = net.nodeIDs[u];
//                 int id_v = net.nodeIDs[v];
//                 double dx = nodes[id_u].beginX - nodes[id_v].beginX;
//                 double dy = nodes[id_u].beginY - nodes[id_v].beginY;
//                 double distance = sqrt(dx*dx + dy*dy);

//                 if (distance < min_edge[v].first) {
//                     min_edge[v] = {distance, u};
//                     pq.push({distance, v});
//                 }
//             }
//         }
//     }

//     return mst_edges;
// }


std::vector<std::pair<int, int>> Reader::getMSTEdges(Net& net, std::vector<Node>& nodes) {
    auto& nodeIDs = net.nodeIDs;
    size_t n = nodeIDs.size();
    if (n <= 1) return {};  // Handle 0 or 1 node

    // Validate node IDs first
    for (int id : nodeIDs) {
        if (id < 0 || id >= static_cast<int>(nodes.size())) {
            std::cerr << "Invalid node ID in net: " << id << std::endl;
            // Handle invalid ID - return empty or throw exception
            return {};
        }
    }

    std::vector<bool> in_mst(n, false);
    std::vector<std::pair<double, int>> min_edge(n, {std::numeric_limits<double>::infinity(), -1});
    min_edge[0] = {0.0, -1};

    using QueueElem = std::pair<double, int>;
    std::priority_queue<QueueElem, std::vector<QueueElem>, std::greater<QueueElem>> pq;
    pq.push({0.0, 0});

    std::vector<std::pair<int, int>> mst_edges;
    mst_edges.reserve(n - 1); // We'll have at most n-1 edges in the MST

    while (!pq.empty()) {
        double current_weight = pq.top().first;
        int u = pq.top().second;
        pq.pop();

        // Skip if already in MST
        if (in_mst[u]) continue;
        
        // Skip stale entries
        if (current_weight > min_edge[u].first) continue;
        
        in_mst[u] = true;

        if (min_edge[u].second != -1) {
            std::pair<int, int> edge = std::make_pair(nodeIDs[min_edge[u].second], nodeIDs[u]);
            mst_edges.push_back(edge);
        }

        for (int v = 0; v < n; ++v) {
            if (!in_mst[v]) {
                int id_u = nodeIDs[u];
                int id_v = nodeIDs[v];
                
                // Use manhattan distance instead of euclidean for FPGA routing
                double distance = std::abs(nodes[id_u].beginX - nodes[id_v].beginX) + 
                                  std::abs(nodes[id_u].beginY - nodes[id_v].beginY);
                
                // Only update if this is a better distance
                if (distance < min_edge[v].first) {
                    min_edge[v] = {distance, u};
                    pq.push({distance, v});
                }
            }
        }
    }

    return mst_edges;
}


// Parse the netlist file
bool Reader::parseNetlist(std::vector<Net>& nets, std::vector<Node>& nodes, std::map<int, std::vector<int>>& x_to_ys) {
    std::ifstream netlistFile(netlistPath);
    if (!netlistFile.is_open()) {
        std::cerr << "Failed to open netlist file: " << netlistPath << std::endl;
        return false;
    }
    
    std::string line;
    if(!std::getline(netlistFile, line)) {
        std::cerr << "Failed to read number of nets from netlist file" << std::endl;
        return false;
    }

    int numNets;

    try {
        numNets = std::stoi(line);
    } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid number of nets in netlist file: " << line << std::endl;
        return false;
    }

    nets.clear();
    nets.resize(numNets);  // Create all Net objects at once

    for(int i = 0; i < numNets; i++) {
        if(!std::getline(netlistFile, line)) {
            std::cerr << "Failed to read net " << i << " from netlist file" << std::endl;
            return false;
        }

        // Access and modify the pre-allocated Net object
        Net& net = nets[i];
        int nodeId;
        std::istringstream iss(line);
        
         
        // Parse ID and name
        iss >> net.id >> net.name;
        
        // Clear any existing node IDs and parse new ones
        net.nodeIDs.clear();
        std::vector<std::pair<int, int>> xy;
        int max_x = 0, min_x = 0, max_y = 0, min_y = 0;
        while(iss >> nodeId) {
            // TODO: Add pin coordinates to layered sorted vector
            // TODO: Calculate max(x,y) & min(x,y) for each net
            if(nodes[nodeId].beginX > max_x) max_x = nodes[nodeId].beginX;
            if(nodes[nodeId].beginX < min_x) min_x = nodes[nodeId].beginX;
            if(nodes[nodeId].beginY > max_y) max_y = nodes[nodeId].beginY;
            if(nodes[nodeId].beginY < min_y) min_y = nodes[nodeId].beginY;
            xy.push_back(std::make_pair(nodes[nodeId].beginX, nodes[nodeId].beginY));
            net.nodeIDs.push_back(nodeId);
        }
        net.max_min_xy = std::make_pair(std::make_pair(max_x, min_x), std::make_pair(max_y, min_y));

        net.mst_edges = getMSTEdges(net, nodes);
        

        for (const auto& p : xy) {
            x_to_ys[p.first].push_back(p.second);
        }
        for (auto& p1 : x_to_ys) {
            std::sort(p1.second.begin(), p1.second.end());
        }
    }

    netlistFile.close();
    return true;
}

// Add method to verify and display device parsing results
void Reader::verifyDeviceParsing(std::vector<Node>& nodes, std::vector<std::vector<int>>& edges) {
    std::cout << "\n==== Device Parsing Verification ====\n";
    std::cout << "Total nodes parsed: " << nodes.size() << std::endl;
    std::cout << "Total edges parsed: " << edges.size() << std::endl;
    
    // Verify node IDs
    if (!nodes.empty()) {
        std::cout << "First node ID: " << nodes.front().id << std::endl;
        std::cout << "Last node ID: " << nodes.back().id << std::endl;
    }
    
    // Verify edge connections
    size_t totalConnections = 0;
    int minConnections = std::numeric_limits<int>::max();
    int maxConnections = 0;
    
    for (const auto& edge : edges) {
        if (!edge.empty()) {
            int connections = edge.size() - 1; // First element is parent ID
            totalConnections += connections;
            minConnections = std::min(minConnections, connections);
            maxConnections = std::max(maxConnections, connections);
        }
    }
    
    if (!edges.empty()) {
        std::cout << "Total connections: " << totalConnections << std::endl;
        std::cout << "Min connections per node: " << (minConnections == std::numeric_limits<int>::max() ? 0 : minConnections) << std::endl;
        std::cout << "Max connections per node: " << maxConnections << std::endl;
        std::cout << "Average connections per node: " << (edges.empty() ? 0 : totalConnections / edges.size()) << std::endl;
        
        // Sample some edges (first, middle, last)
        std::cout << "\nSample edges:" << std::endl;
        
        // First edge
        if (!edges.empty() && !edges.front().empty()) {
            std::cout << "First edge - Parent: " << edges.front()[0] << ", Children: ";
            for (size_t i = 1; i < std::min(edges.front().size(), size_t(5)); i++) {
                std::cout << edges.front()[i] << " ";
            }
            if (edges.front().size() > 5) std::cout << "... (" << edges.front().size() - 1 << " total)";
            std::cout << std::endl;
        }
        
        // Middle edge (if at least 3 edges)
        if (edges.size() >= 3) {
            size_t middle = edges.size() / 2;
            if (!edges[middle].empty()) {
                std::cout << "Middle edge - Parent: " << edges[middle][0] << ", Children: ";
                for (size_t i = 1; i < std::min(edges[middle].size(), size_t(5)); i++) {
                    std::cout << edges[middle][i] << " ";
                }
                if (edges[middle].size() > 5) std::cout << "... (" << edges[middle].size() - 1 << " total)";
                std::cout << std::endl;
            }
        }
        
        // Last edge
        if (!edges.empty() && !edges.back().empty()) {
            std::cout << "Last edge - Parent: " << edges.back()[0] << ", Children: ";
            for (size_t i = 1; i < std::min(edges.back().size(), size_t(5)); i++) {
                std::cout << edges.back()[i] << " ";
            }
            if (edges.back().size() > 5) std::cout << "... (" << edges.back().size() - 1 << " total)";
            std::cout << std::endl;
        }
    }
    
    std::cout << "====================================\n" << std::endl;
}

// Function to print all connections in the edges data structure
void Reader::printAllConnections(std::vector<std::vector<int>>& edges) {
    std::cout << "\n==== All Connections in Device ====\n";
    std::cout << "Total number of edges: " << edges.size() << std::endl;
    
    for (size_t i = 0; i < edges.size(); ++i) {
        const auto& edge = edges[i];
        
        if (!edge.empty()) {
            std::cout << "Edge " << i << " - Parent: " << edge[0] << ", Children: ";
            
            if (edge.size() > 1) {
                for (size_t j = 1; j < edge.size(); ++j) {
                    std::cout << edge[j];
                    if (j < edge.size() - 1) {
                        std::cout << ", ";
                    }
                }
                std::cout << " (Total: " << edge.size() - 1 << ")";
            } else {
                std::cout << "None";
            }
            std::cout << std::endl;
        } else {
            std::cout << "Edge " << i << " - Empty" << std::endl;
        }
    }
    
    std::cout << "==================================\n" << std::endl;
}

// Function to verify the contents of the netlist
void Reader::verifyNetlist(std::vector<Net>& nets) {
    std::cout << "\n==== Netlist Verification ====\n";
    std::cout << "Total number of nets: " << nets.size() << std::endl;
    
    // Count statistics
    size_t totalConnections = 0;
    int minConnections = std::numeric_limits<int>::max();
    int maxConnections = 0;
    
    for (const auto& net : nets) {
        // First node is source, rest are sinks
        int sinkCount = net.nodeIDs.size() > 0 ? net.nodeIDs.size() - 1 : 0;
        totalConnections += sinkCount;
        minConnections = std::min(minConnections, sinkCount);
        maxConnections = std::max(maxConnections, sinkCount);
    }
    
    // Display statistics
    std::cout << "Total sink connections: " << totalConnections << std::endl;
    std::cout << "Min sinks per net: " << (minConnections == std::numeric_limits<int>::max() ? 0 : minConnections) << std::endl;
    std::cout << "Max sinks per net: " << maxConnections << std::endl;
    std::cout << "Average sinks per net: " << (nets.empty() ? 0 : totalConnections / nets.size()) << std::endl;
    
    // Sample some nets
    std::cout << "\nSample nets:" << std::endl;
    
    // First net
    if (!nets.empty()) {
        const auto& net = nets.front();
        std::cout << "First net - ID: " << net.id << ", Name: " << net.name << std::endl;
        
        // Check if net has any nodes
        if (!net.nodeIDs.empty()) {
            // First node is source
            std::cout << "  Source: " << net.nodeIDs[0] << std::endl;
            
            // Rest are sinks
            std::cout << "  Sinks: ";
            for (size_t i = 1; i < std::min(net.nodeIDs.size(), size_t(6)); i++) {
                std::cout << net.nodeIDs[i];
                if (i < std::min(net.nodeIDs.size(), size_t(6)) - 1) std::cout << ", ";
            }
            if (net.nodeIDs.size() > 6) std::cout << "... (" << (net.nodeIDs.size() - 1) << " total)";
            std::cout << std::endl;
        } else {
            std::cout << "  No nodes defined" << std::endl;
        }
    }
    
    // Middle net
    if (nets.size() >= 3) {
        const auto& net = nets[nets.size() / 2];
        std::cout << "Middle net - ID: " << net.id << ", Name: " << net.name << std::endl;
        
        // Check if net has any nodes
        if (!net.nodeIDs.empty()) {
            // First node is source
            std::cout << "  Source: " << net.nodeIDs[0] << std::endl;
            
            // Rest are sinks
            std::cout << "  Sinks: ";
            for (size_t i = 1; i < std::min(net.nodeIDs.size(), size_t(6)); i++) {
                std::cout << net.nodeIDs[i];
                if (i < std::min(net.nodeIDs.size(), size_t(6)) - 1) std::cout << ", ";
            }
            if (net.nodeIDs.size() > 6) std::cout << "... (" << (net.nodeIDs.size() - 1) << " total)";
            std::cout << std::endl;
        } else {
            std::cout << "  No nodes defined" << std::endl;
        }
    }
    
    // Last net
    if (!nets.empty()) {
        const auto& net = nets.back();
        std::cout << "Last net - ID: " << net.id << ", Name: " << net.name << std::endl;
        
        // Check if net has any nodes
        if (!net.nodeIDs.empty()) {
            // First node is source
            std::cout << "  Source: " << net.nodeIDs[0] << std::endl;
            
            // Rest are sinks
            std::cout << "  Sinks: ";
            for (size_t i = 1; i < std::min(net.nodeIDs.size(), size_t(6)); i++) {
                std::cout << net.nodeIDs[i];
                if (i < std::min(net.nodeIDs.size(), size_t(6)) - 1) std::cout << ", ";
            }
            if (net.nodeIDs.size() > 6) std::cout << "... (" << (net.nodeIDs.size() - 1) << " total)";
            std::cout << std::endl;
        } else {
            std::cout << "  No nodes defined" << std::endl;
        }
    }
    
    std::cout << "=============================\n" << std::endl;
}



