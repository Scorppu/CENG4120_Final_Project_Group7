#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <map>
#include <set>
#include <queue>
#include <limits>
#include <algorithm>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <functional>
#include <memory>
#include <cmath>
#include <string>

// Node structure to represent FPGA nodes
struct Node {
    int id;
    int type;
    int length;
    int beginX, beginY, endX, endY;
    std::string name;
    std::vector<int> children;
    
    // Distance estimate for A* search
    double getDistanceTo(const Node& other) const {
        double dx1 = endX - beginX;
        double dy1 = endY - beginY;
        double dx2 = other.beginX - endX;
        double dy2 = other.beginY - endY;
        return std::sqrt(dx2*dx2 + dy2*dy2);
    }
};

// Net structure representing connections to be routed
struct Net {
    int id;
    std::string name;
    int sourceNodeId;
    std::vector<int> sinkNodeIds;
    
    // Routing result
    std::vector<std::pair<int, int>> routingResult; // (parent, child) pairs
    bool isRouted = false;
    
    // Calculate bounding box for this net
    void calculateBoundingBox(const std::unordered_map<int, Node>& nodes, 
                              int& minX, int& minY, int& maxX, int& maxY) const {
        minX = std::numeric_limits<int>::max();
        minY = std::numeric_limits<int>::max();
        maxX = std::numeric_limits<int>::min();
        maxY = std::numeric_limits<int>::min();
        
        auto updateBounds = [&](const Node& node) {
            minX = std::min(minX, std::min(node.beginX, node.endX));
            minY = std::min(minY, std::min(node.beginY, node.endY));
            maxX = std::max(maxX, std::max(node.beginX, node.endX));
            maxY = std::max(maxY, std::max(node.beginY, node.endY));
        };
        
        if (nodes.count(sourceNodeId)) {
            updateBounds(nodes.at(sourceNodeId));
        }
        
        for (int sinkId : sinkNodeIds) {
            if (nodes.count(sinkId)) {
                updateBounds(nodes.at(sinkId));
            }
        }
    }
};

// Global Variables
std::unordered_map<int, Node> nodes;  // Node ID -> Node
std::vector<Net> nets;                // All nets
std::atomic<bool> timedOut{false};    // Time out flag

// Congestion data
std::unordered_map<int, int> nodeUsageCount;  // Node ID -> usage count
std::unordered_map<int, double> nodeHistoryCost;  // Node ID -> history cost

// Mutex for thread safety
std::mutex nodeUsageMutex;

// Parameters for PathFinder algorithm
const double BASE_COST = 1.0;
const double HISTORY_FACTOR_INITIAL = 0.5;
const double HISTORY_FACTOR_INCREMENT = 0.5;
const double PRESENT_CONGESTION_PENALTY = 2.0;
const int MAX_ITERATIONS = 50;

// Parse the device file
bool parseDevice(const std::string& devicePath) {
    std::ifstream deviceFile(devicePath);
    if (!deviceFile.is_open()) {
        std::cerr << "Failed to open device file: " << devicePath << std::endl;
        return false;
    }
    
    std::string line;
    // Read the number of nodes
    int numNodes;
    std::getline(deviceFile, line);
    numNodes = std::stoi(line);
    
    // Read node definitions
    for (int i = 0; i < numNodes; i++) {
        std::getline(deviceFile, line);
        std::istringstream iss(line);
        
        Node node;
        iss >> node.id >> node.type >> node.length >> node.beginX >> node.beginY >> node.endX >> node.endY;
        
        // Read the rest of the line as the node name
        std::string nameTemp;
        iss >> nameTemp;  // Get the first part of the name
        std::getline(iss, line);  // Get the rest of the line
        node.name = nameTemp + line;
        
        nodes[node.id] = node;
        
        // Initialize costs
        nodeHistoryCost[node.id] = 0.0;
        nodeUsageCount[node.id] = 0;
    }
    
    // Read adjacency list (edges)
    while (std::getline(deviceFile, line)) {
        std::istringstream iss(line);
        int parentId;
        iss >> parentId;
        
        int childId;
        while (iss >> childId) {
            nodes[parentId].children.push_back(childId);
        }
    }
    
    deviceFile.close();
    return true;
}

// Parse the netlist file
bool parseNetlist(const std::string& netlistPath) {
    std::ifstream netlistFile(netlistPath);
    if (!netlistFile.is_open()) {
        std::cerr << "Failed to open netlist file: " << netlistPath << std::endl;
        return false;
    }
    
    std::string line;
    // Read the number of nets
    int numNets;
    std::getline(netlistFile, line);
    numNets = std::stoi(line);
    
    // Read net definitions
    for (int i = 0; i < numNets; i++) {
        std::getline(netlistFile, line);
        std::istringstream iss(line);
        
        Net net;
        iss >> net.id >> net.name >> net.sourceNodeId;
        
        int sinkId;
        while (iss >> sinkId) {
            net.sinkNodeIds.push_back(sinkId);
        }
        
        nets.push_back(net);
    }
    
    netlistFile.close();
    return true;
}

// PathFinder algorithm cost function
double calculateNodeCost(int nodeId, bool ignoreCongestion = false) {
    double baseCost = BASE_COST;
    double cost = baseCost + nodeHistoryCost[nodeId];
    
    if (!ignoreCongestion) {
        int usage = nodeUsageCount[nodeId];
        if (usage > 1) {
            cost += PRESENT_CONGESTION_PENALTY * (usage - 1);
        }
    }
    
    return cost;
}

// A* search for routing a single net to a sink
bool routeNetToSink(const Net& net, int sinkNodeId, const std::unordered_map<int, bool>& occupiedNodes,
                    std::unordered_map<int, int>& parentMap, bool ignoreCongestion = false) {
    // Priority queue for A* search
    using QueueItem = std::pair<double, int>; // (cost, nodeId)
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> pq;
    
    // Start from source node
    pq.push({0.0, net.sourceNodeId});
    
    // Track visited nodes and their costs
    std::unordered_map<int, double> gScore;
    gScore[net.sourceNodeId] = 0.0;
    
    // A* search
    while (!pq.empty() && !timedOut) {
        auto [_, currentNodeId] = pq.top();
        pq.pop();
        
        // If we've reached the sink, we're done
        if (currentNodeId == sinkNodeId) {
            return true;
        }
        
        // Get neighbors (children nodes)
        const Node& currentNode = nodes[currentNodeId];
        for (int childId : currentNode.children) {
            // Skip if node is already occupied by another net (unless we're ignoring congestion)
            if (!ignoreCongestion && occupiedNodes.count(childId) && occupiedNodes.at(childId)) {
                continue;
            }
            
            // Calculate cost to this child
            double nodeCost = calculateNodeCost(childId, ignoreCongestion);
            double newGScore = gScore[currentNodeId] + nodeCost;
            
            // If we haven't seen this node or found a better path
            if (gScore.count(childId) == 0 || newGScore < gScore[childId]) {
                // Update cost
                gScore[childId] = newGScore;
                
                // Calculate heuristic (distance to sink)
                double hScore = nodes[childId].getDistanceTo(nodes[sinkNodeId]);
                double fScore = newGScore + hScore;
                
                // Add to priority queue
                pq.push({fScore, childId});
                
                // Update parent map
                parentMap[childId] = currentNodeId;
            }
        }
    }
    
    // If we get here, we couldn't find a path
    return false;
}

// Extract path from source to sink using parent map
std::vector<int> extractPath(int sourceNodeId, int sinkNodeId, const std::unordered_map<int, int>& parentMap) {
    std::vector<int> path;
    int current = sinkNodeId;
    
    while (current != sourceNodeId) {
        path.push_back(current);
        if (parentMap.count(current) == 0) {
            // Should not happen if path was found
            return {};
        }
        current = parentMap.at(current);
    }
    
    path.push_back(sourceNodeId);
    std::reverse(path.begin(), path.end());
    return path;
}

// Route a single net (connects source to all sinks)
bool routeNet(Net& net, std::unordered_map<int, bool>& occupiedNodes, bool ignoreCongestion = false) {
    // Create a routing tree starting from the source
    std::set<int> routedNodes = {net.sourceNodeId};
    std::vector<std::pair<int, int>> routingResult;
    
    // Route to each sink
    for (int sinkId : net.sinkNodeIds) {
        // Track parent nodes for path extraction
        std::unordered_map<int, int> parentMap;
        
        // Find path to this sink from any node in our current tree
        bool foundPath = false;
        int bestStartNodeId = -1;
        
        for (int startNodeId : routedNodes) {
            // Temporarily set the source node to this start node
            Net tempNet = net;
            tempNet.sourceNodeId = startNodeId;
            
            // Find path from this start node to the sink
            parentMap.clear();
            if (routeNetToSink(tempNet, sinkId, occupiedNodes, parentMap, ignoreCongestion)) {
                foundPath = true;
                bestStartNodeId = startNodeId;
                break;
            }
        }
        
        // If we couldn't find a path to this sink, the routing failed
        if (!foundPath) {
            return false;
        }
        
        // Extract the path from the best start node to the sink
        std::vector<int> path = extractPath(bestStartNodeId, sinkId, parentMap);
        
        // Add the path to our routing tree
        for (size_t i = 0; i < path.size() - 1; i++) {
            routingResult.push_back({path[i], path[i + 1]});
            routedNodes.insert(path[i + 1]);
        }
    }
    
    // Update the net's routing result
    net.routingResult = routingResult;
    net.isRouted = true;
    
    // Mark nodes as occupied
    for (const auto& [parent, child] : routingResult) {
        nodeUsageCount[child]++;
        occupiedNodes[child] = true;
    }
    
    return true;
}

// PathFinder algorithm main function
void runPathFinder(int startNetIndex, int endNetIndex) {
    // Track which nodes are occupied by nets
    std::unordered_map<int, bool> occupiedNodes;
    
    // Initial routing (ignore congestion)
    for (int i = startNetIndex; i < endNetIndex && !timedOut; i++) {
        routeNet(nets[i], occupiedNodes, true);
    }
    
    // Iterative rip-up and re-route
    double historyFactor = HISTORY_FACTOR_INITIAL;
    bool hasCongestion = true;
    int iteration = 0;
    
    while (hasCongestion && iteration < MAX_ITERATIONS && !timedOut) {
        // Reset occupancy
        occupiedNodes.clear();
        
        // Reset usage counts
        for (auto& [nodeId, count] : nodeUsageCount) {
            count = 0;
        }
        
        // Route each net
        for (int i = startNetIndex; i < endNetIndex && !timedOut; i++) {
            // Reset routing for this net
            nets[i].routingResult.clear();
            nets[i].isRouted = false;
            
            // Route the net
            routeNet(nets[i], occupiedNodes);
        }
        
        // Check for congestion
        hasCongestion = false;
        
        for (const auto& [nodeId, count] : nodeUsageCount) {
            if (count > 1) {
                hasCongestion = true;
                // Update history cost
                nodeHistoryCost[nodeId] += historyFactor * (count - 1);
            }
        }
        
        // Increase history factor for next iteration
        historyFactor += HISTORY_FACTOR_INCREMENT;
        iteration++;
    }
}

// Parallel PathFinder
void parallelPathFinder(int numThreads) {
    // Sort nets by complexity (sinks count) for better load balancing
    std::sort(nets.begin(), nets.end(), [](const Net& a, const Net& b) {
        return a.sinkNodeIds.size() > b.sinkNodeIds.size();
    });
    
    // Create chunks for parallel processing
    int netsPerThread = (nets.size() + numThreads - 1) / numThreads;
    std::vector<std::thread> threads;
    
    // Launch threads
    for (int i = 0; i < numThreads; i++) {
        int startIdx = i * netsPerThread;
        int endIdx = std::min(startIdx + netsPerThread, static_cast<int>(nets.size()));
        if (startIdx < endIdx) {
            threads.emplace_back(runPathFinder, startIdx, endIdx);
        }
    }
    
    // Join threads
    for (auto& thread : threads) {
        thread.join();
    }
}

// Check if the routing solution has congestion
bool hasCongestion() {
    std::unordered_map<int, int> finalNodeUsage;
    
    for (const auto& net : nets) {
        if (!net.isRouted) continue;
        
        for (const auto& [parent, child] : net.routingResult) {
            finalNodeUsage[child]++;
            if (finalNodeUsage[child] > 1) {
                return true;
            }
        }
    }
    
    return false;
}

// Calculate total wirelength
int calculateWirelength() {
    int totalLength = 0;
    
    for (const auto& net : nets) {
        if (!net.isRouted) continue;
        totalLength += net.routingResult.size();
    }
    
    return totalLength;
}

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
        for (const auto& [parent, child] : net.routingResult) {
            outputFile << parent << " " << child << std::endl;
        }
        outputFile << std::endl;
    }
    
    outputFile.close();
    return true;
}

int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <device> <netlist> <result>" << std::endl;
        return 1;
    }
    
    std::string devicePath = argv[1];
    std::string netlistPath = argv[2];
    std::string resultPath = argv[3];
    
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Parse the device file
    std::cout << "Parsing device file..." << std::endl;
    if (!parseDevice(devicePath)) {
        return 1;
    }
    
    // Parse the netlist file
    std::cout << "Parsing netlist file..." << std::endl;
    if (!parseNetlist(netlistPath)) {
        return 1;
    }
    
    // Set up timeout thread
    int timeLimit = (nets.size() > 10000) ? 250 : 100; // Design 5 has more nets
    std::thread timeoutThread([timeLimit]() {
        std::this_thread::sleep_for(std::chrono::seconds(timeLimit - 5)); // Give 5 seconds for cleanup
        timedOut = true;
    });
    timeoutThread.detach();
    
    // Run PathFinder
    std::cout << "Routing " << nets.size() << " nets..." << std::endl;
    int numThreads = std::min(8, static_cast<int>(std::thread::hardware_concurrency()));
    parallelPathFinder(numThreads);
    
    // Check results
    int routedNets = 0;
    for (const auto& net : nets) {
        if (net.isRouted) {
            routedNets++;
        }
    }
    
    bool congested = hasCongestion();
    int wirelength = calculateWirelength();
    
    std::cout << "Routed " << routedNets << "/" << nets.size() << " nets" << std::endl;
    std::cout << "Congestion: " << (congested ? "Yes" : "No") << std::endl;
    std::cout << "Total wirelength: " << wirelength << std::endl;
    
    // Write routing result
    std::cout << "Writing routing result..." << std::endl;
    if (!writeRoutingResult(resultPath)) {
        return 1;
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    
    std::cout << "Total runtime: " << duration.count() / 1000.0 << " seconds" << std::endl;
    
    return 0;
} 