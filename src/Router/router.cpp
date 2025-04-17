#include "include/Router.hpp"
#include <algorithm>
#include <queue>
#include <thread>
#include <functional>
#include <iostream>
#include <set>

// Global variables implementation
std::atomic<bool> timedOut{false};
std::unordered_map<int, int> nodeUsageCount;
std::unordered_map<int, double> nodeHistoryCost;
std::mutex nodeUsageMutex;

// PathFinder algorithm parameters
const double BASE_COST = 1.0;
const double HISTORY_FACTOR_INITIAL = 0.5;
const double HISTORY_FACTOR_INCREMENT = 0.5;
const double PRESENT_CONGESTION_PENALTY = 2.0;
const int MAX_ITERATIONS = 50;

// PathFinder algorithm cost function
double calculateNodeCost(int nodeId, bool ignoreCongestion) {
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
                   std::unordered_map<int, int>& parentMap, bool ignoreCongestion) {
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
        auto current = pq.top();
        double currentCost = current.first;
        int currentNodeId = current.second;
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
bool routeNet(Net& net, std::unordered_map<int, bool>& occupiedNodes, bool ignoreCongestion) {
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
    for (const auto& edge : routingResult) {
        int parent = edge.first;
        int child = edge.second;
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
        for (auto& entry : nodeUsageCount) {
            entry.second = 0;
        }
        
        // Route each net
        for (int i = startNetIndex; i < endNetIndex && !timedOut; i++) {
            // Reset routing for this net
            nets[i].routingResult.clear();
            nets[i].isRouted = false;
            
            // Route the net
            routeNet(nets[i], occupiedNodes, false);
        }
        
        // Check for congestion
        hasCongestion = false;
        
        for (const auto& entry : nodeUsageCount) {
            int nodeId = entry.first;
            int count = entry.second;
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
        
        for (const auto& edge : net.routingResult) {
            int child = edge.second;
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

// Count routed nets
int countRoutedNets() {
    int count = 0;
    for (const auto& net : nets) {
        if (net.isRouted) {
            count++;
        }
    }
    return count;
} 