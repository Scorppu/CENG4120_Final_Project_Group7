#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <memory>
#include <chrono>
#include "../Datastructure.hpp"
#include "../PathfindingAlgorithms/AStarSearch.hpp"
#include "Router.hpp"

// Constructor
Router::Router() : pathfinder(nullptr) {}

// Destructor that properly cleans up resources
Router::~Router() {
    clearAll();
}

// Get routing results
const std::vector<NetRoute>& Router::getRoutingResults() const {
    return routingResults;
}

// Clear routing results
void Router::clearRoutingResults() {
    routingResults.clear();
}

// Clear the pathfinder to free its memory
void Router::clearPathfinder() {
    pathfinder.reset(nullptr);
}

// Clear all resources to free memory
void Router::clearAll() {
    clearRoutingResults();
    clearPathfinder();
}

// Set verbosity level
void Router::setVerbose(bool verbose) {
    verboseLogging = verbose;
}

// Resolve congestions (Rip up and reroute)
void Router::resolveCongestion() {
    // Implement congestion resolution logic here
}

// Route a single net using A* Search
NetRoute Router::routeSingleNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
    // Create a NetRoute object to store the results
    NetRoute netRoute(net.id, net.name);
    
    // Check if we have enough nodes to route
    if (net.nodeIDs.size() < 2) {
        if (verboseLogging) {
            std::cerr << "Net " << net.id << " (" << net.name << ") has fewer than 2 nodes, skipping" << std::endl;
        }
        return netRoute; // Return empty NetRoute with isRouted = false
    }

    // In the netlist format from design1.netlist:
    // First node is the source, all others are sinks
    int sourceNodeId = net.nodeIDs[0];
    std::vector<int> sinkNodeIds(net.nodeIDs.begin() + 1, net.nodeIDs.end());
    
    // Verify source node exists in the graph
    if (existingNodeIds.find(sourceNodeId) == existingNodeIds.end()) {
        if (verboseLogging) {
            std::cerr << "Source node " << sourceNodeId << " for net " << net.id 
                    << " (" << net.name << ") does not exist in the graph, skipping" << std::endl;
        }
        return netRoute; // Return empty NetRoute with isRouted = false
    }
    
    // Filter sinks to only include those that exist in the graph
    validSinkNodeIds.clear();
    validSinkNodeIds.reserve(sinkNodeIds.size());  // Pre-allocate for efficiency
    
    for (int sinkId : sinkNodeIds) {
        if (existingNodeIds.find(sinkId) != existingNodeIds.end()) {
            validSinkNodeIds.push_back(sinkId);
        } else if (verboseLogging) {
            std::cerr << "Sink node " << sinkId << " for net " << net.id 
                    << " does not exist in the graph, skipping this sink" << std::endl;
        }
    }
    
    if (validSinkNodeIds.empty()) {
        if (verboseLogging) {
            std::cerr << "No valid sink nodes for net " << net.id 
                    << " (" << net.name << "), skipping" << std::endl;
        }
        return netRoute; // Return empty NetRoute with isRouted = false
    }
    
    // Route from source to each sink
    std::unordered_set<int> remainingSinks(validSinkNodeIds.begin(), validSinkNodeIds.end());
    bool allSinksRouted = true;
    
    // Minimal debug information
    if (verboseLogging) {
        std::cout << "Routing net " << net.id << " with " << validSinkNodeIds.size() << " sinks" << std::endl;
    }
    
    for (int sinkId : validSinkNodeIds) {
        // Find path from source to this sink
        std::vector<int> path;
        pathfinder->findPath(sourceNodeId, sinkId, path);
        
        if (path.empty() || path.size() < 2) {
            // Failed to find a path to this sink
            if (verboseLogging) {
                std::cerr << "Failed to find path from " << sourceNodeId << " to " << sinkId << std::endl;
            }
            allSinksRouted = false;
            continue;
        }
        
        // Convert path to edges and add to NetRoute
        std::vector<std::pair<int, int>> newEdges;
        newEdges.reserve(path.size() - 1);
        for (size_t i = 0; i < path.size() - 1; ++i) {
            newEdges.push_back({path[i], path[i+1]});
        }
        netRoute.addEdges(newEdges);
        
        // Mark this sink as routed
        remainingSinks.erase(sinkId);
    }
                
    // Set success flag on NetRoute
    netRoute.isRouted = remainingSinks.empty();
    
    return netRoute;
}

// Helper method to calculate cost between nodes (includes congestion awareness)
double Router::calculateCost(int fromId, int toId) {
    // Base cost of traversing between nodes
    const double baseCost = 1.0;
    
    // Add congestion awareness if pathfinder is initialized
    if (pathfinder) {
        double congestion = pathfinder->getCongestion(toId);
        return baseCost + (congestion * 0.5); // Apply 50% weight to current congestion
    }
    
    return baseCost;
}

double Router::calculateDistance(int fromId, int toId, const std::vector<Node>& nodes) {
    return std::sqrt(std::pow(nodes[fromId].beginX - nodes[toId].beginX, 2) + std::pow(nodes[fromId].beginY - nodes[toId].beginY, 2));
}

// 1. Define priority calculation function
double Router::computePriority(const Net& net, const std::vector<Node>& nodes) {
    int fanout = net.nodeIDs.size() - 1;
    double totalDist = 0.0;
    for (size_t j = 1; j < net.nodeIDs.size(); ++j) {
        totalDist += calculateDistance(net.nodeIDs[0], net.nodeIDs[j], nodes);
    }
    double avgDist = (fanout > 0) ? totalDist / fanout : 0.0;
    
    // Custom formula: fanout * avgDist (prioritize high fanout and long distances)
    return fanout * avgDist;
}

void Router::routeAllNets(std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
    // Clear previous results
    clearRoutingResults();
    
    // Reserve space for results
    routingResults.reserve(nets.size());
    
    // Pre-build set of existing node IDs (do this once)
    existingNodeIds.clear();
    existingNodeIds.reserve(nodes.size());
    for (const auto& node : nodes) {
        existingNodeIds.insert(node.id);
    }
    
    // Initialize the global pathfinder once
    if (!pathfinder) {
        auto startTime = std::chrono::high_resolution_clock::now();
        
        pathfinder = std::make_unique<AStarSearch>(edges, nodes);
        pathfinder->setTimeout(2000); // 2 second timeout per path
        
        // Set cost function once for all paths
        pathfinder->setCostFunction([this](int fromId, int toId) {
            return this->calculateCost(fromId, toId);
        });
        
        // Initialize congestion penalty factor
        pathfinder->setCongestionPenaltyFactor(5.0);
        
        auto endTime = std::chrono::high_resolution_clock::now();
        auto initTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
        std::cout << "Initialized pathfinder in " << initTime << "ms" << std::endl;
    }
    
    // Reset congestion map at the start of routing
    pathfinder->resetCongestion();
    
    // Start timing the full routing process
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Pre-allocate vector for sink IDs to further reduce allocations
    validSinkNodeIds.reserve(100);  // Reasonable size for most nets
    
    // Create indices for nets to sort them without modifying original array
    std::vector<size_t> netIndices(nets.size());
    for (size_t i = 0; i < nets.size(); ++i) {
        netIndices[i] = i;
    }
    
    // // Sort nets by fanout (number of sinks) in descending order
    // std::sort(netIndices.begin(), netIndices.end(), [&nets](size_t a, size_t b) {
    //     // nodeIDs[0] is the source, the rest are sinks, so size()-1 is the fanout
    //     return (nets[a].nodeIDs.size() - 1) > (nets[b].nodeIDs.size() - 1);
    // });


    // 2. Precompute priorities for all nets
    std::vector<double> priorities(nets.size());
    for (size_t i = 0; i < nets.size(); ++i) {
        priorities[i] = computePriority(nets[i], nodes);
    }

    // 3. Sort indices based on computed priorities (descending order)
    std::sort(netIndices.begin(), netIndices.end(), 
        [&priorities](size_t a, size_t b) {
            return priorities[a] > priorities[b]; 
        }
    );
    
    // Show progress every 10% of nets
    size_t progressStep = std::max(size_t(1), nets.size() / 10);
    
    // Route nets in order of descending fanout
    for (size_t idx = 0; idx < netIndices.size(); ++idx) {
        size_t i = netIndices[idx];
        
        // Report progress occasionally
        if (idx % progressStep == 0) {
            std::cout << "Routing progress: " << (idx * 100 / nets.size()) << "%" << std::endl;
            
            if (verboseLogging) {
                std::cout << "  Current net: " << nets[i].id 
                        << " (" << nets[i].name << ") with fanout " 
                        << (nets[i].nodeIDs.size() - 1) << std::endl;
            }
        }
        
        // Route the net and store the result
        NetRoute result = routeSingleNet(nets[i], edges, nodes);
        
        // Always add the result to routingResults, even if it has no edges or wasn't routed
        routingResults.push_back(result);
        
        // If the route was successful, update congestion map
        if (result.isRouted && !result.edges.empty()) {
            // Extract the full path from edges
            std::vector<int> path;
            path.reserve(result.edges.size() + 1);
            
            // Add the first node of the first edge
            if (!result.edges.empty()) {
                path.push_back(result.edges[0].first);
            }
            
            // Add all destination nodes
            for (const auto& edge : result.edges) {
                path.push_back(edge.second);
            }
            
            // Update congestion along this path
            pathfinder->updateCongestion(path);
        }
    }
    
    // Report total time
    auto endTime = std::chrono::high_resolution_clock::now();
    auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
    std::cout << "Complete routing of all nets took " << totalTime << "ms" << std::endl;
    
    // Report high-fanout net performance
    if (verboseLogging) {
        std::cout << "\nHigh-fanout net routing performance:" << std::endl;
        const size_t TOP_NETS = std::min(size_t(10), nets.size());
        
        for (size_t i = 0; i < TOP_NETS; ++i) {
            size_t netIdx = netIndices[i];
            bool found = false;
            
            // Find the corresponding routing result
            for (const auto& route : routingResults) {
                if (route.netId == nets[netIdx].id) {
                    std::cout << "  Net " << nets[netIdx].id << " (" << nets[netIdx].name 
                            << ") with fanout " << (nets[netIdx].nodeIDs.size() - 1)
                            << ": " << (route.isRouted ? "SUCCESS" : "FAILED") << std::endl;
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                std::cout << "  Net " << nets[netIdx].id << " (" << nets[netIdx].name 
                        << ") with fanout " << (nets[netIdx].nodeIDs.size() - 1)
                        << ": NOT ROUTED" << std::endl;
            }
        }
    }
}

// Print routing results
void Router::printRoutingResults() const {
    std::cout << "\n==== Routing Results ====\n";
    std::cout << "Total nets routed: " << routingResults.size() << std::endl;
    
    int successfullyRouted = 0;
    for (const auto& route : routingResults) {
        if (route.isRouted) {
            successfullyRouted++;
        }
    }
    
    std::cout << "Successfully routed: " << successfullyRouted << std::endl;
    std::cout << "Success rate: " << (routingResults.empty() ? 0 : (successfullyRouted * 100.0 / routingResults.size())) << "%" << std::endl;
    
    // Only print detailed per-net results if verbose logging is enabled
    if (verboseLogging) {
        for (const auto& route : routingResults) {
            std::cout << "Net " << route.netId << ": " 
                    << (route.isRouted ? "Successfully routed" : "Failed to route")
                    << " (" << route.edges.size() << " connections)" << std::endl;
        }
    }
    
    std::cout << "==========================\n" << std::endl;
}

// Implementation of SteinerTreeRouter

SteinerTreeRouter::SteinerTreeRouter() : Router() {}

void SteinerTreeRouter::routeAllNets(std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
    // Implement Steiner Tree routing algorithm
    // This is a placeholder implementation
    std::cout << "Steiner Tree routing not implemented yet." << std::endl;
}

NetRoute SteinerTreeRouter::routeSingleNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
    // Implement Steiner Tree routing algorithm for a single net
    // This is a placeholder implementation
    std::cout << "Steiner Tree routing for a single net not implemented yet." << std::endl;
    return NetRoute(net.id, net.name); // Return empty route
}

