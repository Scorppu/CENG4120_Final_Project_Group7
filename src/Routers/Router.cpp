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
#include "Router.hpp"

// Constructor
Router::Router() : pathfinder(nullptr), designNumber(1), timeoutSeconds(249), hasProgramStartTime(false) {}

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

// Set the design number to adjust parameters accordingly
void Router::setDesignNumber(int number) {
    designNumber = number;
}

// Set the global timeout in seconds
void Router::setTimeout(int seconds) {
    timeoutSeconds = seconds;
}

// Set the program start time
void Router::setProgramStartTime(std::chrono::time_point<std::chrono::steady_clock> startTime) {
    programStartTime = startTime;
    hasProgramStartTime = true;
}

// Extract nodes from a route's edges
std::vector<int> Router::extractNodesFromRoute(const NetRoute& route) const {
    std::vector<int> nodes;
    if (route.edges.empty()) {
        return nodes;
    }
    
    // Start with the first node of the first edge
    nodes.push_back(route.edges[0].first);
    
    // Add all destination nodes
    for (const auto& edge : route.edges) {
        nodes.push_back(edge.second);
    }
    
    return nodes;
}

// Calculate congestion score for a net
double Router::calculateCongestionScore(int netId, const std::vector<int>& path) const {
    double score = 0.0;
    
    for (int nodeId : path) {
        auto it = congestedNodes.find(nodeId);
        if (it != congestedNodes.end()) {
            int users = it->second.size();
            if (users > 1) {  // Only consider congested nodes (more than one user)
                score += 1.0 / (users - 1);
            }
        }
    }
    
    return score;
}

// Resolve congestions (Rip up and reroute)
void Router::resolveCongestion() {
    // Skip if no routing results
    if (routingResults.empty()) {
        return;
    }
    
    std::cout << "\n==== Starting Congestion Resolution ====\n";
    
    // First, clean up any empty entries in congestedNodes
    auto it = congestedNodes.begin();
    while (it != congestedNodes.end()) {
        if (it->second.empty()) {
            it = congestedNodes.erase(it);
        } else {
            ++it;
        }
    }
    
    // Maximum number of iterations for rip-up and reroute
    const int MAX_ITERATIONS = 5;
    
    // Timeout check parameters
    auto startTime = std::chrono::steady_clock::now();
    bool timeoutOccurred = false;
    
    // Continue resolving congestion for multiple iterations or until no congestion
    for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
        // Check for timeout
        if (hasProgramStartTime) {
            auto currentTime = std::chrono::steady_clock::now();
            auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(
                currentTime - programStartTime).count();
                
            // Leave buffer time for cleanup
            if (elapsedSeconds >= (timeoutSeconds - 2)) {
                std::cout << "Congestion resolution timeout reached (total program time: " 
                        << elapsedSeconds << " seconds). Stopping." << std::endl;
                timeoutOccurred = true;
                break;
            }
        }
        
        // Count congested nodes
        int congestedNodeCount = 0;
        for (const auto& pair : congestedNodes) {
            if (pair.second.size() > 1) {
                congestedNodeCount++;
            }
        }
        
        std::cout << "Iteration " << (iteration + 1) << ": " 
                  << congestedNodeCount << " congested nodes found." << std::endl;
        
        // If no congestion, we're done
        if (congestedNodeCount == 0) {
            std::cout << "No congestion found. Congestion resolution complete.\n";
            break;
        }
        
        // Step 1: Calculate congestion scores for all nets
        std::vector<std::pair<int, double>> netScores;
        netScores.reserve(routingResults.size());
        
        for (size_t i = 0; i < routingResults.size(); ++i) {
            const NetRoute& route = routingResults[i];
            int netId = route.netId;
            
            // Only consider successfully routed nets
            if (!route.isRouted || route.edges.empty()) {
                continue; // Skip unrouted nets
            }
            
            // Extract nodes from the route
            std::vector<int> nodes = extractNodesFromRoute(route);
            
            // Calculate initial congestion score
            double score = calculateCongestionScore(netId, nodes);
            
            // Only consider nets that are actually involved in congestion
            if (score <= 0.0) {
                continue;
            }
            
            // Step 2: Apply net-size multiplier based on fanout
            // Find the corresponding Net to get fanout
            for (size_t j = 0; j < routingResults.size(); ++j) {
                if (routingResults[j].netId == netId) {
                    // Estimate fanout from number of edges in the route
                    int fanout = std::max(1, (int)nodes.size() - 1);
                    score *= std::log(fanout + 1); // +1 to avoid log(0)
                    break;
                }
            }
            
            // Step 3: Deduct priority for nets rerouted >3 times
            int attempts = rerouteAttempts[netId];
            if (attempts > 3) {
                // Apply penalty: score *= 0.8^attempts
                double penalty = 1.0;
                for (int a = 0; a < (attempts - 3); ++a) {
                    penalty *= 0.8;
                }
                score *= penalty;
            }
            
            // Add to scores list
            netScores.push_back({netId, score});
        }
        
        // If no nets have congestion scores, we're done
        if (netScores.empty()) {
            std::cout << "No nets with congestion found. Congestion resolution complete.\n";
            break;
        }
        
        // Step 4: Sort nets by score (descending)
        std::sort(netScores.begin(), netScores.end(),
            [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                return a.second > b.second;
            }
        );
        
        // Step 5: Select top 20% of nets to rip up
        size_t netsToRipUp = std::max(size_t(1), netScores.size() / 5);
        std::vector<int> ripUpNets;
        ripUpNets.reserve(netsToRipUp);
        
        for (size_t i = 0; i < netsToRipUp && i < netScores.size(); ++i) {
            ripUpNets.push_back(netScores[i].first);
        }
        
        std::cout << "Ripping up " << ripUpNets.size() << " nets with highest congestion scores." << std::endl;
        
        // Clear congestion data for these nets
        for (int netId : ripUpNets) {
            // Increment reroute attempts
            rerouteAttempts[netId]++;
            
            // Find the corresponding route
            for (size_t i = 0; i < routingResults.size(); ++i) {
                if (routingResults[i].netId == netId) {
                    // Remove this net from congestedNodes tracking
                    std::vector<int> nodes = extractNodesFromRoute(routingResults[i]);
                    for (int nodeId : nodes) {
                        auto it = congestedNodes.find(nodeId);
                        if (it != congestedNodes.end()) {
                            it->second.erase(netId);
                            if (it->second.empty()) {
                                congestedNodes.erase(it);
                            }
                        }
                    }
                    
                    // Get net info to reroute
                    int sourceNodeId = 0;
                    std::vector<int> sinkNodeIds;
                    
                    if (!routingResults[i].edges.empty()) {
                        // Source is the first node of the first edge
                        sourceNodeId = routingResults[i].edges[0].first;
                        
                        // Collect unique destination nodes
                        std::unordered_set<int> uniqueSinks;
                        for (const auto& edge : routingResults[i].edges) {
                            uniqueSinks.insert(edge.second);
                        }
                        sinkNodeIds.assign(uniqueSinks.begin(), uniqueSinks.end());
                    }
                    
                    // Only proceed if we have valid source and sinks
                    if (sourceNodeId != 0 && !sinkNodeIds.empty()) {
                        // Clear the previous route
                        routingResults[i].edges.clear();
                        routingResults[i].isRouted = false;
                        
                        // Route from source to each sink
                        bool allSinksRouted = true;
                        for (int sinkId : sinkNodeIds) {
                            // Find path from source to this sink
                            std::vector<int> path;
                            pathfinder->findPath(sourceNodeId, sinkId, path, congestedNodes, netId);
                            
                            if (path.empty() || path.size() < 2) {
                                allSinksRouted = false;
                                continue; // Failed to find a path
                            }
                            
                            // Convert path to edges and add to NetRoute
                            std::vector<std::pair<int, int>> newEdges;
                            newEdges.reserve(path.size() - 1);
                            for (size_t j = 0; j < path.size() - 1; ++j) {
                                newEdges.push_back({path[j], path[j+1]});
                            }
                            routingResults[i].addEdges(newEdges);
                        }
                        
                        // Set success flag on NetRoute
                        routingResults[i].isRouted = allSinksRouted;
                        
                        // If the route wasn't successful, remove its entries from congestedNodes
                        if (!routingResults[i].isRouted) {
                            // Remove this net from all congested nodes
                            auto nodeIt = congestedNodes.begin();
                            while (nodeIt != congestedNodes.end()) {
                                nodeIt->second.erase(netId);
                                if (nodeIt->second.empty()) {
                                    nodeIt = congestedNodes.erase(nodeIt);
                                } else {
                                    ++nodeIt;
                                }
                            }
                        }
                        
                        // Update congestion map if successful
                        if (routingResults[i].isRouted && !routingResults[i].edges.empty()) {
                            std::vector<int> newNodes = extractNodesFromRoute(routingResults[i]);
                            if (!newNodes.empty()) {
                                pathfinder->updateCongestion(newNodes, 2.0);
                            }
                        }
                    }
                    
                    // Done processing this net
                    break;
                }
            }
        }
        
        // Clean up empty entries in congestedNodes
        auto cleanupIt = congestedNodes.begin();
        while (cleanupIt != congestedNodes.end()) {
            if (cleanupIt->second.empty()) {
                cleanupIt = congestedNodes.erase(cleanupIt);
            } else {
                ++cleanupIt;
            }
        }
    }
    
    std::cout << "==== Congestion Resolution Complete ====\n\n";
}

// Route a single net
NetRoute Router::routeSingleNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
    NetRoute netRoute;
    netRoute.netId = net.id;
    
    // Validate that all node IDs exist in the graph
    for (size_t j = 0; j < net.nodeIDs.size(); ++j) {
        if (existingNodeIds.find(net.nodeIDs[j]) == existingNodeIds.end()) {
            std::cerr << "Warning: Node ID " << net.nodeIDs[j] << " in net " << net.id 
                    << " does not exist in the graph. Skipping this node." << std::endl;
        }
    }
    
    // Clear the valid sink node IDs vector (reused across calls)
    validSinkNodeIds.clear();
    
    // First node is the source (driver), rest are sinks
    int sourceNodeId = net.nodeIDs[0];
    
    // Collect only valid sink node IDs
    for (size_t j = 1; j < net.nodeIDs.size(); ++j) {
        int sinkId = net.nodeIDs[j];
        if (existingNodeIds.find(sinkId) != existingNodeIds.end()) {
            validSinkNodeIds.push_back(sinkId);
        }
    }
    
    // Route from source to each sink
    std::unordered_set<int> remainingSinks(validSinkNodeIds.begin(), validSinkNodeIds.end());
    bool allSinksRouted = true;
    
    
    for (int sinkId : validSinkNodeIds) {
        // Find path from source to this sink
        std::vector<int> path;
        pathfinder->findPath(sourceNodeId, sinkId, path, congestedNodes, net.id);
        
        if (path.empty() || path.size() < 2) {
            // Failed to find a path to this sink
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
    
    // If not all sinks were successfully routed, remove this net from congestedNodes
    if (!netRoute.isRouted) {
        // Remove this net from all congested nodes
        for (auto& nodePair : congestedNodes) {
            nodePair.second.erase(net.id);
            
            // If a node no longer has any nets, remove it from the map
            if (nodePair.second.empty()) {
                // Can't erase while iterating, so mark for removal
                // We'll handle cleanup in resolveCongestion
            }
        }
        
        // Remove empty entries
        auto it = congestedNodes.begin();
        while (it != congestedNodes.end()) {
            if (it->second.empty()) {
                it = congestedNodes.erase(it);
            } else {
                ++it;
            }
        }
    }
    
    return netRoute;
}

double Router::calculateDistance(int fromId, int toId, const std::vector<Node>& nodes) {
    return std::sqrt(std::pow(nodes[fromId].beginX - nodes[toId].beginX, 2) + std::pow(nodes[fromId].beginY - nodes[toId].beginY, 2));
}

// Define priority calculation function
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
    
    // Record the routing start time
    auto routingStartTime = std::chrono::steady_clock::now();
    
    // Initialize the global pathfinder once
    if (!pathfinder) {
        auto startTime = std::chrono::steady_clock::now();
        
        // Use new with manual pointer assignment instead of make_unique for C++11
        pathfinder.reset(new AStarSearch(edges, nodes));
        
        // Set per-path timeout based on design size
        int pathTimeoutMs = 2000; // Default 2 seconds per path
        
        if (designNumber == 5) {
            pathTimeoutMs = 1000; // Reduce path timeout for design 5
        }
        
        pathfinder->setTimeout(pathTimeoutMs);
        
        // Set congestion penalty factor based on design
        double congestionFactor = 5.0; // Default
        double initialCongestion = 2.0; // Default
        
        switch (designNumber) {
            case 1:
                congestionFactor = 4.0;
                initialCongestion = 5;
                break;
            case 2:
                congestionFactor = 5.0;
                initialCongestion = 5;
                break;
            case 3:
                congestionFactor = 6.0;
                initialCongestion = 5;
                break;
            case 4:
                congestionFactor = 7.0;
                initialCongestion = 5;
                break;
            case 5:
                congestionFactor = 10.0;
                initialCongestion = 10;
                break;
            default:
                // Use defaults
                break;
        }
        
        // Initialize congestion penalty factor
        pathfinder->setCongestionPenaltyFactor(congestionFactor);
        
        auto endTime = std::chrono::steady_clock::now();
        auto initTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
        std::cout << "Initialized pathfinder in " << initTime << "ms" << std::endl;
        std::cout << "Using congestion factor: " << congestionFactor << " for design " << designNumber << std::endl;
    }
    
    // Reset congestion map at the start of routing
    pathfinder->resetCongestion();
    
    // Pre-allocate vector for sink IDs to further reduce allocations
    validSinkNodeIds.reserve(100);  // Reasonable size for most nets
    
    // Create indices for nets to sort them without modifying original array
    std::vector<size_t> netIndices(nets.size());
    for (size_t i = 0; i < nets.size(); ++i) {
        netIndices[i] = i;
    }
    
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
    bool timeoutOccurred = false;
    


    // Route nets in order of descending fanout
    for (size_t idx = 0; idx < netIndices.size(); ++idx) {
        // Check for global timeout using total program time if available
        if (hasProgramStartTime) {
            auto currentTime = std::chrono::steady_clock::now();
            auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(
                currentTime - programStartTime).count();
                
            // Leave at least 1 second for output and cleanup
            if (elapsedSeconds >= (timeoutSeconds - 1)) {
                std::cout << "Global timeout reached (total program time: " << elapsedSeconds 
                        << " seconds). Stopping routing." << std::endl;
                timeoutOccurred = true;
                break;
            }
        } else {
            // Fall back to routing time if program start time wasn't set
            auto currentTime = std::chrono::steady_clock::now();
            auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(
                currentTime - routingStartTime).count();
                
            if (elapsedSeconds >= timeoutSeconds) {
                std::cout << "Routing timeout reached after " << elapsedSeconds 
                        << " seconds. Stopping routing." << std::endl;
                timeoutOccurred = true;
                break;
            }
        }
        
        size_t i = netIndices[idx];
        
        // Report progress occasionally
        if (idx % progressStep == 0) {
            auto now = std::chrono::steady_clock::now();
            auto elapsedSeconds = hasProgramStartTime ? 
                std::chrono::duration_cast<std::chrono::seconds>(now - programStartTime).count() :
                std::chrono::duration_cast<std::chrono::seconds>(now - routingStartTime).count();
                
            std::cout << "Routing progress: " << (idx * 100 / nets.size()) << "% (Time: " 
                    << elapsedSeconds << "s)" << std::endl;
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
            
            // Update congestion along this path - use design-specific initial congestion
            double initialCongestion = 2.0;
            switch (designNumber) {
                case 1: initialCongestion = 5; break;
                case 2: initialCongestion = 5; break;
                case 3: initialCongestion = 5; break;
                case 4: initialCongestion = 5; break;
                case 5: initialCongestion = 10; break;
                default: break;
            }
            
            pathfinder->updateCongestion(path, initialCongestion);
        }
    }
    
    // Report total time
    auto endTime = std::chrono::steady_clock::now();
    auto routingTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - routingStartTime).count();
    std::cout << "Routing " << (timeoutOccurred ? "(stopped by timeout)" : "of all nets") 
              << " took " << routingTime << "ms" << std::endl;


    // Resolve congestion
    auto resolveStartTime = std::chrono::steady_clock::now();
    resolveCongestion();
    auto resolveEndTime = std::chrono::steady_clock::now();
    auto resolveTime = std::chrono::duration_cast<std::chrono::milliseconds>(resolveEndTime - resolveStartTime).count();
    std::cout << "Resolving congestion took " << resolveTime << "ms" << std::endl;
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
    std::cout << "==========================\n" << std::endl;
}


