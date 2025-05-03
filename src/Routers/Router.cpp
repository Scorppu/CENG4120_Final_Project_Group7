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
void Router::resolveCongestion(const std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
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
    const int MAX_ITERATIONS = 50;
    
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
        
        // std::cout << "Iteration " << (iteration + 1) << ": " 
        //           << congestedNodeCount << " congested nodes found." << std::endl;
        
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
            std::vector<int> congestedRouteNodes = extractNodesFromRoute(route);
            
            // Calculate initial congestion score
            double score = calculateCongestionScore(netId, congestedRouteNodes);
            
            // Only consider nets that are actually involved in congestion
            if (score <= 0.0) {
                continue;
            }
            
            // Step 2: Apply net-size multiplier based on fanout
            // Find the corresponding Net to get fanout
            for (size_t j = 0; j < routingResults.size(); ++j) {
                if (routingResults[j].netId == netId) {
                    // Estimate fanout from number of edges in the route
                    int fanout = std::max(1, (int)congestedRouteNodes.size() - 1);
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
        std::vector<int> ripUpNetIds;
        ripUpNetIds.reserve(netsToRipUp);
        
        for (size_t i = 0; i < netsToRipUp && i < netScores.size(); ++i) {
            ripUpNetIds.push_back(netScores[i].first);
        }
        
        // std::cout << "Ripping up " << ripUpNetIds.size() << " nets with highest congestion scores." << std::endl;
        
        // Extract the actual Net objects from the original netlist that correspond to ripUpNetIds
        std::vector<Net> netsToReroute;
        for (int netId : ripUpNetIds) {
            // Increment reroute attempts
            rerouteAttempts[netId]++;
            
            // Find the corresponding Net object
            for (const auto& net : nets) {
                if (net.id == netId) {
                    netsToReroute.push_back(net);
                    break;
                }
            }
            
            // Remove this net from congestedNodes tracking
            for (auto& nodePair : congestedNodes) {
                nodePair.second.erase(netId);
                
                // If a node no longer has any nets, remove it from the map
                if (nodePair.second.empty()) {
                    // Can't erase while iterating, so mark for removal
                    // We'll handle cleanup below
                }
            }
            
            // Remove the net from routingResults
            for (auto it = routingResults.begin(); it != routingResults.end(); ) {
                if (it->netId == netId) {
                    it = routingResults.erase(it);
                } else {
                    ++it;
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
        
        // If we have nets to reroute, call routeAllNets with just those nets
        if (!netsToReroute.empty()) {
            // std::cout << "Rerouting " << netsToReroute.size() << " nets..." << std::endl;

            // Route the selected nets and append results to routingResults
            std::vector<NetRoute> newRoutes;
            for (auto& net : netsToReroute) {
                NetRoute result = routeSingleNet(net, edges, nodes);
                // std::cout << "netId: " << result.netId << " isRouted: " << result.isRouted << std::endl;
                routingResults.push_back(result);
            }
            
            // std::cout << "Rerouting complete." << std::endl;
        }
    }
    
    std::cout << "==== Congestion Resolution Complete ====\n\n";
}

// Validate if an MST edge has a viable path in the device graph
bool Router::validateMSTEdge(int sourceId, int targetId) {
    // Skip validation if pathfinder not initialized
    if (!pathfinder) return true;

    // Do a quick check using A* to see if a path exists
    std::vector<int> testPath;
    pathfinder->findPath(sourceId, targetId, testPath);
    
    // If path is valid (at least source and target nodes)
    return testPath.size() >= 2;
}

// Route a single net
NetRoute Router::routeSingleNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
    NetRoute netRoute;
    netRoute.netId = net.id;
    
    // First try routing through the MST edges
    bool allEdgesRouted = true;
    int successfulEdges = 0;
    
    // Keep track of the most recently routed path for backtracking
    std::vector<int> lastRoutedPath;
    
    for (size_t i = 0; i < net.mst_edges.size(); ++i) {
        int source = net.mst_edges[i].first;
        int target = net.mst_edges[i].second;
        
        // Find path between the two nodes of this MST edge
        std::vector<int> path;
        
        // If this is the first edge or we have no previous path, use regular routing
        if (lastRoutedPath.empty()) {
            pathfinder->findPath(source, target, path, congestedNodes, net.id);
        } else {
            // Otherwise use backtracking with the previous path
            pathfinder->findPath(source, target, path, congestedNodes, net.id, lastRoutedPath);
        }
        
        // Skip if path is empty or too short
        if (path.size() < 2) {
            // std::cout << "Path is empty or too short for MST edge between " << source << " and " << target << std::endl;
            allEdgesRouted = false;
            continue;
        }
        
        // Convert path to edges and add to NetRoute
        std::vector<std::pair<int, int>> newEdges;
        newEdges.reserve(path.size() - 1);
        
        for (size_t j = 0; j < path.size() - 1; ++j) {
            newEdges.push_back({path[j], path[j+1]});
        }
        netRoute.addEdges(newEdges);
        successfulEdges++;
        
        // Update the last routed path for next iteration
        lastRoutedPath = path;
    }
    
    // If all MST edges were successfully routed, we're done
    if (allEdgesRouted && successfulEdges > 0) {
        netRoute.isRouted = true;
        return netRoute;
    }
    
    // Set success flag based on whether any edges were routed
    netRoute.isRouted = (successfulEdges > 0);
    
    return netRoute;
}

double Router::calculateDistance(int fromId, int toId, const std::vector<Node>& nodes) {
    return std::sqrt(std::pow(nodes[fromId].beginX - nodes[toId].beginX, 2) + std::pow(nodes[fromId].beginY - nodes[toId].beginY, 2));
}

int count_in_box(int x1, int x2, int y1, int y2, std::map<int, std::vector<int>>& x_to_ys) {
    // Make sure x1 <= x2 and y1 <= y2
    if (x1 > x2) std::swap(x1, x2);
    if (y1 > y2) std::swap(y1, y2);
    
    auto it_low = x_to_ys.lower_bound(x1);
    auto it_high = x_to_ys.upper_bound(x2);
    int total = 0;

    for (auto it = it_low; it != it_high && it != x_to_ys.end(); ++it) {
        const auto& ys = it->second;
        
        // Check if the vector is empty
        if (ys.empty()) continue;
        
        // Need to check if vector is sorted
        bool is_sorted = true;
        for (size_t i = 1; i < ys.size(); ++i) {
            if (ys[i-1] > ys[i]) {
                is_sorted = false;
                break;
            }
        }
        
        if (is_sorted) {
            auto y_low = std::lower_bound(ys.begin(), ys.end(), y1);
            auto y_high = std::upper_bound(ys.begin(), ys.end(), y2);
            total += y_high - y_low;
        } else {
            // If not sorted, do a linear count
            for (const auto& y : ys) {
                if (y >= y1 && y <= y2) {
                    total++;
                }
            }
        }
    }
    
    return total;
}


// Define priority calculation function
double Router::computePriority(const Net& net, const std::vector<Node>& nodes, std::map<int, std::vector<int>>& x_to_ys) {
    int fanout = net.nodeIDs.size() - 1;
    double totalDist = 0.0;
    for (size_t j = 1; j < net.nodeIDs.size(); ++j) {
        totalDist += calculateDistance(net.nodeIDs[0], net.nodeIDs[j], nodes);
    }
    double avgDist = (fanout > 0) ? totalDist / fanout : 0.0;
    int pinsInBox = count_in_box(net.max_min_xy.first.first, net.max_min_xy.first.second, net.max_min_xy.second.first, net.max_min_xy.second.second, x_to_ys);
    // std::cout << "fanout: " << fanout << " pinsInBox: " << pinsInBox << " avgDist: " << avgDist << std::endl;
    return 0.4 * fanout + 0.5 / (1 + pinsInBox) + 0.1 / (1 + avgDist);
}

void Router::routeAllNets(std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes, std::map<int, std::vector<int>>& x_to_ys) {
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
        double congestionFactor = 0.1; // Default
        double initialCongestion = 0.1; // Default
        
        switch (designNumber) {
            case 1:
                congestionFactor = 4.0;
                initialCongestion = 5;
                break;
            case 2:
                congestionFactor = 1.0;
                initialCongestion = 1;
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
        priorities[i] = computePriority(nets[i], nodes, x_to_ys);
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
            // Only reserve if we actually have edges
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
            double initialCongestion = 0.0;
            // switch (designNumber) {
            //     case 1: initialCongestion = 5; break;
            //     case 2: initialCongestion = 5; break;
            //     case 3: initialCongestion = 5; break;
            //     case 4: initialCongestion = 5; break;
            //     case 5: initialCongestion = 10; break;
            //     default: break;
            // }
            
            pathfinder->updateCongestion(path, initialCongestion);
        }
    }
    
    // Report total time
    auto endTime = std::chrono::steady_clock::now();
    auto routingTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - routingStartTime).count();
    std::cout << "Routing " << (timeoutOccurred ? "(stopped by timeout)" : "of all nets") 
              << " took " << routingTime << "ms" << std::endl;


    // Resolve congestion
    // auto resolveStartTime = std::chrono::steady_clock::now();
    // resolveCongestion(nets, edges, nodes);
    // auto resolveEndTime = std::chrono::steady_clock::now();
    // auto resolveTime = std::chrono::duration_cast<std::chrono::milliseconds>(resolveEndTime - resolveStartTime).count();
    // std::cout << "Resolving congestion took " << resolveTime << "ms" << std::endl;
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


