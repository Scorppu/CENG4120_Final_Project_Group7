/**
 * Router Implementation
 * 
 * Enhanced with advanced congestion resolution techniques:
 * 
 * 1. Two-stage Rip-up and Reroute framework:
 *    - Local phase: Focus on specific congestion hotspots
 *    - Global phase: More aggressive approach for stubborn congestion
 * 
 * 2. Specialized handling for difficult nets:
 *    - Dedicated phase to retry failed nets with relaxed constraints
 *    - Reduced congestion penalties and increased timeouts
 *    - Prioritization based on net complexity
 * 
 * 3. Adaptive pathfinding parameters:
 *    - Dynamic iteration limits based on estimated path complexity
 *    - Capped congestion penalties to avoid creating barriers
 *    - Partial success handling for multi-sink nets
 * 
 * 4. History-based congestion model:
 *    - Tracks repeatedly congested nodes
 *    - Applies progressive penalties to critical hotspots
 *    - Helps router better avoid problematic regions
 */
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
#include <cstring>
#include <ctime>
#include <cstdlib>
#include <cassert>
#include <stdexcept>
#include <fstream>
#include <functional>
#include <thread>
#include <future>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <limits>
#include <numeric>
#include <iomanip>
#include <random>
#include <sstream>
#include <tuple>
#include <utility>
#include <map>
#include "../PathfindingAlgorithms/AStarSearch.hpp"
#include "../DataStructure.hpp"
#include "Router.hpp"
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
    
    // Sum congestion along the path
    for (int nodeId : path) {
        auto it = congestedNodes.find(nodeId);
        if (it != congestedNodes.end() && it->second.size() > NODE_CAPACITY) {
            score += it->second.size() - NODE_CAPACITY;
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
    
    // Implement a multi-stage RRR approach
    
    // Stage 1: Local rip-up and reroute - focused on specific congestion hotspots
    std::cout << "Starting local congestion resolution..." << std::endl;
    localRipUpAndReroute(nets, edges, nodes);
    
    // Stage 2: Global rip-up and reroute - handle remaining congestion with more aggressive approach
    std::cout << "Starting global congestion resolution..." << std::endl;
    globalRipUpAndReroute(nets, edges, nodes);
    
    // Stage 3: Handle difficult nets with specialized approach
    std::cout << "Starting difficult nets resolution..." << std::endl;
    handleDifficultNets(nets, edges, nodes);
    
    // Stage 4: Final pass to eliminate any remaining congestion
    std::cout << "Starting final congestion elimination..." << std::endl;
    eliminateRemainingCongestion(nets, edges, nodes);
    
    std::cout << "==== Congestion Resolution Complete ====\n\n";
}

// Local stage: Focus on localized congestion with minimal disruption
void Router::localRipUpAndReroute(const std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
    // Maximum iterations for local phase
    const int MAX_LOCAL_ITERATIONS = 30;
    
    // Timeout check parameters
    auto startTime = std::chrono::steady_clock::now();
    bool timeoutOccurred = false;
    
    // Track congestion history for the first time
    static int lastCongestedNodeCount = INT_MAX;
    
    // Continue resolving congestion for multiple iterations or until no congestion
    for (int iteration = 0; iteration < MAX_LOCAL_ITERATIONS; ++iteration) {
        // Check for timeout
        if (hasProgramStartTime) {
            auto currentTime = std::chrono::steady_clock::now();
            auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(
                currentTime - programStartTime).count();
                
            // Leave buffer time for cleanup
            if (elapsedSeconds >= (timeoutSeconds - 5)) {
                std::cout << "Congestion resolution timeout reached. Stopping local phase." << std::endl;
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
        
        std::cout << "Local iteration " << (iteration + 1) << ": " 
                  << congestedNodeCount << " congested nodes" << std::endl;
        
        // If no congestion, we're done
        if (congestedNodeCount == 0) {
            std::cout << "No congestion found in local phase. Complete." << std::endl;
            return; // Skip global phase if no congestion
        }
        
        // Check if we're making progress
        if (congestedNodeCount >= lastCongestedNodeCount && iteration > 5) {
            // Adapt routing effort based on progress
            adjustRoutingEffort(iteration, congestedNodeCount);
            std::cout << "Adjusting routing effort for local phase..." << std::endl;
        }
        lastCongestedNodeCount = congestedNodeCount;
        
        // Find bounding boxes of congested regions (local approach)
        std::vector<std::tuple<int, int, int, int>> congestedBoxes;
        findCongestedRegions(nodes, congestedBoxes);
        
        // Process each congested region separately
        std::vector<Net> allNetsToReroute;
        
        for (const auto& box : congestedBoxes) {
            int minX = std::get<0>(box);
            int maxX = std::get<1>(box);
            int minY = std::get<2>(box);
            int maxY = std::get<3>(box);
            
            // Find nets that go through this congested box
            std::vector<int> netsInBox;
            for (const auto& route : routingResults) {
                // Skip unrouted nets
                if (!route.isRouted || route.edges.empty()) continue;
                
                // Check if route goes through the congested box
                std::vector<int> routeNodes = extractNodesFromRoute(route);
                
                bool intersectsBox = false;
                for (int nodeId : routeNodes) {
                    if (nodes[nodeId].beginX >= minX && nodes[nodeId].beginX <= maxX &&
                        nodes[nodeId].beginY >= minY && nodes[nodeId].beginY <= maxY) {
                        intersectsBox = true;
                        netsInBox.push_back(route.netId);
                        break;
                    }
                }
            }
            
            // Calculate scores for nets in this box
            std::vector<std::pair<int, double>> netScores;
            for (int netId : netsInBox) {
                // Find corresponding route
                NetRoute route;
                for (const auto& r : routingResults) {
                    if (r.netId == netId) {
                        route = r;
                        break;
                    }
                }
                
                // Skip if not found
                if (route.netId != netId) continue;
                
                // Calculate score - using base congestion and net characteristics
                std::vector<int> routeNodes = extractNodesFromRoute(route);
                double score = calculateCongestionScore(netId, routeNodes);
                
                // Apply adjustments for history and net characteristics
                score *= historyPenaltyFactor(netId);
                
                // Only consider nets that are actually involved in congestion
                if (score > 0.0) {
                    netScores.push_back({netId, score});
                }
            }
            
            // Sort nets by score
            std::sort(netScores.begin(), netScores.end(),
                [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                    return a.second > b.second;
                }
            );
            
            // Select up to 30% of nets in this box to reroute
            int netsToSelect = std::max(1, (int)(netScores.size() * 0.3));
            
            for (int i = 0; i < netsToSelect && i < (int)netScores.size(); i++) {
                int netId = netScores[i].first;
                
                // Find the corresponding Net object
                for (const auto& net : nets) {
                    if (net.id == netId) {
                        allNetsToReroute.push_back(net);
                        
                        // Increment reroute attempts
                        rerouteAttempts[netId]++;
                        
                        // Remove this net from congestedNodes tracking
                        for (auto& nodePair : congestedNodes) {
                            nodePair.second.erase(netId);
                        }
                        
                        // Remove the net from routingResults
                        for (auto it = routingResults.begin(); it != routingResults.end(); ) {
                            if (it->netId == netId) {
                                it = routingResults.erase(it);
                            } else {
                                ++it;
                            }
                        }
                        
                        break;
                    }
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
        
        // Route all selected nets using pattern routing first for speed
        if (!allNetsToReroute.empty()) {
            std::cout << "Rerouting " << allNetsToReroute.size() << " nets in local phase..." << std::endl;
            
            // Use lower congestion penalty for local phase - focus on wirelength
            double originalFactor = pathfinder->getCongestionPenaltyFactor();
            double localFactor = originalFactor * 1.5; // Mild increase
            pathfinder->setCongestionPenaltyFactor(localFactor);
            
            // Route the selected nets with pattern routing first
            for (auto& net : allNetsToReroute) {
                NetRoute result = routeSingleNet(net, edges, nodes);
                routingResults.push_back(result);
                
                // Update congestion history for this net's path
                if (result.isRouted && !result.edges.empty()) {
                    std::vector<int> path = extractNodesFromRoute(result);
                    for (int nodeId : path) {
                        nodeHistoryCongestion[nodeId]++;
                    }
                }
            }
            
            // Restore original factor
            pathfinder->setCongestionPenaltyFactor(originalFactor);
        }
        
        // Update congestion history for all nodes
        updateCongestionHistory();
    }
}

// Global stage: Handle remaining congestion with more aggressive approach
void Router::globalRipUpAndReroute(const std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
    // Maximum iterations for global phase
    const int MAX_GLOBAL_ITERATIONS = 100;
    
    // Timeout check parameters
    auto startTime = std::chrono::steady_clock::now();
    bool timeoutOccurred = false;
    
    // Continue resolving congestion for multiple iterations or until no congestion
    for (int iteration = 0; iteration < MAX_GLOBAL_ITERATIONS; ++iteration) {
        // Check for timeout
        if (hasProgramStartTime) {
            auto currentTime = std::chrono::steady_clock::now();
            auto elapsedSeconds = std::chrono::duration_cast<std::chrono::seconds>(
                currentTime - programStartTime).count();
                
            // Leave buffer time for cleanup
            if (elapsedSeconds >= (timeoutSeconds - 2)) {
                std::cout << "Congestion resolution timeout reached. Stopping global phase." << std::endl;
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
        
        std::cout << "Global iteration " << (iteration + 1) << ": " 
                  << congestedNodeCount << " congested nodes" << std::endl;
        
        // If no congestion, we're done
        if (congestedNodeCount == 0) {
            std::cout << "No congestion found in global phase. Complete." << std::endl;
            break;
        }
        
        // Calculate scores for all nets
        std::vector<std::pair<int, double>> netScores;
        netScores.reserve(routingResults.size());
        
        for (const auto& route : routingResults) {
            // Skip unrouted nets
            if (!route.isRouted || route.edges.empty()) continue;
            
            int netId = route.netId;
            std::vector<int> routeNodes = extractNodesFromRoute(route);
            double score = calculateCongestionScore(netId, routeNodes);
            
            // Only consider nets that are actually involved in congestion
            if (score <= 0.0) continue;
            
            // Apply net characteristics (fanout, etc.)
            int fanout = 0;
            for (const auto& net : nets) {
                if (net.id == netId) {
                    fanout = net.nodeIDs.size() - 1;
                    break;
                }
            }
            
            // Higher score for high-fanout nets in later iterations
            if (iteration > 15) {
                score *= std::log(fanout + 1);
            }
            
            // Apply iteration-dependent adjustments 
            if (iteration > 20) {
                // For later iterations, prioritize nets that haven't been rerouted much
                if (rerouteAttempts[netId] < 2) {
                    score *= 1.5; // Boost score for "fresh" nets
                }
            } else {
                // For earlier iterations, penalize frequently rerouted nets
                score *= historyPenaltyFactor(netId);
            }
            
            // Add randomization to escape local minima
            if (iteration > 10) {
                score *= (0.9 + (rand() % 20) / 100.0); // +/- 10% random variation
            }
            
            netScores.push_back({netId, score});
        }
        
        // If no nets have congestion scores, we're done
        if (netScores.empty()) {
            std::cout << "No nets with congestion found. Complete." << std::endl;
            break;
        }
        
        // Sort nets by score (descending)
        std::sort(netScores.begin(), netScores.end(),
            [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                return a.second > b.second;
            }
        );
        
        // Select nets to rip up - adaptive percentage based on phase
        float ripupPercentage;
        if (iteration < 10) {
            ripupPercentage = 0.1; // Start conservative
        } else if (iteration < 20) {
            ripupPercentage = 0.2; // Mid phase - be more aggressive
        } else {
            // Late phase - adjust based on remaining congestion
            ripupPercentage = 0.1 + (0.4 * (float)congestedNodeCount / routingResults.size());
            ripupPercentage = std::min(ripupPercentage, 0.5f); // Cap at 50%
        }
        
        size_t netsToRipUp = std::max(size_t(1), (size_t)(netScores.size() * ripupPercentage));
        std::vector<Net> netsToReroute;
        
        // Extract nets to reroute
        for (size_t i = 0; i < netsToRipUp && i < netScores.size(); ++i) {
            int netId = netScores[i].first;
            
            // Skip nets rerouted too many times
            if (rerouteAttempts[netId] > 7) {
                continue;
            }
            
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
        
        // Reroute the selected nets
        if (!netsToReroute.empty()) {
            std::cout << "Rerouting " << netsToReroute.size() << " nets in global phase..." << std::endl;
            
            // Use higher congestion penalty for global phase
            double originalFactor = pathfinder->getCongestionPenaltyFactor();
            
            // Higher factor that increases with iteration number
            double globalFactor = originalFactor * (1.0 + (std::min(iteration, 20) * 0.05));
            pathfinder->setCongestionPenaltyFactor(globalFactor);
            
            // Dynamic ordering based on iteration phase
            std::vector<Net> orderedNets = orderNetsForRerouting(netsToReroute, iteration, congestedNodeCount);
            
            // Route the selected nets
            for (auto& net : orderedNets) {
                NetRoute result = routeSingleNet(net, edges, nodes);
                routingResults.push_back(result);
            }
            
            // Restore original factor
            pathfinder->setCongestionPenaltyFactor(originalFactor);
        }
        
        // Update congestion history
        updateCongestionHistory();
    }
}

// Find congested regions by grouping nearby congested nodes into bounding boxes
void Router::findCongestedRegions(const std::vector<Node>& nodes, std::vector<std::tuple<int, int, int, int>>& congestedBoxes) {
    const int PROXIMITY_THRESHOLD = 50; // Nodes closer than this are considered part of the same region
    
    // Collect all congested nodes
    std::vector<int> congestedNodeIds;
    for (const auto& pair : congestedNodes) {
        if (pair.second.size() > 1) {
            congestedNodeIds.push_back(pair.first);
        }
    }
    
    // Simple clustering algorithm to group nearby congested nodes
    std::vector<std::vector<int>> clusters;
    std::vector<bool> visited(congestedNodeIds.size(), false);
    
    for (size_t i = 0; i < congestedNodeIds.size(); i++) {
        if (visited[i]) continue;
        
        // Start a new cluster
        std::vector<int> cluster;
        cluster.push_back(congestedNodeIds[i]);
        visited[i] = true;
        
        // Find all nodes close to this one
        for (size_t j = 0; j < congestedNodeIds.size(); j++) {
            if (visited[j]) continue;
            
            int node1 = congestedNodeIds[i];
            int node2 = congestedNodeIds[j];
            
            double distance = std::sqrt(
                std::pow(nodes[node1].beginX - nodes[node2].beginX, 2) +
                std::pow(nodes[node1].beginY - nodes[node2].beginY, 2)
            );
            
            if (distance < PROXIMITY_THRESHOLD) {
                cluster.push_back(node2);
                visited[j] = true;
            }
        }
        
        if (!cluster.empty()) {
            clusters.push_back(cluster);
        }
    }
    
    // Create bounding boxes for each cluster
    for (const auto& cluster : clusters) {
        int minX = INT_MAX, maxX = INT_MIN, minY = INT_MAX, maxY = INT_MIN;
        
        for (int nodeId : cluster) {
            minX = std::min(minX, nodes[nodeId].beginX);
            maxX = std::max(maxX, nodes[nodeId].beginX);
            minY = std::min(minY, nodes[nodeId].beginY);
            maxY = std::max(maxY, nodes[nodeId].beginY);
        }
        
        // Expand the box slightly
        int expansion = 20;
        minX -= expansion;
        maxX += expansion;
        minY -= expansion;
        maxY += expansion;
        
        congestedBoxes.push_back(std::make_tuple(minX, maxX, minY, maxY));
    }
}

// Update congestion history after each iteration
void Router::updateCongestionHistory() {
    for (const auto& pair : congestedNodes) {
        if (pair.second.size() > 1) {
            nodeHistoryCongestion[pair.first]++;
        }
    }
}

// Get node congestion cost based on history
double Router::getNodeCongestionCost(int nodeId) {
    double baseCost = 1.0;
    auto it = nodeHistoryCongestion.find(nodeId);
    if (it != nodeHistoryCongestion.end()) {
        // Cap the exponential penalty to avoid creating impassable barriers
        double historyPenalty = std::min(10.0, pow(1.25, it->second));
        baseCost *= historyPenalty;
    }
    return baseCost;
}

// Apply penalty factor based on reroute history
double Router::historyPenaltyFactor(int netId) {
    int attempts = rerouteAttempts[netId];
    if (attempts > 3) {
        // Apply penalty: score *= 0.8^(attempts-3)
        double penalty = 1.0;
        for (int a = 0; a < (attempts - 3); ++a) {
            penalty *= 0.8;
        }
        return penalty;
    } else if (attempts > 0) {
        // Mild penalty for nets rerouted 1-3 times
        return 1.0 - (attempts * 0.1);
    }
    return 1.0;
}

// Adjust routing effort based on iteration and congestion
void Router::adjustRoutingEffort(int iteration, int congestedNodeCount) {
    // This would need to be implemented in AStarSearch to actually work
    // Since we don't have direct access to modify AStarSearch, this is just a stub
    
    // Early iterations - use fast, approximate methods
    if (iteration < 10) {
        // pathfinder->setAccuracyLevel(LOW);
        pathfinder->setTimeout(500); // Short timeout
    }
    // Middle iterations - balance speed and quality
    else if (iteration < 30) {
        // pathfinder->setAccuracyLevel(MEDIUM);
        pathfinder->setTimeout(1000);
    }
    // Final iterations - use high-quality methods for remaining issues
    else {
        // pathfinder->setAccuracyLevel(HIGH);
        pathfinder->setTimeout(2000);
    }
}

// Dynamically order nets for rerouting
std::vector<Net> Router::orderNetsForRerouting(const std::vector<Net>& nets, int iteration, int congestedNodeCount) {
    // For now, a simple pass-through
    return nets;
    
    // This could be enhanced to:
    // - For early iterations, prioritize high-fanout nets
    // - For later iterations, prioritize nets in critical regions
    // - For final iterations, consider nets that haven't been rerouted yet
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
        
        // Check if the path would cause overcongestion and skip if so
        if (pathfinder->wouldPathCauseCongestion(path)) {
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
    
    // Only for logging - don't change isRouted flag
    if (!netRoute.isRouted && netRoute.edges.size() > 0 && validSinkNodeIds.size() > 1) {
        // Compute success percentage
        double successRate = (validSinkNodeIds.size() - remainingSinks.size()) / 
                            (double)validSinkNodeIds.size();
        
        if (successRate > 0.6) {
            std::cout << "Net " << net.id << " partially routed (" 
                    << (int)(successRate * 100) << "% of sinks) but marked as FAILED" << std::endl;
        }
    }
    
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
    //double totalDist = 0.0;
    //for (size_t j = 1; j < net.nodeIDs.size(); ++j) {
    //    totalDist += calculateDistance(net.nodeIDs[0], net.nodeIDs[j], nodes);
    //}
    //double avgDist = (fanout > 0) ? totalDist / fanout : 0.0;
    int pinsInBox = count_in_box(net.max_min_xy.first.first, net.max_min_xy.first.second, net.max_min_xy.second.first, net.max_min_xy.second.second, x_to_ys);
    // std::cout << "fanout: " << fanout << " pinsInBox: " << pinsInBox << " avgDist: " << avgDist << std::endl;
    return 1 / (1 + pinsInBox);
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
        
        pathfinder->setTimeout(pathTimeoutMs);
        
        // Set congestion penalty factor based on design
        double congestionFactor = 2; // Default
        double initialCongestion = 0; // Default

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
    auto resolveStartTime = std::chrono::steady_clock::now();
    resolveCongestion(nets, edges, nodes);
    auto resolveEndTime = std::chrono::steady_clock::now();
    auto resolveTime = std::chrono::duration_cast<std::chrono::milliseconds>(resolveEndTime - resolveStartTime).count();
    std::cout << "Resolving congestion took " << resolveTime << "ms" << std::endl;
    
    std::cout << "Total routing time: " << (routingTime + resolveTime) / 1000 << "s" << std::endl;
    
    // Print routing results
    printRoutingResults();
    
    // Perform internal evaluation to match external evaluation tool's criteria
    evaluateRouting(nets);
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

// Handle difficult nets that couldn't be routed in previous phases
void Router::handleDifficultNets(const std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
    // Find nets that failed to route
    std::vector<Net> failedNets;
    std::unordered_set<int> failedNetIds;
    
    for (const auto& route : routingResults) {
        if (!route.isRouted) {
            failedNetIds.insert(route.netId);
        }
    }
    
    if (failedNetIds.empty()) {
        std::cout << "No failed nets to handle. Skipping difficult nets phase." << std::endl;
        return;
    }
    
    // Find the corresponding Net objects
    for (const auto& net : nets) {
        if (failedNetIds.find(net.id) != failedNetIds.end()) {
            failedNets.push_back(net);
        }
    }
    
    std::cout << "Found " << failedNets.size() << " difficult nets to retry." << std::endl;
    
    // Remove failed nets from routing results
    for (auto it = routingResults.begin(); it != routingResults.end();) {
        if (failedNetIds.find(it->netId) != failedNetIds.end()) {
            it = routingResults.erase(it);
        } else {
            ++it;
        }
    }
    
    // Setup for difficult net routing
    int originalTimeout = pathfinder->getTimeout();
    pathfinder->setTimeout(5000); // 5 seconds per path
    
    // Sort failed nets by complexity (number of sinks)
    std::sort(failedNets.begin(), failedNets.end(), 
        [](const Net& a, const Net& b) {
            return a.nodeIDs.size() < b.nodeIDs.size(); // Try simpler nets first
        }
    );
    
    int successCount = 0;
    
    // Try multiple approaches with increasing aggressiveness
    for (int attempt = 0; attempt < 3; attempt++) {
        if (failedNets.empty()) break;
        
        // Increase penalty for each retry
        double attemptFactor = 5.0 * (1.0 + attempt);
        pathfinder->setCongestionPenaltyFactor(attemptFactor);
        
        std::cout << "Difficult nets retry attempt " << (attempt + 1) 
                << " with congestion factor " << attemptFactor << std::endl;
        
        // Attempt to route each failed net
        auto netIt = failedNets.begin();
        while (netIt != failedNets.end()) {
            // Reset congestion for this attempt
            for (auto& nodePair : congestedNodes) {
                nodePair.second.erase(netIt->id);
            }
            
            // Try with relaxed cost function for difficult nets
            NetRoute result = routeDifficultNet(*netIt, edges, nodes);
            
            if (result.isRouted) {
                successCount++;
                std::cout << "Successfully routed difficult net " << netIt->id << std::endl;
                routingResults.push_back(result);
                netIt = failedNets.erase(netIt);
            } else {
                std::cout << "Still unable to route net " << netIt->id << " (" 
                        << netIt->nodeIDs.size() - 1 << " sinks)" << std::endl;
                ++netIt;
            }
        }
    }
    
    // Try progressive routing approach for remaining difficult nets
    if (!failedNets.empty()) {
        std::cout << "Trying progressive routing for " << failedNets.size() << " remaining difficult nets..." << std::endl;
        pathfinder->setCongestionPenaltyFactor(3.0); // Reduce penalty for more flexibility
        
        auto netIt = failedNets.begin();
        while (netIt != failedNets.end()) {
            // Reset congestion for this attempt
            for (auto& nodePair : congestedNodes) {
                nodePair.second.erase(netIt->id);
            }
            
            // Try progressive routing
            NetRoute result;
            bool success = progressiveRouting(*netIt, result, edges, nodes);
            
            if (success) {
                successCount++;
                std::cout << "Successfully routed difficult net " << netIt->id << " with progressive routing" << std::endl;
                routingResults.push_back(result);
                netIt = failedNets.erase(netIt);
            } else {
                std::cout << "Progressive routing failed for net " << netIt->id << std::endl;
                ++netIt;
            }
        }
    }
    
    // Final attempt - try pattern routing as fallback
    if (!failedNets.empty()) {
        std::cout << "Final attempt with pattern routing for " << failedNets.size() << " remaining difficult nets..." << std::endl;
        
        for (auto& net : failedNets) {
            // Reset congestion for this attempt
            for (auto& nodePair : congestedNodes) {
                nodePair.second.erase(net.id);
            }
            
            // Try pattern routing
            NetRoute result = patternRouteSingleNet(net, edges, nodes);
            
            if (result.isRouted) {
                successCount++;
                std::cout << "Successfully routed difficult net " << net.id << " with pattern routing" << std::endl;
            } else {
                std::cout << "Failed to route net " << net.id << " (" 
                        << net.nodeIDs.size() - 1 << " sinks)" << std::endl;
            }
            
            routingResults.push_back(result);
        }
    }
    
    // Restore original parameters
    pathfinder->setTimeout(originalTimeout);
    
    std::cout << "Difficult nets phase completed. " << successCount << " of " 
            << failedNetIds.size() << " difficult nets successfully routed." << std::endl;
}

// Final pass to eliminate any remaining congestion
void Router::eliminateRemainingCongestion(const std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
    // Maximum iterations for congestion elimination
    const int MAX_ITERATIONS = 20;
    
    // Reset pathfinder congestion map to match congestedNodes
    pathfinder->resetCongestion();
    
    // Build a mapping of node usage
    std::unordered_map<int, int> nodeUsage;
    for (const auto& pair : congestedNodes) {
        nodeUsage[pair.first] = pair.second.size();
    }
    
    // Count genuinely congested nodes (more than one net using them)
    std::vector<int> congestedNodeIds;
    for (const auto& pair : nodeUsage) {
        if (pair.second > NODE_CAPACITY) {
            congestedNodeIds.push_back(pair.first);
        }
    }
    
    if (congestedNodeIds.empty()) {
        std::cout << "No congested nodes found. Skipping final elimination." << std::endl;
        return;
    }
    
    std::cout << "Found " << congestedNodeIds.size() << " congested nodes to resolve." << std::endl;
    
    // Update pathfinder's congestion map to reflect the current congestion state
    for (const auto& pair : nodeUsage) {
        int nodeId = pair.first;
        int usage = pair.second;
        std::vector<int> dummy = {nodeId};
        pathfinder->updateCongestion(dummy, usage);
    }
    
    // Attempt to eliminate congestion
    for (int iteration = 0; iteration < MAX_ITERATIONS; iteration++) {
        // Re-evaluate congested nodes
        congestedNodeIds.clear();
        for (const auto& pair : congestedNodes) {
            if (pair.second.size() > NODE_CAPACITY) {
                congestedNodeIds.push_back(pair.first);
            }
        }
        
        if (congestedNodeIds.empty()) {
            std::cout << "All congestion eliminated at iteration " << iteration << "." << std::endl;
            break;
        }
        
        std::cout << "Iteration " << (iteration + 1) << ": " 
                  << congestedNodeIds.size() << " congested nodes" << std::endl;
        
        // Identify nets using congested nodes
        std::unordered_map<int, int> netCongestionCount;
        for (int nodeId : congestedNodeIds) {
            for (int netId : congestedNodes[nodeId]) {
                netCongestionCount[netId]++;
            }
        }
        
        // Sort nets by congestion impact
        std::vector<std::pair<int, int>> netsToRip;
        for (const auto& pair : netCongestionCount) {
            netsToRip.push_back({pair.first, pair.second});
        }
        
        // Sort by congestion impact (descending)
        std::sort(netsToRip.begin(), netsToRip.end(),
            [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                return a.second > b.second;
            });
        
        // Choose top 25% of nets to rip up
        size_t netsToSelect = std::max(size_t(1), netsToRip.size() / 4);
        netsToSelect = std::min(netsToSelect, netsToRip.size());
        
        std::vector<Net> netsToReroute;
        std::unordered_set<int> selectedNetIds;
        
        for (size_t i = 0; i < netsToSelect; i++) {
            int netId = netsToRip[i].first;
            selectedNetIds.insert(netId);
            
            // Find the corresponding Net object
            for (const auto& net : nets) {
                if (net.id == netId) {
                    netsToReroute.push_back(net);
                    break;
                }
            }
            
            // Remove this net from congestedNodes
            for (auto& nodePair : congestedNodes) {
                nodePair.second.erase(netId);
            }
            
            // Remove the net from routingResults
            for (auto it = routingResults.begin(); it != routingResults.end();) {
                if (it->netId == netId) {
                    it = routingResults.erase(it);
                } else {
                    ++it;
                }
            }
        }
        
        // Clean up empty entries
        auto cleanupIt = congestedNodes.begin();
        while (cleanupIt != congestedNodes.end()) {
            if (cleanupIt->second.empty()) {
                cleanupIt = congestedNodes.erase(cleanupIt);
            } else {
                ++cleanupIt;
            }
        }
        
        if (netsToReroute.empty()) {
            std::cout << "No nets selected for rerouting. Breaking." << std::endl;
            break;
        }
        
        std::cout << "Rerouting " << netsToReroute.size() << " nets to eliminate congestion..." << std::endl;
        
        // Use very high congestion penalty for final elimination
        double originalFactor = pathfinder->getCongestionPenaltyFactor();
        pathfinder->setCongestionPenaltyFactor(20.0); // Extremely high penalty to avoid congestion at all costs
        
        // Route the selected nets
        for (auto& net : netsToReroute) {
            NetRoute result = routeSingleNet(net, edges, nodes);
            
            if (result.isRouted) {
                std::cout << "Successfully rerouted net " << net.id << " without congestion." << std::endl;
            } else {
                std::cout << "Failed to reroute net " << net.id << " without congestion." << std::endl;
            }
            
            routingResults.push_back(result);
        }
        
        // Restore original congestion penalty
        pathfinder->setCongestionPenaltyFactor(originalFactor);
    }
    
    // Final congestion check
    int remainingCongestion = 0;
    for (const auto& pair : congestedNodes) {
        if (pair.second.size() > NODE_CAPACITY) {
            remainingCongestion++;
        }
    }
    
    if (remainingCongestion == 0) {
        std::cout << "All congestion successfully eliminated!" << std::endl;
    } else {
        std::cout << "Unable to eliminate all congestion. " << remainingCongestion 
                << " congested nodes remain." << std::endl;
    }
}

// Pattern routing for difficult nets as fallback
NetRoute Router::patternRouteSingleNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
    NetRoute netRoute;
    netRoute.netId = net.id;
    
    // First node is the source (driver), rest are sinks
    if (net.nodeIDs.size() < 2) {
        // No sinks to route to
        netRoute.isRouted = true;
        return netRoute;
    }
    
    int sourceNodeId = net.nodeIDs[0];
    const Node& sourceNode = nodes[sourceNodeId];
    
    // Track which sinks have been routed
    std::unordered_set<int> remainingSinks;
    for (size_t j = 1; j < net.nodeIDs.size(); ++j) {
        int sinkId = net.nodeIDs[j];
        // Only consider valid sink nodes
        if (existingNodeIds.find(sinkId) != existingNodeIds.end()) {
            remainingSinks.insert(sinkId);
        }
    }
    
    // Route to each sink using pattern routing
    for (size_t j = 1; j < net.nodeIDs.size(); ++j) {
        int sinkId = net.nodeIDs[j];
        
        // Skip invalid nodes
        if (existingNodeIds.find(sinkId) == existingNodeIds.end()) {
            continue;
        }
        
        const Node& sinkNode = nodes[sinkId];
        
        // Try different patterns (L-shape and Z-shape)
        std::vector<std::vector<int>> patternOptions;
        
        // Option 1: L-shape - horizontal first, then vertical
        std::vector<int> lShape1;
        bool lShape1Valid = tryLShapeRoute(sourceNode, sinkNode, lShape1, true, nodes);
        if (lShape1Valid) patternOptions.push_back(lShape1);
        
        // Option 2: L-shape - vertical first, then horizontal
        std::vector<int> lShape2;
        bool lShape2Valid = tryLShapeRoute(sourceNode, sinkNode, lShape2, false, nodes);
        if (lShape2Valid) patternOptions.push_back(lShape2);
        
        // Option 3: Z-shape - combination of horizontal and vertical segments
        std::vector<int> zShape;
        bool zShapeValid = tryZShapeRoute(sourceNode, sinkNode, zShape, nodes);
        if (zShapeValid) patternOptions.push_back(zShape);
        
        // Select the best pattern - choose the one with minimum congestion
        if (!patternOptions.empty()) {
            std::vector<int> bestPattern;
            double minCongestion = std::numeric_limits<double>::max();
            
            for (const auto& pattern : patternOptions) {
                double totalCongestion = 0;
                for (int nodeId : pattern) {
                    totalCongestion += pathfinder->getCongestion(nodeId);
                }
                
                if (totalCongestion < minCongestion) {
                    minCongestion = totalCongestion;
                    bestPattern = pattern;
                }
            }
            
            // Add this pattern to the NetRoute
            for (size_t i = 0; i < bestPattern.size() - 1; ++i) {
                netRoute.addEdge(bestPattern[i], bestPattern[i+1]);
                
                // Track congested nodes
                congestedNodes[bestPattern[i]].insert(net.id);
                congestedNodes[bestPattern[i+1]].insert(net.id);
            }
            
            // Mark this sink as routed
            remainingSinks.erase(sinkId);
        } else {
            // No valid pattern found for this sink - STRICT MODE
            // If any sink can't be routed, the whole net fails
            netRoute.isRouted = false;
            return netRoute;
        }
    }
    
    // STRICT MODE: All sinks must be routed
    netRoute.isRouted = remainingSinks.empty();
    
    // Only for logging - don't change isRouted flag
    if (!netRoute.isRouted && netRoute.edges.size() > 0 && validSinkNodeIds.size() > 1) {
        // Compute success percentage
        double successRate = (validSinkNodeIds.size() - remainingSinks.size()) / 
                            (double)validSinkNodeIds.size();
        
        if (successRate > 0.6) {
            std::cout << "Net " << net.id << " partially routed (" 
                    << (int)(successRate * 100) << "% of sinks) but marked as FAILED" << std::endl;
        }
    }
    
    return netRoute;
}

// Try L-shape routing
bool Router::tryLShapeRoute(const Node& sourceNode, const Node& sinkNode, std::vector<int>& path, bool horizontalFirst, const std::vector<Node>& nodes) {
    // Clear the path
    path.clear();
    
    // Source and sink coordinates
    int sourceX = sourceNode.beginX;
    int sourceY = sourceNode.beginY;
    int sinkX = sinkNode.beginX;
    int sinkY = sinkNode.beginY;
    
    // Find intermediate node based on pattern
    int intermediateX, intermediateY;
    if (horizontalFirst) {
        intermediateX = sinkX;
        intermediateY = sourceY;
    } else {
        intermediateX = sourceX;
        intermediateY = sinkY;
    }
    
    // Find node IDs for these coordinates
    int intermediateNodeId = -1;
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (nodes[i].beginX == intermediateX && nodes[i].beginY == intermediateY) {
            intermediateNodeId = nodes[i].id;
            break;
        }
    }
    
    // If intermediate node not found, pattern is invalid
    if (intermediateNodeId == -1) return false;
    
    // Check if nodes would cause congestion
    if (pathfinder->isNodeOvercongested(intermediateNodeId) || 
        pathfinder->isNodeOvercongested(sinkNode.id)) {
        return false;
    }
    
    // Create the L-shaped path
    path.push_back(sourceNode.id);
    path.push_back(intermediateNodeId);
    path.push_back(sinkNode.id);
    
    return true;
}

// Try Z-shape routing
bool Router::tryZShapeRoute(const Node& sourceNode, const Node& sinkNode, std::vector<int>& path, const std::vector<Node>& nodes) {
    // Clear the path
    path.clear();
    
    // Source and sink coordinates
    int sourceX = sourceNode.beginX;
    int sourceY = sourceNode.beginY;
    int sinkX = sinkNode.beginX;
    int sinkY = sinkNode.beginY;
    
    // Calculate midpoint for Z-shape
    int midX = (sourceX + sinkX) / 2;
    
    // Find intermediate nodes for Z-shape
    int intermediate1NodeId = -1;
    int intermediate2NodeId = -1;
    
    for (size_t i = 0; i < nodes.size(); ++i) {
        if (nodes[i].beginX == midX && nodes[i].beginY == sourceY) {
            intermediate1NodeId = nodes[i].id;
        }
        if (nodes[i].beginX == midX && nodes[i].beginY == sinkY) {
            intermediate2NodeId = nodes[i].id;
        }
    }
    
    // If either intermediate node not found, pattern is invalid
    if (intermediate1NodeId == -1 || intermediate2NodeId == -1) return false;
    
    // Check if nodes would cause congestion
    if (pathfinder->isNodeOvercongested(intermediate1NodeId) || 
        pathfinder->isNodeOvercongested(intermediate2NodeId) ||
        pathfinder->isNodeOvercongested(sinkNode.id)) {
        return false;
    }
    
    // Create the Z-shaped path
    path.push_back(sourceNode.id);
    path.push_back(intermediate1NodeId);
    path.push_back(intermediate2NodeId);
    path.push_back(sinkNode.id);
    
    return true;
}

// Route with relaxed constraints for difficult nets
NetRoute Router::routeDifficultNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
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
    
    // Route from source to each sink with relaxed constraints
    std::unordered_set<int> remainingSinks(validSinkNodeIds.begin(), validSinkNodeIds.end());
    bool allSinksRouted = true;
    
    
    for (int sinkId : validSinkNodeIds) {
        // Find path from source to this sink with relaxed constraints
        std::vector<int> path;
        pathfinder->findPathForDifficultNet(sourceNodeId, sinkId, path, congestedNodes, net.id);
        
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
                
    // Set success flag on NetRoute - STRICT MODE: all sinks must be routed
    netRoute.isRouted = remainingSinks.empty();
    
    // Only for logging - don't change isRouted flag
    if (!netRoute.isRouted && netRoute.edges.size() > 0 && validSinkNodeIds.size() > 1) {
        // Compute success percentage
        double successRate = (validSinkNodeIds.size() - remainingSinks.size()) / 
                            (double)validSinkNodeIds.size();
        
        if (successRate > 0.6) {
            std::cout << "Net " << net.id << " partially routed (" 
                    << (int)(successRate * 100) << "% of sinks) but marked as FAILED" << std::endl;
        }
    }
    
    return netRoute;
}

// Progressive routing for multi-sink nets
bool Router::progressiveRouting(Net& net, NetRoute& netRoute, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
    netRoute.netId = net.id;
    
    // No sinks to route
    if (net.nodeIDs.size() <= 1) {
        netRoute.isRouted = true;
        return true;
    }
    
    int sourceNodeId = net.nodeIDs[0];
    
    // Sort sinks by distance from source (nearest first)
    std::vector<std::pair<int, double>> sinkDistances;
    for (size_t i = 1; i < net.nodeIDs.size(); i++) {
        int sinkId = net.nodeIDs[i];
        if (existingNodeIds.find(sinkId) == existingNodeIds.end()) continue;
        
        double distance = calculateDistance(sourceNodeId, sinkId, nodes);
        sinkDistances.push_back({sinkId, distance});
    }
    
    // Sort by distance (ascending)
    std::sort(sinkDistances.begin(), sinkDistances.end(),
        [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
            return a.second < b.second;
        });
    
    // Current routing tree nodes (start with source)
    std::unordered_set<int> routedNodes;
    routedNodes.insert(sourceNodeId);
    
    // Try to route each sink
    int routedSinkCount = 0;
    for (const auto& sinkPair : sinkDistances) {
        int sinkId = sinkPair.first;
        
        // Find closest node in current tree to route from
        int nearestTreeNodeId = sourceNodeId;
        double minDistance = std::numeric_limits<double>::max();
        
        for (int nodeId : routedNodes) {
            double distance = calculateDistance(nodeId, sinkId, nodes);
            if (distance < minDistance) {
                minDistance = distance;
                nearestTreeNodeId = nodeId;
            }
        }
        
        // Try to route from nearest tree node to this sink
        std::vector<int> path;
        pathfinder->findPathForDifficultNet(nearestTreeNodeId, sinkId, path, congestedNodes, net.id);
        
        if (!path.empty() && path.size() >= 2) {
            // Add new edges to route
            for (size_t i = 0; i < path.size() - 1; ++i) {
                netRoute.addEdge(path[i], path[i+1]);
                routedNodes.insert(path[i]);
                routedNodes.insert(path[i+1]);
            }
            routedSinkCount++;
        } else {
            // STRICT MODE: If any sink fails to route, the entire net fails
            // No need to continue trying other sinks
            netRoute.isRouted = false;
            
            // Log partial progress for debugging purposes only
            if (routedSinkCount > 0) {
                double successRate = (double)routedSinkCount / sinkDistances.size();
                std::cout << "Progressive routing: Net " << net.id << " partially routed (" 
                        << (int)(successRate * 100) << "% of sinks) but FAILED due to unreachable sink" << std::endl;
            }
            
            return false;
        }
    }
    
    // STRICT MODE: Check if all sinks were routed
    bool allSinksRouted = (routedSinkCount == sinkDistances.size());
    netRoute.isRouted = allSinksRouted;
    
    // Only for logging - report partial success but don't change isRouted flag
    if (!allSinksRouted && routedSinkCount > 0) {
        double successRate = (double)routedSinkCount / sinkDistances.size();
        if (successRate > 0.6) {
            std::cout << "Progressive routing: Net " << net.id << " partially routed (" 
                    << (int)(successRate * 100) << "% of sinks) but marked as FAILED" << std::endl;
        }
    }
    
    return allSinksRouted;
}

// Validate if a net is fully routed according to evaluation tool criteria
bool Router::validateNetRoute(const NetRoute& netRoute, const Net& net) const {
    // Must have edges and be marked as routed
    if (netRoute.edges.empty() || !netRoute.isRouted)
        return false;
        
    // Collect all nodes in the route
    std::unordered_set<int> routedNodes;
    for (const auto& edge : netRoute.edges) {
        routedNodes.insert(edge.first);
        routedNodes.insert(edge.second);
    }
    
    // Source node must be in the route
    if (routedNodes.find(net.nodeIDs[0]) == routedNodes.end())
        return false;
    
    // Check every sink is connected - even a single missing sink means failure
    for (size_t i = 1; i < net.nodeIDs.size(); i++) {
        int sinkId = net.nodeIDs[i];
        
        // Skip invalid nodes (those not in the graph)
        if (existingNodeIds.find(sinkId) == existingNodeIds.end())
            continue;
            
        if (routedNodes.find(sinkId) == routedNodes.end())
            return false; // Sink not found in route
    }
    
    return true;
}

// Compare our routing success with what the evaluation tool would report
void Router::compareRoutingToEval(const std::vector<Net>& nets) const {
    int ourSuccessCount = 0;
    int evalSuccessCount = 0;
    std::vector<int> mismatchedNets;
    
    // Check each net
    for (const auto& route : routingResults) {
        if (route.isRouted) {
            ourSuccessCount++;
            
            // Find the corresponding Net
            for (const auto& net : nets) {
                if (net.id == route.netId) {
                    // Validate according to evaluation tool criteria
                    if (validateNetRoute(route, net)) {
                        evalSuccessCount++;
                    } else {
                        // This net would be considered failed by the eval tool
                        mismatchedNets.push_back(net.id);
                    }
                    break;
                }
            }
        }
    }
    
    if (ourSuccessCount != evalSuccessCount) {
        std::cout << "\n==== Routing Validation Warning ====\n";
        std::cout << "Router reports " << ourSuccessCount << " successful nets\n";
        std::cout << "Evaluation tool would report " << evalSuccessCount << " successful nets\n";
        std::cout << "Discrepancy: " << (ourSuccessCount - evalSuccessCount) << " nets\n";
        
        if (!mismatchedNets.empty()) {
            std::cout << "Mismatched nets (router says success, eval would say failure): ";
            for (size_t i = 0; i < std::min(size_t(10), mismatchedNets.size()); i++) {
                std::cout << mismatchedNets[i] << " ";
            }
            if (mismatchedNets.size() > 10) {
                std::cout << "... (" << (mismatchedNets.size() - 10) << " more)";
            }
            std::cout << std::endl;
        }
        std::cout << "==================================\n\n";
    }
}

// Perform internal evaluation matching external evaluation tool
void Router::evaluateRouting(const std::vector<Net>& nets) const {
    int successCount = 0;
    int congestionCount = 0;
    unsigned long totalWirelength = 0;
    
    // Check each net
    for (const auto& route : routingResults) {
        // Must be marked as routed
        if (route.isRouted) {
            // Find the corresponding Net
            for (const auto& net : nets) {
                if (net.id == route.netId) {
                    // Validate according to evaluation tool criteria
                    if (validateNetRoute(route, net)) {
                        successCount++;
                        totalWirelength += route.edges.size();
                    }
                    break;
                }
            }
        }
    }
    
    // Check congestion
    for (const auto& pair : congestedNodes) {
        if (pair.second.size() > NODE_CAPACITY) {
            congestionCount++;
        }
    }
    
    std::cout << "\n==== Internal Evaluation Results ====\n";
    std::cout << "# congested nodes: " << congestionCount << std::endl;
    std::cout << "# successfully routed nets: " << successCount << "/" << nets.size() << std::endl;
    std::cout << "total wirelength: " << totalWirelength << std::endl;
    std::cout << "Success rate: " << (nets.empty() ? 0 : (successCount * 100.0 / nets.size())) << "%" << std::endl;
    std::cout << "======================================\n\n";
    
    // Compare our results with what the evaluation tool would report
    compareRoutingToEval(nets);
}

// Find congested regions and return as Region objects
std::vector<Region> Router::findCongestedRegionsGrouped() {
    // Since we don't have access to nodes, we need to implement a different approach
    // This method should be called with the nodes parameter when needed
    // For now, return an empty vector
    return {};
}

// Check if a net passes through any congestion hotspot
bool Router::isNetInCongestionHotspot(int netId, const std::vector<Region>& hotspots) {
    // First, find the net's route
    NetRoute netRoute;
    for (const auto& route : routingResults) {
        if (route.netId == netId) {
            netRoute = route;
            break;
        }
    }
    
    // If the net has no route or is not routed, it can't be in a hotspot
    if (netRoute.edges.empty() || !netRoute.isRouted) {
        return false;
    }
    
    // Extract all nodes in the route
    std::vector<int> routeNodes = extractNodesFromRoute(netRoute);
    
    // Without access to node coordinates, we can just check if any route node is in any hotspot's nodeIds
    for (int nodeId : routeNodes) {
        for (const auto& hotspot : hotspots) {
            // Check if node is in the hotspot's node list
            if (std::find(hotspot.nodeIds.begin(), hotspot.nodeIds.end(), nodeId) != hotspot.nodeIds.end()) {
                return true;
            }
        }
    }
    
    return false;
}

// Print statistics about congestion
void Router::printCongestionStats() {
    // Count congested nodes by usage level
    std::unordered_map<int, int> usageCounts;
    int maxUsage = 0;
    for (const auto& pair : congestedNodes) {
        int usage = pair.second.size();
        usageCounts[usage]++;
        maxUsage = std::max(maxUsage, usage);
    }
    
    std::cout << "\n==== Congestion Statistics ====\n";
    std::cout << "Total nodes with at least one net: " << congestedNodes.size() << std::endl;
    
    // Print usage distribution
    std::cout << "\nNode usage distribution:\n";
    for (int i = 1; i <= maxUsage; i++) {
        std::cout << "  Nodes with " << i << " nets: " << usageCounts[i] << std::endl;
    }
    
    // Count congested nodes (usage > 1)
    int congestedCount = 0;
    for (int i = NODE_CAPACITY + 1; i <= maxUsage; i++) {
        congestedCount += usageCounts[i];
    }
    
    std::cout << "\nCongested nodes (>" << NODE_CAPACITY << " nets): " << congestedCount << std::endl;
    std::cout << "Congestion percentage: " 
              << (congestedNodes.empty() ? 0 : (congestedCount * 100.0 / congestedNodes.size())) 
              << "%" << std::endl;
    
    std::cout << "===============================\n\n";
}


