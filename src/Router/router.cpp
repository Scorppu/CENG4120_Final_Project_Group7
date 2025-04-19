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

class Router {
    private:
        // Collection of routing trees for result printing
        std::vector<std::shared_ptr<RoutingTree>> routingResults;
        
        // Shared pathfinding instance to avoid costly recreation
        std::unique_ptr<AStarSearch> pathfinder;
        
    public:
        Router() : pathfinder(nullptr) {}
        
        // Destructor that properly cleans up resources
        ~Router() {
            clearAll();
        }
        
        // Get routing results
        const std::vector<std::shared_ptr<RoutingTree>>& getRoutingResults() const {
            return routingResults;
        }
        
        // Clear routing results
        void clearRoutingResults() {
            routingResults.clear();
        }
        
        // Clear the pathfinder to free its memory
        void clearPathfinder() {
            pathfinder.reset(nullptr);
        }
        
        // Clear all resources to free memory
        void clearAll() {
            clearRoutingResults();
            clearPathfinder();
        }

        // Resolve congestions (Rip up and reroute)
        virtual void resolveCongestion() {
            // Implement congestion resolution logic here
        }

        // Route a single net using A* Search
        virtual std::shared_ptr<RoutingTree> routeSingleNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
            // Check if we have enough nodes to route
            if (net.nodeIDs.size() < 2) {
                std::cerr << "Net " << net.id << " (" << net.name << ") has fewer than 2 nodes, skipping" << std::endl;
                return nullptr;
            }

            // In the netlist format from design1.netlist:
            // First node is the source, all others are sinks
            int sourceNodeId = net.nodeIDs[0];
            std::vector<int> sinkNodeIds(net.nodeIDs.begin() + 1, net.nodeIDs.end());
            
            // Create a routing tree to store the results
            std::shared_ptr<RoutingTree> routingTree = std::make_shared<RoutingTree>(net.id, sourceNodeId, sinkNodeIds);
            
            // Use a more memory-efficient approach for checking node existence
            std::unordered_set<int> existingNodeIds;
            existingNodeIds.reserve(nodes.size());  // Pre-allocate for efficiency
            
            for (const auto& node : nodes) {
                existingNodeIds.insert(node.id);
            }
            
            // Verify source node exists in the graph
            if (existingNodeIds.find(sourceNodeId) == existingNodeIds.end()) {
                std::cerr << "Source node " << sourceNodeId << " for net " << net.id 
                          << " (" << net.name << ") does not exist in the graph, skipping" << std::endl;
                return nullptr;
            }
            
            // Filter sinks to only include those that exist in the graph
            std::vector<int> validSinkNodeIds;
            validSinkNodeIds.reserve(sinkNodeIds.size());  // Pre-allocate for efficiency
            
            for (int sinkId : sinkNodeIds) {
                if (existingNodeIds.find(sinkId) != existingNodeIds.end()) {
                    validSinkNodeIds.push_back(sinkId);
                } else {
                    std::cerr << "Sink node " << sinkId << " for net " << net.id 
                              << " does not exist in the graph, skipping this sink" << std::endl;
                }
            }
            
            if (validSinkNodeIds.empty()) {
                std::cerr << "No valid sink nodes for net " << net.id 
                          << " (" << net.name << "), skipping" << std::endl;
                return nullptr;
            }
            
            // Ensure we have a valid pathfinder
            if (!pathfinder) {
                std::cout << "Creating A* pathfinder..." << std::endl;
                auto startTime = std::chrono::high_resolution_clock::now();
                
                pathfinder = std::make_unique<AStarSearch>(edges, nodes);
                pathfinder->setTimeout(2000);  // Set 2 second timeout
                
                auto endTime = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
                std::cout << "Pathfinder initialization took " << duration << "ms" << std::endl;
            }
            
            // Set custom cost function if needed for congestion
            pathfinder->setCostFunction([this](int fromId, int toId) {
                return this->calculateCost(fromId, toId);
            });
            
            // Route from source to each sink
            std::unordered_set<int> remainingSinks(validSinkNodeIds.begin(), validSinkNodeIds.end());
            bool allSinksRouted = true;
            
            // Debug information
            std::cout << "Routing net " << net.id << " (" << net.name << ") with source " 
                      << sourceNodeId << " and " << validSinkNodeIds.size() << " sinks" << std::endl;
            
            // Start timing
            auto startTime = std::chrono::high_resolution_clock::now();
            
            for (int sinkId : validSinkNodeIds) {
                // Find path from source to this sink
                std::vector<int> path = pathfinder->findPath(sourceNodeId, sinkId);
                
                if (path.empty() || path.size() < 2) {
                    // Failed to find a path to this sink
                    std::cerr << "Failed to find path from " << sourceNodeId << " to " << sinkId << std::endl;
                    allSinksRouted = false;
                    continue;
                }
                
                std::cout << "Found path from " << sourceNodeId << " to " << sinkId 
                          << " with " << path.size() << " nodes" << std::endl;
                
                // Convert path to edges and add to routing tree
                for (size_t i = 0; i < path.size() - 1; ++i) {
                    routingTree->edges.push_back({path[i], path[i+1]});
                }
                
                // Mark this sink as routed
                remainingSinks.erase(sinkId);
            }
            
            // Report total routing time
            auto endTime = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
            std::cout << "Completed routing net " << net.id << " in " << duration << "ms" << std::endl;
                        
            // Set success flag on routing tree
            routingTree->isRouted = remainingSinks.empty();
            
            return routingTree;
        }

        // Helper method to calculate cost between nodes (can be extended for congestion)
        double calculateCost(int fromId, int toId) {
            // Simple uniform cost for now, can be updated with congestion later
            return 1.0;
        }

        virtual void routeAllNets(std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
            // Clear previous results
            clearRoutingResults();
            
            // Reserve space for results
            routingResults.reserve(nets.size());
            
            // Initialize the global pathfinder once
            auto startTime = std::chrono::high_resolution_clock::now();
            
            // Create the pathfinder
            pathfinder = std::make_unique<AStarSearch>(edges, nodes);
            pathfinder->setTimeout(2000); // 2 second timeout per path
            
            auto endTime = std::chrono::high_resolution_clock::now();
            auto initTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
            std::cout << "Initialized pathfinder in " << initTime << "ms" << std::endl;
            
            // Start timing the full routing process
            startTime = std::chrono::high_resolution_clock::now();
            
            for (Net& net : nets) {
                // Route the net and store the result
                std::shared_ptr<RoutingTree> result = routeSingleNet(net, edges, nodes);
                if (result) {
                    routingResults.push_back(result);
                }
            }
            
            // Report total time
            endTime = std::chrono::high_resolution_clock::now();
            auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
            std::cout << "Complete routing of all nets took " << totalTime << "ms" << std::endl;

            // resolve congestions
            // resolveCongestion();
        }
        
        // Print routing results
        void printRoutingResults() const {
            std::cout << "\n==== Routing Results ====\n";
            std::cout << "Total nets routed: " << routingResults.size() << std::endl;
            
            int successfullyRouted = 0;
            for (const auto& tree : routingResults) {
                if (tree->isRouted) {
                    successfullyRouted++;
                }
            }
            
            std::cout << "Successfully routed: " << successfullyRouted << std::endl;
            std::cout << "Success rate: " << (routingResults.empty() ? 0 : (successfullyRouted * 100.0 / routingResults.size())) << "%" << std::endl;
            
            // Print details for each net if needed
            for (const auto& tree : routingResults) {
                std::cout << "Net " << tree->NetId << ": " 
                          << (tree->isRouted ? "Successfully routed" : "Failed to route")
                          << " (" << tree->edges.size() << " connections)" << std::endl;
            }
            
            std::cout << "==========================\n" << std::endl;
        }
};