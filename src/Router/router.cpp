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
#include "../PathfindingAlgorithms/SteinerArborescence.hpp"
#include "../PathfindingAlgorithms/SteinerSALT.hpp"

class Router {
    private:
        // Collection of routing trees for result printing
        std::vector<std::shared_ptr<RoutingTree>> routingResults;
        
        // Shared pathfinding instance - can be either SteinerArborescence or SteinerSALT
        std::unique_ptr<SteinerArborescence> pathfinderArborescence;
        std::unique_ptr<SteinerSALT> pathfinderSALT;
        
        // Flag to indicate which algorithm to use
        bool useSALT = true;
        
        // Reusable data structures to avoid repeated allocation
        std::unordered_set<int> existingNodeIds;
        std::vector<int> validSinkNodeIds;

        // Verbosity level for logging
        bool verboseLogging = false;
        
        // Timeout for pathfinder
        int pathfinderTimeout = 0;
        
        // Congestion factor for pathfinder
        double pathfinderCongestionFactor = 0;
        
        // Trade-off parameter for SALT
        double epsilon = 0.5;
        
    public:
        Router() : pathfinderArborescence(nullptr), pathfinderSALT(nullptr) {}
        
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
        
        // Clear the pathfinder to free memory
        void clearPathfinder() {
            pathfinderArborescence.reset(nullptr);
            pathfinderSALT.reset(nullptr);
        }
        
        // Clear all resources to free memory
        void clearAll() {
            clearRoutingResults();
            clearPathfinder();
        }

        // Set verbosity level
        void setVerbose(bool verbose) {
            verboseLogging = verbose;
        }
        
        // Set pathfinder timeout in milliseconds
        void setTimeout(int timeoutMs) {
            pathfinderTimeout = timeoutMs;
            if (pathfinderArborescence) {
                pathfinderArborescence->setTimeout(timeoutMs);
            }
            if (pathfinderSALT) {
                pathfinderSALT->setTimeout(timeoutMs);
            }
        }
        
        // Set congestion penalty factor
        void setCongestionFactor(double factor) {
            pathfinderCongestionFactor = factor;
            if (pathfinderArborescence) {
                pathfinderArborescence->setCongestionPenaltyFactor(factor);
            }
            if (pathfinderSALT) {
                pathfinderSALT->setCongestionPenaltyFactor(factor);
            }
        }
        
        // Set epsilon (trade-off parameter) for SALT
        void setEpsilon(double eps) {
            epsilon = eps;
            if (pathfinderSALT) {
                pathfinderSALT->setEpsilon(eps);
            }
        }
        
        // Set which algorithm to use
        void setUseSALT(bool use) {
            useSALT = use;
        }

        // Resolve congestions (Rip up and reroute)
        virtual void resolveCongestion() {
            // Implement congestion resolution logic here
        }

        // Route a single net using Steiner Arborescence or SALT
        virtual std::shared_ptr<RoutingTree> routeSingleNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
            // Check if we have enough nodes to route
            if (net.nodeIDs.size() < 2) {
                if (verboseLogging) {
                    std::cerr << "Net " << net.id << " (" << net.name << ") has fewer than 2 nodes, skipping" << std::endl;
                }
                return nullptr;
            }

            // In the netlist format from design1.netlist:
            // First node is the source, all others are sinks
            int sourceNodeId = net.nodeIDs[0];
            std::vector<int> sinkNodeIds(net.nodeIDs.begin() + 1, net.nodeIDs.end());
            
            // Create a routing tree to store the results
            std::shared_ptr<RoutingTree> routingTree = std::make_shared<RoutingTree>(net.id, sourceNodeId, sinkNodeIds);
            
            // Verify source node exists in the graph
            if (existingNodeIds.find(sourceNodeId) == existingNodeIds.end()) {
                if (verboseLogging) {
                    std::cerr << "Source node " << sourceNodeId << " for net " << net.id 
                            << " (" << net.name << ") does not exist in the graph, skipping" << std::endl;
                }
                return nullptr;
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
                return nullptr;
            }
            
            // Minimal debug information
            if (verboseLogging) {
                std::cout << "Routing net " << net.id << " with " << validSinkNodeIds.size() << " sinks" << std::endl;
            }
            
            // Find Steiner tree connecting source to all sinks
            std::vector<int> path;
            if (useSALT && pathfinderSALT) {
                path = pathfinderSALT->findSteinerTree(sourceNodeId, validSinkNodeIds);
            } else if (pathfinderArborescence) {
                path = pathfinderArborescence->findSteinerTree(sourceNodeId, validSinkNodeIds);
            } else {
                std::cerr << "Error: No pathfinder initialized" << std::endl;
                return nullptr;
            }
            
            if (path.empty()) {
                // More verbose error reporting
                std::cerr << "Failed to find Steiner tree for net " << net.id << " (" << net.name << ")" 
                        << " from source " << sourceNodeId 
                        << " to " << validSinkNodeIds.size() << " sinks" << std::endl;
                
                // Add the first few sink IDs for debugging
                if (!validSinkNodeIds.empty()) {
                    std::cerr << "Sample sinks: ";
                    for (size_t i = 0; i < std::min(size_t(5), validSinkNodeIds.size()); ++i) {
                        std::cerr << validSinkNodeIds[i] << " ";
                    }
                    std::cerr << std::endl;
                }
                
                // Even though we failed to route, return the routing tree to track this failure
                routingTree->isRouted = false;
                return routingTree;
            }
            
            // Convert path to edges and add to routing tree
            for (size_t i = 0; i < path.size() - 1; ++i) {
                routingTree->edges.push_back({path[i], path[i+1]});
            }
            
            // Debug output about the path
            if (verboseLogging) {
                std::cout << "Successfully routed net " << net.id << " with path length " << path.size() 
                        << " and " << routingTree->edges.size() << " edges" << std::endl;
            }
            
            // Update congestion for the entire path
            if (useSALT && pathfinderSALT) {
                pathfinderSALT->updateCongestion(path, 1.0);
            } else if (pathfinderArborescence) {
                pathfinderArborescence->updateCongestion(path);
            }
            
            // Mark all sinks as routed (since Steiner tree connects them all)
            routingTree->isRouted = true;
            
            return routingTree;
        }

        // Helper method to calculate cost between nodes (includes congestion awareness)
        double calculateCost(int fromId, int toId) {
            // Base cost of traversing between nodes
            const double baseCost = 1.0;
            
            // Add congestion awareness if pathfinder is initialized
            if (useSALT && pathfinderSALT) {
                double congestion = pathfinderSALT->getCongestion(toId);
                return baseCost + (congestion * pathfinderCongestionFactor);
            } else if (pathfinderArborescence) {
                double congestion = pathfinderArborescence->getCongestion(toId);
                return baseCost + (congestion * pathfinderCongestionFactor);
            }
            
            return baseCost;
        }

        virtual void routeAllNets(std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
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
            
            // Initialize the pathfinder
            auto startTime = std::chrono::high_resolution_clock::now();
            
            // Create pathfinder based on selected algorithm
            if (useSALT) {
                pathfinderSALT = std::make_unique<SteinerSALT>(edges, nodes);
                
                // Apply settings
                if (pathfinderTimeout > 0) {
                    pathfinderSALT->setTimeout(pathfinderTimeout);
                } else {
                    pathfinderSALT->setTimeout(5000); // Default 5 second timeout
                }
                
                if (pathfinderCongestionFactor > 0) {
                    pathfinderSALT->setCongestionPenaltyFactor(pathfinderCongestionFactor);
                } else {
                    pathfinderSALT->setCongestionPenaltyFactor(1.5); // Default congestion factor
                }
                
                pathfinderSALT->setEpsilon(epsilon);
                
                // Reset congestion map
                pathfinderSALT->resetCongestion();
            } else {
                pathfinderArborescence = std::make_unique<SteinerArborescence>(edges, nodes);
                
                // Apply settings
                if (pathfinderTimeout > 0) {
                    pathfinderArborescence->setTimeout(pathfinderTimeout);
                } else {
                    pathfinderArborescence->setTimeout(5000); // Default 5 second timeout
                }
                
                if (pathfinderCongestionFactor > 0) {
                    pathfinderArborescence->setCongestionPenaltyFactor(pathfinderCongestionFactor);
                } else {
                    pathfinderArborescence->setCongestionPenaltyFactor(1.5); // Default congestion factor
                }
                
                // Reset congestion map
                pathfinderArborescence->resetCongestion();
            }
            
            auto endTime = std::chrono::high_resolution_clock::now();
            auto initTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
            std::cout << "Initialized pathfinder in " << initTime << "ms" << std::endl;
            
            // Start timing the full routing process
            startTime = std::chrono::high_resolution_clock::now();
            
            // Pre-allocate vector for sink IDs to further reduce allocations
            validSinkNodeIds.reserve(100);  // Reasonable size for most nets
            
            // Create indices for nets to sort them without modifying original array
            std::vector<size_t> netIndices(nets.size());
            for (size_t i = 0; i < nets.size(); ++i) {
                netIndices[i] = i;
            }
            
            // Sort nets by fanout (number of sinks) in descending order
            std::sort(netIndices.begin(), netIndices.end(), [&nets](size_t a, size_t b) {
                // nodeIDs[0] is the source, the rest are sinks, so size()-1 is the fanout
                return (nets[a].nodeIDs.size() - 1) > (nets[b].nodeIDs.size() - 1);
            });
            
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
                std::shared_ptr<RoutingTree> result = routeSingleNet(nets[i], edges, nodes);
                if (result) {
                    routingResults.push_back(result);
                }
            }
            
            // Report total time
            endTime = std::chrono::high_resolution_clock::now();
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
                    for (const auto& tree : routingResults) {
                        if (tree->NetId == nets[netIdx].id) {
                            std::cout << "  Net " << nets[netIdx].id << " (" << nets[netIdx].name 
                                    << ") with fanout " << (nets[netIdx].nodeIDs.size() - 1)
                                    << ": " << (tree->isRouted ? "SUCCESS" : "FAILED") << std::endl;
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
            
            // Only print detailed per-net results if verbose logging is enabled
            if (verboseLogging) {
                for (const auto& tree : routingResults) {
                    std::cout << "Net " << tree->NetId << ": " 
                            << (tree->isRouted ? "Successfully routed" : "Failed to route")
                            << " (" << tree->edges.size() << " connections)" << std::endl;
                }
            }
            
            std::cout << "==========================\n" << std::endl;
        }
};