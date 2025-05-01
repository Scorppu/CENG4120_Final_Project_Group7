#ifndef ROUTER_HPP
#define ROUTER_HPP

#include <vector>
#include <memory>
#include <unordered_set>
#include <chrono>
#include "../DataStructure.hpp"
#include "../PathfindingAlgorithms/AStarSearch.hpp"

class Router {
private:
    // Collection of routing results using the simpler NetRoute structure
    std::vector<NetRoute> routingResults;
    
    // Shared pathfinding instance to avoid costly recreation
    std::unique_ptr<AStarSearch> pathfinder;
    
    // Reusable data structures to avoid repeated allocation
    std::unordered_set<int> existingNodeIds;
    std::vector<int> validSinkNodeIds;
    
    // Keep track of congested nodes
    std::unordered_map<int, std::unordered_set<int>> congestedNodes;
    
    // Keep track of reroute attempts for each net
    std::unordered_map<int, int> rerouteAttempts;

    // Design-specific settings
    int designNumber;     // Current design number (1-5)
    int timeoutSeconds;   // Global timeout in seconds
    
    // Program start time
    std::chrono::time_point<std::chrono::steady_clock> programStartTime;
    bool hasProgramStartTime;
public:
    // Constructor
    Router();
    
    // Destructor that properly cleans up resources
    ~Router();

    // Calculate distance between two nodes
    double calculateDistance(int fromId, int toId, const std::vector<Node>& nodes);

    // Compute priority for a net
    double computePriority(const Net& net, const std::vector<Node>& nodes, std::map<int, std::vector<int>>& x_to_ys);
    
    // Get routing results
    const std::vector<NetRoute>& getRoutingResults() const;
    
    // Clear routing results
    void clearRoutingResults();
    
    // Clear the pathfinder to free its memory
    void clearPathfinder();
    
    // Clear all resources to free memory
    void clearAll();

    // Set verbosity level
    void setVerbose(bool verbose);
    
    // Set the design number to adjust parameters accordingly
    void setDesignNumber(int number);
    
    // Set the global timeout in seconds
    void setTimeout(int seconds);
    
    // Set the program start time to calculate total elapsed time
    void setProgramStartTime(std::chrono::time_point<std::chrono::steady_clock> startTime);

    // Resolve congestions (Rip up and reroute)
    virtual void resolveCongestion(const std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes);

    // Route a single net using A* Search
    virtual NetRoute routeSingleNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes);

    // Helper method to calculate cost between nodes (includes congestion awareness)
    double calculateCost(int fromId, int toId);

    // Route all nets
    virtual void routeAllNets(std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes, std::map<int, std::vector<int>>& x_to_ys);
    
    // Print routing results
    void printRoutingResults() const;
    
    // Extract nodes from a route's edges
    std::vector<int> extractNodesFromRoute(const NetRoute& route) const;
    
    // Calculate congestion score for a net
    double calculateCongestionScore(int netId, const std::vector<int>& path) const;
};

// Derived class for Steiner Tree routing
// class SteinerTreeRouter : public Router {
// private:
//     // Collection of routing results using the simpler NetRoute structure
//     std::vector<NetRoute> routingResults;
    
//     // Shared pathfinding instance to avoid costly recreation
//     std::unique_ptr<AStarSearch> pathfinder;
    
//     // Reusable data structures to avoid repeated allocation
//     std::unordered_set<int> existingNodeIds;
//     std::vector<int> validSinkNodeIds;
// public:
//     // Constructor
//     SteinerTreeRouter();
    
//     std::vector<int> get_shortest_path(int src, int dest, const std::vector<std::vector<int>>& edges);

//     std::vector<std::vector<std::vector<int>>> compute_all_shortest_paths(
//         const std::unordered_set<int>& terminals, 
//         const std::vector<std::vector<int>>& edges
//     );

//     // Override routeSingleNet to use Steiner Tree algorithm
//     NetRoute routeSingleNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) override;
    
//     // Override routeAllNets to use Steiner Tree algorithm
//     void routeAllNets(std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) override;
// };

#endif // ROUTER_HPP
