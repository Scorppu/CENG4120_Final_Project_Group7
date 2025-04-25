#ifndef ROUTER_HPP
#define ROUTER_HPP

#include <vector>
#include <memory>
#include <unordered_set>
#include "../Datastructure.hpp"
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
public:
    // Constructor
    Router();
    
    // Destructor that properly cleans up resources
    ~Router();

    // Calculate distance between two nodes
    double calculateDistance(int fromId, int toId, const std::vector<Node>& nodes);

    // Compute priority for a net
    double computePriority(const Net& net, const std::vector<Node>& nodes);
    
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

    // Resolve congestions (Rip up and reroute)
    virtual void resolveCongestion();

    // Route a single net using A* Search
    virtual NetRoute routeSingleNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes);

    // Helper method to calculate cost between nodes (includes congestion awareness)
    double calculateCost(int fromId, int toId);

    // Route all nets
    virtual void routeAllNets(std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes);
    
    // Print routing results
    void printRoutingResults() const;
};

// Derived class for Steiner Tree routing
class SteinerTreeRouter : public Router {
private:
    // Collection of routing results using the simpler NetRoute structure
    std::vector<NetRoute> routingResults;
    
    // Shared pathfinding instance to avoid costly recreation
    std::unique_ptr<AStarSearch> pathfinder;
    
    // Reusable data structures to avoid repeated allocation
    std::unordered_set<int> existingNodeIds;
    std::vector<int> validSinkNodeIds;
public:
    // Constructor
    SteinerTreeRouter();
    
    std::vector<int> get_shortest_path(int src, int dest, const std::vector<std::vector<int>>& edges);

    std::vector<std::vector<std::vector<int>>> compute_all_shortest_paths(
        const std::unordered_set<int>& terminals, 
        const std::vector<std::vector<int>>& edges
    );

    // Override routeSingleNet to use Steiner Tree algorithm
    NetRoute routeSingleNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) override;
    
    // Override routeAllNets to use Steiner Tree algorithm
    void routeAllNets(std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) override;
};

#endif // ROUTER_HPP
