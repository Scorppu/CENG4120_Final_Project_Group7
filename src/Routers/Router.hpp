#ifndef ROUTER_HPP
#define ROUTER_HPP

#include <vector>
#include <memory>
#include <unordered_set>
#include <chrono>
#include <tuple>
#include "../DataStructure.hpp"
#include "../PathfindingAlgorithms/AStarSearch.hpp"

// Routing accuracy levels
enum AccuracyLevel {
    LOW,    // Fast, less accurate
    MEDIUM, // Balanced
    HIGH    // Slow, most accurate
};

// Define Region struct for congested regions
struct Region {
    int minX, minY, maxX, maxY;
    std::vector<int> nodeIds;
};

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

    // History-based congestion tracking
    std::unordered_map<int, int> nodeHistoryCongestion;

    // Design-specific settings
    int designNumber;     // Current design number (1-5)
    int timeoutSeconds;   // Global timeout in seconds
    
    // Program start time
    std::chrono::time_point<std::chrono::steady_clock> programStartTime;
    bool hasProgramStartTime;

    // Advanced congestion resolution methods
    void localRipUpAndReroute(const std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes);
    void globalRipUpAndReroute(const std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes);
    void handleDifficultNets(const std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes);
    void eliminateRemainingCongestion(const std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes);
    
    // Validation and evaluation
    bool validateNetRoute(const NetRoute& netRoute, const Net& net) const;
    void evaluateRouting(const std::vector<Net>& nets) const;
    void compareRoutingToEval(const std::vector<Net>& nets) const; // Compare router results with evaluation tool results
    
    // Congestion helper methods
    void updateCongestionHistory();
    void findCongestedRegions(const std::vector<Node>& nodes, std::vector<std::tuple<int, int, int, int>>& congestedBoxes);
    std::vector<Region> findCongestedRegionsGrouped();
    void printCongestionStats();
    bool isNetInCongestionHotspot(int netId, const std::vector<Region>& hotspots);
    double calculateCongestionScore(int netId, const std::vector<int>& path) const;
    std::vector<Net> orderNetsForRerouting(const std::vector<Net>& nets, int iteration, int congestedNodeCount);
    double getNodeCongestionCost(int nodeId);
    double historyPenaltyFactor(int netId);
    void adjustRoutingEffort(int iteration, int congestedNodeCount);
    
    // Helper methods
    double calculateDistance(int fromId, int toId, const std::vector<Node>& nodes);
    double computePriority(const Net& net, const std::vector<Node>& nodes, std::map<int, std::vector<int>>& x_to_ys);
    double getNetCongestionScore(int netId);
    
    // State management
    void clearRoutingResults() { routingResults.clear(); }
    void clearPathfinder() { pathfinder.reset(nullptr); }
    void clearAll() { clearRoutingResults(); clearPathfinder(); }
    
public:
    // Constructor
    Router() : pathfinder(nullptr), designNumber(0), timeoutSeconds(600), hasProgramStartTime(false) {}
    
    // Destructor
    virtual ~Router() {}
    
    // Get routing results for writing
    const std::vector<NetRoute>& getRoutingResults() const { return routingResults; }
    
    // Set design number for design-specific optimizations
    void setDesignNumber(int number);
    
    // Set global timeout for the entire routing process (in seconds)
    void setTimeout(int seconds);
    
    // Set the program start time for global timeout calculation
    void setProgramStartTime(std::chrono::time_point<std::chrono::steady_clock> startTime);

    // Resolve congestions (Rip up and reroute)
    virtual void resolveCongestion(const std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes);

    // Route a single net using A* Search
    virtual NetRoute routeSingleNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes);

    // Pattern routing for difficult nets as fallback
    NetRoute patternRouteSingleNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes);
    
    // Try L-shape routing
    bool tryLShapeRoute(const Node& sourceNode, const Node& sinkNode, std::vector<int>& path, bool horizontalFirst, const std::vector<Node>& nodes);
    
    // Try Z-shape routing
    bool tryZShapeRoute(const Node& sourceNode, const Node& sinkNode, std::vector<int>& path, const std::vector<Node>& nodes);
    
    // Route with relaxed constraints for difficult nets
    NetRoute routeDifficultNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes);
    
    // Progressive routing for multi-sink nets
    bool progressiveRouting(Net& net, NetRoute& netRoute, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes);

    // Helper method to calculate cost between nodes (includes congestion awareness)
    double calculateCost(int fromId, int toId);

    // Route all nets
    virtual void routeAllNets(std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes, std::map<int, std::vector<int>>& x_to_ys);
    
    // Print routing results
    void printRoutingResults() const;
    
    // Extract nodes from a route's edges
    std::vector<int> extractNodesFromRoute(const NetRoute& route) const;
};

#endif // ROUTER_HPP
