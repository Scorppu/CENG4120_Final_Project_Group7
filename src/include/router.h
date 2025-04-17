#ifndef ROUTER_H
#define ROUTER_H

#include "include/input.h"
#include <atomic>
#include <unordered_map>
#include <mutex>

// Global variables for routing
extern std::atomic<bool> timedOut;  // Time out flag
extern std::unordered_map<int, int> nodeUsageCount;  // Node ID -> usage count
extern std::unordered_map<int, double> nodeHistoryCost;  // Node ID -> history cost
extern std::mutex nodeUsageMutex;  // Mutex for thread safety

// Parameters for PathFinder algorithm
extern const double BASE_COST;
extern const double HISTORY_FACTOR_INITIAL;
extern const double HISTORY_FACTOR_INCREMENT;
extern const double PRESENT_CONGESTION_PENALTY;
extern const int MAX_ITERATIONS;

// PathFinder algorithm functions
double calculateNodeCost(int nodeId, bool ignoreCongestion = false);
bool routeNetToSink(const Net& net, int sinkNodeId, const std::unordered_map<int, bool>& occupiedNodes,
                   std::unordered_map<int, int>& parentMap, bool ignoreCongestion = false);
std::vector<int> extractPath(int sourceNodeId, int sinkNodeId, const std::unordered_map<int, int>& parentMap);
bool routeNet(Net& net, std::unordered_map<int, bool>& occupiedNodes, bool ignoreCongestion = false);
void runPathFinder(int startNetIndex, int endNetIndex);
void parallelPathFinder(int numThreads);

// Analysis functions
bool hasCongestion();
int calculateWirelength();
int countRoutedNets();

#endif // ROUTER_H 