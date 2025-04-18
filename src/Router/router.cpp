 #include <iostream>
 #include <string>
 #include <vector>
 #include <algorithm>
 #include <cmath>
 #include <unordered_map>
 #include <unordered_set>
 #include <queue>
 #include "../Datastructure.hpp"
 
class Router {
    private:
        std::vector<Node> nodes;
        std::vector<std::vector<int>> edges;
        std::vector<Net> nets;

            
    public:
        Router(const std::vector<Node>& nodes, const std::vector<std::vector<int>>& edges, const std::vector<Net>& nets) {
            this->nodes = nodes;
            this->edges = edges;
            this->nets = nets;
        }

        // Resolve congestions (Rip up and reroute)
        virtual void resolveCongestion() {
            // Implement congestion resolution logic here
        }

        // Route a single net (A* Search)
        virtual bool routeSingleNet(const Net& net) {

            // Structures to track costs, parents, and used nodes
            std::unordered_map<int, int> costMap; // nodeId -> lowest known cost
            std::unordered_map<int, int> parentMap; // nodeId -> parent nodeId
            std::unordered_set<int> usedNodes; // nodes already occupied by other nets
            std::unordered_set<int> remainingSinks; // sinks not yet routed


            // Initialize Priority Queue sorted by path cost (wirelength + congestion)
            using QueueElement = std::pair<int,int>;
            std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<>> openQueue;

            // Initialize with the source node
            int sourceNodeId = net.nodeIDs[0];
            openQueue.emplace(0, sourceNodeId);
            costMap[sourceNodeId] = 0;
            parentMap[sourceNodeId] = -1; // no parent for the source node
            usedNodes.insert(sourceNodeId); // source node is occupied
            remainingSinks.insert(net.nodeIDs.begin() + 1, net.nodeIDs.end()); // all other nodes are remaining sinks
            
            
            while (!openQueue.empty() && !remainingSinks.empty()) {
                auto [currentCost, currentNodeId] = openQueue.top();
                openQueue.pop();

                // Skip if not the lowest cost path
                if (currentCost > costMap[currentNodeId]) continue;

                if (remainingSinks.erase(currentNodeId) > 0){
                    // Handle backtracking later

                    if (remainingSinks.empty()) {
                        // All sinks routed successfully
                        break;
                    }
                }

                for (int childNodeId : edges[currentNodeId]) {
                    // Skip if node is already used by another net 
                    if (usedNodes.contains(childNodeId)) continue;

                    // Calculate new cost (wirelength + congestion)
                }
            }
        }

        virtual void routeAllNets(const std::vector<Net>& nets) {
            for (const Net& net : nets) {
                routeSingleNet(net);
            }

            // resolve congestions
            // resolveCongestion();
        }   
};