#include "SteinerArborescence.hpp"
#include <queue>
#include <set>
#include <iostream>
#include <chrono>
#include <functional>
#include <cmath>

// Constructor
SteinerArborescence::SteinerArborescence(const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes)
    : edges(edges), 
      nodes(nodes), 
      timeoutMs(5000),
      congestionPenaltyFactor(1.0)
{
    // Initialize node ID to index map
    nodeIdToIndex.reserve(nodes.size());
    for (size_t i = 0; i < nodes.size(); ++i) {
        nodeIdToIndex[nodes[i].id] = i;
        
        // Build coordinate to node ID map for efficient spatial lookups
        int x = nodes[i].beginX;
        int y = nodes[i].beginY;
        coordToNodeId[x][y] = nodes[i].id;
    }
    
    // Initialize congestion map
    resetCongestion();
}

// Set timeout in milliseconds
void SteinerArborescence::setTimeout(int milliseconds) {
    timeoutMs = milliseconds;
}

// Set congestion penalty factor
void SteinerArborescence::setCongestionPenaltyFactor(double factor) {
    congestionPenaltyFactor = factor;
}

// Update congestion map after routing a path
void SteinerArborescence::updateCongestion(const std::vector<int>& path, double congestionIncrement) {
    for (int nodeId : path) {
        congestionMap[nodeId] += congestionIncrement;
    }
}

// Reset congestion map
void SteinerArborescence::resetCongestion() {
    congestionMap.clear();
}

// Get current congestion at a node
double SteinerArborescence::getCongestion(int nodeId) const {
    auto it = congestionMap.find(nodeId);
    return (it != congestionMap.end()) ? it->second : 0.0;
}

// Helper method to get node coordinates by ID
std::pair<int, int> SteinerArborescence::getNodeCoordinates(int nodeId) {
    auto it = nodeIdToIndex.find(nodeId);
    if (it == nodeIdToIndex.end()) {
        // Return invalid coordinates if node not found
        return {std::numeric_limits<int>::min(), std::numeric_limits<int>::min()};
    }
    
    const Node& node = nodes[it->second];
    return {node.beginX, node.beginY};
}

// Helper method to find the node in the graph closest to the given coordinates
int SteinerArborescence::findClosestNode(int x, int y) {
    // First check if there's a node exactly at these coordinates
    auto xIt = coordToNodeId.find(x);
    if (xIt != coordToNodeId.end()) {
        auto yIt = xIt->second.find(y);
        if (yIt != xIt->second.end()) {
            return yIt->second;
        }
    }
    
    // Otherwise find the closest node using Manhattan distance
    int closestId = -1;
    int minDistance = std::numeric_limits<int>::max();
    
    for (const auto& node : nodes) {
        int distance = std::abs(node.beginX - x) + std::abs(node.beginY - y);
        if (distance < minDistance) {
            minDistance = distance;
            closestId = node.id;
        }
    }
    
    return closestId;
}

// Connect two nodes with a rectilinear path (Manhattan distance)
std::vector<int> SteinerArborescence::connectNodesRectilinear(int fromId, int toId) {
    auto fromCoords = getNodeCoordinates(fromId);
    auto toCoords = getNodeCoordinates(toId);
    
    if (fromCoords.first == std::numeric_limits<int>::min() || 
        toCoords.first == std::numeric_limits<int>::min()) {
        // One or both nodes not found
        return {};
    }
    
    int fromX = fromCoords.first;
    int fromY = fromCoords.second;
    int toX = toCoords.first;
    int toY = toCoords.second;
    
    // Create path of nodes following Manhattan distance
    std::vector<int> path;
    path.push_back(fromId);
    
    // If nodes are the same, return just that node
    if (fromId == toId) {
        return path;
    }
    
    // First go horizontally, then vertically
    // Find intermediate nodes if they exist
    if (fromX != toX) {
        int x = toX;
        int y = fromY;
        
        // Find a node at this intermediate point if it exists
        int intermediateId = findClosestNode(x, y);
        if (intermediateId != -1 && intermediateId != fromId && intermediateId != toId) {
            // Recursively connect from -> intermediate -> to
            auto path1 = connectNodesRectilinear(fromId, intermediateId);
            auto path2 = connectNodesRectilinear(intermediateId, toId);
            
            // Combine paths (remove duplicate intermediate node)
            path1.pop_back();
            path1.insert(path1.end(), path2.begin(), path2.end());
            return path1;
        }
    }
    
    // Add destination node
    path.push_back(toId);
    return path;
}

// Internal implementation of the RSA algorithm
SteinerTree SteinerArborescence::buildRectilinearSteinerArborescence(
    int sourceX, int sourceY, 
    const std::vector<std::pair<int, int>>& sinkCoordinates,
    const std::vector<int>& sinkIds)
{
    // Check if we have sinks
    if (sinkCoordinates.empty()) {
        return SteinerTree();
    }
    
    // Start timing to enforce timeout
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // 1. Initialize S as an empty set of subtrees
    std::vector<SteinerTree> S;
    
    // 2. Insert into S n trees where each tree t_i is a single node v_i
    for (size_t i = 0; i < sinkCoordinates.size(); ++i) {
        auto [x, y] = sinkCoordinates[i];
        int id = (i < sinkIds.size()) ? sinkIds[i] : -1;
        
        auto node = std::make_shared<SteinerNode>(x, y, id, true);
        S.push_back(SteinerTree(node));
    }
    
    // 3. While |S| > 1 do
    while (S.size() > 1) {
        // Check for timeout periodically
        auto currentTime = std::chrono::high_resolution_clock::now();
        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            currentTime - startTime).count();
        
        if (elapsedMs > timeoutMs) {
            std::cerr << "Timeout in Steiner tree construction" << std::endl;
            break;
        }
        
        // 4. Find 2 trees t_j and t_k in S s.t. x_r + y_r is maximum where
        //    x_r = min{x_j, x_k} and y_r = min{y_j, y_k}
        double maxSum = -std::numeric_limits<double>::max();
        size_t bestJ = 0, bestK = 0;
        
        for (size_t j = 0; j < S.size(); ++j) {
            for (size_t k = j + 1; k < S.size(); ++k) {
                if (!S[j].root || !S[k].root) continue;
                
                int xj = S[j].root->x;
                int yj = S[j].root->y;
                int xk = S[k].root->x;
                int yk = S[k].root->y;
                
                int xr = std::min(xj, xk);
                int yr = std::min(yj, yk);
                
                // Calculate sum with congestion penalty
                double sum = xr + yr;
                
                // Find the closest actual node to these coordinates
                int nodeId = findClosestNode(xr, yr);
                if (nodeId != -1) {
                    // Apply congestion penalty
                    double congestion = getCongestion(nodeId);
                    sum -= congestion * congestionPenaltyFactor;
                }
                
                if (sum > maxSum) {
                    maxSum = sum;
                    bestJ = j;
                    bestK = k;
                }
            }
        }
        
        // 5. Merge t_j and t_k to create a new tree t with root r = (x_r, y_r)
        if (bestJ < S.size() && bestK < S.size() && S[bestJ].root && S[bestK].root) {
            int xj = S[bestJ].root->x;
            int yj = S[bestJ].root->y;
            int xk = S[bestK].root->x;
            int yk = S[bestK].root->y;
            
            int xr = std::min(xj, xk);
            int yr = std::min(yj, yk);
            
            // Create new Steiner point as the root
            auto newRoot = std::make_shared<SteinerNode>(xr, yr);
            
            // Find closest actual node to this Steiner point
            int closestNodeId = findClosestNode(xr, yr);
            if (closestNodeId != -1) {
                newRoot->id = closestNodeId;
            }
            
            // Add the two trees as children
            newRoot->children.push_back(S[bestJ].root);
            newRoot->children.push_back(S[bestK].root);
            
            // Create the new tree
            SteinerTree newTree(newRoot);
            
            // 6. Add t to S. Remove t_j and t_k from S.
            S.erase(S.begin() + std::max(bestJ, bestK));
            S.erase(S.begin() + std::min(bestJ, bestK));
            S.push_back(newTree);
        } else {
            // Something went wrong, break to avoid infinite loop
            break;
        }
    }
    
    // 7. Construct a tree T by connecting (0, 0) to the root of the last tree in S
    if (!S.empty() && S[0].root) {
        // Create the source node at (0, 0)
        auto sourceNode = std::make_shared<SteinerNode>(sourceX, sourceY);
        
        // Find closest actual node to the source coordinates
        int closestNodeId = findClosestNode(sourceX, sourceY);
        if (closestNodeId != -1) {
            sourceNode->id = closestNodeId;
        }
        
        // Add the remaining tree as a child
        sourceNode->children.push_back(S[0].root);
        
        // Return the final tree
        return SteinerTree(sourceNode);
    } else {
        // Return an empty tree if something went wrong
        return SteinerTree();
    }
}

// Convert Steiner tree to a path in the original graph
std::vector<int> SteinerArborescence::steinerTreeToPath(const SteinerTree& tree, int sourceNodeId) {
    if (!tree.root) {
        return {};
    }
    
    std::vector<int> path;
    std::set<int> visitedNodes; // To avoid duplicates
    
    // Helper function to traverse the tree and build the path
    std::function<void(std::shared_ptr<SteinerNode>, int)> traverse = 
        [&](std::shared_ptr<SteinerNode> node, int parentId) {
            if (!node) return;
            
            int currentId = node->id;
            if (currentId == -1) {
                // This is a Steiner point without a corresponding node
                // Find the closest actual node
                currentId = findClosestNode(node->x, node->y);
                if (currentId == -1) return; // Cannot map this Steiner point
            }
            
            // Connect parent to current node with a rectilinear path
            if (parentId != -1 && parentId != currentId) {
                auto segment = connectNodesRectilinear(parentId, currentId);
                
                // Add all nodes except the last one (to avoid duplicates)
                for (size_t i = 0; i < segment.size() - 1; ++i) {
                    int nodeId = segment[i];
                    if (visitedNodes.insert(nodeId).second) {
                        path.push_back(nodeId);
                    }
                }
            }
            
            // Add current node if not already visited
            if (visitedNodes.insert(currentId).second) {
                path.push_back(currentId);
            }
            
            // Recursively process children
            for (const auto& child : node->children) {
                traverse(child, currentId);
            }
        };
    
    // Start traversal from the source
    traverse(tree.root, -1);
    
    return path;
}

// Find a Steiner arborescence connecting source to multiple sinks
std::vector<int> SteinerArborescence::findSteinerTree(int sourceNodeId, const std::vector<int>& sinkNodeIds) {
    // Get source coordinates
    auto sourceCoords = getNodeCoordinates(sourceNodeId);
    if (sourceCoords.first == std::numeric_limits<int>::min()) {
        // Source node not found
        return {};
    }
    
    // Get sink coordinates
    std::vector<std::pair<int, int>> sinkCoordinates;
    sinkCoordinates.reserve(sinkNodeIds.size());
    
    std::vector<int> validSinkIds;
    validSinkIds.reserve(sinkNodeIds.size());
    
    for (int sinkId : sinkNodeIds) {
        auto coords = getNodeCoordinates(sinkId);
        if (coords.first != std::numeric_limits<int>::min()) {
            // Valid sink node
            sinkCoordinates.push_back(coords);
            validSinkIds.push_back(sinkId);
        }
    }
    
    // Build the rectilinear Steiner arborescence
    SteinerTree steinerTree = buildRectilinearSteinerArborescence(
        sourceCoords.first, sourceCoords.second, 
        sinkCoordinates, validSinkIds);
    
    // Convert the Steiner tree to a path in the original graph
    std::vector<int> path = steinerTreeToPath(steinerTree, sourceNodeId);
    
    return path;
} 