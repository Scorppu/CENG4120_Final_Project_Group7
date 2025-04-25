#include "SteinerSALT.hpp"
#include <iostream>
#include <chrono>
#include <functional>
#include <queue>
#include <set>
#include <limits>
#include <cmath>
#include <algorithm>

// Default constructor
SteinerSALT::SteinerSALT()
    : epsilon(0.5),     // Default trade-off parameter
      timeoutMs(5000),  // Default timeout: 5 seconds
      congestionPenaltyFactor(1.0),
      verbose(false)
{
    // Empty initialization - no graph to work with yet
    resetCongestion();
}

// Constructor
SteinerSALT::SteinerSALT(const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes)
    : edges(edges), 
      nodes(nodes), 
      epsilon(0.5),     // Default trade-off parameter
      timeoutMs(5000),  // Default timeout: 5 seconds
      congestionPenaltyFactor(1.0),
      verbose(false)
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

// Destructor
SteinerSALT::~SteinerSALT() {
    // Nothing to do here - all memory is managed by smart pointers or the stack
}

// Set trade-off parameter epsilon
void SteinerSALT::setEpsilon(double eps) {
    epsilon = eps;
}

// Set timeout in milliseconds
void SteinerSALT::setTimeout(int milliseconds) {
    timeoutMs = milliseconds;
}

// Set congestion penalty factor
void SteinerSALT::setCongestionPenaltyFactor(double factor) {
    congestionPenaltyFactor = factor;
}

// Set verbosity
void SteinerSALT::setVerbose(bool v) {
    verbose = v;
}

// Update congestion map after routing a path
void SteinerSALT::updateCongestion(const std::vector<int>& path, double congestionIncrement) {
    // Only update congestion if path is valid
    if (path.empty()) {
        return;
    }
    
    // Verify all nodes in path are legal before updating congestion
    for (int nodeId : path) {
        if (!isLegalNode(nodeId)) {
            return; // Don't update congestion if any node is illegal
        }
    }
    
    // Update congestion for all nodes in the path
    for (int nodeId : path) {
        congestionMap[nodeId] += congestionIncrement;
    }
}

// Reset congestion map
void SteinerSALT::resetCongestion() {
    congestionMap.clear();
}

// Get current congestion at a node
double SteinerSALT::getCongestion(int nodeId) const {
    auto it = congestionMap.find(nodeId);
    return (it != congestionMap.end()) ? it->second : 0.0;
}

// Helper method to get node coordinates by ID
std::pair<int, int> SteinerSALT::getNodeCoordinates(int nodeId) const {
    auto it = nodeIdToIndex.find(nodeId);
    if (it == nodeIdToIndex.end()) {
        // Return invalid coordinates if node not found
        return {std::numeric_limits<int>::min(), std::numeric_limits<int>::min()};
    }
    
    const Node& node = nodes[it->second];
    return {node.beginX, node.beginY};
}

// Helper method to validate if a node is legal for routing
bool SteinerSALT::isLegalNode(int nodeId) const {
    // Check if node exists in the graph
    auto it = nodeIdToIndex.find(nodeId);
    if (it == nodeIdToIndex.end()) {
        return false;
    }
    
    // For now, all existing nodes are considered legal
    // With obstacle handling, we'll check if the node is in an obstacle
    auto coords = getNodeCoordinates(nodeId);
    if (coords.first == std::numeric_limits<int>::min()) {
        return false; // Invalid node
    }
    
    // Check if the node is inside any obstacle
    if (isPointInObstacle(coords.first, coords.second)) {
        return false; // Node is inside an obstacle
    }
    
    return true;
}

// Helper method to find the node in the graph closest to the given coordinates
int SteinerSALT::findClosestNode(int x, int y) const {
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

// Helper method to find the closest legal node to the given coordinates
int SteinerSALT::findClosestLegalNode(int x, int y) const {
    // First check if there's a node exactly at these coordinates and if it's legal
    auto xIt = coordToNodeId.find(x);
    if (xIt != coordToNodeId.end()) {
        auto yIt = xIt->second.find(y);
        if (yIt != xIt->second.end() && isLegalNode(yIt->second)) {
            return yIt->second;
        }
    }
    
    // Otherwise find the closest legal node using Manhattan distance
    int closestId = -1;
    int minDistance = std::numeric_limits<int>::max();
    
    for (const auto& node : nodes) {
        if (!isLegalNode(node.id)) {
            continue;
        }
        
        int distance = std::abs(node.beginX - x) + std::abs(node.beginY - y);
        if (distance < minDistance) {
            minDistance = distance;
            closestId = node.id;
        }
    }
    
    return closestId;
}

// Helper method to calculate Manhattan distance
int SteinerSALT::manhattanDistance(int x1, int y1, int x2, int y2) const {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

// Manhattan distance between two nodes
int SteinerSALT::manhattanDistance(int nodeId1, int nodeId2) const {
    auto coords1 = getNodeCoordinates(nodeId1);
    auto coords2 = getNodeCoordinates(nodeId2);
    
    if (coords1.first == std::numeric_limits<int>::min() || 
        coords2.first == std::numeric_limits<int>::min()) {
        return std::numeric_limits<int>::max();
    }
    
    return manhattanDistance(coords1.first, coords1.second, coords2.first, coords2.second);
}

// Convert coordinates to a unique integer identifier
int64_t SteinerSALT::coordToId(int x, int y) const {
    // Combine x and y into a single integer, ensuring uniqueness
    return (static_cast<int64_t>(x) << 32) | static_cast<uint32_t>(y);
}

// Add obstacle to the graph
void SteinerSALT::addObstacle(const Obstacle& obstacle) {
    obstacles.push_back(obstacle);
}

// Clear all obstacles
void SteinerSALT::clearObstacles() {
    obstacles.clear();
}

// Check if a point is inside any obstacle
bool SteinerSALT::isPointInObstacle(int x, int y) const {
    for (const auto& obstacle : obstacles) {
        if (obstacle.contains(x, y)) {
            return true;
        }
    }
    return false;
}

// Check if a line segment intersects any obstacle
bool SteinerSALT::doesLineIntersectObstacle(int x1, int y1, int x2, int y2) const {
    for (const auto& obstacle : obstacles) {
        if (obstacle.intersectsLine(x1, y1, x2, y2)) {
            return true;
        }
    }
    return false;
}

// Check if an edge between two nodes crosses an obstacle
bool SteinerSALT::edgeCrossesObstacle(int fromId, int toId) const {
    auto fromCoords = getNodeCoordinates(fromId);
    auto toCoords = getNodeCoordinates(toId);
    
    if (fromCoords.first == std::numeric_limits<int>::min() || 
        toCoords.first == std::numeric_limits<int>::min()) {
        return true; // Consider invalid nodes as blocked
    }
    
    return doesLineIntersectObstacle(fromCoords.first, fromCoords.second, 
                                    toCoords.first, toCoords.second);
}

// Get rectilinear neighbors (nodes in the four cardinal directions)
std::vector<int> SteinerSALT::getRectilinearNeighbors(int nodeId) const {
    std::vector<int> neighbors;
    auto coords = getNodeCoordinates(nodeId);
    if (coords.first == std::numeric_limits<int>::min()) {
        return neighbors; // Invalid node
    }
    
    int x = coords.first;
    int y = coords.second;
    
    // Check in the four cardinal directions
    std::vector<std::pair<int, int>> directions = {
        {x+1, y}, {x-1, y}, {x, y+1}, {x, y-1}
    };
    
    for (const auto& dir : directions) {
        int neighborId = findClosestNode(dir.first, dir.second);
        if (neighborId != -1 && isLegalNode(neighborId)) {
            neighbors.push_back(neighborId);
        }
    }
    
    return neighbors;
}

// Get the cost of an edge (including congestion)
double SteinerSALT::getEdgeCost(int fromId, int toId) const {
    // Base cost is the Manhattan distance
    double baseCost = manhattanDistance(fromId, toId);
    
    // Add congestion penalty
    double congestion = (getCongestion(fromId) + getCongestion(toId)) / 2.0;
    double congestionPenalty = congestion * congestionPenaltyFactor;
    
    return baseCost + congestionPenalty;
}

// Find all corner points of obstacles
std::vector<Point> SteinerSALT::findObstacleCorners() const {
    std::vector<Point> corners;
    for (const auto& obstacle : obstacles) {
        // Add the four corners of the rectangle
        corners.push_back(Point(obstacle.x1, obstacle.y1)); // Top-left
        corners.push_back(Point(obstacle.x2, obstacle.y1)); // Top-right
        corners.push_back(Point(obstacle.x1, obstacle.y2)); // Bottom-left
        corners.push_back(Point(obstacle.x2, obstacle.y2)); // Bottom-right
    }
    return corners;
}

// Create a new node or find an existing one at the given coordinates
int SteinerSALT::findOrCreateNode(int x, int y) {
    // First check if there's a node exactly at these coordinates
    auto xIt = coordToNodeId.find(x);
    if (xIt != coordToNodeId.end()) {
        auto yIt = xIt->second.find(y);
        if (yIt != xIt->second.end()) {
            return yIt->second;
        }
    }
    
    // Create a new node
    int newId = nodes.size() > 0 ? nodes.back().id + 1 : 0;
    Node newNode;
    newNode.id = newId;
    newNode.beginX = x;
    newNode.beginY = y;
    
    // Add to the graph
    nodes.push_back(newNode);
    nodeIdToIndex[newId] = nodes.size() - 1;
    coordToNodeId[x][y] = newId;
    
    return newId;
}

// Line intersection check between two SALTNode pairs
bool SteinerSALT::edgesIntersect(const SALTNode* a1, const SALTNode* a2, 
                              const SALTNode* b1, const SALTNode* b2) const {
    // Implement line segment intersection algorithm
    // For simplicity, check if the two line segments intersect
    // This is a placeholder - actual implementation would use proper line intersection math
    int a1x = a1->x, a1y = a1->y;
    int a2x = a2->x, a2y = a2->y;
    int b1x = b1->x, b1y = b1->y;
    int b2x = b2->x, b2y = b2->y;
    
    // Check if the lines are parallel
    int dxa = a2x - a1x;
    int dya = a2y - a1y;
    int dxb = b2x - b1x;
    int dyb = b2y - b1y;
    
    int crossProduct = dxa * dyb - dya * dxb;
    if (crossProduct == 0) {
        return false; // Parallel lines don't intersect for our purposes
    }
    
    // Find intersection point
    double t = ((b1x - a1x) * dyb - (b1y - a1y) * dxb) / static_cast<double>(crossProduct);
    double u = ((b1x - a1x) * dya - (b1y - a1y) * dxa) / static_cast<double>(crossProduct);
    
    // Check if intersection point is within both line segments
    return (t >= 0 && t <= 1 && u >= 0 && u <= 1);
}

// Find intersection point of two line segments
Point SteinerSALT::findIntersection(const SALTNode* a1, const SALTNode* a2, 
                                 const SALTNode* b1, const SALTNode* b2) const {
    // Implement line segment intersection to find the intersection point
    // This is a placeholder - actual implementation would use proper line intersection math
    int a1x = a1->x, a1y = a1->y;
    int a2x = a2->x, a2y = a2->y;
    int b1x = b1->x, b1y = b1->y;
    int b2x = b2->x, b2y = b2->y;
    
    // Calculate determinants
    int dxa = a2x - a1x;
    int dya = a2y - a1y;
    int dxb = b2x - b1x;
    int dyb = b2y - b1y;
    
    int crossProduct = dxa * dyb - dya * dxb;
    if (crossProduct == 0) {
        // Parallel lines - return midpoint of one line as a fallback
        return Point((a1x + a2x) / 2, (a1y + a2y) / 2);
    }
    
    // Find intersection point
    double t = ((b1x - a1x) * dyb - (b1y - a1y) * dxb) / static_cast<double>(crossProduct);
    
    // Calculate intersection coordinates
    int ix = a1x + static_cast<int>(t * dxa);
    int iy = a1y + static_cast<int>(t * dya);
    
    return Point(ix, iy);
}

// Implementation of the line-obstacle intersection
bool Obstacle::intersectsLine(int sx, int sy, int tx, int ty) const {
    // Check if either endpoint is inside the obstacle
    if (contains(sx, sy) || contains(tx, ty)) {
        return true;
    }
    
    // Check if the line intersects any of the four edges of the obstacle
    
    // Line parameters
    int dx = tx - sx;
    int dy = ty - sy;
    
    // Time values for intersections
    double tmin = 0.0;
    double tmax = 1.0;
    
    // Check horizontal edges
    if (dx != 0) {
        double txMin = static_cast<double>(x1 - sx) / dx;
        double txMax = static_cast<double>(x2 - sx) / dx;
        
        if (txMin > txMax) std::swap(txMin, txMax);
        
        tmin = std::max(tmin, txMin);
        tmax = std::min(tmax, txMax);
        
        if (tmin > tmax) return false;
    } else if (sx < x1 || sx > x2) {
        // Line is vertical and outside horizontal bounds
        return false;
    }
    
    // Check vertical edges
    if (dy != 0) {
        double tyMin = static_cast<double>(y1 - sy) / dy;
        double tyMax = static_cast<double>(y2 - sy) / dy;
        
        if (tyMin > tyMax) std::swap(tyMin, tyMax);
        
        tmin = std::max(tmin, tyMin);
        tmax = std::min(tmax, tyMax);
        
        if (tmin > tmax) return false;
    } else if (sy < y1 || sy > y2) {
        // Line is horizontal and outside vertical bounds
        return false;
    }
    
    return true; // The line intersects the obstacle
}

// SALT algorithm implementation (Algorithm 2)
SALTTree* SteinerSALT::buildSALT(int sourceNodeId, const std::vector<int>& sinkNodeIds) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Step 1: Initialize
    std::vector<int> allNodeIds = {sourceNodeId};
    allNodeIds.insert(allNodeIds.end(), sinkNodeIds.begin(), sinkNodeIds.end());
    
    // Step 2: Build MST
    SALTTree* mst = buildMST(sourceNodeId, allNodeIds);
    if (!mst || !mst->root) {
        std::cerr << "Failed to build MST" << std::endl;
        return nullptr;
    }
    
    // Initialize result tree
    SALTTree* result = new SALTTree();
    result->root = new SALTNode(sourceNodeId);
    auto rootCoords = getNodeCoordinates(sourceNodeId);
    result->root->x = rootCoords.first;
    result->root->y = rootCoords.second;
    result->root->distance = 0;  // Distance from root to itself is 0
    result->nodes.push_back(result->root);
    result->terminals.push_back(result->root);
    
    // Step 3: Perform DFS
    std::vector<SALTNode*> breakpoints;
    dfs(mst->root, mst, result, breakpoints);
    
    // Step 4-5: Build Steiner SPT for breakpoints and combine
    if (!breakpoints.empty()) {
        SALTTree* steinerSPT = buildLightSteinerSPT(sourceNodeId, breakpoints);
        if (steinerSPT) {
            // Merge trees
            // This is a simplified approach - in practice, we need to carefully merge the trees
            for (auto node : steinerSPT->nodes) {
                if (node != steinerSPT->root) { // Skip root, as it's already in result
                    result->nodes.push_back(node);
                    if (node->id != -1) { // not a Steiner point
                        result->terminals.push_back(node);
                    } else {
                        result->steinerPoints.push_back(node);
                    }
                }
            }
            
            // Clean up, but keep the nodes (they're now part of result)
            steinerSPT->nodes.clear();
            delete steinerSPT;
        }
    }
    
    // Clean up MST
    delete mst;
    
    // Do refinement (Algorithm 5)
    refineSALT(result);
    
    return result;
}

// Initialize MST (for SALT algorithm)
SALTTree* SteinerSALT::buildMST(int sourceNodeId, const std::vector<int>& nodeIds) {
    // Implementation of Prim's algorithm for MST
    SALTTree* tree = new SALTTree();
    if (nodeIds.empty()) {
        return tree;
    }
    
    // Create nodes for all terminals
    std::unordered_map<int, SALTNode*> idToNode;
    for (int id : nodeIds) {
        auto coords = getNodeCoordinates(id);
        if (coords.first != std::numeric_limits<int>::min()) {
            SALTNode* node = new SALTNode(id, coords.first, coords.second);
            idToNode[id] = node;
            tree->nodes.push_back(node);
            tree->terminals.push_back(node);
        }
    }
    
    // Check if we have the source node
    if (idToNode.find(sourceNodeId) == idToNode.end()) {
        std::cerr << "Source node not found" << std::endl;
        delete tree;
        return nullptr;
    }
    
    // Initialize distances for Prim's algorithm
    for (auto& pair : idToNode) {
        pair.second->distance = std::numeric_limits<double>::infinity();
    }
    
    // Start from source node
    tree->root = idToNode[sourceNodeId];
    tree->root->distance = 0;
    
    // Priority queue for Prim's algorithm
    auto cmp = [](SALTNode* a, SALTNode* b) { return a->distance > b->distance; };
    std::priority_queue<SALTNode*, std::vector<SALTNode*>, decltype(cmp)> pq(cmp);
    pq.push(tree->root);
    
    // Set to keep track of processed nodes
    std::set<int> processed;
    
    while (!pq.empty()) {
        SALTNode* current = pq.top();
        pq.pop();
        
        if (processed.find(current->id) != processed.end()) {
            continue;  // Skip if already processed
        }
        
        processed.insert(current->id);
        
        // Find all other nodes as potential neighbors
        for (auto& pair : idToNode) {
            SALTNode* neighbor = pair.second;
            if (processed.find(neighbor->id) != processed.end()) {
                continue;  // Skip processed nodes
            }
            
            int distance = manhattanDistance(current->id, neighbor->id);
            if (distance < neighbor->distance) {
                neighbor->distance = distance;
                neighbor->parent = current;
                current->children.push_back(neighbor);
                pq.push(neighbor);
            }
        }
    }
    
    return tree;
}

// DFS traversal for finding breakpoints (as in Algorithm 2)
void SteinerSALT::dfs(SALTNode* v, SALTTree* mst, SALTTree* result, std::vector<SALTNode*>& breakpoints) {
    if (!v) return;
    
    // Check if v is a breakpoint
    if (v != mst->root && v->distance > (1 + epsilon) * manhattanDistance(mst->root->id, v->id)) {
        breakpoints.push_back(v);
        v->isBreakpoint = true;
        
        // Update distance to the direct distance from root
        v->distance = manhattanDistance(mst->root->id, v->id);
    }
    
    // Process children
    for (SALTNode* child : v->children) {
        // Relax the edge v -> child
        relax(v, child, mst);
        
        // Recursive DFS
        dfs(child, mst, result, breakpoints);
        
        // Relax the edge child -> v (for undirected graphs)
        relax(child, v, mst);
    }
}

// Relax function (as in Algorithm 2)
void SteinerSALT::relax(SALTNode* u, SALTNode* v, SALTTree* mst) {
    if (!u || !v) return;
    
    int weight = manhattanDistance(u->id, v->id);
    
    // First condition: if d[v] > d[u] + w(uv)
    if (v->distance > u->distance + weight) {
        v->distance = u->distance + weight;
        v->parent = u;
    } 
    // Second condition: if d[v] = d[u] + w(uv) and w(p[v]v) > w(uv)
    else if (v->distance == u->distance + weight && 
             v->parent && manhattanDistance(v->parent->id, v->id) > weight) {
        v->parent = u;
    }
}

// Light Steiner SPT construction (Algorithm 3)
SALTTree* SteinerSALT::buildLightSteinerSPT(int sourceNodeId, const std::vector<SALTNode*>& terminals) {
    if (terminals.empty()) {
        return nullptr;
    }
    
    // Create a new tree
    SALTTree* tree = new SALTTree();
    
    // Create nodes for all terminals including source
    std::unordered_map<int, SALTNode*> idToNode;
    
    // Add source node
    auto sourceCoords = getNodeCoordinates(sourceNodeId);
    if (sourceCoords.first == std::numeric_limits<int>::min()) {
        delete tree;
        return nullptr;
    }
    
    SALTNode* sourceNode = new SALTNode(sourceNodeId, sourceCoords.first, sourceCoords.second);
    sourceNode->distance = 0;
    tree->root = sourceNode;
    tree->nodes.push_back(sourceNode);
    tree->terminals.push_back(sourceNode);
    idToNode[sourceNodeId] = sourceNode;
    
    // Add all terminal nodes
    for (SALTNode* terminal : terminals) {
        if (terminal->id == sourceNodeId) continue; // Skip if it's the source
        
        SALTNode* node = new SALTNode(terminal->id, terminal->x, terminal->y);
        node->distance = manhattanDistance(sourceNodeId, terminal->id);
        tree->nodes.push_back(node);
        tree->terminals.push_back(node);
        idToNode[terminal->id] = node;
    }
    
    // Create a circle based on MST
    // This is a simplification - in practice, we would need a Hamiltonian circle
    std::vector<SALTNode*> circle = tree->terminals;
    
    // Iteratively merge nodes
    while (circle.size() > 1) {
        // Calculate costs for adjacent pairs
        std::vector<std::pair<double, std::pair<SALTNode*, SALTNode*>>> edgeCosts;
        for (size_t i = 0; i < circle.size(); ++i) {
            size_t next = (i + 1) % circle.size();
            SALTNode* node1 = circle[i];
            SALTNode* node2 = circle[next];
            
            // Calculate s and b values
            double s = node1->distance + node2->distance;
            double b = std::abs(node1->distance - node2->distance);
            
            // Calculate c value
            double c = std::max(s, b);
            
            edgeCosts.push_back({c, {node1, node2}});
        }
        
        // Sort edges by cost
        std::sort(edgeCosts.begin(), edgeCosts.end());
        
        // Create matching (simplified - just pick pairs in order)
        std::vector<std::pair<SALTNode*, SALTNode*>> matching;
        std::set<SALTNode*> matched;
        
        for (const auto& edge : edgeCosts) {
            SALTNode* node1 = edge.second.first;
            SALTNode* node2 = edge.second.second;
            
            if (matched.find(node1) != matched.end() || 
                matched.find(node2) != matched.end()) {
                continue;
            }
            
            matching.push_back({node1, node2});
            matched.insert(node1);
            matched.insert(node2);
            
            // Break if we can't make any more pairs
            if (matched.size() >= circle.size() - 1) {
                break;
            }
        }
        
        // Create new circle for next iteration
        std::vector<SALTNode*> newCircle;
        
        // Process each matching pair
        for (const auto& pair : matching) {
            SALTNode* left = pair.first;
            SALTNode* right = pair.second;
            
            // Create Steiner point
            int steinerX = (left->x + right->x) / 2;
            int steinerY = (left->y + right->y) / 2;
            SALTNode* steiner = new SALTNode(-1, steinerX, steinerY);
            
            // Calculate edge weights
            double s = left->distance + right->distance;
            double b = std::abs(left->distance - right->distance);
            
            if (b <= s) {
                steiner->distance = (s + b) / 2;
                // Set edge weights (for future use)
                double w1 = (s + b) / 2;
                double w2 = (s - b) / 2;
            } else {
                steiner->distance = std::max(b, 0.0);
                // Set edge weights (for future use)
                double w1 = std::max(b, 0.0);
                double w2 = std::max(-b, 0.0);
            }
            
            // Connect Steiner point to left and right
            steiner->children.push_back(left);
            steiner->children.push_back(right);
            left->parent = steiner;
            right->parent = steiner;
            
            // Add to the tree
            tree->nodes.push_back(steiner);
            tree->steinerPoints.push_back(steiner);
            
            // Add to new circle
            newCircle.push_back(steiner);
        }
        
        // Add unmatched node if one exists (when circle size is odd)
        for (SALTNode* node : circle) {
            if (matched.find(node) == matched.end()) {
                newCircle.push_back(node);
                break;
            }
        }
        
        // Update circle for next iteration
        circle = newCircle;
    }
    
    // Set the root of the tree
    if (!circle.empty()) {
        tree->root = circle[0];
    }
    
    return tree;
}

// Algorithm 4: Rectilinear SALT
SALTTree* SteinerSALT::buildRectilinearSALT(int sourceNodeId, const std::vector<int>& sinkNodeIds) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // Build visibility graph to consider obstacles
    buildVisibilityGraph();
    
    // Step 1: Initialize
    std::vector<int> allNodeIds = {sourceNodeId};
    allNodeIds.insert(allNodeIds.end(), sinkNodeIds.begin(), sinkNodeIds.end());
    
    // Step 2: Build obstacle-aware MST
    SALTTree* mst = buildObstacleAwareMST(sourceNodeId, allNodeIds);
    if (!mst || !mst->root) {
        std::cerr << "Failed to build obstacle-aware MST" << std::endl;
        return nullptr;
    }
    
    // Initialize result tree
    SALTTree* result = new SALTTree();
    result->root = new SALTNode(sourceNodeId);
    auto rootCoords = getNodeCoordinates(sourceNodeId);
    result->root->x = rootCoords.first;
    result->root->y = rootCoords.second;
    result->root->distance = 0;  // Distance from root to itself is 0
    result->nodes.push_back(result->root);
    result->terminals.push_back(result->root);
    
    // Step 3: Perform DFS
    std::vector<SALTNode*> breakpoints;
    dfs(mst->root, mst, result, breakpoints);
    
    // Step 4-5: Build Steiner SPT for breakpoints and combine
    if (!breakpoints.empty()) {
        SALTTree* steinerSPT = buildLightSteinerSPT(sourceNodeId, breakpoints);
        if (steinerSPT) {
            // Merge trees
            for (auto node : steinerSPT->nodes) {
                if (node != steinerSPT->root) { // Skip root, as it's already in result
                    result->nodes.push_back(node);
                    if (node->id != -1) { // not a Steiner point
                        result->terminals.push_back(node);
                    } else {
                        result->steinerPoints.push_back(node);
                    }
                }
            }
            
            // Clean up, but keep the nodes (they're now part of result)
            steinerSPT->nodes.clear();
            delete steinerSPT;
        }
    }
    
    // Clean up MST
    delete mst;
    
    // Do refinement with obstacle awareness
    refineSALT(result);
    
    return result;
}

// Algorithm 5: Refinement
void SteinerSALT::refineSALT(SALTTree* tree) {
    if (!tree) return;
    
    // Do each refinement step
    cancelIntersectedEdges(tree);
    performLShapeFlipping(tree);
    performUShapeShifting(tree);
}

// Cancel intersecting edges (part of refinement)
void SteinerSALT::cancelIntersectedEdges(SALTTree* tree) {
    if (!tree) return;
    
    // Get all edges in the tree
    std::vector<std::pair<SALTNode*, SALTNode*>> edges;
    
    // Helper function to collect edges
    std::function<void(SALTNode*)> collectEdges = [&](SALTNode* node) {
        if (!node) return;
        
        for (SALTNode* child : node->children) {
            edges.push_back({node, child});
            collectEdges(child);
        }
    };
    
    collectEdges(tree->root);
    
    // Check each pair of edges for intersection
    for (size_t i = 0; i < edges.size(); ++i) {
        for (size_t j = i + 1; j < edges.size(); ++j) {
            SALTNode* a1 = edges[i].first;
            SALTNode* a2 = edges[i].second;
            SALTNode* b1 = edges[j].first;
            SALTNode* b2 = edges[j].second;
            
            // Check if edges intersect
            if (edgesIntersect(a1, a2, b1, b2)) {
                // Find the intersection point
                Point intersection = findIntersection(a1, a2, b1, b2);
                
                // Skip if intersection point is in an obstacle
                if (isPointInObstacle(intersection.x, intersection.y)) {
                    continue;
                }
                
                // Create a new Steiner point at the intersection
                SALTNode* steiner = new SALTNode(-1, intersection.x, intersection.y);
                tree->nodes.push_back(steiner);
                tree->steinerPoints.push_back(steiner);
                
                // Replace the original edges with connections through the Steiner point
                
                // Disconnect original edges
                auto removeChild = [](SALTNode* parent, SALTNode* child) {
                    auto it = std::find(parent->children.begin(), parent->children.end(), child);
                    if (it != parent->children.end()) {
                        parent->children.erase(it);
                    }
                    child->parent = nullptr;
                };
                
                removeChild(a1, a2);
                removeChild(b1, b2);
                
                // Connect through Steiner point
                a1->children.push_back(steiner);
                steiner->parent = a1;
                
                steiner->children.push_back(a2);
                a2->parent = steiner;
                
                b1->children.push_back(steiner);
                // Note: steiner already has a parent, so we don't set it again
                
                steiner->children.push_back(b2);
                b2->parent = steiner;
                
                // Update distance values
                steiner->distance = std::min(a1->distance + manhattanDistance(a1->x, a1->y, steiner->x, steiner->y),
                                           b1->distance + manhattanDistance(b1->x, b1->y, steiner->x, steiner->y));
                
                // After modifying the tree, we need to recollect edges and restart
                edges.clear();
                collectEdges(tree->root);
                i = 0;
                j = 0;
                break;
            }
        }
    }
}

// L-shape flipping (part of refinement) 
void SteinerSALT::performLShapeFlipping(SALTTree* tree) {
    if (!tree) return;
    
    // Find all L-shapes in the tree
    bool madeChange;
    do {
        madeChange = false;
        
        // Helper function to check for L-shapes
        std::function<void(SALTNode*)> findLShapes = [&](SALTNode* node) {
            if (!node || madeChange) return;
            
            for (SALTNode* child : node->children) {
                if (madeChange) break;
                
                // Check if node->child is an L-shape
                if (child->children.size() == 1 && node->x != child->x && node->y != child->y) {
                    SALTNode* grandchild = child->children[0];
                    
                    // Check if this forms an L-shape
                    if ((grandchild->x == node->x || grandchild->y == node->y) &&
                        node->x != grandchild->x && node->y != grandchild->y) {
                        
                        // Check if the direct connection would cross an obstacle
                        if (!isPointInObstacle(node->x, node->y) &&
                            !isPointInObstacle(grandchild->x, grandchild->y) &&
                            !doesLineIntersectObstacle(node->x, node->y, grandchild->x, grandchild->y)) {
                            
                            // Calculate current and proposed path costs
                            double currentCost = manhattanDistance(node->x, node->y, child->x, child->y) +
                                                 manhattanDistance(child->x, child->y, grandchild->x, grandchild->y);
                            
                            double proposedCost = manhattanDistance(node->x, node->y, grandchild->x, grandchild->y);
                            
                            if (proposedCost <= currentCost) {
                                // Flip the L-shape
                                node->children.erase(std::find(node->children.begin(), node->children.end(), child));
                                
                                for (auto it = child->children.begin(); it != child->children.end(); ++it) {
                                    if (*it == grandchild) {
                                        child->children.erase(it);
                                        break;
                                    }
                                }
                                
                                // Add grandchild directly to node
                                node->children.push_back(grandchild);
                                grandchild->parent = node;
                                
                                // Remove the middle node
                                tree->nodes.erase(std::find(tree->nodes.begin(), tree->nodes.end(), child));
                                tree->steinerPoints.erase(std::find(tree->steinerPoints.begin(), tree->steinerPoints.end(), child));
                                delete child;
                                
                                madeChange = true;
                                break;
                            }
                        }
                    }
                }
                
                findLShapes(child);
            }
        };
        
        findLShapes(tree->root);
        
    } while (madeChange);
}

// U-shape shifting (part of refinement)
void SteinerSALT::performUShapeShifting(SALTTree* tree) {
    if (!tree) return;
    
    // Find all U-shapes in the tree
    bool madeChange;
    do {
        madeChange = false;
        
        // Helper function to check for U-shapes
        std::function<void(SALTNode*)> findUShapes = [&](SALTNode* node) {
            if (!node || madeChange) return;
            
            // Check for U-shape pattern
            if (node->children.size() >= 2) {
                for (size_t i = 0; i < node->children.size(); ++i) {
                    if (madeChange) break;
                    
                    SALTNode* child1 = node->children[i];
                    
                    for (size_t j = i + 1; j < node->children.size(); ++j) {
                        SALTNode* child2 = node->children[j];
                        
                        // Check if both children have a single child (forming a U shape)
                        if (child1->children.size() == 1 && child2->children.size() == 1) {
                            SALTNode* grandchild1 = child1->children[0];
                            SALTNode* grandchild2 = child2->children[0];
                            
                            // Check if the grandchildren are adjacent or the same node
                            int dist = manhattanDistance(grandchild1->x, grandchild1->y, 
                                                       grandchild2->x, grandchild2->y);
                            
                            if (dist <= 1 || grandchild1 == grandchild2) {
                                // Check if the path can be improved
                                double currentCost = manhattanDistance(node->x, node->y, child1->x, child1->y) +
                                                   manhattanDistance(child1->x, child1->y, grandchild1->x, grandchild1->y) +
                                                   manhattanDistance(node->x, node->y, child2->x, child2->y) +
                                                   manhattanDistance(child2->x, child2->y, grandchild2->x, grandchild2->y);
                                
                                // Find a potential better connection point
                                int midX = (grandchild1->x + grandchild2->x) / 2;
                                int midY = (grandchild1->y + grandchild2->y) / 2;
                                
                                // Skip if midpoint is inside an obstacle
                                if (isPointInObstacle(midX, midY)) {
                                    continue;
                                }
                                
                                // Calculate cost of the new configuration
                                double newCost = manhattanDistance(node->x, node->y, midX, midY) +
                                               manhattanDistance(midX, midY, grandchild1->x, grandchild1->y) +
                                               manhattanDistance(midX, midY, grandchild2->x, grandchild2->y);
                                
                                if (newCost < currentCost) {
                                    // Create a new Steiner point
                                    SALTNode* steiner = new SALTNode(-1, midX, midY);
                                    tree->nodes.push_back(steiner);
                                    tree->steinerPoints.push_back(steiner);
                                    
                                    // Update connections
                                    node->children.erase(std::find(node->children.begin(), node->children.end(), child1));
                                    node->children.erase(std::find(node->children.begin(), node->children.end(), child2));
                                    
                                    child1->children.clear();
                                    child2->children.clear();
                                    
                                    // Add new connections
                                    node->children.push_back(steiner);
                                    steiner->parent = node;
                                    
                                    steiner->children.push_back(grandchild1);
                                    grandchild1->parent = steiner;
                                    
                                    // If the grandchildren are different nodes
                                    if (grandchild1 != grandchild2) {
                                        steiner->children.push_back(grandchild2);
                                        grandchild2->parent = steiner;
                                    }
                                    
                                    // Remove the unused nodes
                                    tree->nodes.erase(std::find(tree->nodes.begin(), tree->nodes.end(), child1));
                                    tree->nodes.erase(std::find(tree->nodes.begin(), tree->nodes.end(), child2));
                                    
                                    tree->steinerPoints.erase(std::find(tree->steinerPoints.begin(), tree->steinerPoints.end(), child1));
                                    tree->steinerPoints.erase(std::find(tree->steinerPoints.begin(), tree->steinerPoints.end(), child2));
                                    
                                    delete child1;
                                    delete child2;
                                    
                                    madeChange = true;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
            
            // Recursively check children if no change was made
            if (!madeChange) {
                for (SALTNode* child : node->children) {
                    findUShapes(child);
                    if (madeChange) break;
                }
            }
        };
        
        findUShapes(tree->root);
        
    } while (madeChange);
}

// Convert SALT tree to a path in the original graph
std::vector<int> SteinerSALT::saltTreeToPath(SALTTree* tree) {
    if (!tree || !tree->root) {
        return {};
    }
    
    std::vector<int> path;
    std::set<int> visitedNodes; // To avoid duplicates
    
    // Helper function to traverse the tree and build the path
    std::function<void(SALTNode*, SALTNode*)> traverse = 
        [&](SALTNode* node, SALTNode* parent) {
            if (!node) return;
            
            int currentId = node->id;
            if (currentId == -1) {
                // This is a Steiner point without a corresponding node
                currentId = findClosestLegalNode(node->x, node->y);
                if (currentId == -1) return; // Cannot map this Steiner point
            }
            
            // Connect parent to current node with a rectilinear path
            if (parent) {
                int parentId = parent->id;
                if (parentId == -1) {
                    parentId = findClosestLegalNode(parent->x, parent->y);
                    if (parentId == -1) return; // Cannot map parent Steiner point
                }
                
                if (parentId != currentId) {
                    std::vector<int> segment;
                    
                    // Check if there are obstacles
                    if (!obstacles.empty() && edgeCrossesObstacle(parentId, currentId)) {
                        // Use obstacle-aware path finding
                        segment = findObstacleAwareRectilinearPath(parentId, currentId);
                    } else {
                        // Use simple rectilinear connection
                        segment = connectNodesRectilinear(parentId, currentId);
                    }
                    
                    // Add all nodes except the last one (to avoid duplicates)
                    for (size_t i = 0; i < segment.size() - 1; ++i) {
                        int nodeId = segment[i];
                        if (visitedNodes.insert(nodeId).second) {
                            path.push_back(nodeId);
                        }
                    }
                }
            }
            
            // Add current node if not already visited
            if (visitedNodes.insert(currentId).second) {
                path.push_back(currentId);
            }
            
            // Recursively process children
            for (SALTNode* child : node->children) {
                traverse(child, node);
            }
        };
    
    // Start traversal from the root
    traverse(tree->root, nullptr);
    
    return path;
}

// Connect two nodes with a rectilinear path (Manhattan distance)
std::vector<int> SteinerSALT::connectNodesRectilinear(int fromId, int toId) {
    // First check if both nodes are legal
    if (!isLegalNode(fromId) || !isLegalNode(toId)) {
        std::cerr << "Warning: connectNodesRectilinear called with illegal nodes: " 
                  << fromId << ", " << toId << std::endl;
        return {};
    }
    
    auto fromCoords = getNodeCoordinates(fromId);
    auto toCoords = getNodeCoordinates(toId);
    
    if (fromCoords.first == std::numeric_limits<int>::min() || 
        toCoords.first == std::numeric_limits<int>::min()) {
        // One or both nodes not found
        std::cerr << "Warning: Cannot get coordinates for nodes: " 
                  << fromId << ", " << toId << std::endl;
        return {};
    }
    
    // If there are obstacles, use obstacle-aware path finding
    if (!obstacles.empty() && edgeCrossesObstacle(fromId, toId)) {
        return findObstacleAwareRectilinearPath(fromId, toId);
    }
    
    int fromX = fromCoords.first;
    int fromY = fromCoords.second;
    int toX = toCoords.first;
    int toY = toCoords.second;
    
    // Create path of nodes following rectilinear (L-shaped) route
    std::vector<int> path;
    path.push_back(fromId);
    
    // If nodes are the same, return just that node
    if (fromId == toId) {
        return path;
    }
    
    // Create an L-shaped path (first horizontal, then vertical)
    if (fromX != toX) {
        // Add intermediate node at (toX, fromY)
        int intermediateId = findOrCreateNode(toX, fromY);
        if (intermediateId != fromId) {  // Avoid duplicates
            path.push_back(intermediateId);
        }
    }
    
    // Add destination node
    if (path.back() != toId) {  // Avoid duplicates
        path.push_back(toId);
    }
    
    return path;
}

// Find an obstacle-aware path between two nodes
std::vector<int> SteinerSALT::findObstacleAwareRectilinearPath(int fromId, int toId) {
    // If no obstacles or direct path doesn't cross obstacles, use direct connection
    if (obstacles.empty() || !edgeCrossesObstacle(fromId, toId)) {
        return connectNodesRectilinear(fromId, toId);
    }
    
    // Get coordinates of nodes
    auto fromCoords = getNodeCoordinates(fromId);
    auto toCoords = getNodeCoordinates(toId);
    
    if (fromCoords.first == std::numeric_limits<int>::min() || 
        toCoords.first == std::numeric_limits<int>::min()) {
        return {}; // Invalid nodes
    }
    
    // Priority queue for A* search
    using QueueItem = std::pair<double, int>; // <estimated cost, nodeId>
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
    
    // Maps for tracking
    std::unordered_map<int, int> cameFrom;  // nodeId -> previous nodeId
    std::unordered_map<int, double> costSoFar;  // nodeId -> cost from source
    
    // Begin the search
    queue.push({0.0, fromId});
    cameFrom[fromId] = -1;  // No previous for start node
    costSoFar[fromId] = 0.0;
    
    bool found = false;
    
    while (!queue.empty() && !found) {
        // Get the node with lowest estimated cost
        int currentId = queue.top().second;
        queue.pop();
        
        // Check if we've reached the destination
        if (currentId == toId) {
            found = true;
            break;
        }
        
        // Get current node coordinates
        auto currentCoords = getNodeCoordinates(currentId);
        
        // Define possible movement directions (rectilinear movements)
        std::vector<std::pair<int, int>> directions = {
            {1, 0}, {-1, 0}, {0, 1}, {0, -1}  // Right, Left, Up, Down
        };
        
        // Check each direction
        for (const auto& dir : directions) {
            int nextX = currentCoords.first + dir.first;
            int nextY = currentCoords.second + dir.second;
            
            // Skip if the point is inside an obstacle
            if (isPointInObstacle(nextX, nextY)) {
                continue;
            }
            
            // Find or create node at this position
            int nextId = findOrCreateNode(nextX, nextY);
            
            // Skip if this would cross an obstacle
            if (edgeCrossesObstacle(currentId, nextId)) {
                continue;
            }
            
            // Calculate cost to move to this node
            double newCost = costSoFar[currentId] + getEdgeCost(currentId, nextId);
            
            // If we haven't visited this node yet, or we found a better path
            if (costSoFar.find(nextId) == costSoFar.end() || newCost < costSoFar[nextId]) {
                costSoFar[nextId] = newCost;
                
                // Calculate heuristic (Manhattan distance to goal)
                auto nextCoords = getNodeCoordinates(nextId);
                double heuristic = manhattanDistance(nextCoords.first, nextCoords.second, 
                                                   toCoords.first, toCoords.second);
                
                // Add to queue with priority = cost + heuristic
                queue.push({newCost + heuristic, nextId});
                cameFrom[nextId] = currentId;
            }
        }
    }
    
    // If no path found, return empty vector
    if (!found) {
        if (verbose) {
            std::cerr << "No obstacle-aware path found from " << fromId << " to " << toId << std::endl;
        }
        return {};
    }
    
    // Reconstruct the path
    std::vector<int> path;
    int current = toId;
    
    while (current != -1) {
        path.push_back(current);
        current = cameFrom[current];
    }
    
    // Reverse the path to get it from source to destination
    std::reverse(path.begin(), path.end());
    
    return path;
}

// Build visibility graph for obstacle-aware routing
void SteinerSALT::buildVisibilityGraph() {
    if (obstacles.empty()) {
        return; // No obstacles to consider
    }
    
    // Get obstacle corners
    std::vector<Point> corners = findObstacleCorners();
    
    // Add corner points to the graph as nodes
    std::vector<int> cornerNodeIds;
    for (const auto& corner : corners) {
        // Skip if point is inside an obstacle (this can happen at inner corners)
        if (isPointInObstacle(corner.x, corner.y)) {
            continue;
        }
        
        int nodeId = findOrCreateNode(corner.x, corner.y);
        cornerNodeIds.push_back(nodeId);
    }
    
    // For each existing node, try connecting to corner nodes
    for (const auto& node : nodes) {
        for (int cornerNodeId : cornerNodeIds) {
            // Skip self-connections
            if (node.id == cornerNodeId) {
                continue;
            }
            
            // Check if direct connection is possible (no obstacles in the way)
            if (!edgeCrossesObstacle(node.id, cornerNodeId)) {
                // This would be where we add an edge to the graph
                // In our implementation, we're using the nodes and the obstacle check
                // rather than explicitly storing edges
            }
        }
    }
}

// Build obstacle-aware MST
SALTTree* SteinerSALT::buildObstacleAwareMST(int sourceNodeId, const std::vector<int>& nodeIds) {
    // Build or update the visibility graph first
    buildVisibilityGraph();
    
    // Create a new tree
    SALTTree* tree = new SALTTree();
    if (nodeIds.empty()) {
        return tree;
    }
    
    // Create nodes for all terminals
    std::unordered_map<int, SALTNode*> idToNode;
    for (int id : nodeIds) {
        auto coords = getNodeCoordinates(id);
        if (coords.first != std::numeric_limits<int>::min()) {
            SALTNode* node = new SALTNode(id, coords.first, coords.second);
            idToNode[id] = node;
            tree->nodes.push_back(node);
            tree->terminals.push_back(node);
        }
    }
    
    // Check if we have the source node
    if (idToNode.find(sourceNodeId) == idToNode.end()) {
        std::cerr << "Source node not found" << std::endl;
        delete tree;
        return nullptr;
    }
    
    // Initialize distances for Prim's algorithm
    for (auto& pair : idToNode) {
        pair.second->distance = std::numeric_limits<double>::infinity();
    }
    
    // Start from source node
    tree->root = idToNode[sourceNodeId];
    tree->root->distance = 0;
    
    // Priority queue for Prim's algorithm
    auto cmp = [](SALTNode* a, SALTNode* b) { return a->distance > b->distance; };
    std::priority_queue<SALTNode*, std::vector<SALTNode*>, decltype(cmp)> pq(cmp);
    pq.push(tree->root);
    
    // Set to keep track of processed nodes
    std::set<int> processed;
    
    while (!pq.empty()) {
        SALTNode* current = pq.top();
        pq.pop();
        
        if (processed.find(current->id) != processed.end()) {
            continue;  // Skip if already processed
        }
        
        processed.insert(current->id);
        
        // Find all other nodes as potential neighbors
        for (auto& pair : idToNode) {
            SALTNode* neighbor = pair.second;
            if (processed.find(neighbor->id) != processed.end()) {
                continue;  // Skip processed nodes
            }
            
            // Find obstacle-aware path cost between current and neighbor
            std::vector<int> path = findObstacleAwareRectilinearPath(current->id, neighbor->id);
            
            // Skip if no path was found
            if (path.empty()) {
                continue;
            }
            
            // Calculate path cost
            double pathCost = 0.0;
            for (size_t i = 1; i < path.size(); ++i) {
                pathCost += getEdgeCost(path[i-1], path[i]);
            }
            
            if (pathCost < neighbor->distance) {
                neighbor->distance = pathCost;
                neighbor->parent = current;
                current->children.push_back(neighbor);
                pq.push(neighbor);
            }
        }
    }
    
    return tree;
}

// Find a SALT tree connecting source to multiple sinks
std::vector<int> SteinerSALT::findSteinerTree(int sourceNodeId, const std::vector<int>& sinkNodeIds) {
    // Check if graph is initialized
    if (nodes.empty() || edges.empty()) {
        if (verbose) {
            std::cerr << "Error: Graph not initialized" << std::endl;
        }
        return {};
    }
    
    // Validate source node
    if (!isLegalNode(sourceNodeId)) {
        if (verbose) {
            std::cerr << "Error: Source node " << sourceNodeId << " is not legal" << std::endl;
        }
        return {}; // Source node is not legal
    }
    
    // Get source coordinates
    auto sourceCoords = getNodeCoordinates(sourceNodeId);
    if (sourceCoords.first == std::numeric_limits<int>::min()) {
        // Source node not found
        if (verbose) {
            std::cerr << "Error: Source node " << sourceNodeId << " coordinates not found" << std::endl;
        }
        return {};
    }
    
    // Get sink coordinates and validate sinks
    std::vector<int> validSinkIds;
    validSinkIds.reserve(sinkNodeIds.size());
    
    for (int sinkId : sinkNodeIds) {
        if (!isLegalNode(sinkId)) {
            if (verbose) {
                std::cerr << "Warning: Sink node " << sinkId << " is not legal, skipping" << std::endl;
            }
            continue; // Skip illegal sink nodes
        }
        
        auto coords = getNodeCoordinates(sinkId);
        if (coords.first != std::numeric_limits<int>::min()) {
            // Valid sink node
            validSinkIds.push_back(sinkId);
        } else {
            if (verbose) {
                std::cerr << "Warning: Sink node " << sinkId << " coordinates not found, skipping" << std::endl;
            }
        }
    }
    
    // If no valid sinks, return empty path
    if (validSinkIds.empty()) {
        if (verbose) {
            std::cerr << "Error: No valid sink nodes for source " << sourceNodeId << std::endl;
        }
        return {};
    }
    
    if (verbose) {
        std::cout << "Building obstacle-aware SALT tree for source " << sourceNodeId 
                << " with " << validSinkIds.size() << " sinks" << std::endl;
    }
    
    // Use the obstacle-aware implementation
    SALTTree* tree = buildRectilinearSALT(sourceNodeId, validSinkIds);
    std::vector<int> path = saltTreeToPath(tree);
    delete tree;
    
    return path;
} 