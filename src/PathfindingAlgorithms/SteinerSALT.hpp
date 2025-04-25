#ifndef STEINERSALT_HPP
#define STEINERSALT_HPP

#include <vector>
#include <unordered_map>
#include <map>
#include <cstdint>
#include "../Datastructure.hpp"

// Point structure for generic 2D points
struct Point {
    int x, y;
    
    Point() : x(0), y(0) {}
    Point(int x, int y) : x(x), y(y) {}
};

// Rectangle obstacle representation
struct Obstacle {
    int x1, y1;  // Top-left corner
    int x2, y2;  // Bottom-right corner
    
    Obstacle() : x1(0), y1(0), x2(0), y2(0) {}
    Obstacle(int x1, int y1, int x2, int y2) : x1(x1), y1(y1), x2(x2), y2(y2) {}
    
    bool contains(int x, int y) const {
        return x >= x1 && x <= x2 && y >= y1 && y <= y2;
    }
    
    bool intersectsLine(int sx, int sy, int tx, int ty) const;
};

// SALT Node structure
struct SALTNode {
    int id;           // Original node ID or -1 for Steiner point
    int x, y;         // Coordinates
    double distance;  // Distance from root
    bool isBreakpoint;  // Whether this is a breakpoint
    
    SALTNode* parent;
    std::vector<SALTNode*> children;
    
    SALTNode() : id(-1), x(0), y(0), distance(0), isBreakpoint(false), parent(nullptr) {}
    SALTNode(int id) : id(id), x(0), y(0), distance(0), isBreakpoint(false), parent(nullptr) {}
    SALTNode(int id, int x, int y) : id(id), x(x), y(y), distance(0), isBreakpoint(false), parent(nullptr) {}
};

// SALT Tree structure
struct SALTTree {
    SALTNode* root;
    std::vector<SALTNode*> nodes;
    std::vector<SALTNode*> terminals;
    std::vector<SALTNode*> steinerPoints;
    
    SALTTree() : root(nullptr) {}
    ~SALTTree() {
        for (auto node : nodes) {
            delete node;
        }
    }
};

// SteinerSALT class
class SteinerSALT {
private:
    // Graph data
    std::vector<std::vector<int>> edges;  // Adjacency list
    std::vector<Node> nodes;              // All nodes
    std::vector<Obstacle> obstacles;      // Obstacles in the graph
    
    // Mappings for efficient lookup
    std::unordered_map<int, int> nodeIdToIndex;          // Node ID to index
    std::map<int, std::map<int, int>> coordToNodeId;     // (x,y) to Node ID
    std::unordered_map<int, double> congestionMap;       // Node ID to congestion
    
    // Algorithm parameters
    double epsilon;                // Trade-off parameter
    int timeoutMs;                 // Timeout in milliseconds
    double congestionPenaltyFactor; // Congestion penalty factor
    bool verbose;                  // Verbosity flag
    
    // Helper methods
    SALTTree* buildMST(int sourceNodeId, const std::vector<int>& nodeIds);
    void dfs(SALTNode* v, SALTTree* mst, SALTTree* result, std::vector<SALTNode*>& breakpoints);
    void relax(SALTNode* u, SALTNode* v, SALTTree* mst);
    SALTTree* buildLightSteinerSPT(int sourceNodeId, const std::vector<SALTNode*>& terminals);
    void refineSALT(SALTTree* tree);
    void cancelIntersectedEdges(SALTTree* tree);
    void performLShapeFlipping(SALTTree* tree);
    void performUShapeShifting(SALTTree* tree);
    
    // Obstacle handling methods
    bool isPointInObstacle(int x, int y) const;
    bool doesLineIntersectObstacle(int x1, int y1, int x2, int y2) const;
    std::vector<Point> findObstacleCorners() const;
    void buildVisibilityGraph();
    std::vector<int> findObstacleAwareRectilinearPath(int fromId, int toId);
    bool edgeCrossesObstacle(int fromId, int toId) const;
    std::vector<int> getRectilinearNeighbors(int nodeId) const;
    double getEdgeCost(int fromId, int toId) const;
    bool edgesIntersect(const SALTNode* a1, const SALTNode* a2, 
                        const SALTNode* b1, const SALTNode* b2) const;
    Point findIntersection(const SALTNode* a1, const SALTNode* a2, 
                          const SALTNode* b1, const SALTNode* b2) const;
    
    // Utility methods
    bool isLegalNode(int nodeId) const;
    int findClosestNode(int x, int y) const;
    int findClosestLegalNode(int x, int y) const;
    int manhattanDistance(int x1, int y1, int x2, int y2) const;
    int manhattanDistance(int nodeId1, int nodeId2) const;
    int64_t coordToId(int x, int y) const;
    int findOrCreateNode(int x, int y);

public:
    // Constructors and destructor
    SteinerSALT();
    SteinerSALT(const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes);
    ~SteinerSALT();
    
    // Parameter setters
    void setEpsilon(double eps);
    void setTimeout(int milliseconds);
    void setCongestionPenaltyFactor(double factor);
    void setVerbose(bool v);
    
    // Obstacle management
    void addObstacle(const Obstacle& obstacle);
    void clearObstacles();
    
    // Congestion management
    void updateCongestion(const std::vector<int>& path, double congestionIncrement);
    void resetCongestion();
    double getCongestion(int nodeId) const;
    
    // Main algorithms
    SALTTree* buildSALT(int sourceNodeId, const std::vector<int>& sinkNodeIds);
    SALTTree* buildRectilinearSALT(int sourceNodeId, const std::vector<int>& sinkNodeIds);
    SALTTree* buildObstacleAwareMST(int sourceNodeId, const std::vector<int>& nodeIds);
    
    // Path finding
    std::vector<int> findSteinerTree(int sourceNodeId, const std::vector<int>& sinkNodeIds);
    std::vector<int> connectNodesRectilinear(int fromId, int toId);
    std::vector<int> saltTreeToPath(SALTTree* tree);
    
    // Made public to support testing
    std::pair<int, int> getNodeCoordinates(int nodeId) const;
};

#endif // STEINERSALT_HPP 