#include <iostream>
#include <vector>
#include <chrono>
#include <queue>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include "../Datastructure.hpp"
#include "../PathfindingAlgorithms/AStarSearch.hpp"
#include "Router.hpp"

// Simple structure to represent edges for MST
struct MST_Edge {
    int from;
    int to;
    double weight;
    
    MST_Edge(int f, int t, double w) : from(f), to(t), weight(w) {}
    
    bool operator<(const MST_Edge& other) const {
        return weight < other.weight;
    }
};

SteinerTreeRouter::SteinerTreeRouter() : Router() {
    // Base constructor already initializes the pathfinder
}

// Helper function to expand the subgraph
std::unordered_set<int> get_k_hop_subgraph(const std::vector<std::vector<int>>& edges, const std::unordered_set<int>& terminals, int k) {
    std::unordered_set<int> neighbors;
    if (k < 0) {
        return neighbors;
    }

    // Track visited nodes efficiently using a vector (node IDs are consecutive)
    std::vector<bool> visited(edges.size(), false);
    std::queue<int> queue;

    // Initialize with terminals (0-hop)
    for (int node : terminals) {
        if (node >= 0 && node < edges.size() && !visited[node]) {
            visited[node] = true;
            queue.push(node);
            neighbors.insert(node);
        }
    }

    // Explore up to k hops using BFS
    for (int level = 0; level < k && !queue.empty(); ++level) {
        int level_size = queue.size();
        for (int i = 0; i < level_size; ++i) {
            int current = queue.front();
            queue.pop();

            for (int neighbor : edges[current]) {
                if (neighbor >= 0 && neighbor < edges.size() && !visited[neighbor]) {
                    visited[neighbor] = true;
                    queue.push(neighbor);
                    neighbors.insert(neighbor);
                }
            }
        }
    }
    
    return neighbors;
}

// Find the representative of a set (for Union-Find algorithm)
int find_set(std::unordered_map<int, int>& parent, int node) {
    if (parent[node] != node) {
        parent[node] = find_set(parent, parent[node]); // Path compression
    }
    return parent[node];
}

// Merge two sets (for Union-Find algorithm)
void union_sets(std::unordered_map<int, int>& parent, std::unordered_map<int, int>& rank, int x, int y) {
    int root_x = find_set(parent, x);
    int root_y = find_set(parent, y);
    
    if (root_x == root_y) return;
    
    // Union by rank
    if (rank[root_x] < rank[root_y]) {
        parent[root_x] = root_y;
    } else {
        parent[root_y] = root_x;
        if (rank[root_x] == rank[root_y]) {
            rank[root_x]++;
        }
    }
}

// Computer all shortest paths between terminals
std::vector<std::vector<std::vector<int>>> SteinerTreeRouter::compute_all_shortest_paths(
    const std::unordered_set<int>& terminals, 
    const std::vector<std::vector<int>>& edges
) {
    // Use a more memory-efficient approach with a map instead of a large 3D vector
    // This avoids allocating memory for all possible terminal pairs
    std::vector<std::vector<std::vector<int>>> shortest_paths;
    
    // Pre-determine the size needed
    size_t terminal_count = terminals.size();
    shortest_paths.resize(terminal_count);
    
    // Create a mapping from terminal IDs to indices in our vectors
    std::unordered_map<int, size_t> terminal_to_index;
    size_t index = 0;
    for (int terminal : terminals) {
        terminal_to_index[terminal] = index++;
        shortest_paths[terminal_to_index[terminal]].resize(terminal_count);
    }
    
    // Compute shortest paths only between actual terminals
    for (int src : terminals) {
        for (int dest : terminals) {
            if (src == dest) continue;
            
            std::vector<int> path;
            pathfinder->findPath(src, dest, path);
            
            // Store path in our optimized data structure
            size_t src_idx = terminal_to_index[src];
            size_t dest_idx = terminal_to_index[dest];
            shortest_paths[src_idx][dest_idx] = path;
        }
    }
    return shortest_paths;
}

// Helper to get a path from the terminal-indexed shortest paths structure
std::vector<int> get_path_from_terminal_paths(
    int src, int dest,
    const std::unordered_map<int, size_t>& terminal_to_index,
    const std::vector<std::vector<std::vector<int>>>& shortest_paths
) {
    auto src_it = terminal_to_index.find(src);
    auto dest_it = terminal_to_index.find(dest);
    if (src_it != terminal_to_index.end() && dest_it != terminal_to_index.end()) {
        return shortest_paths[src_it->second][dest_it->second];
    }
    return {};
}

// Build MST from terminals using Kruskal's algorithm
std::vector<std::pair<int, int>> built_mst(
    const std::unordered_set<int>& terminals, 
    const std::vector<std::vector<std::vector<int>>>& shortest_paths,
    const std::unordered_map<int, size_t>& terminal_to_index
) {
    std::vector<MST_Edge> edges;
    std::vector<std::pair<int, int>> mst_edges;
    
    // Create all edges between terminals with their weights
    for (auto it1 = terminals.begin(); it1 != terminals.end(); ++it1) {
        for (auto it2 = std::next(it1); it2 != terminals.end(); ++it2) {
            int u = *it1;
            int v = *it2;
            
            // Use the terminal indices to access the paths
            size_t u_idx = terminal_to_index.at(u);
            size_t v_idx = terminal_to_index.at(v);
            
            if (!shortest_paths[u_idx][v_idx].empty()) {
                double weight = shortest_paths[u_idx][v_idx].size() - 1; // Path length
                edges.emplace_back(u, v, weight);
            }
        }
    }
    
    // Sort edges by weight
    std::sort(edges.begin(), edges.end());
    
    // Initialize Union-Find data structures
    std::unordered_map<int, int> parent;
    std::unordered_map<int, int> rank;
    for (int terminal : terminals) {
        parent[terminal] = terminal;
        rank[terminal] = 0;
    }
    
    // Run Kruskal's algorithm
    for (const auto& edge : edges) {
        if (find_set(parent, edge.from) != find_set(parent, edge.to)) {
            mst_edges.emplace_back(edge.from, edge.to);
            union_sets(parent, rank, edge.from, edge.to);
        }
        
        // Check if we have enough edges for the MST
        if (mst_edges.size() == terminals.size() - 1) {
            break;
        }
    }
    
    return mst_edges;
}

// Check if a node is congested
bool is_congested(int node_id) {
    // A simple threshold-based congestion check
    // In a real implementation, this would check the congestion map
    // For now, just return false (no congestion)
    return false;
}

// Reroute a net when congestion is detected
void reroute_net(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
    // This would implement congestion-aware rerouting
    // For now, do nothing (placeholder)
}

// Prune redundant Steiner points
void prune_redundant_nodes(std::vector<std::pair<int, int>>& edges, const std::unordered_set<int>& terminals) {
    // This function would remove unnecessary Steiner points
    // For now, do nothing (placeholder)
}

// Get the shortest path between two nodes
std::vector<int> SteinerTreeRouter::get_shortest_path(int src, int dest, const std::vector<std::vector<int>>& edges) {
    std::vector<int> path;
    pathfinder->findPath(src, dest, path);
    return path;
}

// Route a single net using Steiner Tree algorithm
NetRoute SteinerTreeRouter::routeSingleNet(Net& net, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
    // Create a NetRoute object to store results
    NetRoute netRoute(net.id, net.name);
    
    // Check if the net has enough nodes
    if (net.nodeIDs.size() < 2) {
        std::cout << "Warning: Net " << net.id << " has less than 2 nodes. Skipping routing." << std::endl;
        return netRoute;
    }

    // Get the source and sink nodes
    int source = net.nodeIDs[0];
    std::vector<int> sinkNodeIds(net.nodeIDs.begin() + 1, net.nodeIDs.end());
    
    // Construct the routing subgraph
    std::unordered_set<int> terminals = {source};
    terminals.insert(sinkNodeIds.begin(), sinkNodeIds.end());

    // Expand the subgraph to include nodes within k hops of terminals
    std::unordered_set<int> subgraph_nodes = get_k_hop_subgraph(edges, terminals, 3);

    // Initialize our pathfinder if it hasn't been done yet
    if (!pathfinder) {
        pathfinder.reset(new AStarSearch(edges, nodes));
        pathfinder->setTimeout(2000); // 2 second timeout per path
        pathfinder->setCongestionPenaltyFactor(5.0);
    }

    try {
        // Create terminal-to-index mapping for memory-efficient paths
        std::unordered_map<int, size_t> terminal_to_index;
        size_t index = 0;
        for (int terminal : terminals) {
            terminal_to_index[terminal] = index++;
        }
        
        // Compute metric closure (shortest paths between all terminals)
        auto shortest_paths = compute_all_shortest_paths(terminals, edges);

        // Step 2: Build the Steiner Tree
        auto mst_edges = built_mst(terminals, shortest_paths, terminal_to_index);

        // Step 3: Replace MST edges with actual FPGA paths
        std::vector<std::pair<int, int>> steiner_edges;
        for (const auto& edge : mst_edges) {
            // Get path using terminal indices
            std::vector<int> path;
            if (terminal_to_index.count(edge.first) > 0 && terminal_to_index.count(edge.second) > 0) {
                size_t src_idx = terminal_to_index[edge.first];
                size_t dest_idx = terminal_to_index[edge.second];
                if (src_idx < shortest_paths.size() && dest_idx < shortest_paths[src_idx].size()) {
                    path = shortest_paths[src_idx][dest_idx];
                }
            }
            
            // If path lookup failed, get it directly
            if (path.empty()) {
                path = get_shortest_path(edge.first, edge.second, edges);
            }
            
            // Convert path to edges
            for (size_t i = 0; i < path.size() - 1; ++i) {
                steiner_edges.emplace_back(path[i], path[i+1]);
            }
        }

        // Filter valid PIPs
        std::vector<std::pair<int, int>> valid_edges;
        for (const auto& edge_pair : steiner_edges) {
            int u = edge_pair.first;
            int v = edge_pair.second;
            if (u < edges.size() && std::find(edges[u].begin(), edges[u].end(), v) != edges[u].end()) {
                valid_edges.emplace_back(u, v);
            }
        }

        // Prune redundant Steiner points (if needed)
        prune_redundant_nodes(valid_edges, terminals);
        
        // Check for congestion (simplified)
        bool congestion_detected = false;
        for (const auto& edge_pair : valid_edges) {
            if (is_congested(edge_pair.first) || is_congested(edge_pair.second)) {
                reroute_net(net, edges, nodes);
                congestion_detected = true;
                break;
            }
        }

        // If no congestion or rerouting was successful, add edges to result
        if (!congestion_detected && !valid_edges.empty()) {
            netRoute.addEdges(valid_edges);
            netRoute.isRouted = true;
            return netRoute;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error routing net " << net.id << ": " << e.what() << std::endl;
    }

    // Must return a NetRoute object since we can't return void
    // Fallback to base router if our implementation fails
    return Router::routeSingleNet(net, edges, nodes);
}

// Route all nets using Steiner Tree algorithm
void SteinerTreeRouter::routeAllNets(std::vector<Net>& nets, const std::vector<std::vector<int>>& edges, const std::vector<Node>& nodes) {
    std::cout << "Routing " << nets.size() << " nets using Steiner Tree algorithm..." << std::endl;
    
    // Clear previous results
    clearRoutingResults();
    
    // Reserve space for results
    routingResults.reserve(nets.size());
    
    // Initialize pathfinder if needed
    if (!pathfinder) {
        auto startTime = std::chrono::steady_clock::now();
        
        pathfinder.reset(new AStarSearch(edges, nodes));
        pathfinder->setTimeout(2000); // 2 second timeout per path
        pathfinder->setCongestionPenaltyFactor(5.0);
        
        auto endTime = std::chrono::steady_clock::now();
        auto initTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
        std::cout << "Initialized pathfinder in " << initTime << "ms" << std::endl;
    }
    
    // Pre-build set of existing node IDs (do this once)
    existingNodeIds.clear();
    existingNodeIds.reserve(nodes.size());
    for (const auto& node : nodes) {
        existingNodeIds.insert(node.id);
    }
    
    // Start timing for complete routing process
    auto startTime = std::chrono::steady_clock::now();
    
    // Sort nets by priority (similar to the base Router)
    std::vector<size_t> netIndices(nets.size());
    for (size_t i = 0; i < nets.size(); ++i) {
        netIndices[i] = i;
    }
    
    // Precompute priorities
    std::vector<double> priorities(nets.size());
    for (size_t i = 0; i < nets.size(); ++i) {
        priorities[i] = computePriority(nets[i], nodes);
    }
    
    // Sort indices based on priorities (descending)
    std::sort(netIndices.begin(), netIndices.end(), 
        [&priorities](size_t a, size_t b) {
            return priorities[a] > priorities[b]; 
        }
    );
    
    // Show progress every 10% of nets
    size_t progressStep = std::max(size_t(1), nets.size() / 10);
    
    // Route nets in priority order
    for (size_t idx = 0; idx < netIndices.size(); ++idx) {
        size_t i = netIndices[idx];
        
        // Report progress occasionally
        if (idx % progressStep == 0) {
            std::cout << "Routing progress: " << (idx * 100 / nets.size()) << "%" << std::endl;
        }
        
        // Route the net and store the result
        NetRoute result = routeSingleNet(nets[i], edges, nodes);
        routingResults.push_back(result);
        
        // If the route was successful, update congestion map
        if (result.isRouted && !result.edges.empty()) {
            // Extract the full path from edges
            std::vector<int> path;
            path.reserve(result.edges.size() + 1);
            
            // Add the first node of the first edge
            if (!result.edges.empty()) {
                path.push_back(result.edges[0].first);
            }
            
            // Add all destination nodes
            for (const auto& edge : result.edges) {
                path.push_back(edge.second);
            }
            
            // Update congestion along this path
            pathfinder->updateCongestion(path, 2.0);
        }
    }
    
    // Report total time
    auto endTime = std::chrono::steady_clock::now();
    auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
    std::cout << "Complete Steiner Tree routing of all nets took " << totalTime << "ms" << std::endl;
    
    // Print results
    printRoutingResults();
}