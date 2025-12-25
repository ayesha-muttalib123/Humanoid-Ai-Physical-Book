---
sidebar_label: Motion Planning Algorithms
title: Motion Planning Algorithms - Path Planning and Obstacle Avoidance
description: Understanding motion planning algorithms for robotics including path planning, obstacle avoidance, and navigation
keywords: [motion planning, path planning, obstacle avoidance, navigation, robotics, A*, RRT, PRM, algorithms]
---

# 6.3 Motion Planning Algorithms

## Introduction

Motion planning is a fundamental capability in robotics that enables robots to navigate from their current state to a desired goal while avoiding obstacles and satisfying various constraints. In Physical AI systems, motion planning bridges the gap between high-level task planning and low-level control, generating feasible paths that respect the robot's kinematic and dynamic constraints while considering environmental obstacles.

The complexity of motion planning stems from the need to search through high-dimensional configuration spaces while accounting for the robot's physical constraints, environmental obstacles, and various optimization criteria. Modern motion planning algorithms must balance computational efficiency with solution quality, finding paths that are not only collision-free but also optimal with respect to criteria like path length, execution time, or energy consumption.

This chapter explores the theoretical foundations of motion planning, various algorithmic approaches, and practical implementation considerations for Physical AI systems. We'll cover both sampling-based methods for high-dimensional spaces and graph-based methods for lower-dimensional problems.

## Motion Planning Fundamentals

### Configuration Space (C-Space)

The configuration space represents all possible configurations of a robot. For a robot with n degrees of freedom, the configuration space is an n-dimensional space where each point represents a possible configuration.

#### Free Space and Obstacles
- **C-Free**: The portion of configuration space where the robot doesn't collide with obstacles
- **C-Obstacle**: The portion where the robot collides with obstacles
- **Dimensionality**: The complexity of motion planning increases exponentially with the number of dimensions

#### Obstacle Representation
- **Explicit**: Direct representation of obstacle geometry in C-space
- **Implicit**: Function that determines if a configuration is in collision
- **Approximate**: Simplified representations for computational efficiency

### Motion Planning Problem Definition

The motion planning problem can be formally defined as:
- **Input**: Robot geometry, start configuration, goal configuration, obstacles
- **Output**: Collision-free path from start to goal
- **Constraints**: Kinematic, dynamic, and environmental constraints
- **Optimization**: Criteria to optimize (length, time, energy, etc.)

#### Mathematical Formulation
Given:
- Start configuration: q_start ∈ C_free
- Goal configuration: q_goal ∈ C_free
- Robot geometry: R(q) where q is configuration
- Obstacles: O_i ⊂ R^3

Find: A continuous path τ: [0,1] → C_free such that τ(0) = q_start and τ(1) = q_goal

### Planning Paradigms

#### Combinatorial Planning
- **Approach**: Decompose C-space into cells
- **Examples**: Visibility graphs, cell decomposition
- **Advantages**: Complete (finds solution if one exists)
- **Disadvantages**: Exponential complexity in high dimensions

#### Sampling-Based Planning
- **Approach**: Sample random configurations in C-space
- **Examples**: PRM, RRT, RRT*
- **Advantages**: Scalable to high dimensions, probabilistically complete
- **Disadvantages**: No guarantee of optimality in finite time

#### Optimization-Based Planning
- **Approach**: Formulate as optimization problem
- **Examples**: CHOMP, TrajOpt, STOMP
- **Advantages**: Optimal solutions, smooth paths
- **Disadvantages**: Local minima, computational complexity

## Graph-Based Planning Algorithms

### A* Algorithm

A* is a graph search algorithm that guarantees optimal solutions by using heuristics to guide the search toward the goal.

#### Algorithm Overview
1. **Initialization**: Add start node to open set with cost f = g + h
2. **Node Selection**: Select node with minimum f-cost from open set
3. **Goal Check**: If selected node is goal, reconstruct path
4. **Expansion**: Generate successors and update costs
5. **Iteration**: Repeat until goal found or open set empty

#### Implementation Example
```cpp
#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>

class AStarPlanner {
private:
    struct Node {
        int x, y;
        double g_cost;  // Cost from start
        double h_cost;  // Heuristic cost to goal
        double f_cost;  // g_cost + h_cost
        Node* parent;
        
        bool operator>(const Node& other) const {
            return f_cost > other.f_cost;
        }
    };
    
    std::vector<std::vector<bool>> occupancy_grid_;
    int width_, height_;
    double resolution_;
    
public:
    AStarPlanner(const std::vector<std::vector<bool>>& grid, double resolution)
        : occupancy_grid_(grid), resolution_(resolution) {
        width_ = grid.size();
        height_ = grid[0].size();
    }
    
    std::vector<std::pair<int, int>> planPath(
        std::pair<int, int> start, 
        std::pair<int, int> goal) {
        
        // Priority queue for nodes to visit
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open_set;
        
        // Keep track of visited nodes and their costs
        std::unordered_map<int, std::unordered_map<int, double>> g_costs;
        std::unordered_map<int, std::unordered_map<int, bool>> closed_set;
        
        // Add start node
        Node start_node;
        start_node.x = start.first;
        start_node.y = start.second;
        start_node.g_cost = 0.0;
        start_node.h_cost = heuristic(start_node.x, start_node.y, goal.first, goal.second);
        start_node.f_cost = start_node.g_cost + start_node.h_cost;
        start_node.parent = nullptr;
        
        open_set.push(start_node);
        g_costs[start_node.x][start_node.y] = start_node.g_cost;
        
        // Directions: up, down, left, right, and diagonals
        std::vector<std::pair<int, int>> directions = {
            {-1, 0}, {1, 0}, {0, -1}, {0, 1},
            {-1, -1}, {-1, 1}, {1, -1}, {1, 1}
        };
        
        while (!open_set.empty()) {
            Node current = open_set.top();
            open_set.pop();
            
            // Check if we reached the goal
            if (current.x == goal.first && current.y == goal.second) {
                return reconstructPath(current);
            }
            
            // Mark as visited
            closed_set[current.x][current.y] = true;
            
            // Explore neighbors
            for (const auto& dir : directions) {
                int new_x = current.x + dir.first;
                int new_y = current.y + dir.second;
                
                // Check bounds
                if (new_x < 0 || new_x >= width_ || new_y < 0 || new_y >= height_) {
                    continue;
                }
                
                // Check if in closed set
                if (closed_set[new_x][new_y]) {
                    continue;
                }
                
                // Check if obstacle
                if (occupancy_grid_[new_x][new_y]) {
                    continue;
                }
                
                // Calculate tentative g_cost
                double move_cost = (dir.first == 0 || dir.second == 0) ? 1.0 : std::sqrt(2.0);
                double tentative_g = current.g_cost + move_cost;
                
                // Check if this path is better than previous ones
                if (tentative_g < g_costs[new_x][new_y]) {
                    // This path is better
                    Node neighbor;
                    neighbor.x = new_x;
                    neighbor.y = new_y;
                    neighbor.g_cost = tentative_g;
                    neighbor.h_cost = heuristic(new_x, new_y, goal.first, goal.second);
                    neighbor.f_cost = neighbor.g_cost + neighbor.h_cost;
                    neighbor.parent = &current;
                    
                    g_costs[new_x][new_y] = tentative_g;
                    open_set.push(neighbor);
                }
            }
        }
        
        // No path found
        return {};
    }

private:
    double heuristic(int x1, int y1, int x2, int y2) const {
        // Euclidean distance heuristic
        return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }
    
    std::vector<std::pair<int, int>> reconstructPath(const Node& goal_node) const {
        std::vector<std::pair<int, int>> path;
        const Node* current = &goal_node;
        
        while (current != nullptr) {
            path.push_back({current->x, current->y});
            current = current->parent;
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }
    
    bool isOccupied(int x, int y) const {
        if (x < 0 || x >= width_ || y < 0 || y >= height_) {
            return true;  // Treat out-of-bounds as occupied
        }
        return occupancy_grid_[x][y];
    }
};
```

### Dijkstra's Algorithm

Dijkstra's algorithm finds the shortest path in a weighted graph without using heuristics:

```cpp
class DijkstraPlanner {
private:
    std::vector<std::vector<double>> cost_map_;
    int width_, height_;
    
public:
    std::vector<std::pair<int, int>> findShortestPath(
        std::pair<int, int> start, 
        std::pair<int, int> goal) {
        
        std::vector<std::vector<double>> distances(width_, 
                                                   std::vector<double>(height_, 
                                                                       std::numeric_limits<double>::infinity()));
        std::vector<std::vector<std::pair<int, int>>> predecessors(width_, 
                                                                  std::vector<std::pair<int, int>>(height_, {-1, -1}));
        
        distances[start.first][start.second] = 0.0;
        
        // Priority queue of (distance, position)
        auto comp = [](const std::pair<double, std::pair<int, int>>& a, 
                      const std::pair<double, std::pair<int, int>>& b) {
            return a.first > b.first;
        };
        
        std::priority_queue<
            std::pair<double, std::pair<int, int>>,
            std::vector<std::pair<double, std::pair<int, int>>>,
            decltype(comp)> pq(comp);
        
        pq.push({0.0, start});
        
        while (!pq.empty()) {
            auto [current_dist, current_pos] = pq.top();
            pq.pop();
            
            if (current_pos == goal) {
                break;  // Found shortest path
            }
            
            if (current_dist > distances[current_pos.first][current_pos.second]) {
                continue;  // Already found a better path
            }
            
            // Explore neighbors
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0) continue;  // Skip self
                    
                    int nx = current_pos.first + dx;
                    int ny = current_pos.second + dy;
                    
                    if (isValidPosition(nx, ny) && !isObstacle(nx, ny)) {
                        double move_cost = (dx == 0 || dy == 0) ? 1.0 : std::sqrt(2.0);
                        double new_dist = current_dist + move_cost;
                        
                        if (new_dist < distances[nx][ny]) {
                            distances[nx][ny] = new_dist;
                            predecessors[nx][ny] = current_pos;
                            pq.push({new_dist, {nx, ny}});
                        }
                    }
                }
            }
        }
        
        return reconstructPath(predecessors, start, goal);
    }

private:
    bool isValidPosition(int x, int y) const {
        return x >= 0 && x < width_ && y >= 0 && y < height_;
    }
    
    bool isObstacle(int x, int y) const {
        // Implementation depends on how obstacles are represented
        return false;  // Placeholder
    }
    
    std::vector<std::pair<int, int>> reconstructPath(
        const std::vector<std::vector<std::pair<int, int>>>& predecessors,
        std::pair<int, int> start,
        std::pair<int, int> goal) const {
        
        std::vector<std::pair<int, int>> path;
        std::pair<int, int> current = goal;
        
        while (current != start && predecessors[current.first][current.second] != std::make_pair(-1, -1)) {
            path.push_back(current);
            current = predecessors[current.first][current.second];
        }
        
        path.push_back(start);
        std::reverse(path.begin(), path.end());
        
        return path;
    }
};
```

## Sampling-Based Planning Algorithms

### Probabilistic Roadmap (PRM)

PRM is a multi-query planning algorithm that pre-computes a roadmap of the free space that can be reused for multiple planning queries.

#### Algorithm Steps
1. **Sampling**: Sample random configurations in C-free
2. **Connection**: Connect each sample to nearby samples
3. **Path Query**: Find path using graph search on the roadmap

#### Implementation
```cpp
#include <random>
#include <algorithm>

class PRMPlanner {
private:
    struct Vertex {
        std::vector<double> config;
        std::vector<size_t> neighbors;
        std::vector<double> edge_costs;
    };
    
    std::vector<Vertex> roadmap_;
    size_t max_vertices_;
    double connection_radius_;
    std::random_device rd_;
    std::mt19937 gen_;
    
    // Robot and environment information
    std::function<bool(const std::vector<double>&)> isCollisionFree_;
    std::function<double(const std::vector<double>&, const std::vector<double>&)> distanceMetric_;
    
public:
    PRMPlanner(size_t max_verts, double conn_radius)
        : max_vertices_(max_verts), connection_radius_(conn_radius), gen_(rd_()) {}
    
    void buildRoadmap(int dof, 
                     const std::vector<double>& min_bounds,
                     const std::vector<double>& max_bounds) {
        
        roadmap_.clear();
        
        std::vector<std::uniform_real_distribution<double>> distributions;
        for (int i = 0; i < dof; i++) {
            distributions.emplace_back(min_bounds[i], max_bounds[i]);
        }
        
        // Sample vertices
        while (roadmap_.size() < max_vertices_) {
            std::vector<double> config(dof);
            for (int i = 0; i < dof; i++) {
                config[i] = distributions[i](gen_);
            }
            
            if (isCollisionFree_(config)) {
                roadmap_.push_back({config, {}, {}});
            }
        }
        
        // Connect vertices within radius
        for (size_t i = 0; i < roadmap_.size(); i++) {
            for (size_t j = i + 1; j < roadmap_.size(); j++) {
                double dist = distanceMetric_(roadmap_[i].config, roadmap_[j].config);
                
                if (dist <= connection_radius_) {
                    if (isPathCollisionFree(roadmap_[i].config, roadmap_[j].config)) {
                        roadmap_[i].neighbors.push_back(j);
                        roadmap_[i].edge_costs.push_back(dist);
                        roadmap_[j].neighbors.push_back(i);
                        roadmap_[j].edge_costs.push_back(dist);
                    }
                }
            }
        }
    }
    
    std::vector<std::vector<double>> planPath(
        const std::vector<double>& start, 
        const std::vector<double>& goal) {
        
        // Find nearest roadmap vertices to start and goal
        size_t start_vertex = findNearestVertex(start);
        size_t goal_vertex = findNearestVertex(goal);
        
        // Add start and goal to roadmap temporarily
        roadmap_.push_back({start, {}, {}});
        roadmap_.push_back({goal, {}, {}});
        
        size_t start_idx = roadmap_.size() - 2;
        size_t goal_idx = roadmap_.size() - 1;
        
        // Connect start and goal to nearby roadmap vertices
        connectToRoadmap(start_idx, start);
        connectToRoadmap(goal_idx, goal);
        
        // Perform A* search on the roadmap
        auto path_indices = aStarSearch(start_idx, goal_idx);
        
        // Convert indices to configurations
        std::vector<std::vector<double>> path;
        for (auto idx : path_indices) {
            path.push_back(roadmap_[idx].config);
        }
        
        // Remove temporary start and goal vertices
        roadmap_.pop_back();
        roadmap_.pop_back();
        
        return path;
    }

private:
    size_t findNearestVertex(const std::vector<double>& config) {
        size_t nearest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < roadmap_.size(); i++) {
            double dist = distanceMetric_(config, roadmap_[i].config);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        
        return nearest_idx;
    }
    
    void connectToRoadmap(size_t vertex_idx, const std::vector<double>& config) {
        for (size_t i = 0; i < roadmap_.size() - 1; i++) {  // Don't connect to self
            if (i == vertex_idx) continue;
            
            double dist = distanceMetric_(config, roadmap_[i].config);
            if (dist <= connection_radius_ && isPathCollisionFree(config, roadmap_[i].config)) {
                roadmap_[vertex_idx].neighbors.push_back(i);
                roadmap_[vertex_idx].edge_costs.push_back(dist);
                roadmap_[i].neighbors.push_back(vertex_idx);
                roadmap_[i].edge_costs.push_back(dist);
            }
        }
    }
    
    std::vector<size_t> aStarSearch(size_t start, size_t goal) {
        // Implementation of A* search on the roadmap
        // Similar to the grid-based A* but using the roadmap graph
        std::vector<double> g_costs(roadmap_.size(), std::numeric_limits<double>::max());
        std::vector<size_t> predecessors(roadmap_.size(), SIZE_MAX);
        std::vector<bool> closed_set(roadmap_.size(), false);
        
        g_costs[start] = 0.0;
        std::priority_queue<std::pair<double, size_t>, 
                           std::vector<std::pair<double, size_t>>, 
                           std::greater<std::pair<double, size_t>>> open_set;
        open_set.push({0.0, start});
        
        while (!open_set.empty()) {
            auto [f_cost, current] = open_set.top();
            open_set.pop();
            
            if (current == goal) {
                // Reconstruct path
                std::vector<size_t> path;
                size_t curr = goal;
                while (curr != SIZE_MAX) {
                    path.push_back(curr);
                    curr = predecessors[curr];
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            
            if (closed_set[current]) continue;
            closed_set[current] = true;
            
            // Explore neighbors
            for (size_t i = 0; i < roadmap_[current].neighbors.size(); i++) {
                size_t neighbor = roadmap_[current].neighbors[i];
                double edge_cost = roadmap_[current].edge_costs[i];
                
                double tentative_g = g_costs[current] + edge_cost;
                if (tentative_g < g_costs[neighbor]) {
                    g_costs[neighbor] = tentative_g;
                    double h_cost = distanceMetric_(roadmap_[neighbor].config, roadmap_[goal].config);
                    double f_cost = tentative_g + h_cost;
                    
                    predecessors[neighbor] = current;
                    open_set.push({f_cost, neighbor});
                }
            }
        }
        
        return {};  // No path found
    }
    
    bool isPathCollisionFree(const std::vector<double>& start, 
                           const std::vector<double>& end) {
        // Check if path between start and end is collision-free
        // This would involve checking multiple intermediate points
        int num_checks = 10;
        for (int i = 1; i < num_checks; i++) {
            double t = static_cast<double>(i) / num_checks;
            std::vector<double> intermediate;
            for (size_t j = 0; j < start.size(); j++) {
                intermediate.push_back(start[j] + t * (end[j] - start[j]));
            }
            
            if (!isCollisionFree_(intermediate)) {
                return false;
            }
        }
        return true;
    }
};
```

### Rapidly-Exploring Random Trees (RRT)

RRT is a single-query planning algorithm that builds a tree of configurations by randomly exploring the space.

#### Basic RRT Algorithm
```cpp
class RRTPlanner {
private:
    struct TreeNode {
        std::vector<double> config;
        size_t parent;
        std::vector<size_t> children;
    };
    
    std::vector<TreeNode> tree_;
    std::function<bool(const std::vector<double>&)> isCollisionFree_;
    std::function<double(const std::vector<double>&, const std::vector<double>&)> distanceMetric_;
    std::function<std::vector<double>()> sampleConfiguration_;
    
    double step_size_;
    size_t max_iterations_;
    std::random_device rd_;
    std::mt19937 gen_;
    
public:
    RRTPlanner(double step_size, size_t max_iter) 
        : step_size_(step_size), max_iterations_(max_iter), gen_(rd_()) {}
    
    std::vector<std::vector<double>> planPath(
        const std::vector<double>& start, 
        const std::vector<double>& goal) {
        
        // Initialize tree with start configuration
        tree_.clear();
        TreeNode root;
        root.config = start;
        root.parent = SIZE_MAX;  // Root has no parent
        tree_.push_back(root);
        
        for (size_t iter = 0; iter < max_iterations_; iter++) {
            // Sample random configuration or bias toward goal
            std::vector<double> random_config = 
                (iter % 10 == 0) ? goal : sampleConfiguration_();  // 10% chance to sample goal
            
            // Find nearest node in tree
            size_t nearest_idx = findNearestNode(random_config);
            
            // Steer from nearest node toward random configuration
            std::vector<double> new_config = steer(tree_[nearest_idx].config, random_config, step_size_);
            
            // Check if new configuration is collision-free
            if (isCollisionFree_(new_config) && 
                isPathCollisionFree(tree_[nearest_idx].config, new_config)) {
                
                // Add new node to tree
                TreeNode new_node;
                new_node.config = new_config;
                new_node.parent = nearest_idx;
                size_t new_idx = tree_.size();
                tree_.push_back(new_node);
                
                // Add new node as child of parent
                tree_[nearest_idx].children.push_back(new_idx);
                
                // Check if we're close enough to goal
                if (distanceMetric_(new_config, goal) < step_size_) {
                    return extractPath(new_idx);
                }
            }
        }
        
        // No path found within max iterations
        return {};
    }

private:
    size_t findNearestNode(const std::vector<double>& config) {
        size_t nearest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < tree_.size(); i++) {
            double dist = distanceMetric_(config, tree_[i].config);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        
        return nearest_idx;
    }
    
    std::vector<double> steer(const std::vector<double>& from, 
                             const std::vector<double>& to, 
                             double max_dist) {
        
        std::vector<double> result = from;
        double dist = distanceMetric_(from, to);
        
        if (dist <= max_dist) {
            return to;  // Direct connection
        }
        
        // Move from toward to by max_dist
        for (size_t i = 0; i < from.size(); i++) {
            double direction = to[i] - from[i];
            result[i] += (direction / dist) * max_dist;
        }
        
        return result;
    }
    
    std::vector<std::vector<double>> extractPath(size_t goal_node_idx) {
        std::vector<std::vector<double>> path;
        size_t current_idx = goal_node_idx;
        
        while (current_idx != SIZE_MAX) {
            path.push_back(tree_[current_idx].config);
            current_idx = tree_[current_idx].parent;
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }
    
    bool isPathCollisionFree(const std::vector<double>& start, 
                           const std::vector<double>& end) {
        // Check if path between start and end is collision-free
        int num_checks = 10;
        for (int i = 1; i < num_checks; i++) {
            double t = static_cast<double>(i) / num_checks;
            std::vector<double> intermediate;
            for (size_t j = 0; j < start.size(); j++) {
                intermediate.push_back(start[j] + t * (end[j] - start[j]));
            }
            
            if (!isCollisionFree_(intermediate)) {
                return false;
            }
        }
        return true;
    }
};
```

### RRT* (Optimal RRT)

RRT* is an extension of RRT that provides asymptotic optimality by rewiring the tree to find lower-cost paths.

```cpp
class RRTStarPlanner {
private:
    struct TreeNode {
        std::vector<double> config;
        size_t parent;
        double cost;  // Cost from root
        std::vector<size_t> children;
    };
    
    std::vector<TreeNode> tree_;
    double step_size_;
    double gamma_;
    size_t max_iterations_;
    
    std::function<bool(const std::vector<double>&)> isCollisionFree_;
    std::function<double(const std::vector<double>&, const std::vector<double>&)> distanceMetric_;
    std::function<std::vector<double>()> sampleConfiguration_;
    
public:
    RRTStarPlanner(double step_size, size_t max_iter, double gamma = 1.0) 
        : step_size_(step_size), max_iterations_(max_iter), gamma_(gamma) {}
    
    std::vector<std::vector<double>> planPath(
        const std::vector<double>& start, 
        const std::vector<double>& goal) {
        
        // Initialize tree with start configuration
        tree_.clear();
        TreeNode root;
        root.config = start;
        root.parent = SIZE_MAX;
        root.cost = 0.0;
        tree_.push_back(root);
        
        for (size_t iter = 0; iter < max_iterations_; iter++) {
            // Sample random configuration or bias toward goal
            std::vector<double> random_config = 
                (iter % 20 == 0) ? goal : sampleConfiguration_();  // 5% chance to sample goal
            
            // Find nearest node in tree
            size_t nearest_idx = findNearestNode(random_config);
            
            // Steer from nearest node toward random configuration
            std::vector<double> new_config = steer(tree_[nearest_idx].config, random_config, step_size_);
            
            // Check if new configuration is collision-free
            if (isCollisionFree_(new_config) && 
                isPathCollisionFree(tree_[nearest_idx].config, new_config)) {
                
                // Find best parent for new configuration
                auto [best_parent, best_cost] = findBestParent(new_config, nearest_idx);
                
                if (best_parent != SIZE_MAX) {
                    // Add new node to tree
                    TreeNode new_node;
                    new_node.config = new_config;
                    new_node.parent = best_parent;
                    new_node.cost = best_cost;
                    size_t new_idx = tree_.size();
                    tree_.push_back(new_node);
                    
                    // Add new node as child of parent
                    tree_[best_parent].children.push_back(new_idx);
                    
                    // Rewire nearby nodes if cheaper path is found
                    rewireNearby(new_idx);
                    
                    // Check if we're close enough to goal
                    if (distanceMetric_(new_config, goal) < step_size_ && 
                        iter > max_iterations_ - 100) {  // Only consider late in planning
                        return extractOptimalPath(findBestGoalNode());
                    }
                }
            }
        }
        
        // Return best path to goal found
        size_t best_goal_node = findBestGoalNode();
        if (best_goal_node != SIZE_MAX) {
            return extractOptimalPath(best_goal_node);
        }
        
        return {};
    }

private:
    std::pair<size_t, double> findBestParent(const std::vector<double>& new_config, size_t nearest_idx) {
        // Find radius for neighborhood based on number of nodes and dimensionality
        double radius = gamma_ * std::pow(std::log(tree_.size()) / tree_.size(), 1.0 / new_config.size());
        
        size_t best_parent = nearest_idx;
        double best_cost = tree_[nearest_idx].cost + distanceMetric_(tree_[nearest_idx].config, new_config);
        
        // Check all nodes within radius
        for (size_t i = 0; i < tree_.size() - 1; i++) {  // Don't check the new node itself
            if (distanceMetric_(tree_[i].config, new_config) <= radius) {
                double potential_cost = tree_[i].cost + distanceMetric_(tree_[i].config, new_config);
                
                if (potential_cost < best_cost && 
                    isPathCollisionFree(tree_[i].config, new_config)) {
                    best_cost = potential_cost;
                    best_parent = i;
                }
            }
        }
        
        return {best_parent, best_cost};
    }
    
    void rewireNearby(size_t new_node_idx) {
        double radius = gamma_ * std::pow(std::log(tree_.size()) / tree_.size(), 1.0 / tree_[new_node_idx].config.size());
        
        for (size_t i = 0; i < tree_.size() - 1; i++) {  // Don't check the new node itself
            if (i == new_node_idx) continue;
            
            if (distanceMetric_(tree_[i].config, tree_[new_node_idx].config) <= radius) {
                double potential_cost = tree_[new_node_idx].cost + 
                                       distanceMetric_(tree_[new_node_idx].config, tree_[i].config);
                
                if (potential_cost < tree_[i].cost && 
                    isPathCollisionFree(tree_[new_node_idx].config, tree_[i].config)) {
                    // Update parent of node i
                    size_t old_parent = tree_[i].parent;
                    tree_[i].parent = new_node_idx;
                    tree_[i].cost = potential_cost;
                    
                    // Update parent's children list
                    if (old_parent != SIZE_MAX) {
                        auto& old_children = tree_[old_parent].children;
                        old_children.erase(std::remove(old_children.begin(), old_children.end(), i), 
                                          old_children.end());
                    }
                    
                    tree_[new_node_idx].children.push_back(i);
                    
                    // Propagate cost updates to descendants
                    updateDescendantCosts(i);
                }
            }
        }
    }
    
    void updateDescendantCosts(size_t node_idx) {
        double base_cost = tree_[node_idx].cost;
        
        for (size_t child_idx : tree_[node_idx].children) {
            tree_[child_idx].cost = base_cost + 
                                  distanceMetric_(tree_[node_idx].config, 
                                                tree_[child_idx].config);
            updateDescendantCosts(child_idx);  // Recursive update
        }
    }
    
    size_t findBestGoalNode() {
        size_t best_node = SIZE_MAX;
        double best_cost = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < tree_.size(); i++) {
            if (distanceMetric_(tree_[i].config, goal_) < step_size_ && 
                tree_[i].cost < best_cost) {
                best_cost = tree_[i].cost;
                best_node = i;
            }
        }
        
        return best_node;
    }
    
    std::vector<std::vector<double>> extractOptimalPath(size_t goal_node_idx) {
        if (goal_node_idx == SIZE_MAX) return {};
        
        std::vector<std::vector<double>> path;
        size_t current_idx = goal_node_idx;
        
        while (current_idx != SIZE_MAX) {
            path.push_back(tree_[current_idx].config);
            current_idx = tree_[current_idx].parent;
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }
    
    size_t findNearestNode(const std::vector<double>& config) {
        size_t nearest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < tree_.size(); i++) {
            double dist = distanceMetric_(config, tree_[i].config);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }
        
        return nearest_idx;
    }
    
    std::vector<double> steer(const std::vector<double>& from, 
                             const std::vector<double>& to, 
                             double max_dist) {
        
        std::vector<double> result = from;
        double dist = distanceMetric_(from, to);
        
        if (dist <= max_dist) {
            return to;  // Direct connection
        }
        
        // Move from toward to by max_dist
        for (size_t i = 0; i < from.size(); i++) {
            double direction = to[i] - from[i];
            result[i] += (direction / dist) * max_dist;
        }
        
        return result;
    }
    
    bool isPathCollisionFree(const std::vector<double>& start, 
                           const std::vector<double>& end) {
        // Check if path between start and end is collision-free
        int num_checks = 10;
        for (int i = 1; i < num_checks; i++) {
            double t = static_cast<double>(i) / num_checks;
            std::vector<double> intermediate;
            for (size_t j = 0; j < start.size(); j++) {
                intermediate.push_back(start[j] + t * (end[j] - start[j]));
            }
            
            if (!isCollisionFree_(intermediate)) {
                return false;
            }
        }
        return true;
    }
};
```

## Advanced Motion Planning Techniques

### Kinodynamic Planning

Kinodynamic planning considers both kinematic and dynamic constraints of the robot:

```cpp
class KinodynamicRRT {
private:
    struct StateNode {
        std::vector<double> position;
        std::vector<double> velocity;
        size_t parent;
        double time;
    };
    
    std::vector<StateNode> tree_;
    std::function<bool(const std::vector<double>&)> isCollisionFree_;
    std::function<bool(const std::vector<double>&, const std::vector<double>&, const std::vector<double>&)> isValidControl_;
    std::function<std::vector<double>(const std::vector<double>&, const std::vector<double>&, double, double)> integrateDynamics_;
    
    double integration_dt_;
    size_t max_iterations_;
    
public:
    KinodynamicRRT(double dt, size_t max_iter) 
        : integration_dt_(dt), max_iterations_(max_iter) {}
    
    std::vector<std::pair<std::vector<double>, std::vector<double>>> planPath(
        const std::vector<double>& start_pos,
        const std::vector<double>& start_vel,
        const std::vector<double>& goal_pos,
        const std::vector<double>& goal_vel) {
        
        // Initialize tree with start state
        tree_.clear();
        StateNode root;
        root.position = start_pos;
        root.velocity = start_vel;
        root.parent = SIZE_MAX;
        root.time = 0.0;
        tree_.push_back(root);
        
        for (size_t iter = 0; iter < max_iterations_; iter++) {
            // Sample random state or bias toward goal
            std::vector<double> random_pos, random_vel;
            if (iter % 10 == 0) {
                random_pos = goal_pos;
                random_vel = goal_vel;
            } else {
                random_pos = sampleConfiguration_();
                random_vel = sampleVelocity_();  // Sample random velocity
            }
            
            // Find nearest node in tree
            size_t nearest_idx = findNearestState({random_pos, random_vel});
            
            // Generate control to reach random state
            auto [control, duration] = generateControlToState(nearest_idx, random_pos, random_vel);
            
            if (isValidControl_(tree_[nearest_idx].position, 
                               tree_[nearest_idx].velocity, 
                               control)) {
                
                // Integrate dynamics to get new state
                auto [new_pos, new_vel] = integrateDynamics(
                    tree_[nearest_idx].position, 
                    tree_[nearest_idx].velocity, 
                    control, 
                    duration);
                
                // Check if path is collision-free
                if (isCollisionFree_(new_pos) && 
                    isPathCollisionFree(tree_[nearest_idx], new_pos, control, duration)) {
                    
                    // Add new state to tree
                    StateNode new_node;
                    new_node.position = new_pos;
                    new_node.velocity = new_vel;
                    new_node.parent = nearest_idx;
                    new_node.time = tree_[nearest_idx].time + duration;
                    tree_.push_back(new_node);
                    
                    // Check if we're close enough to goal
                    if (isNearGoal(new_node, goal_pos, goal_vel)) {
                        return extractStatePath(tree_.size() - 1);
                    }
                }
            }
        }
        
        return {};  // No path found
    }

private:
    struct StatePair {
        std::vector<double> position;
        std::vector<double> velocity;
    };
    
    size_t findNearestState(const StatePair& target) {
        size_t nearest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < tree_.size(); i++) {
            double pos_dist = distanceMetric_(tree_[i].position, target.position);
            double vel_dist = distanceMetric_(tree_[i].velocity, target.velocity);
            
            double combined_dist = pos_dist + vel_dist;  // Weighted combination
            if (combined_dist < min_dist) {
                min_dist = combined_dist;
                nearest_idx = i;
            }
        }
        
        return nearest_idx;
    }
    
    std::vector<std::pair<std::vector<double>, std::vector<double>>> extractStatePath(size_t goal_node_idx) {
        std::vector<std::pair<std::vector<double>, std::vector<double>>> path;
        size_t current_idx = goal_node_idx;
        
        while (current_idx != SIZE_MAX) {
            path.push_back({tree_[current_idx].position, tree_[current_idx].velocity});
            current_idx = tree_[current_idx].parent;
        }
        
        std::reverse(path.begin(), path.end());
        return path;
    }
    
    bool isNearGoal(const StateNode& state, 
                   const std::vector<double>& goal_pos, 
                   const std::vector<double>& goal_vel) {
        double pos_threshold = 0.1;  // 10cm threshold
        double vel_threshold = 0.1;  // 0.1 rad/s threshold
        
        double pos_dist = distanceMetric_(state.position, goal_pos);
        double vel_dist = distanceMetric_(state.velocity, goal_vel);
        
        return (pos_dist < pos_threshold && vel_dist < vel_threshold);
    }
    
    std::pair<std::vector<double>, double> generateControlToState(
        size_t from_idx, 
        const std::vector<double>& target_pos, 
        const std::vector<double>& target_vel) {
        
        // Simple control generation - in practice, this would solve a control problem
        std::vector<double> control(tree_[from_idx].position.size());
        
        for (size_t i = 0; i < control.size(); i++) {
            control[i] = (target_pos[i] - tree_[from_idx].position[i]) * 0.1;  // Proportional control
        }
        
        double duration = 1.0;  // Fixed duration for simplicity
        
        return {control, duration};
    }
    
    bool isPathCollisionFree(const StateNode& start_node,
                           const std::vector<double>& end_pos,
                           const std::vector<double>& control,
                           double duration) {
        // Simulate path and check for collisions at intermediate points
        int num_steps = static_cast<int>(duration / integration_dt_);
        std::vector<double> current_pos = start_node.position;
        std::vector<double> current_vel = start_node.velocity;
        
        for (int step = 1; step <= num_steps; step++) {
            double t = step * integration_dt_;
            auto [next_pos, next_vel] = integrateDynamics(current_pos, current_vel, control, integration_dt_);
            
            if (!isCollisionFree_(next_pos)) {
                return false;
            }
            
            current_pos = next_pos;
            current_vel = next_vel;
        }
        
        return true;
    }
};
```

### Optimization-Based Planning

#### CHOMP (Covariant Hamiltonian Optimization for Motion Planning)
```cpp
class CHOMPPlanner {
private:
    std::vector<std::vector<double>> trajectory_;
    std::function<double(const std::vector<double>&)> collisionCost_;
    std::function<std::vector<double>(const std::vector<double>&)> gradientCollisionCost_;
    std::function<double(const std::vector<double>&, const std::vector<double>&)> distanceCost_;
    
    double learning_rate_;
    int max_iterations_;
    double obstacle_influence_distance_;
    
public:
    CHOMPPlanner(double lr, int max_iter, double obs_dist) 
        : learning_rate_(lr), max_iterations_(max_iter), 
          obstacle_influence_distance_(obs_dist) {}
    
    std::vector<std::vector<double>> planPath(
        const std::vector<double>& start, 
        const std::vector<double>& goal,
        const std::vector<std::vector<double>>& initial_trajectory) {
        
        trajectory_ = initial_trajectory;
        
        // Optimize trajectory using gradient descent
        for (int iter = 0; iter < max_iterations_; iter++) {
            // Compute gradients for each point in trajectory
            std::vector<std::vector<double>> gradients = computeGradients();
            
            // Update trajectory
            updateTrajectory(gradients, learning_rate_);
            
            // Check for convergence
            if (isConverged(gradients)) {
                break;
            }
        }
        
        return trajectory_;
    }

private:
    std::vector<std::vector<double>> computeGradients() {
        std::vector<std::vector<double>> gradients(trajectory_.size());
        
        for (size_t i = 0; i < trajectory_.size(); i++) {
            // Gradient of collision cost
            auto collision_grad = gradientCollisionCost_(trajectory_[i]);
            
            // Gradient of smoothness cost (penalizes acceleration)
            std::vector<double> smoothness_grad(trajectory_[i].size(), 0.0);
            if (i > 0 && i < trajectory_.size() - 1) {
                for (size_t j = 0; j < trajectory_[i].size(); j++) {
                    // Approximate acceleration: (x[i-1] - 2*x[i] + x[i+1])
                    smoothness_grad[j] = trajectory_[i-1][j] - 2*trajectory_[i][j] + trajectory_[i+1][j];
                }
            }
            
            // Combine gradients
            gradients[i].resize(trajectory_[i].size());
            for (size_t j = 0; j < gradients[i].size(); j++) {
                gradients[i][j] = collision_grad[j] + 0.1 * smoothness_grad[j];  // Weighted combination
            }
        }
        
        return gradients;
    }
    
    void updateTrajectory(const std::vector<std::vector<double>>& gradients, 
                         double learning_rate) {
        for (size_t i = 1; i < trajectory_.size() - 1; i++) {  // Don't update start/end points
            for (size_t j = 0; j < trajectory_[i].size(); j++) {
                trajectory_[i][j] -= learning_rate * gradients[i][j];
            }
        }
    }
    
    bool isConverged(const std::vector<std::vector<double>>& gradients) {
        double threshold = 0.001;
        
        for (const auto& grad : gradients) {
            double norm = 0.0;
            for (double g : grad) {
                norm += g * g;
            }
            if (std::sqrt(norm) > threshold) {
                return false;
            }
        }
        
        return true;
    }
    
    double computeTotalCost() {
        double collision_cost = 0.0;
        double smoothness_cost = 0.0;
        
        for (size_t i = 0; i < trajectory_.size(); i++) {
            collision_cost += collisionCost_(trajectory_[i]);
        }
        
        for (size_t i = 1; i < trajectory_.size() - 1; i++) {
            for (size_t j = 0; j < trajectory_[i].size(); j++) {
                double acceleration = trajectory_[i-1][j] - 2*trajectory_[i][j] + trajectory_[i+1][j];
                smoothness_cost += acceleration * acceleration;
            }
        }
        
        return collision_cost + 0.1 * smoothness_cost;
    }
};
```

## Integration with Physical AI Systems

### Motion Planning in ROS 2

#### MoveIt! Integration
```cpp
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/display_trajectory.hpp"

class MotionPlanningInterface {
private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_publisher_;
    
    std::string planning_group_;
    
public:
    MotionPlanningInterface(const std::string& group_name) 
        : planning_group_(group_name) {
        
        node_ = std::make_shared<rclcpp::Node>("motion_planning_interface");
        
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            node_, planning_group_);
        
        planning_scene_interface_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
        
        display_trajectory_publisher_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
            "display_planned_path", 1);
    }
    
    bool planToPose(const geometry_msgs::msg::Pose& target_pose) {
        // Set target pose
        move_group_->setPoseTarget(target_pose);
        
        // Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Motion plan successful!");
            
            // Publish trajectory for visualization
            moveit_msgs::msg::DisplayTrajectory display_trajectory;
            display_trajectory.trajectory_start = plan.start_state_;
            display_trajectory.trajectory.push_back(plan.trajectory_);
            
            display_trajectory_publisher_->publish(display_trajectory);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Motion planning failed!");
        }
        
        return success;
    }
    
    bool planToJointValues(const std::vector<double>& joint_values) {
        // Set target joint values
        move_group_->setJointValueTarget(joint_values);
        
        // Plan the motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Joint space motion plan successful!");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Joint space motion planning failed!");
        }
        
        return success;
    }
    
    bool executePlan() {
        // Execute the most recently planned motion
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        move_group_->getCurrentState()->setJointGroupPositions(
            move_group_->getCurrentState()->getJointModelGroup(planning_group_),
            move_group_->getJointValueTarget());
        
        bool success = (move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(node_->get_logger(), "Motion execution successful!");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Motion execution failed!");
        }
        
        return success;
    }
    
    void addCollisionObjects() {
        // Add collision objects to the planning scene
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
        
        // Example: Add a table
        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = move_group_->getPlanningFrame();
        table.id = "table";
        
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {1.0, 1.5, 0.7};  // width, depth, height
        
        geometry_msgs::msg::Pose pose;
        pose.position.x = 0.5;
        pose.position.y = 0.0;
        pose.position.z = 0.35;
        pose.orientation.w = 1.0;
        
        table.primitives.push_back(primitive);
        table.primitive_poses.push_back(pose);
        table.operation = table.ADD;
        
        collision_objects.push_back(table);
        
        // Apply collision objects to planning scene
        planning_scene_interface_->applyCollisionObjects(collision_objects);
    }
    
    std::vector<double> getCurrentJointValues() {
        return move_group_->getCurrentJointValues();
    }
    
    geometry_msgs::msg::Pose getCurrentPose() {
        return move_group_->getCurrentPose().pose;
    }
};
```

### Navigation Integration

#### Navigation2 with Custom Planners
```cpp
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"

class CustomPlannerServer : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::ComputePathToPose> {
public:
    CustomPlannerServer(
        const std::string& action_name,
        const BT::NodeConfiguration& conf)
        : nav2_behavior_tree::BtActionNode<nav2_msgs::action::ComputePathToPose>(
            action_name, conf)
    {
        // Initialize custom motion planner
        motion_planner_ = std::make_unique<RRTStarPlanner>(0.1, 10000, 10.0);
    }

    BT::NodeStatus on_tick() override {
        // Get goal from blackboard
        geometry_msgs::msg::PoseStamped goal;
        if (!getInput("goal", goal)) {
            RCLCPP_ERROR(node_->get_logger(), "Could not get goal from blackboard");
            return BT::NodeStatus::FAILURE;
        }
        
        // Get start pose
        geometry_msgs::msg::PoseStamped start;
        if (!getInput("start", start)) {
            RCLCPP_ERROR(node_->get_logger(), "Could not get start from blackboard");
            return BT::NodeStatus::FAILURE;
        }
        
        // Get costmap
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros;
        getInput("costmap_ros", costmap_ros);
        
        // Plan path using custom planner
        auto path = planPath(start.pose, goal.pose, costmap_ros->getCostmap());
        
        if (!path.poses.empty()) {
            // Set output path
            setOutput("path", path);
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Could not find a path to the goal");
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    std::unique_ptr<RRTStarPlanner> motion_planner_;
    
    nav_msgs::msg::Path planPath(const geometry_msgs::msg::Pose& start_pose,
                                const geometry_msgs::msg::Pose& goal_pose,
                                const nav2_costmap_2d::Costmap2D* costmap) {
        
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = node_->now();
        
        // Convert ROS poses to planner format
        std::vector<double> start_config = {start_pose.position.x, start_pose.position.y};
        std::vector<double> goal_config = {goal_pose.position.x, goal_pose.position.y};
        
        // Create collision checker function
        auto collision_checker = [costmap](const std::vector<double>& config) -> bool {
            unsigned int mx, my;
            if (costmap->worldToMap(config[0], config[1], mx, my)) {
                unsigned char cost = costmap->getCost(mx, my);
                return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;  // Not in obstacle
            }
            return false;  // Outside map bounds
        };
        
        // Create distance metric function
        auto distance_metric = [](const std::vector<double>& a, 
                                const std::vector<double>& b) -> double {
            return std::sqrt((a[0] - b[0]) * (a[0] - b[0]) + 
                           (a[1] - b[1]) * (a[1] - b[1]));
        };
        
        // Create sampling function
        auto sampler = [costmap]() -> std::vector<double> {
            // Sample random configuration in map bounds
            double x = (rand() / (double)RAND_MAX) * costmap->getSizeInMetersX() + costmap->getOriginX();
            double y = (rand() / (double)RAND_MAX) * costmap->getSizeInMetersY() + costmap->getOriginY();
            return {x, y};
        };
        
        // Set up planner
        motion_planner_->setCollisionChecker(collision_checker);
        motion_planner_->setDistanceMetric(distance_metric);
        motion_planner_->setSampler(sampler);
        
        // Plan path
        auto planned_path = motion_planner_->planPath(start_config, goal_config);
        
        // Convert to ROS message format
        for (const auto& config : planned_path) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path_msg.header;
            pose_stamped.pose.position.x = config[0];
            pose_stamped.pose.position.y = config[1];
            pose_stamped.pose.position.z = 0.0;
            pose_stamped.pose.orientation.w = 1.0;  // Identity quaternion
            
            path_msg.poses.push_back(pose_stamped);
        }
        
        return path_msg;
    }
    
    // Helper function to check if path is collision-free
    bool isPathCollisionFree(const std::vector<double>& start, 
                           const std::vector<double>& end,
                           const nav2_costmap_2d::Costmap2D* costmap) {
        // Check multiple intermediate points along the path
        int num_checks = 10;
        for (int i = 1; i < num_checks; i++) {
            double t = static_cast<double>(i) / num_checks;
            double x = start[0] + t * (end[0] - start[0]);
            double y = start[1] + t * (end[1] - start[1]);
            
            unsigned int mx, my;
            if (costmap->worldToMap(x, y, mx, my)) {
                unsigned char cost = costmap->getCost(mx, my);
                if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                    return false;  // Collision detected
                }
            }
        }
        return true;  // Path is collision-free
    }
};
```

## Performance Optimization

### Algorithm Performance

#### Computational Complexity
- **A* and Dijkstra**: O(E + V log V) where E is edges and V is vertices
- **PRM**: O(V²) for connection phase, O(log V) for query phase
- **RRT**: O(n) per iteration, probabilistically complete
- **RRT***: O(n) per iteration, asymptotically optimal

#### Memory Usage
- **Grid-based**: Memory scales with resolution cubed
- **Sampling-based**: Memory scales linearly with samples
- **Optimization-based**: Memory for trajectory storage and gradients

### Real-time Considerations

#### Incremental Planning
- **Replanning**: Update plan as new information becomes available
- **Partial Execution**: Execute part of plan while computing next part
- **Dynamic Obstacles**: Handle moving obstacles in real-time

#### Multi-threading
- **Parallel Sampling**: Sample multiple configurations simultaneously
- **Path Validation**: Validate multiple paths in parallel
- **Collision Checking**: Check collisions in parallel for different configurations

### Planning Optimization Techniques

#### Lazy Collision Checking
```cpp
class LazyCollisionChecker {
private:
    std::function<bool(const std::vector<double>&)> expensiveCollisionCheck_;
    std::function<bool(const std::vector<double>&)> cheapCollisionCheck_;
    std::vector<std::vector<double>> cachedCollisions_;
    
public:
    LazyCollisionChecker(
        std::function<bool(const std::vector<double>&)> expensive_check,
        std::function<bool(const std::vector<double>&)> cheap_check)
        : expensiveCollisionCheck_(expensive_check), cheapCollisionCheck_(cheap_check) {}
    
    bool isCollisionFree(const std::vector<double>& config) {
        // First, try cheap collision check
        if (!cheapCollisionCheck_(config)) {
            return false;  // Definitely in collision
        }
        
        // If cheap check passes, do expensive check with caching
        auto it = std::find(cachedCollisions_.begin(), cachedCollisions_.end(), config);
        if (it != cachedCollisions_.end()) {
            return true;  // Cached as collision-free
        }
        
        // Check expensive collision
        bool collision_free = expensiveCollisionCheck_(config);
        if (collision_free) {
            cachedCollisions_.push_back(config);  // Cache result
        }
        
        return collision_free;
    }
};
```

#### Hierarchical Planning
- **Coarse-to-Fine**: Plan on coarse grid, refine on finer grids
- **Multi-Resolution**: Use different resolutions for different areas
- **Subspace Planning**: Plan in lower-dimensional subspaces first

## Troubleshooting Common Issues

### Planning Problems

#### No Path Found
- **Symptoms**: Planner fails to find any path between start and goal
- **Causes**: Start or goal in collision, disconnected free space, insufficient planning time
- **Solutions**: Verify start/goal validity, increase planning time, check map quality

#### Infeasible Paths
- **Symptoms**: Planned path is collision-free but robot cannot follow it
- **Causes**: Ignored kinematic/dynamic constraints, discretization errors
- **Solutions**: Use kinodynamic planners, add constraint checking, smooth paths

#### Poor Path Quality
- **Symptoms**: Path is valid but suboptimal (too long, jerky, etc.)
- **Causes**: Insufficient planning time, poor heuristics, inadequate sampling
- **Solutions**: Use optimization-based methods, improve sampling strategies, add post-processing

### Performance Issues

#### Slow Planning
- **Symptoms**: Long planning times, real-time performance issues
- **Causes**: High-dimensional spaces, dense maps, inefficient algorithms
- **Solutions**: Use sampling-based methods, improve heuristics, reduce dimensionality

#### Memory Issues
- **Symptoms**: High memory usage, system slowdown
- **Causes**: Large roadmap, many samples, complex data structures
- **Solutions**: Memory optimization, data structure optimization, pruning strategies

## Safety Considerations

### Safety in Motion Planning

#### Safety Margins
- **Inflation**: Inflate obstacles to create safety margins
- **Velocity Limits**: Plan paths that respect velocity constraints
- **Acceleration Limits**: Plan paths that respect acceleration constraints
- **Dynamic Obstacles**: Account for moving obstacles in planning

#### Emergency Planning
- **Emergency Stops**: Plan for immediate stopping capability
- **Safe Positions**: Pre-plan safe positions for emergency situations
- **Escape Routes**: Plan alternative routes for obstacle avoidance

### Validation and Verification

#### Path Validation
- **Collision Checking**: Verify entire path is collision-free
- **Kinematic Validation**: Verify path respects kinematic constraints
- **Dynamic Validation**: Verify path respects dynamic constraints
- **Execution Validation**: Test path execution in simulation

## Future Developments

### Learning-Based Motion Planning

#### Neural Motion Planning
- **Learning Planners**: Neural networks that learn to plan
- **Imitation Learning**: Learning from expert demonstrations
- **Reinforcement Learning**: Learning optimal planning strategies
- **Generalization**: Learning to plan in unseen environments

#### Data-Driven Approaches
- **Experience-Based Planning**: Using past planning experience
- **Demonstration Integration**: Learning from human demonstrations
- **Multi-Task Learning**: Learning to plan for multiple tasks simultaneously

### Advanced Integration

#### Perception-Action Integration
- **Online Planning**: Planning based on real-time perception
- **Uncertainty Handling**: Planning with uncertain perception data
- **Adaptive Planning**: Adjusting plans based on perception feedback

#### Human-Robot Collaboration
- **Intent Prediction**: Planning based on predicted human intent
- **Collaborative Planning**: Planning that considers human actions
- **Interactive Planning**: Planning with human input and feedback

## Conclusion

Motion planning is a critical component of Physical AI systems, enabling robots to navigate their environment while avoiding obstacles and respecting their physical constraints. The choice of planning algorithm depends on the specific requirements of the application, including the dimensionality of the configuration space, the complexity of the environment, and the real-time performance requirements.

Modern motion planning algorithms balance computational efficiency with solution quality, providing both feasible and optimal solutions for a wide range of robotic applications. The integration of these algorithms with other Physical AI components like perception, control, and navigation creates comprehensive systems capable of operating effectively in complex environments.

As robotics applications become more sophisticated and operate in more dynamic environments, motion planning algorithms must continue to evolve with improvements in efficiency, robustness, and adaptability. The future of motion planning lies in the integration of traditional planning methods with learning-based approaches that can adapt to new environments and tasks.

Understanding these motion planning techniques and their integration with Physical AI systems is essential for creating robots that can operate autonomously and safely in real-world environments.

## Exercises

1. Implement and compare different motion planning algorithms (A*, RRT, RRT*) for a 2D navigation task, analyzing their performance characteristics.
2. Design a motion planning system that incorporates both kinematic and dynamic constraints for a robotic manipulator.
3. Create a hierarchical planning system that uses coarse global planning with fine local planning for navigation.

## Further Reading

- LaValle, S. M. (2006). "Planning Algorithms." Cambridge University Press.
- Siciliano, B., & Khatib, O. (Eds.). (2016). "Springer Handbook of Robotics." Springer.
- Kavraki, L. E., Svestka, P., Latombe, J. C., & Overmars, M. H. (1996). "Probabilistic roadmaps for path planning in high-dimensional configuration spaces."
- LaValle, S. M., & Kuffner, J. J. (2001). "Randomized kinodynamic planning."
- Kalakrishnan, M., et al. (2011). "STOMP: Stochastic trajectory optimization for motion planning."