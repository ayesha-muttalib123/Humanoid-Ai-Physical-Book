---
sidebar_label: Motion Planning Algorithms
title: Motion Planning Algorithms - Path Planning and Obstacle Avoidance
description: Understanding motion planning algorithms for robotics including path planning, obstacle avoidance, and navigation
keywords: [motion planning, path planning, obstacle avoidance, navigation, robotics, A*, RRT, PRM, algorithms]
---

# 9.3 Motion Planning Algorithms

## Introduction

Motion planning is a critical component of Physical AI systems that enables robots to navigate from their current state to a desired goal while avoiding obstacles and satisfying various constraints. In robotics applications, motion planning bridges the gap between high-level task planning and low-level control, generating feasible paths that respect the robot's kinematic and dynamic constraints while considering environmental obstacles.

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

## Optimization-Based Planning Algorithms

### CHOMP (Covariant Hamiltonian Optimization for Motion Planning)

CHOMP is an optimization-based planning algorithm that improves path quality by optimizing the entire trajectory.

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
    double smoothness_cost_weight_;
    double collision_cost_weight_;
    
public:
    CHOMPPlanner(double lr, int max_iter, double obs_dist, 
                double smooth_weight = 0.1, double collision_weight = 1.0) 
        : learning_rate_(lr), max_iterations_(max_iter), 
          obstacle_influence_distance_(obs_dist),
          smoothness_cost_weight_(smooth_weight),
          collision_cost_weight_(collision_weight) {}
    
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
            for (size_t j = 0; i < gradients[i].size(); i++) {
                gradients[i][j] = collision_cost_weight_ * collision_grad[j] + 
                                 smoothness_cost_weight_ * smoothness_grad[j];
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
        
        return collision_cost + smoothness_cost;
    }
};
```

### STOMP (Stochastic Trajectory Optimization for Motion Planning)

STOMP uses stochastic optimization to improve trajectory quality.

```cpp
class STOMPPlanner {
private:
    std::vector<std::vector<double>> trajectory_;
    std::function<double(const std::vector<double>&)> costFunction_;
    std::function<std::vector<std::vector<double>>()> generatePerturbations_;
    
    int num_iterations_;
    int num_perturbations_;
    double learning_rate_;
    double initial_temp_;
    double final_temp_;
    
public:
    STOMPPlanner(int num_iter, int num_pert, double lr, double init_temp, double final_temp)
        : num_iterations_(num_iter), num_perturbations_(num_pert), 
          learning_rate_(lr), initial_temp_(init_temp), final_temp_(final_temp) {}
    
    std::vector<std::vector<double>> planPath(
        const std::vector<double>& start, 
        const std::vector<double>& goal,
        const std::vector<std::vector<double>>& initial_trajectory) {
        
        trajectory_ = initial_trajectory;
        
        // Run STOMP optimization
        for (int iter = 0; iter < num_iterations_; iter++) {
            // Generate perturbations
            auto perturbations = generatePerturbations_();
            
            // Evaluate cost of perturbed trajectories
            std::vector<double> perturbation_costs(num_perturbations_);
            for (int i = 0; i < num_perturbations_; i++) {
                auto perturbed_traj = applyPerturbation(trajectory_, perturbations[i]);
                perturbation_costs[i] = evaluateTrajectoryCost(perturbed_traj);
            }
            
            // Calculate temperature for this iteration
            double temp = calculateTemperature(iter);
            
            // Calculate probability weights for perturbations
            auto weights = calculateProbabilityWeights(perturbation_costs, temp);
            
            // Update trajectory based on weighted perturbations
            updateTrajectoryWithWeights(perturbations, weights);
        }
        
        return trajectory_;
    }

private:
    std::vector<std::vector<double>> generatePerturbations() {
        std::vector<std::vector<double>> perturbations(num_perturbations_);
        
        // Generate random perturbations for each point in trajectory
        for (int i = 0; i < num_perturbations_; i++) {
            perturbations[i].resize(trajectory_.size());
            for (size_t j = 0; j < trajectory_.size(); j++) {
                // Generate random perturbation vector
                std::vector<double> perturbation(trajectory_[j].size());
                for (size_t k = 0; k < trajectory_[j].size(); k++) {
                    perturbation[k] = generateRandomPerturbation(j, k);
                }
                perturbations[i][j] = perturbation;
            }
        }
        
        return perturbations;
    }
    
    std::vector<std::vector<double>> applyPerturbation(
        const std::vector<std::vector<double>>& base_trajectory,
        const std::vector<std::vector<double>>& perturbation) {
        
        auto result = base_trajectory;
        
        // Apply perturbation to each point in trajectory
        for (size_t i = 1; i < result.size() - 1; i++) {  // Don't perturb start/end points
            for (size_t j = 0; j < result[i].size(); j++) {
                result[i][j] += perturbation[i][j];
            }
        }
        
        return result;
    }
    
    double evaluateTrajectoryCost(const std::vector<std::vector<double>>& trajectory) {
        double total_cost = 0.0;
        
        // Calculate collision cost for each point in trajectory
        for (const auto& point : trajectory) {
            total_cost += costFunction_(point);
        }
        
        // Add smoothness cost
        total_cost += calculateSmoothnessCost(trajectory);
        
        return total_cost;
    }
    
    double calculateSmoothnessCost(const std::vector<std::vector<double>>& trajectory) {
        double smoothness_cost = 0.0;
        
        // Penalize sharp changes in trajectory (approximate acceleration)
        for (size_t i = 1; i < trajectory.size() - 1; i++) {
            for (size_t j = 0; j < trajectory[i].size(); j++) {
                double acceleration = trajectory[i-1][j] - 2*trajectory[i][j] + trajectory[i+1][j];
                smoothness_cost += acceleration * acceleration;
            }
        }
        
        return smoothness_cost;
    }
    
    std::vector<double> calculateProbabilityWeights(
        const std::vector<double>& costs, double temperature) {
        
        std::vector<double> weights(costs.size());
        
        // Calculate maximum cost to avoid numerical issues
        double max_cost = *std::max_element(costs.begin(), costs.end());
        
        // Calculate Boltzmann weights
        double sum_weights = 0.0;
        for (size_t i = 0; i < costs.size(); i++) {
            weights[i] = std::exp(-(costs[i] - max_cost) / temperature);
            sum_weights += weights[i];
        }
        
        // Normalize weights
        for (auto& weight : weights) {
            weight /= sum_weights;
        }
        
        return weights;
    }
    
    void updateTrajectoryWithWeights(
        const std::vector<std::vector<std::vector<double>>>& perturbations,
        const std::vector<double>& weights) {
        
        // Calculate weighted average of perturbations
        for (size_t i = 1; i < trajectory_.size() - 1; i++) {  // Don't update start/end points
            for (size_t j = 0; j < trajectory_[i].size(); j++) {
                double weighted_perturbation = 0.0;
                for (size_t k = 0; k < perturbations.size(); k++) {
                    weighted_perturbation += weights[k] * perturbations[k][i][j];
                }
                
                // Update trajectory with weighted perturbation
                trajectory_[i][j] += learning_rate_ * weighted_perturbation;
            }
        }
    }
    
    double generateRandomPerturbation(size_t point_idx, size_t dim_idx) {
        // Generate random perturbation using appropriate distribution
        // This could be Gaussian, uniform, or other distribution
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::normal_distribution<> d(0.0, 0.1);  // Mean 0, std 0.1
        
        return d(gen);
    }
    
    double calculateTemperature(int iteration) {
        // Linear temperature schedule
        double progress = static_cast<double>(iteration) / num_iterations_;
        return initial_temp_ + progress * (final_temp_ - initial_temp_);
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
        root.parent = SIZE_MAX;  // Root has no parent
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
                    
                    // Update null-space projector for lower-priority tasks
                    Eigen::MatrixXd N = Eigen::MatrixXd::Identity(tree_[nearest_idx].position.size(), tree_[nearest_idx].position.size()) - 
                                       computeNullSpaceProjection(nearest_idx);
                    updateNullSpaceProjector(N);
                    
                    // Check if we're close enough to goal
                    if (isNearGoal(new_node, goal_pos, goal_vel)) {
                        return extractStatePath(tree_.size() - 1);
                    }
                }
            }
        }
        
        // No path found within max iterations
        return {};
    }

private:
    size_t findNearestState(const std::pair<std::vector<double>, std::vector<double>>& target_state) {
        size_t nearest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t i = 0; i < tree_.size(); i++) {
            // Calculate distance in both position and velocity space
            double pos_dist = distanceMetric_(tree_[i].position, target_state.first);
            double vel_dist = distanceMetric_(tree_[i].velocity, target_state.second);
            
            // Combined distance metric
            double combined_dist = pos_dist + vel_dist;  // Weighted combination
            if (combined_dist < min_dist) {
                min_dist = combined_dist;
                nearest_idx = i;
            }
        }
        
        return nearest_idx;
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
    
    std::pair<std::vector<double>, std::vector<double>> integrateDynamics(
        const std::vector<double>& pos, 
        const std::vector<double>& vel,
        const std::vector<double>& control, 
        double duration) {
        
        // Simple dynamics integration
        // In practice, this would use more sophisticated integration
        auto new_pos = pos;
        auto new_vel = vel;
        
        // Integration steps
        int num_steps = static_cast<int>(duration / integration_dt_);
        double dt = duration / num_steps;
        
        for (int step = 0; step < num_steps; step++) {
            // Simple integration (in practice, use RK4 or other methods)
            for (size_t i = 0; i < new_pos.size(); i++) {
                new_vel[i] += control[i] * dt;  // Apply control
                new_pos[i] += new_vel[i] * dt;  // Apply velocity
            }
        }
        
        return {new_pos, new_vel};
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
    
    bool isNearGoal(const StateNode& state, 
                   const std::vector<double>& goal_pos, 
                   const std::vector<double>& goal_vel) {
        double pos_threshold = 0.1;  // 10cm threshold
        double vel_threshold = 0.1;  // 0.1 rad/s threshold
        
        double pos_dist = distanceMetric_(state.position, goal_pos);
        double vel_dist = distanceMetric_(state.velocity, goal_vel);
        
        return (pos_dist < pos_threshold && vel_dist < vel_threshold);
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
    
    Eigen::MatrixXd computeNullSpaceProjection(size_t node_idx) {
        // Compute null-space projection for the state node
        // This would involve computing the Jacobian and null-space
        return Eigen::MatrixXd();  // Placeholder
    }
    
    void updateNullSpaceProjector(const Eigen::MatrixXd& N) {
        // Update the null-space projector for future nodes
        // This would involve maintaining the projector in the planning process
    }
    
    std::function<double(const std::vector<double>&, const std::vector<double>&)> distanceMetric_ = 
        [](const std::vector<double>& a, const std::vector<double>& b) -> double {
            double sum = 0.0;
            for (size_t i = 0; i < a.size(); i++) {
                sum += (a[i] - b[i]) * (a[i] - b[i]);
            }
            return std::sqrt(sum);
        };
    
    std::function<std::vector<double>()> sampleConfiguration_ = 
        []() -> std::vector<double> {
            // Sample random configuration
            return std::vector<double>();  // Placeholder
        };
    
    std::function<std::vector<double>()> sampleVelocity_ = 
        []() -> std::vector<double> {
            // Sample random velocity
            return std::vector<double>();  // Placeholder
        };
};
```

## Integration with Physical AI Systems

### Motion Planning in ROS 2

#### Navigation2 Integration
```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/behavior_tree_engine.hpp"

class AdvancedMotionPlannerNode : public rclcpp::Node {
private:
    rclcpp_action::Server<nav2_msgs::action::NavigateToPose>::SharedPtr navigation_server_;
    std::unique_ptr<AdvancedMotionPlanner> motion_planner_;
    std::unique_ptr<CollisionDetector> collision_detector_;
    std::unique_ptr<CostmapInterface> costmap_interface_;
    
    rclcpp::TimerBase::SharedPtr planning_timer_;
    std::chrono::milliseconds planning_frequency_;
    
    // Current navigation state
    geometry_msgs::msg::PoseStamped current_goal_;
    std::vector<geometry_msgs::msg::PoseStamped> current_path_;
    bool navigation_active_;

public:
    AdvancedMotionPlannerNode() : Node("advanced_motion_planner"),
                                planning_frequency_(std::chrono::milliseconds(100)),
                                navigation_active_(false) {
        
        // Initialize motion planner
        motion_planner_ = std::make_unique<RRTStarPlanner>(0.1, 10000, 10.0);
        
        // Initialize collision detector
        collision_detector_ = std::make_unique<CollisionDetector>();
        
        // Initialize costmap interface
        costmap_interface_ = std::make_unique<CostmapInterface>();
        
        // Create navigation server
        navigation_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "navigate_to_pose",
            std::bind(&AdvancedMotionPlannerNode::handleGoal, this, 
                     std::placeholders::_1, std::placeholders::_2),
            std::bind(&AdvancedMotionPlannerNode::handleCancel, this, 
                     std::placeholders::_1),
            std::bind(&AdvancedMotionPlannerNode::handleAccepted, this, 
                     std::placeholders::_1)
        );
        
        // Initialize planning timer
        planning_timer_ = this->create_wall_timer(
            planning_frequency_,
            std::bind(&AdvancedMotionPlannerNode::planningCallback, this));
    }

private:
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal) {
        
        RCLCPP_INFO(this->get_logger(), "Received navigation goal");
        
        // Check if navigation is currently active
        if (navigation_active_) {
            RCLCPP_WARN(this->get_logger(), "Navigation already active, rejecting new goal");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        // Validate goal pose
        if (!isGoalValid(goal->pose)) {
            RCLCPP_WARN(this->get_logger(), "Goal pose is invalid, rejecting goal");
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        // Store goal
        current_goal_ = goal->pose;
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
        
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        
        // Cancel current navigation
        navigation_active_ = false;
        
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handleAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
        
        // This needs to be run in a separate thread to avoid blocking
        using namespace std::placeholders;
        std::thread{std::bind(&AdvancedMotionPlannerNode::execute, this, _1), goal_handle}.detach();
    }
    
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing navigation goal...");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<nav2_msgs::action::NavigateToPose::Feedback>();
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
        
        // Get current robot pose
        auto current_pose = getCurrentRobotPose();
        
        // Plan path using advanced motion planner
        auto path = motion_planner_->planPath(
            convertToVector(current_pose.pose.position),
            convertToVector(current_pose.pose.orientation),
            convertToVector(goal->pose.pose.position),
            convertToVector(goal->pose.pose.orientation));
        
        if (path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Could not find a path to the goal");
            result->error_code = nav2_msgs::action::NavigateToPose::Result::FAILURE;
            goal_handle->succeed(result);
            return;
        }
        
        // Convert path to ROS message format
        current_path_ = convertToPoseStampedPath(path);
        
        // Execute path following
        rclcpp::Rate rate(10);  // 10 Hz control rate
        size_t current_waypoint = 0;
        
        while (rclcpp::ok() && current_waypoint < current_path_.size()) {
            // Check if goal was canceled
            if (goal_handle->is_canceling()) {
                // Stop robot and set result
                stopRobot();
                result->error_code = nav2_msgs::action::NavigateToPose::Result::CANCELED;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Navigation was canceled");
                return;
            }
            
            // Follow current waypoint
            auto robot_pose = getCurrentRobotPose();
            auto control_cmd = calculateControlCommand(robot_pose, current_path_[current_waypoint]);
            
            // Send control command to robot
            sendControlCommand(control_cmd);
            
            // Check if reached current waypoint
            if (isNearWaypoint(robot_pose, current_path_[current_waypoint])) {
                current_waypoint++;
            }
            
            // Publish feedback
            feedback->current_pose = robot_pose;
            goal_handle->publish_feedback(feedback);
            
            rate.sleep();
        }
        
        // Check if completed successfully
        auto final_pose = getCurrentRobotPose();
        if (isNearGoal(final_pose, goal->pose)) {
            RCLCPP_INFO(this->get_logger(), "Navigation completed successfully");
            result->error_code = nav2_msgs::action::NavigateToPose::Result::SUCCESS;
            goal_handle->succeed(result);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Navigation did not reach goal within tolerance");
            result->error_code = nav2_msgs::action::NavigateToPose::Result::FAILURE;
            goal_handle->succeed(result);
        }
    }
    
    void planningCallback() {
        // Periodic planning updates for dynamic environments
        if (navigation_active_ && current_path_.size() > 0) {
            // Check for changes in environment
            auto updated_costmap = costmap_interface_->getUpdatedCostmap();
            
            // Check if current path is still valid
            if (!isPathStillValid(current_path_, updated_costmap)) {
                // Replan path if environment has changed significantly
                RCLCPP_INFO(this->get_logger(), "Replanning path due to environment changes");
                
                auto current_pose = getCurrentRobotPose();
                auto new_path = motion_planner_->planPath(
                    convertToVector(current_pose.pose.position),
                    convertToVector(current_pose.pose.orientation),
                    convertToVector(current_goal_.pose.position),
                    convertToVector(current_goal_.pose.orientation));
                
                if (!new_path.empty()) {
                    current_path_ = convertToPoseStampedPath(new_path);
                }
            }
        }
    }
    
    geometry_msgs::msg::Twist calculateControlCommand(
        const geometry_msgs::msg::PoseStamped& robot_pose,
        const geometry_msgs::msg::PoseStamped& target_waypoint) {
        
        geometry_msgs::msg::Twist control_cmd;
        
        // Calculate position error
        double dx = target_waypoint.pose.position.x - robot_pose.pose.position.x;
        double dy = target_waypoint.pose.position.y - robot_pose.pose.position.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Calculate angle to target
        double target_angle = std::atan2(dy, dx);
        double robot_angle = getYawFromQuaternion(robot_pose.pose.orientation);
        
        // Proportional control
        double kp_linear = 0.5;
        double kp_angular = 1.0;
        
        control_cmd.linear.x = std::min(0.5, kp_linear * distance);  // Limit max speed
        control_cmd.angular.z = kp_angular * normalizeAngle(target_angle - robot_angle);
        
        return control_cmd;
    }
    
    bool isNearWaypoint(const geometry_msgs::msg::PoseStamped& robot_pose,
                       const geometry_msgs::msg::PoseStamped& waypoint) {
        double dx = waypoint.pose.position.x - robot_pose.pose.position.x;
        double dy = waypoint.pose.position.y - robot_pose.pose.position.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        return distance < WAYPOINT_TOLERANCE;
    }
    
    bool isNearGoal(const geometry_msgs::msg::PoseStamped& robot_pose,
                   const geometry_msgs::msg::PoseStamped& goal_pose) {
        double dx = goal_pose.pose.position.x - robot_pose.pose.position.x;
        double dy = goal_pose.pose.position.y - robot_pose.pose.position.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        return distance < GOAL_TOLERANCE;
    }
    
    geometry_msgs::msg::PoseStamped getCurrentRobotPose() {
        // Get current robot pose from localization system
        geometry_msgs::msg::PoseStamped pose;
        // Implementation would get actual pose from localization
        return pose;  // Placeholder
    }
    
    void sendControlCommand(const geometry_msgs::msg::Twist& cmd) {
        // Send control command to robot
        // Implementation would send to appropriate topic
    }
    
    void stopRobot() {
        // Send stop command to robot
        geometry_msgs::msg::Twist stop_cmd;
        stop_cmd.linear.x = 0.0;
        stop_cmd.angular.z = 0.0;
        sendControlCommand(stop_cmd);
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> convertToPoseStampedPath(
        const std::vector<std::vector<double>>& path) {
        
        std::vector<geometry_msgs::msg::PoseStamped> pose_path;
        
        for (const auto& point : path) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.stamp = this->now();
            pose_stamped.header.frame_id = "map";
            
            pose_stamped.pose.position.x = point[0];
            pose_stamped.pose.position.y = point[1];
            pose_stamped.pose.position.z = point[2];
            
            pose_stamped.pose.orientation.w = 1.0;  // Identity quaternion for simplicity
            pose_path.push_back(pose_stamped);
        }
        
        return pose_path;
    }
    
    std::vector<double> convertToVector(const geometry_msgs::msg::Point& point) {
        std::vector<double> vec = {point.x, point.y, point.z};
        return vec;
    }
    
    std::vector<double> convertToVector(const geometry_msgs::msg::Quaternion& quat) {
        std::vector<double> vec = {quat.x, quat.y, quat.z, quat.w};
        return vec;
    }
    
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
        return std::atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z)
        );
    }
    
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }
    
    bool isGoalValid(const geometry_msgs::msg::PoseStamped& goal) {
        // Check if goal pose is in a valid location (not in obstacle)
        auto cost = costmap_interface_->getCostAtPosition(
            goal.pose.position.x, goal.pose.position.y);
        
        return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
    }
    
    bool isPathStillValid(const std::vector<geometry_msgs::msg::PoseStamped>& path,
                         const nav2_costmap_2d::Costmap2D& costmap) {
        // Check if path is still collision-free with updated costmap
        for (const auto& pose : path) {
            auto cost = costmap.getCost(
                static_cast<int>(pose.pose.position.x / costmap.getResolution()),
                static_cast<int>(pose.pose.position.y / costmap.getResolution()));
            
            if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
                return false;  // Path is no longer valid
            }
        }
        
        return true;
    }
    
    static constexpr double WAYPOINT_TOLERANCE = 0.2;  // 20cm tolerance
    static constexpr double GOAL_TOLERANCE = 0.1;      // 10cm tolerance
    static constexpr double MAX_LINEAR_SPEED = 0.5;    // 0.5 m/s max
};
```

### Multi-Modal Motion Planning

#### Integration of Multiple Sensors and Planning Approaches
```cpp
class MultiModalMotionPlanner {
private:
    std::unique_ptr<PRMPlanner> prm_planner_;
    std::unique_ptr<RRTStarPlanner> rrt_star_planner_;
    std::unique_ptr<CHOMPPlanner> chomp_planner_;
    
    // Sensor integration
    std::unique_ptr<CameraInterface> camera_interface_;
    std::unique_ptr<LidarInterface> lidar_interface_;
    std::unique_ptr<ImuInterface> imu_interface_;
    std::unique_ptr<EnvironmentModel> environment_model_;
    
    // Planning selection based on situation
    std::unique_ptr<PlanningSelector> planning_selector_;
    
    // Safety and constraint checking
    std::unique_ptr<SafetyChecker> safety_checker_;
    std::unique_ptr<ConstraintChecker> constraint_checker_;
    
    // Dynamic environment adaptation
    std::unique_ptr<DynamicEnvironmentAdapter> dynamic_adapter_;

public:
    MultiModalMotionPlanner() {
        prm_planner_ = std::make_unique<PRMPlanner>(1000, 0.5);  // 1000 vertices, 0.5m radius
        rrt_star_planner_ = std::make_unique<RRTStarPlanner>(0.1, 10000, 10.0);  // 0.1m step, 10k max, gamma 10
        chomp_planner_ = std::make_unique<CHOMPPlanner>(0.01, 100, 0.3);  // 0.01 learning rate, 100 iter, 0.3m influence
        
        camera_interface_ = std::make_unique<CameraInterface>();
        lidar_interface_ = std::make_unique<LidarInterface>();
        imu_interface_ = std::make_unique<ImuInterface>();
        environment_model_ = std::make_unique<EnvironmentModel>();
        
        planning_selector_ = std::make_unique<PlanningSelector>();
        safety_checker_ = std::make_unique<SafetyChecker>();
        constraint_checker_ = std::make_unique<ConstraintChecker>();
        dynamic_adapter_ = std::make_unique<DynamicEnvironmentAdapter>();
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> planPath(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const PlanningConstraints& constraints) {
        
        // Update environment model with current sensor data
        updateEnvironmentModel();
        
        // Select appropriate planning algorithm based on situation
        auto selected_algorithm = planning_selector_->selectAlgorithm(
            start, goal, constraints, environment_model_->getEnvironmentState());
        
        // Plan path using selected algorithm
        std::vector<geometry_msgs::msg::PoseStamped> path;
        
        switch (selected_algorithm) {
            case PlanningAlgorithm::PRM:
                path = planWithPRM(start, goal, constraints);
                break;
            case PlanningAlgorithm::RRT_STAR:
                path = planWithRRTStar(start, goal, constraints);
                break;
            case PlanningAlgorithm::CHOMP:
                path = planWithCHOMP(start, goal, constraints);
                break;
            case PlanningAlgorithm::HYBRID:
                path = planWithHybrid(start, goal, constraints);
                break;
            default:
                // Default to RRT* for general planning
                path = planWithRRTStar(start, goal, constraints);
                break;
        }
        
        if (path.empty()) {
            // Try fallback planning algorithm
            path = planWithFallback(start, goal, constraints);
        }
        
        // Validate path
        if (isPathValid(path, constraints)) {
            // Optimize path if needed
            return optimizePath(path, constraints);
        } else {
            RCLCPP_WARN(rclcpp::get_logger("multi_modal_planner"), 
                       "Generated path is invalid, attempting repair");
            return repairPath(path, start, goal, constraints);
        }
    }

private:
    std::vector<geometry_msgs::msg::PoseStamped> planWithPRM(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const PlanningConstraints& constraints) {
        
        // Convert start/goal to appropriate format for PRM
        auto start_config = convertPoseToConfiguration(start.pose);
        auto goal_config = convertPoseToConfiguration(goal.pose);
        
        // Build roadmap based on current environment
        auto env_bounds = environment_model_->getEnvironmentBounds();
        prm_planner_->buildRoadmap(ROBOT_DOF, env_bounds.min, env_bounds.max);
        
        // Plan path
        auto path_configs = prm_planner_->planPath(start_config, goal_config);
        
        // Convert back to PoseStamped format
        return convertConfigurationPathToPoseStamped(path_configs);
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> planWithRRTStar(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const PlanningConstraints& constraints) {
        
        // Convert start/goal to appropriate format for RRT*
        auto start_config = convertPoseToConfiguration(start.pose);
        auto goal_config = convertPoseToConfiguration(goal.pose);
        
        // Plan path with RRT*
        auto path_configs = rrt_star_planner_->planPath(start_config, goal_config);
        
        // Convert back to PoseStamped format
        return convertConfigurationPathToPoseStamped(path_configs);
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> planWithCHOMP(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const std::vector<geometry_msgs::msg::PoseStamped>& initial_path,
        const PlanningConstraints& constraints) {
        
        // Convert to appropriate format for CHOMP
        auto start_config = convertPoseToConfiguration(start.pose);
        auto goal_config = convertPoseToConfiguration(goal.pose);
        
        // Plan path with CHOMP optimization
        auto path_configs = chomp_planner_->planPath(start_config, goal_config, initial_path);
        
        // Convert back to PoseStamped format
        return convertConfigurationPathToPoseStamped(path_configs);
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> planWithHybrid(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const PlanningConstraints& constraints) {
        
        // Use PRM for initial path planning
        auto initial_path = planWithPRM(start, goal, constraints);
        
        // Use CHOMP for path optimization
        if (!initial_path.empty()) {
            auto start_config = convertPoseToConfiguration(start.pose);
            auto goal_config = convertPoseToConfiguration(goal.pose);
            
            auto optimized_path = chomp_planner_->planPath(
                start_config, goal_config, initial_path);
            
            return convertConfigurationPathToPoseStamped(optimized_path);
        }
        
        return {};
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> planWithFallback(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const PlanningConstraints& constraints) {
        
        // Implement fallback planning with simpler approach
        // This might be a straight-line path with obstacle avoidance
        auto straight_path = generateStraightLinePath(start, goal);
        
        // Apply obstacle avoidance to straight path
        return applyObstacleAvoidance(straight_path, constraints);
    }
    
    void updateEnvironmentModel() {
        // Get current sensor data
        auto camera_data = camera_interface_->getLatestData();
        auto lidar_data = lidar_interface_->getLatestData();
        auto imu_data = imu_interface_->getLatestData();
        
        // Update environment model with sensor data
        environment_model_->updateWithSensorData(camera_data, lidar_data, imu_data);
        
        // Detect dynamic obstacles
        dynamic_adapter_->detectDynamicObstacles(lidar_data, camera_data);
    }
    
    bool isPathValid(const std::vector<geometry_msgs::msg::PoseStamped>& path,
                    const PlanningConstraints& constraints) {
        
        // Check if path is collision-free
        for (const auto& pose : path) {
            if (environment_model_->isInCollision(pose.pose.position)) {
                return false;
            }
        }
        
        // Check if path satisfies constraints
        if (!constraint_checker_->checkPathConstraints(path, constraints)) {
            return false;
        }
        
        // Check if path is dynamically feasible
        if (!isDynamicallyFeasible(path)) {
            return false;
        }
        
        return true;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> optimizePath(
        const std::vector<geometry_msgs::msg::PoseStamped>& path,
        const PlanningConstraints& constraints) {
        
        // Apply path optimization to improve quality
        auto optimized_path = applyPathSmoothing(path);
        
        // Apply optimization to minimize energy consumption
        optimized_path = minimizeEnergyConsumption(optimized_path, constraints);
        
        return optimized_path;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> repairPath(
        const std::vector<geometry_msgs::msg::PoseStamped>& path,
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const PlanningConstraints& constraints) {
        
        // Identify collision points in path
        auto collision_indices = findCollisionPoints(path);
        
        // Repair path around collision points
        auto repaired_path = path;
        
        for (auto idx : collision_indices) {
            // Find alternative path around collision
            auto alternative_path = findAlternativePathSegment(
                repaired_path[idx-1], repaired_path[idx+1], constraints);
            
            if (!alternative_path.empty()) {
                // Replace collision segment with alternative
                repaired_path.erase(repaired_path.begin() + idx-1, 
                                  repaired_path.begin() + idx+1);
                repaired_path.insert(repaired_path.begin() + idx-1, 
                                   alternative_path.begin(), 
                                   alternative_path.end());
            }
        }
        
        return repaired_path;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> generateStraightLinePath(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) {
        
        // Generate straight-line path with resolution
        std::vector<geometry_msgs::msg::PoseStamped> path;
        
        double dx = goal.pose.position.x - start.pose.position.x;
        double dy = goal.pose.position.y - start.pose.position.y;
        double dz = goal.pose.position.z - start.pose.position.z;
        
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        double resolution = 0.1;  // 10cm resolution
        
        int num_points = static_cast<int>(distance / resolution) + 1;
        
        for (int i = 0; i <= num_points; i++) {
            double t = static_cast<double>(i) / num_points;
            
            geometry_msgs::msg::PoseStamped intermediate_pose;
            intermediate_pose.header.stamp = rclcpp::Clock().now();
            intermediate_pose.header.frame_id = "map";
            
            intermediate_pose.pose.position.x = start.pose.position.x + t * dx;
            intermediate_pose.pose.position.y = start.pose.position.y + t * dy;
            intermediate_pose.pose.position.z = start.pose.position.z + t * dz;
            
            // For orientation, interpolate between start and goal
            intermediate_pose.pose.orientation = interpolateOrientation(
                start.pose.orientation, goal.pose.orientation, t);
            
            path.push_back(intermediate_pose);
        }
        
        return path;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> applyObstacleAvoidance(
        const std::vector<geometry_msgs::msg::PoseStamped>& path,
        const PlanningConstraints& constraints) {
        
        // Apply local obstacle avoidance to path
        std::vector<geometry_msgs::msg::PoseStamped> modified_path = path;
        
        for (size_t i = 0; i < modified_path.size(); i++) {
            auto pose = modified_path[i].pose.position;
            
            if (environment_model_->isInCollision(pose)) {
                // Apply local obstacle avoidance
                auto avoidance_pose = calculateObstacleAvoidancePose(pose, constraints);
                modified_path[i].pose.position = avoidance_pose;
            }
        }
        
        return modified_path;
    }
    
    geometry_msgs::msg::Point calculateObstacleAvoidancePose(
        const geometry_msgs::msg::Point& current_pose,
        const PlanningConstraints& constraints) {
        
        // Calculate new pose that avoids obstacle
        // This might involve using potential field or other local avoidance methods
        geometry_msgs::msg::Point new_pose = current_pose;
        
        // Simple potential field approach
        auto repulsive_force = calculateRepulsiveForce(current_pose);
        auto attractive_force = calculateAttractiveForce(current_pose, constraints.goal_pose);
        
        // Combine forces
        new_pose.x += (attractive_force.x + repulsive_force.x) * AVOIDANCE_GAIN;
        new_pose.y += (attractive_force.y + repulsive_force.y) * AVOIDANCE_GAIN;
        new_pose.z += (attractive_force.z + repulsive_force.z) * AVOIDANCE_GAIN;
        
        return new_pose;
    }
    
    struct EnvironmentBounds {
        std::vector<double> min;
        std::vector<double> max;
    };
    
    enum class PlanningAlgorithm {
        PRM, RRT_STAR, CHOMP, HYBRID, FALLBACK
    };
    
    struct PlanningConstraints {
        geometry_msgs::msg::PoseStamped goal_pose;
        std::vector<geometry_msgs::msg::Point> forbidden_areas;
        double max_velocity;
        double max_acceleration;
        double safety_margin;
    };
};
```

## Performance Considerations

### Real-time Performance

#### Planning Frequency Requirements
- **Static Planning**: 0.1-1 Hz for path planning in static environments
- **Dynamic Planning**: 1-10 Hz for replanning in dynamic environments
- **Local Planning**: 10-50 Hz for obstacle avoidance and local path adjustments
- **Control Planning**: 50-200 Hz for trajectory following and control

#### Computation Optimization
- **Pre-computation**: Pre-compute common paths and store in cache
- **Incremental Updates**: Update only parts of path affected by changes
- **Parallel Processing**: Use multi-threading for path planning and validation
- **Approximation Methods**: Use faster approximation when exact solution isn't needed

#### Example: Real-time Path Planning
```cpp
class RealTimePathPlanner {
private:
    std::unique_ptr<AdvancedMotionPlanner> planner_;
    std::unique_ptr<EnvironmentModel> env_model_;
    
    // Planning cache
    std::map<std::string, std::vector<geometry_msgs::msg::PoseStamped>> path_cache_;
    
    // Planning timer
    rclcpp::TimerBase::SharedPtr planning_timer_;
    std::chrono::milliseconds planning_period_;
    
    // Planning state
    geometry_msgs::msg::PoseStamped current_goal_;
    std::vector<geometry_msgs::msg::PoseStamped> current_path_;
    rclcpp::Time last_planning_time_;
    
    // Performance metrics
    std::vector<double> planning_times_;
    std::vector<double> path_lengths_;
    std::vector<bool> planning_successes_;
    size_t max_metrics_history_;

public:
    RealTimePathPlanner(double planning_freq_hz = 10.0)
        : planning_period_(std::chrono::milliseconds(
                             static_cast<int>(1000.0 / planning_freq_hz))),
          max_metrics_history_(100) {
        
        // Initialize planner
        planner_ = std::make_unique<RRTStarPlanner>(0.1, 5000, 10.0);
        
        // Initialize environment model
        env_model_ = std::make_unique<EnvironmentModel>();
        
        // Initialize planning timer
        planning_timer_ = this->create_wall_timer(
            planning_period_,
            std::bind(&RealTimePathPlanner::planningCallback, this));
    }
    
    void setCurrentGoal(const geometry_msgs::msg::PoseStamped& goal) {
        current_goal_ = goal;
        need_replanning_ = true;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> getCurrentPath() const {
        return current_path_;
    }
    
    bool isPathValid() const {
        return !current_path_.empty() && 
               last_planning_time_ > (this->now() - rclcpp::Duration::from_seconds(5.0));
    }

private:
    void planningCallback() {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Check if replanning is needed
        if (needReplanning()) {
            // Update environment model
            env_model_->updateWithLatestSensorData();
            
            // Get current robot pose
            auto current_pose = getCurrentRobotPose();
            
            // Plan path from current pose to goal
            auto new_path = planner_->planPath(current_pose, current_goal_);
            
            if (!new_path.empty()) {
                // Update current path
                current_path_ = new_path;
                last_planning_time_ = this->now();
                
                // Store in cache
                addToPathCache(current_pose, current_goal_, new_path);
                
                // Log performance metrics
                auto end_time = std::chrono::high_resolution_clock::now();
                auto planning_time = std::chrono::duration<double, std::milli>(
                    end_time - start_time).count();
                
                logPerformanceMetrics(planning_time, new_path.size());
                
                RCLCPP_DEBUG(this->get_logger(), 
                            "Path planning completed in %.2f ms", planning_time);
            } else {
                RCLCPP_WARN(this->get_logger(), "Path planning failed to find solution");
                
                // Try fallback planning
                auto fallback_path = tryFallbackPlanning(current_pose, current_goal_);
                if (!fallback_path.empty()) {
                    current_path_ = fallback_path;
                    last_planning_time_ = this->now();
                }
            }
        }
    }
    
    bool needReplanning() {
        // Check various conditions that require replanning
        if (current_path_.empty()) {
            return true;  // Need initial path
        }
        
        if (need_replanning_) {
            need_replanning_ = false;  // Reset flag
            return true;
        }
        
        // Check if environment has changed significantly
        auto env_change = env_model_->getEnvironmentChangeLevel();
        if (env_change > ENV_CHANGE_THRESHOLD_FOR_REPLANNING) {
            return true;
        }
        
        // Check if path is no longer valid (due to new obstacles)
        if (!isCurrentPathStillValid()) {
            return true;
        }
        
        // Check if goal has changed significantly
        auto goal_change = calculateGoalChange();
        if (goal_change > GOAL_CHANGE_THRESHOLD_FOR_REPLANNING) {
            return true;
        }
        
        return false;
    }
    
    bool isCurrentPathStillValid() {
        // Check if current path is still collision-free with updated environment
        for (const auto& pose : current_path_) {
            if (env_model_->isInCollision(pose.pose.position)) {
                return false;
            }
        }
        return true;
    }
    
    double calculateGoalChange() {
        // Calculate how much the goal has changed since last planning
        // Implementation would compare current goal to previous goal
        return 0.0;  // Placeholder
    }
    
    void logPerformanceMetrics(double planning_time, size_t path_length) {
        // Store planning time metrics
        if (planning_times_.size() >= max_metrics_history_) {
            planning_times_.erase(planning_times_.begin());
        }
        planning_times_.push_back(planning_time);
        
        // Store path length metrics
        if (path_lengths_.size() >= max_metrics_history_) {
            path_lengths_.erase(path_lengths_.begin());
        }
        path_lengths_.push_back(path_length);
        
        // Calculate and log performance statistics
        RCLCPP_DEBUG(this->get_logger(), 
                    "Planning Performance: Avg Time=%.2f ms, Avg Length=%.2f", 
                    calculateAverage(planning_times_), 
                    calculateAverage(path_lengths_));
    }
    
    double calculateAverage(const std::vector<double>& values) {
        if (values.empty()) return 0.0;
        
        double sum = 0.0;
        for (double val : values) {
            sum += val;
        }
        return sum / values.size();
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> tryFallbackPlanning(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) {
        
        // Try simpler, faster planning algorithm as fallback
        auto straight_path = generateStraightLinePath(start, goal);
        
        // Apply local obstacle avoidance
        return applyLocalObstacleAvoidance(straight_path);
    }
    
    void addToPathCache(const geometry_msgs::msg::PoseStamped& start,
                       const geometry_msgs::msg::PoseStamped& goal,
                       const std::vector<geometry_msgs::msg::PoseStamped>& path) {
        
        // Create cache key from start and goal poses
        std::string cache_key = createPathCacheKey(start, goal);
        
        // Add to cache
        path_cache_[cache_key] = path;
        
        // Limit cache size
        if (path_cache_.size() > MAX_CACHE_SIZE) {
            // Remove oldest entry
            path_cache_.erase(path_cache_.begin());
        }
    }
    
    std::string createPathCacheKey(const geometry_msgs::msg::PoseStamped& start,
                                  const geometry_msgs::msg::PoseStamped& goal) {
        // Create cache key from start and goal poses
        // This should be robust to small changes in poses
        return "path_" + 
               std::to_string(static_cast<int>(start.pose.position.x * 10)) + "_" +
               std::to_string(static_cast<int>(start.pose.position.y * 10)) + "_" +
               std::to_string(static_cast<int>(goal.pose.position.x * 10)) + "_" +
               std::to_string(static_cast<int>(goal.pose.position.y * 10));
    }
    
    static constexpr double ENV_CHANGE_THRESHOLD_FOR_REPLANNING = 0.3;
    static constexpr double GOAL_CHANGE_THRESHOLD_FOR_REPLANNING = 0.5;
    static constexpr size_t MAX_CACHE_SIZE = 50;
    
    bool need_replanning_ = true;
};
```

## Safety and Reliability

### Safe Motion Planning

#### Safety Constraints Integration
```cpp
class SafeMotionPlanner {
private:
    std::unique_ptr<AdvancedMotionPlanner> motion_planner_;
    std::unique_ptr<SafetyConstraintManager> safety_constraints_;
    std::unique_ptr<CollisionDetector> collision_detector_;
    std::unique_ptr<HumanDetector> human_detector_;
    
    // Safety parameters
    double safety_margin_;
    double human_safety_distance_;
    double max_path_curvature_;
    double min_turning_radius_;
    
    // Emergency planning
    std::unique_ptr<EmergencyPlanner> emergency_planner_;
    bool emergency_mode_active_;

public:
    SafeMotionPlanner(double safety_margin = 0.3, 
                    double human_safety_dist = 1.0)
        : safety_margin_(safety_margin), 
          human_safety_distance_(human_safety_dist),
          emergency_mode_active_(false) {
        
        motion_planner_ = std::make_unique<RRTStarPlanner>(0.1, 10000, 10.0);
        safety_constraints_ = std::make_unique<SafetyConstraintManager>();
        collision_detector_ = std::make_unique<CollisionDetector>();
        human_detector_ = std::make_unique<HumanDetector>();
        emergency_planner_ = std::make_unique<EmergencyPlanner>();
        
        // Set safety parameters
        safety_constraints_->setSafetyMargin(safety_margin_);
        safety_constraints_->setHumanSafetyDistance(human_safety_distance_);
        safety_constraints_->setMaxPathCurvature(max_path_curvature_);
        safety_constraints_->setMinTurningRadius(min_turning_radius_);
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> planSafePath(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const PlanningConstraints& constraints) {
        
        // Validate safety constraints
        if (!safety_constraints_->areConstraintsValid(start, goal, constraints)) {
            RCLCPP_ERROR(this->get_logger(), "Safety constraints are not valid");
            return {};
        }
        
        // Check for humans in environment
        auto humans = human_detector_->detectHumans();
        if (humans.size() > 0) {
            // Add human safety constraints to planning
            auto constrained_goal = addHumanSafetyConstraints(start, goal, humans);
            
            // Plan path with human safety constraints
            return motion_planner_->planPath(start, constrained_goal, constraints);
        }
        
        // Plan path with general safety constraints
        auto path = motion_planner_->planPath(start, goal, constraints);
        
        // Validate path safety
        if (isPathSafe(path)) {
            return path;
        } else {
            RCLCPP_WARN(this->get_logger(), "Planned path is not safe, attempting repair");
            return repairUnsafePath(path, start, goal, constraints);
        }
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> planEmergencyPath(
        const geometry_msgs::msg::PoseStamped& current_pose,
        const EmergencyType& emergency_type) {
        
        emergency_mode_active_ = true;
        
        // Plan emergency path based on type of emergency
        auto emergency_path = emergency_planner_->planEmergencyPath(
            current_pose, emergency_type);
        
        // Validate emergency path is safe
        if (isEmergencyPathSafe(emergency_path)) {
            return emergency_path;
        } else {
            // Fall back to safest possible path
            return emergency_planner_->planSafestPath(current_pose);
        }
    }

private:
    bool isPathSafe(const std::vector<geometry_msgs::msg::PoseStamped>& path) {
        // Check if path is collision-free
        for (const auto& pose : path) {
            if (collision_detector_->isInCollision(pose.pose.position)) {
                return false;
            }
            
            // Check safety margin around path
            if (!isPathWithSafetyMargin(pose.pose.position)) {
                return false;
            }
        }
        
        // Check if path satisfies dynamic constraints
        if (!isPathDynamicallyFeasible(path)) {
            return false;
        }
        
        return true;
    }
    
    bool isPathWithSafetyMargin(const geometry_msgs::msg::Point& position) {
        // Check if there's sufficient safety margin around position
        auto obstacles = collision_detector_->getNearbyObstacles(position, safety_margin_);
        return obstacles.empty();
    }
    
    bool isPathDynamicallyFeasible(const std::vector<geometry_msgs::msg::PoseStamped>& path) {
        // Check if path curvature is within robot's capabilities
        for (size_t i = 1; i < path.size() - 1; i++) {
            auto curvature = calculatePathCurvature(path[i-1], path[i], path[i+1]);
            if (curvature > max_path_curvature_) {
                return false;
            }
        }
        return true;
    }
    
    double calculatePathCurvature(const geometry_msgs::msg::PoseStamped& p1,
                                 const geometry_msgs::msg::PoseStamped& p2,
                                 const geometry_msgs::msg::PoseStamped& p3) {
        // Calculate curvature of path segment between three points
        // This is a simplified approach - in practice would be more complex
        double dx1 = p2.pose.position.x - p1.pose.position.x;
        double dy1 = p2.pose.position.y - p1.pose.position.y;
        double dx2 = p3.pose.position.x - p2.pose.position.x;
        double dy2 = p3.pose.position.y - p2.pose.position.y;
        
        // Calculate angle change
        double angle1 = std::atan2(dy1, dx1);
        double angle2 = std::atan2(dy2, dx2);
        double angle_change = std::abs(angle2 - angle1);
        
        // Calculate curvature as angle change per distance
        double distance1 = std::sqrt(dx1*dx1 + dy1*dy1);
        double distance2 = std::sqrt(dx2*dx2 + dy2*dy2);
        double avg_distance = (distance1 + distance2) / 2.0;
        
        return angle_change / avg_distance;
    }
    
    geometry_msgs::msg::PoseStamped addHumanSafetyConstraints(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const std::vector<geometry_msgs::msg::PoseStamped>& humans) {
        
        // Modify goal to maintain safe distance from humans
        geometry_msgs::msg::PoseStamped constrained_goal = goal;
        
        for (const auto& human : humans) {
            double dx = human.pose.position.x - goal.pose.position.x;
            double dy = human.pose.position.y - goal.pose.position.y;
            double distance = std::sqrt(dx*dx + dy*dy);
            
            if (distance < human_safety_distance_) {
                // Move goal away from human
                double push_distance = human_safety_distance_ - distance;
                double direction_x = dx / distance;
                double direction_y = dy / distance;
                
                constrained_goal.pose.position.x -= direction_x * push_distance;
                constrained_goal.pose.position.y -= direction_y * push_distance;
            }
        }
        
        return constrained_goal;
    }
    
    std::vector<geometry_msgs::msg::PoseStamped> repairUnsafePath(
        const std::vector<geometry_msgs::msg::PoseStamped>& path,
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal,
        const PlanningConstraints& constraints) {
        
        // Identify unsafe segments in path
        auto unsafe_segments = identifyUnsafeSegments(path);
        
        // Repair each unsafe segment
        auto repaired_path = path;
        
        for (const auto& segment : unsafe_segments) {
            // Plan alternative path around unsafe area
            auto alternative_segment = motion_planner_->planPath(
                repaired_path[segment.start_idx], 
                repaired_path[segment.end_idx], 
                constraints);
            
            if (!alternative_segment.empty()) {
                // Replace unsafe segment with alternative
                repaired_path.erase(
                    repaired_path.begin() + segment.start_idx + 1,
                    repaired_path.begin() + segment.end_idx);
                
                repaired_path.insert(
                    repaired_path.begin() + segment.start_idx + 1,
                    alternative_segment.begin() + 1,  // Skip first as it's the same as start
                    alternative_segment.end() - 1);   // Skip last as it's the same as end
            }
        }
        
        // Final validation
        if (isPathSafe(repaired_path)) {
            return repaired_path;
        } else {
            // Could not repair path safely - return empty
            RCLCPP_ERROR(this->get_logger(), "Could not repair path to be safe");
            return {};
        }
    }
    
    struct UnsafeSegment {
        size_t start_idx;
        size_t end_idx;
        std::string reason;
    };
    
    std::vector<UnsafeSegment> identifyUnsafeSegments(
        const std::vector<geometry_msgs::msg::PoseStamped>& path) {
        
        std::vector<UnsafeSegment> unsafe_segments;
        
        for (size_t i = 0; i < path.size(); i++) {
            if (collision_detector_->isInCollision(path[i].pose.position)) {
                // Find start and end of collision segment
                size_t start_idx = i;
                while (i < path.size() && 
                       collision_detector_->isInCollision(path[i].pose.position)) {
                    i++;
                }
                size_t end_idx = i;
                
                unsafe_segments.push_back({
                    start_idx, end_idx, "Collision with obstacle"
                });
            }
        }
        
        return unsafe_segments;
    }
    
    bool isEmergencyPathSafe(const std::vector<geometry_msgs::msg::PoseStamped>& path) {
        // For emergency paths, use more relaxed safety criteria
        // but still ensure no immediate danger
        for (const auto& pose : path) {
            if (collision_detector_->isImmediateDanger(pose.pose.position)) {
                return false;
            }
        }
        return true;
    }
    
    enum class EmergencyType {
        COLLISION_IMMINENT,
        HUMAN_APPROACHING,
        SYSTEM_FAILURE,
        POWER_LOW
    };
};
```

## Troubleshooting Common Issues

### Motion Planning Issues

#### Path Planning Failures
- **Symptoms**: Planner fails to find path between start and goal
- **Causes**: Environment too constrained, obstacles blocking all paths
- **Solutions**: Increase planning time, relax constraints, use alternative planners
- **Prevention**: Verify environment model accuracy, check for valid paths

#### Performance Degradation
- **Symptoms**: Planning takes too long, robot response is slow
- **Causes**: Large configuration space, complex environment, inefficient algorithm
- **Solutions**: Use hierarchical planning, optimize algorithm, increase hardware
- **Monitoring**: Track planning times, solution quality metrics

#### Collision Detection Problems
- **Symptoms**: Robot collides with obstacles despite collision checking
- **Causes**: Inaccurate environment model, sensor delays, prediction errors
- **Solutions**: Improve environment model, add prediction, increase safety margins
- **Validation**: Test with known obstacles, verify sensor accuracy

### Integration Issues

#### Multi-Sensor Fusion Problems
- **Symptoms**: Perception results inconsistent across sensors
- **Causes**: Calibration errors, temporal misalignment, sensor noise
- **Solutions**: Improve calibration, add temporal alignment, use filtering
- **Prevention**: Regular calibration verification, sensor health monitoring

#### Real-time Performance Issues
- **Symptoms**: Control loop misses timing deadlines, robot behaves erratically
- **Causes**: Heavy computation, resource contention, memory allocation
- **Solutions**: Optimize algorithms, use efficient data structures, improve scheduling
- **Monitoring**: Track loop timing, CPU/memory usage, system performance

## Future Developments

### Emerging Technologies

#### AI-Enhanced Motion Planning
- **Learning-Based Planning**: Neural networks that learn to plan from experience
- **Imitation Learning**: Learning from expert demonstrations
- **Reinforcement Learning**: Learning optimal planning strategies
- **Predictive Planning**: Planning based on predicted environment changes

#### Multi-Modal Integration
- **Vision-Language Planning**: Planning based on natural language instructions
- **Haptic Integration**: Planning that considers haptic feedback
- **Social Planning**: Planning considering human behavior and interaction
- **Cross-Modal Learning**: Learning from multiple sensor modalities

### Advanced Integration Approaches

#### Human-Robot Collaborative Planning
- **Shared Intentions**: Planning that considers human intentions
- **Predictive Human Modeling**: Modeling human behavior for planning
- **Interactive Planning**: Humans and robots collaborating on plans
- **Social Compliance**: Plans that follow social norms and conventions

## Conclusion

Advanced perception and control techniques are essential for Physical AI systems that must operate effectively in complex, dynamic environments. These techniques enable robots to understand their environment in rich detail, plan complex trajectories that avoid obstacles and respect safety constraints, and execute precise movements that achieve their objectives.

The integration of multiple perception modalities (camera, LiDAR, IMU, etc.) with advanced planning algorithms (RRT*, CHOMP, etc.) creates sophisticated systems that can handle the challenges of real-world operation. The key to success lies in understanding the trade-offs between different approaches and selecting the appropriate techniques for each application.

As robotics systems become more complex and operate in more diverse environments, the importance of advanced perception and control techniques continues to grow. These systems must handle sensor noise, environmental uncertainties, real-time performance requirements, and safety considerations simultaneously.

Understanding these advanced techniques and their integration is essential for creating Physical AI systems that can operate effectively in the real world, bridging the gap between digital AI and physical interaction.

## Exercises

1. Implement a multi-modal perception system that combines camera and LiDAR data for 3D object detection and pose estimation.
2. Design and implement an advanced motion planner that uses RRT* for path planning with collision avoidance.
3. Create a safe control system that integrates perception and planning with safety constraints for robot operation.

## Further Reading

- LaValle, S. M. (2006). "Planning Algorithms." Cambridge University Press.
- Siciliano, B., & Khatib, O. (Eds.). (2016). "Springer Handbook of Robotics." Springer.
- Spong, M.W., et al. (2006). "Robot Modeling and Control." Wiley.
- Khatib, O., et al. (2018). "Robotics: Perception-Action Cycle, Multiagent Systems, and Applications."
- Fox, D., et al. (2019). "AI-Enabled Robotics: Current Approaches and Future Directions."