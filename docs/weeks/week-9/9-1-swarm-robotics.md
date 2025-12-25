---
sidebar_label: Swarm Robotics and Multi-Robot Systems
title: Swarm Robotics and Multi-Robot Systems - Coordinated Behavior of Multiple Robots
description: Understanding swarm robotics and multi-robot coordination systems for Physical AI applications
keywords: [swarm robotics, multi-robot, coordination, distributed systems, collective behavior, robotics, AI]
---

# 9.1 Swarm Robotics and Multi-Robot Systems

## Introduction

Swarm robotics represents a paradigm where large numbers of relatively simple robots work together to achieve complex goals that would be difficult or impossible for a single robot to accomplish. Inspired by biological systems such as ant colonies, bird flocks, and fish schools, swarm robotics leverages collective behavior and emergent intelligence to solve problems in distributed and scalable ways.

Physical AI systems in swarm configurations exhibit unique properties that emerge from the interaction of multiple embodied agents. These systems can be more robust, adaptable, and efficient than single-robot approaches for certain applications. The decentralized nature of swarm robotics makes these systems resilient to individual robot failures and enables them to operate in environments where a single robot might be insufficient.

This chapter explores the principles, algorithms, and implementation strategies for swarm robotics systems, focusing on how multiple Physical AI agents can coordinate their behavior to achieve collective goals. We'll examine communication protocols, coordination algorithms, and the challenges of implementing swarm intelligence in real physical systems.

## Swarm Robotics Fundamentals

### Key Characteristics

#### Emergence
- **Definition**: Complex behaviors arising from simple local interactions
- **Example**: Formation of complex patterns from simple rules
- **Importance**: Enables sophisticated behaviors without complex central planning
- **Implementation**: Local rules that lead to global behavior

#### Self-Organization
- **Decentralized Control**: No central coordinator required
- **Adaptive Behavior**: Systems adapt to changing conditions
- **Scalability**: Performance scales with number of agents
- **Robustness**: System continues to function despite individual failures

#### Collective Intelligence
- **Information Sharing**: Agents share information to improve collective performance
- **Parallel Processing**: Multiple agents work simultaneously on tasks
- **Distributed Problem Solving**: Complex problems divided among agents
- **Adaptive Learning**: Collective learning from environmental interaction

### Biological Inspiration

#### Ant Colony Optimization
- **Foraging Behavior**: Finding shortest paths to food sources
- **Pheromone Trails**: Indirect communication through environment
- **Distributed Decision Making**: No central coordination required
- **Applications**: Path planning, resource allocation, task assignment

#### Flocking Behavior
- **Separation**: Avoid crowding neighbors
- **Alignment**: Steer toward average heading of neighbors
- **Cohesion**: Steer toward average position of neighbors
- **Applications**: Formation control, area coverage, collective navigation

#### Honeybee Communication
- **Waggle Dance**: Communicating location of resources
- **Quorum Sensing**: Decision-making through collective evaluation
- **Division of Labor**: Task allocation based on colony needs
- **Applications**: Task assignment, collective decision making

### Swarm Robotics vs. Multi-Robot Systems

#### Swarm Robotics
- **Many Simple Robots**: Emphasizes large numbers of simple agents
- **Emergent Behavior**: Complex behavior emerges from simple rules
- **Biological Inspiration**: Draws heavily from biological systems
- **Decentralized**: No central control authority

#### Multi-Robot Systems
- **Few Complex Robots**: Emphasizes fewer but more sophisticated robots
- **Coordinated Planning**: Central or distributed planning with coordination
- **Task Allocation**: Explicit assignment of roles and tasks
- **Communication**: Direct communication between agents

## Communication in Swarm Systems

### Communication Architectures

#### Centralized Communication
- **Hub-and-Spoke**: All robots communicate with central hub
- **Advantages**: Simple coordination, global information
- **Disadvantages**: Single point of failure, communication bottleneck
- **Applications**: Small swarms with simple coordination needs

#### Decentralized Communication
- **Peer-to-Peer**: Direct communication between robots
- **Advantages**: Robust to robot failures, scalable
- **Disadvantages**: Complex coordination, limited information
- **Applications**: Large swarms, robust systems

#### Distributed Communication
- **Hierarchical**: Groups of robots with local coordinators
- **Advantages**: Balance of scalability and coordination
- **Disadvantages**: Complex architecture, potential bottlenecks
- **Applications**: Medium-sized swarms with task specialization

### Communication Protocols

#### ROS 2 for Multi-Robot Communication

```cpp
#include "rclcpp/rclcpp.hpp"
#include "multirobot_msgs/msg/swarm_state.hpp"
#include "multirobot_msgs/msg/task_assignment.hpp"
#include "multirobot_msgs/srv/swarm_coordination.hpp"

class SwarmCoordinator : public rclcpp::Node {
private:
    // Communication with other robots
    rclcpp::Subscription<multirobot_msgs::msg::SwarmState>::SharedPtr swarm_state_sub_;
    rclcpp::Publisher<multirobot_msgs::msg::SwarmState>::SharedPtr swarm_state_pub_;
    rclcpp::Publisher<multirobot_msgs::msg::TaskAssignment>::SharedPtr task_assignment_pub_;
    
    // Service for swarm coordination
    rclcpp::Service<multirobot_msgs::srv::SwarmCoordination>::SharedPtr coordination_srv_;
    
    // Swarm state management
    std::map<std::string, RobotState> robot_states_;
    std::vector<Task> available_tasks_;
    std::string robot_id_;
    int swarm_size_;
    
    // Communication parameters
    double communication_range_;
    double heartbeat_frequency_;
    double task_reassignment_threshold_;

public:
    SwarmCoordinator(const std::string& robot_id, int swarm_size = 10)
        : Node("swarm_coordinator"), robot_id_(robot_id), swarm_size_(swarm_size) {
        
        communication_range_ = 10.0;  // 10 meters communication range
        heartbeat_frequency_ = 10.0;  // 10 Hz heartbeat
        task_reassignment_threshold_ = 0.8;  // 80% completion threshold
        
        // Publishers and subscribers
        swarm_state_pub_ = this->create_publisher<multirobot_msgs::msg::SwarmState>(
            "swarm_state", 10);
        
        swarm_state_sub_ = this->create_subscription<multirobot_msgs::msg::SwarmState>(
            "swarm_state", 10,
            std::bind(&SwarmCoordinator::swarmStateCallback, this, std::placeholders::_1));
        
        task_assignment_pub_ = this->create_publisher<multirobot_msgs::msg::TaskAssignment>(
            "task_assignment", 10);
        
        // Heartbeat timer
        heartbeat_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / heartbeat_frequency_)),
            std::bind(&SwarmCoordinator::publishHeartbeat, this));
        
        // Task assignment timer
        task_assignment_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),  // 1 Hz task assignment
            std::bind(&SwarmCoordinator::updateTaskAssignment, this));
        
        // Service for coordination requests
        coordination_srv_ = this->create_service<multirobot_msgs::srv::SwarmCoordination>(
            "swarm_coordination",
            std::bind(&SwarmCoordinator::coordinationRequest, this, 
                     std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    }

private:
    void swarmStateCallback(const multirobot_msgs::msg::SwarmState::SharedPtr msg) {
        // Update robot state in swarm
        RobotState state;
        state.id = msg->robot_id;
        state.position = msg->position;
        state.orientation = msg->orientation;
        state.task_status = msg->task_status;
        state.battery_level = msg->battery_level;
        state.timestamp = msg->header.stamp;
        
        robot_states_[msg->robot_id] = state;
        
        // Check if we have complete swarm information
        if (robot_states_.size() >= swarm_size_) {
            // Update swarm visualization
            updateSwarmVisualization();
        }
    }
    
    void publishHeartbeat() {
        auto msg = multirobot_msgs::msg::SwarmState();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.robot_id = robot_id_;
        msg.position = getCurrentPosition();
        msg.orientation = getCurrentOrientation();
        msg.task_status = getCurrentTaskStatus();
        msg.battery_level = getBatteryLevel();
        msg.swarm_size = swarm_size_;
        
        swarm_state_publisher_->publish(msg);
    }
    
    void updateTaskAssignment() {
        // Check for task reassignment opportunities
        for (auto& task : available_tasks_) {
            if (task.status == TaskStatus::PENDING) {
                // Find robot closest to task location
                auto best_robot = findClosestRobot(task.location);
                if (best_robot != robot_states_.end()) {
                    // Assign task to closest robot
                    auto assignment_msg = multirobot_msgs::msg::TaskAssignment();
                    assignment_msg.header.stamp = this->now();
                    assignment_msg.task_id = task.id;
                    assignment_msg.assigned_robot = best_robot->first;
                    assignment_msg.task_location = task.location;
                    assignment_msg.task_type = task.type;
                    
                    task_assignment_publisher_->publish(assignment_msg);
                    
                    // Update task status
                    task.assigned_robot = best_robot->first;
                    task.status = TaskStatus::ASSIGNED;
                }
            }
        }
    }
    
    void coordinationRequest(
        const std::shared_ptr<multirobot_msgs::srv::SwarmCoordination::Request> request,
        std::shared_ptr<multirobot_msgs::srv::SwarmCoordination::Response> response) {
        
        if (request->request_type == "formation_change") {
            // Handle formation change request
            auto new_formation = calculateFormation(request->new_formation_type, 
                                                  request->formation_parameters);
            response->success = executeFormationChange(new_formation);
            response->message = response.success ? "Formation change successful" : "Formation change failed";
        } else if (request->request_type == "task_reassignment") {
            // Handle task reassignment request
            response->success = handleTaskReassignment(request->task_id, request->new_robot);
            response->message = response.success ? "Task reassigned successfully" : "Task reassignment failed";
        } else if (request->request_type == "emergency_stop") {
            // Handle emergency stop for all robots
            for (auto& [id, state] : robot_states_) {
                if (id != robot_id_) {  // Don't stop ourselves
                    requestEmergencyStop(id);
                }
            }
            response->success = true;
            response->message = "Emergency stop requested for all robots";
        }
        
        response->swarm_size = robot_states_.size();
        response->active_robots = countActiveRobots();
    }
    
    geometry_msgs::msg::Point getCurrentPosition() {
        // Get current robot position from localization system
        geometry_msgs::msg::Point pos;
        // Implementation would get actual position
        return pos;  // Placeholder
    }
    
    geometry_msgs::msg::Quaternion getCurrentOrientation() {
        // Get current robot orientation from localization system
        geometry_msgs::msg::Quaternion orient;
        orient.w = 1.0;  // Identity orientation
        return orient;  // Placeholder
    }
    
    std::string getCurrentTaskStatus() {
        // Get current task status
        return "idle";  // Placeholder
    }
    
    double getBatteryLevel() {
        // Get current battery level
        return 0.8;  // Placeholder
    }
    
    std::map<std::string, RobotState>::iterator findClosestRobot(
        const geometry_msgs::msg::Point& location) {
        
        double min_distance = std::numeric_limits<double>::max();
        auto closest_it = robot_states_.end();
        
        for (auto it = robot_states_.begin(); it != robot_states_.end(); ++it) {
            if (it->first != robot_id_) {  // Don't consider ourself
                double distance = calculateDistance(it->second.position, location);
                if (distance < min_distance) {
                    min_distance = distance;
                    closest_it = it;
                }
            }
        }
        
        return closest_it;
    }
    
    double calculateDistance(const geometry_msgs::msg::Point& p1,
                           const geometry_msgs::msg::Point& p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + 
                        std::pow(p1.y - p2.y, 2) + 
                        std::pow(p1.z - p2.z, 2));
    }
    
    void updateSwarmVisualization() {
        // Publish visualization markers for swarm
        visualization_msgs::msg::MarkerArray markers;
        
        // Create markers for each robot
        int id = 0;
        for (const auto& [robot_id, state] : robot_states_) {
            visualization_msgs::msg::Marker robot_marker;
            robot_marker.header.frame_id = "map";
            robot_marker.header.stamp = this->now();
            robot_marker.ns = "swarm_robots";
            robot_marker.id = id++;
            robot_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            robot_marker.action = visualization_msgs::msg::Marker::ADD;
            
            robot_marker.pose.position = state.position;
            robot_marker.pose.orientation = state.orientation;
            
            robot_marker.scale.x = 0.3;
            robot_marker.scale.y = 0.3;
            robot_marker.scale.z = 0.2;
            
            // Color based on robot status
            if (state.task_status == "working") {
                robot_marker.color.r = 0.0;
                robot_marker.color.g = 1.0;
                robot_marker.color.b = 0.0;
            } else if (state.task_status == "idle") {
                robot_marker.color.r = 1.0;
                robot_marker.color.g = 1.0;
                robot_marker.color.b = 0.0;
            } else if (state.task_status == "returning") {
                robot_marker.color.r = 0.0;
                robot_marker.color.g = 0.0;
                robot_marker.color.b = 1.0;
            }
            
            robot_marker.color.a = 0.8;
            
            markers.markers.push_back(robot_marker);
        }
        
        // Publish markers
        visualization_publisher_->publish(markers);
    }
    
    struct RobotState {
        std::string id;
        geometry_msgs::msg::Point position;
        geometry_msgs::msg::Quaternion orientation;
        std::string task_status;
        double battery_level;
        rclcpp::Time timestamp;
    };
    
    struct Task {
        std::string id;
        geometry_msgs::msg::Point location;
        std::string type;
        TaskStatus status;
        std::string assigned_robot;
        double priority;
    };
    
    enum class TaskStatus {
        PENDING,
        ASSIGNED,
        IN_PROGRESS,
        COMPLETED,
        FAILED
    };
    
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
    rclcpp::TimerBase::SharedPtr task_assignment_timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_publisher_;
};

// Example swarm robot node
class SwarmRobotNode : public rclcpp::Node {
private:
    std::unique_ptr<SwarmCoordinator> coordinator_;
    std::unique_ptr<RobotController> robot_controller_;
    std::unique_ptr<NavigationStack> navigation_stack_;
    
    // Robot-specific parameters
    std::string robot_id_;
    double max_speed_;
    double communication_range_;
    
    // Task management
    std::queue<Task> task_queue_;
    Task current_task_;
    bool has_active_task_;
    
public:
    SwarmRobotNode(const std::string& robot_id, int swarm_size = 10)
        : Node("swarm_robot_" + robot_id), robot_id_(robot_id) {
        
        coordinator_ = std::make_unique<SwarmCoordinator>(robot_id_, swarm_size);
        robot_controller_ = std::make_unique<RobotController>();
        navigation_stack_ = std::make_unique<NavigationStack>();
        
        // Task assignment subscription
        task_assignment_sub_ = this->create_subscription<multirobot_msgs::msg::TaskAssignment>(
            "task_assignment", 10,
            std::bind(&SwarmRobotNode::taskAssignmentCallback, this, std::placeholders::_1));
        
        // Task execution timer
        task_execution_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10 Hz task execution
            std::bind(&SwarmRobotNode::executeCurrentTask, this));
        
        has_active_task_ = false;
    }

private:
    void taskAssignmentCallback(const multirobot_msgs::msg::TaskAssignment::SharedPtr msg) {
        if (msg->assigned_robot == robot_id_) {
            Task new_task;
            new_task.id = msg->task_id;
            new_task.location = msg->task_location;
            new_task.type = msg->task_type;
            new_task.status = TaskStatus::IN_PROGRESS;
            
            task_queue_.push(new_task);
            RCLCPP_INFO(this->get_logger(), "Task %s assigned to robot %s", 
                       msg->task_id.c_str(), robot_id_.c_str());
        }
    }
    
    void executeCurrentTask() {
        if (!has_active_task_ && !task_queue_.empty()) {
            current_task_ = task_queue_.front();
            task_queue_.pop();
            has_active_task_ = true;
            
            // Execute the task based on its type
            if (current_task_.type == "navigation") {
                executeNavigationTask(current_task_);
            } else if (current_task_.type == "inspection") {
                executeInspectionTask(current_task_);
            } else if (current_task_.type == "transport") {
                executeTransportTask(current_task_);
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown task type: %s", 
                           current_task_.type.c_str());
                completeTask(TaskStatus::FAILED);
            }
        }
        
        if (has_active_task_) {
            // Monitor task execution
            if (isTaskComplete(current_task_)) {
                completeTask(TaskStatus::COMPLETED);
            } else if (isTaskFailed(current_task_)) {
                completeTask(TaskStatus::FAILED);
            }
        }
    }
    
    void executeNavigationTask(const Task& task) {
        // Navigate to task location
        geometry_msgs::msg::PoseStamped goal_pose;
        goal_pose.header.frame_id = "map";
        goal_pose.header.stamp = this->now();
        goal_pose.pose.position = task.location;
        goal_pose.pose.orientation.w = 1.0;  // Default orientation
        
        navigation_stack_->sendGoal(goal_pose);
    }
    
    void executeInspectionTask(const Task& task) {
        // Perform inspection at task location
        // This might involve detailed sensing or mapping
        RCLCPP_INFO(this->get_logger(), "Performing inspection at (%f, %f, %f)",
                   task.location.x, task.location.y, task.location.z);
        
        // Example: Take detailed sensor readings
        auto detailed_scan = performDetailedScan(task.location);
        
        // Process and store inspection results
        storeInspectionData(detailed_scan, task.id);
    }
    
    void executeTransportTask(const Task& task) {
        // Navigate to pickup location, grasp object, navigate to destination, place object
        // This is a complex multi-step task
        if (transport_state_ == TransportState::NAVIGATE_TO_PICKUP) {
            // Navigate to pickup location
            navigateToLocation(task.pickup_location);
            if (isAtLocation(task.pickup_location)) {
                transport_state_ = TransportState::GRASP_OBJECT;
            }
        } else if (transport_state_ == TransportState::GRASP_OBJECT) {
            // Grasp the object
            if (attemptGrasp(task.object_id)) {
                transport_state_ = TransportState::NAVIGATE_TO_DESTINATION;
            } else {
                completeTask(TaskStatus::FAILED);
            }
        } else if (transport_state_ == TransportState::NAVIGATE_TO_DESTINATION) {
            // Navigate to destination
            navigateToLocation(task.destination_location);
            if (isAtLocation(task.destination_location)) {
                transport_state_ = TransportState::PLACE_OBJECT;
            }
        } else if (transport_state_ == TransportState::PLACE_OBJECT) {
            // Place the object
            if (attemptPlace()) {
                completeTask(TaskStatus::COMPLETED);
            } else {
                completeTask(TaskStatus::FAILED);
            }
        }
    }
    
    void completeTask(TaskStatus status) {
        has_active_task_ = false;
        
        // Update task status in coordinator
        // This might involve publishing task completion message
        RCLCPP_INFO(this->get_logger(), "Task %s completed with status: %s", 
                   current_task_.id.c_str(), 
                   status == TaskStatus::COMPLETED ? "SUCCESS" : "FAILED");
        
        // Report task completion to swarm coordinator
        reportTaskCompletion(current_task_.id, status);
    }
    
    enum class TransportState {
        NAVIGATE_TO_PICKUP,
        GRASP_OBJECT,
        NAVIGATE_TO_DESTINATION,
        PLACE_OBJECT
    };
    
    TransportState transport_state_ = TransportState::NAVIGATE_TO_PICKUP;
    
    rclcpp::Subscription<multirobot_msgs::msg::TaskAssignment>::SharedPtr task_assignment_sub_;
    rclcpp::TimerBase::SharedPtr task_execution_timer_;
};
```

### Communication Topologies

#### Network Topologies for Swarms
- **Fully Connected**: Every robot communicates with every other robot
- **Ring Topology**: Robots communicate with adjacent neighbors
- **Star Topology**: All robots communicate through central hub
- **Mesh Topology**: Robots communicate with nearby neighbors

#### Communication Range Considerations
```cpp
class CommunicationManager {
private:
    double communication_radius_;
    std::vector<RobotConnection> connections_;
    rclcpp::TimerBase::SharedPtr discovery_timer_;
    
    struct RobotConnection {
        std::string robot_id;
        geometry_msgs::msg::Point position;
        rclcpp::Time last_seen;
        double signal_strength;
        bool is_connected;
    };

public:
    CommunicationManager(double radius = 10.0) : communication_radius_(radius) {
        // Timer for neighbor discovery
        discovery_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),  // 2 Hz discovery
            std::bind(&CommunicationManager::discoverNeighbors, this));
    }
    
    std::vector<std::string> getNeighbors() {
        std::vector<std::string> neighbors;
        auto current_pos = getCurrentPosition();
        
        for (const auto& conn : connections_) {
            if (conn.is_connected && 
                calculateDistance(current_pos, conn.position) < communication_radius_) {
                neighbors.push_back(conn.robot_id);
            }
        }
        
        return neighbors;
    }

private:
    void discoverNeighbors() {
        auto current_pos = getCurrentPosition();
        auto all_known_robots = getAllKnownRobotPositions();
        
        connections_.clear();
        
        for (const auto& [robot_id, position] : all_known_robots) {
            double distance = calculateDistance(current_pos, position);
            
            if (distance < communication_radius_) {
                RobotConnection conn;
                conn.robot_id = robot_id;
                conn.position = position;
                conn.last_seen = this->now();
                conn.signal_strength = calculateSignalStrength(distance);
                conn.is_connected = true;
                
                connections_.push_back(conn);
            }
        }
        
        // Publish neighbor information
        publishNeighborList(connections_);
    }
    
    double calculateSignalStrength(double distance) {
        // Simple inverse square law model
        // In practice, this would be more complex with fading, obstacles, etc.
        if (distance < 0.1) return 1.0;  // Avoid division by zero
        return std::min(1.0, 1.0 / (distance * distance));
    }
    
    void publishNeighborList(const std::vector<RobotConnection>& neighbors) {
        auto msg = multirobot_msgs::msg::NeighborList();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        
        for (const auto& neighbor : neighbors) {
            multirobot_msgs::msg::RobotInfo robot_info;
            robot_info.robot_id = neighbor.robot_id;
            robot_info.position = neighbor.position;
            robot_info.signal_strength = neighbor.signal_strength;
            robot_info.last_seen = neighbor.last_seen;
            
            msg.neighbors.push_back(robot_info);
        }
        
        neighbor_publisher_->publish(msg);
    }
    
    geometry_msgs::msg::Point getCurrentPosition() {
        // Implementation to get current robot position
        geometry_msgs::msg::Point pos;
        // This would interface with localization system
        return pos;  // Placeholder
    }
    
    std::map<std::string, geometry_msgs::msg::Point> getAllKnownRobotPositions() {
        // Implementation to get positions of all known robots
        // This would typically interface with swarm state management
        return {};  // Placeholder
    }
    
    rclcpp::Publisher<multirobot_msgs::msg::NeighborList>::SharedPtr neighbor_publisher_;
};
```

## Coordination Algorithms

### Consensus Algorithms

#### Average Consensus
Robots iteratively update their state to approach the average of their neighbors' states:

```cpp
class AverageConsensus {
private:
    std::string robot_id_;
    double current_value_;
    std::vector<std::string> neighbors_;
    std::map<std::string, double> neighbor_values_;
    double consensus_weight_;
    double epsilon_;  // Convergence threshold
    
public:
    AverageConsensus(const std::string& id, double initial_value)
        : robot_id_(id), current_value_(initial_value), 
          consensus_weight_(0.1), epsilon_(0.001) {}
    
    void updateConsensusValue() {
        // Get current neighbors
        neighbors_ = communication_manager_->getNeighbors();
        
        // Collect values from neighbors
        for (const auto& neighbor : neighbors_) {
            auto value = getNeighborValue(neighbor);
            if (value != INVALID_VALUE) {
                neighbor_values_[neighbor] = value;
            }
        }
        
        // Update value using weighted average of neighbors
        double sum = current_value_;
        double weight_sum = 1.0;
        
        for (const auto& [neighbor_id, neighbor_value] : neighbor_values_) {
            if (neighbor_id != robot_id_) {
                sum += consensus_weight_ * neighbor_value;
                weight_sum += consensus_weight_;
            }
        }
        
        if (weight_sum > 0) {
            current_value_ = sum / weight_sum;
        }
        
        // Publish updated value
        publishValue(current_value_);
    }
    
    bool isConverged() {
        // Check if consensus has been reached
        double max_difference = 0.0;
        
        for (const auto& [neighbor_id, neighbor_value] : neighbor_values_) {
            double diff = std::abs(current_value_ - neighbor_value);
            max_difference = std::max(max_difference, diff);
        }
        
        return max_difference < epsilon_;
    }

private:
    double getNeighborValue(const std::string& neighbor_id) {
        // Get value from specific neighbor
        // This would involve subscribing to neighbor value topics
        return INVALID_VALUE;  // Placeholder
    }
    
    void publishValue(double value) {
        // Publish current value to neighbors
        auto msg = std_msgs::msg::Float64();
        msg.data = value;
        consensus_publisher_->publish(msg);
    }
    
    static constexpr double INVALID_VALUE = -999.0;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr consensus_publisher_;
    std::unique_ptr<CommunicationManager> communication_manager_;
};
```

### Task Allocation Algorithms

#### Market-Based Task Allocation
```cpp
class MarketBasedTaskAllocation {
private:
    std::vector<Task> available_tasks_;
    std::vector<RobotBid> robot_bids_;
    std::map<std::string, std::vector<Task>> robot_assignments_;
    
    struct Task {
        std::string id;
        geometry_msgs::msg::Point location;
        std::string type;
        double value;  // Task value/benefit
        double cost;   // Minimum cost to complete task
        rclcpp::Time deadline;
    };
    
    struct RobotBid {
        std::string robot_id;
        std::string task_id;
        double bid_price;  // Cost robot is willing to accept
        double capability_score;  // How well robot can perform task
        rclcpp::Time timestamp;
    };

public:
    std::map<std::string, std::vector<Task>> allocateTasks(
        const std::vector<Task>& tasks,
        const std::vector<RobotCapabilities>& robot_capabilities) {
        
        available_tasks_ = tasks;
        robot_assignments_.clear();
        
        // Initialize robot assignments
        for (const auto& robot_cap : robot_capabilities) {
            robot_assignments_[robot_cap.id] = {};
        }
        
        // Iterative bidding process
        for (int iteration = 0; iteration < MAX_BIDDING_ITERATIONS; iteration++) {
            // Clear bids from previous iteration
            robot_bids_.clear();
            
            // Each robot submits bids for tasks it can perform
            for (const auto& robot_cap : robot_capabilities) {
                auto bids = calculateBids(robot_cap, available_tasks_);
                robot_bids_.insert(robot_bids_.end(), bids.begin(), bids.end());
            }
            
            // Assign tasks to highest bidders
            auto assignments = assignTasks(robot_bids_, robot_capabilities);
            
            // Check for convergence
            if (hasConverged(assignments)) {
                RCLCPP_INFO(this->get_logger(), "Task allocation converged after %d iterations", iteration + 1);
                break;
            }
            
            // Update available tasks based on assignments
            updateAvailableTasks(assignments);
        }
        
        return robot_assignments_;
    }

private:
    std::vector<RobotBid> calculateBids(const RobotCapabilities& robot,
                                       const std::vector<Task>& tasks) {
        std::vector<RobotBid> bids;
        
        for (const auto& task : tasks) {
            // Calculate if robot can perform task
            if (canRobotPerformTask(robot, task)) {
                // Calculate cost to perform task
                double travel_cost = calculateTravelCost(robot.position, task.location);
                double execution_cost = calculateExecutionCost(robot, task);
                double total_cost = travel_cost + execution_cost;
                
                // Calculate bid price (could include profit margin)
                double bid_price = total_cost * BID_MULTIPLIER;
                
                RobotBid bid;
                bid.robot_id = robot.id;
                bid.task_id = task.id;
                bid.bid_price = bid_price;
                bid.capability_score = calculateCapabilityScore(robot, task);
                bid.timestamp = this->now();
                
                bids.push_back(bid);
            }
        }
        
        return bids;
    }
    
    std::map<std::string, std::vector<Task>> assignTasks(
        const std::vector<RobotBid>& bids,
        const std::vector<RobotCapabilities>& robots) {
        
        std::map<std::string, std::vector<Task>> assignments;
        
        // Sort bids by value/cost ratio (efficiency)
        std::vector<RobotBid> sorted_bids = bids;
        std::sort(sorted_bids.begin(), sorted_bids.end(),
                 [](const RobotBid& a, const RobotBid& b) {
                     return (a.capability_score / a.bid_price) > 
                            (b.capability_score / b.bid_price);
                 });
        
        // Assign tasks greedily
        std::set<std::string> assigned_tasks;
        
        for (const auto& bid : sorted_bids) {
            if (assigned_tasks.find(bid.task_id) == assigned_tasks.end()) {
                // Task not yet assigned
                auto task_it = std::find_if(available_tasks_.begin(), available_tasks_.end(),
                                          [&bid](const Task& t) { 
                                              return t.id == bid.task_id; 
                                          });
                
                if (task_it != available_tasks_.end()) {
                    assignments[bid.robot_id].push_back(*task_it);
                    assigned_tasks.insert(bid.task_id);
                }
            }
        }
        
        return assignments;
    }
    
    bool hasConverged(const std::map<std::string, std::vector<Task>>& assignments) {
        // Check if assignments have stabilized
        // Implementation would compare with previous iteration
        return false;  // Placeholder
    }
    
    bool canRobotPerformTask(const RobotCapabilities& robot, const Task& task) {
        // Check if robot has required capabilities for task
        if (task.type == "navigation") {
            return robot.can_navigate;
        } else if (task.type == "manipulation") {
            return robot.has_manipulator && 
                   robot.max_payload >= task.required_payload;
        } else if (task.type == "inspection") {
            return robot.has_appropriate_sensors;
        }
        return false;
    }
    
    double calculateTravelCost(const geometry_msgs::msg::Point& robot_pos,
                              const geometry_msgs::msg::Point& task_pos) {
        // Calculate cost of traveling to task location
        double distance = std::sqrt(std::pow(robot_pos.x - task_pos.x, 2) + 
                                  std::pow(robot_pos.y - task_pos.y, 2) + 
                                  std::pow(robot_pos.z - task_pos.z, 2));
        
        // Cost could be time, energy, or other metric
        return distance * COST_PER_METER;
    }
    
    double calculateCapabilityScore(const RobotCapabilities& robot, const Task& task) {
        // Calculate how well the robot can perform the task
        double score = 1.0;
        
        if (robot.max_payload < task.required_payload) {
            score *= 0.5;  // Robot can't carry required payload
        }
        
        if (task.deadline < (this->now() + rclcpp::Duration::from_seconds(
                calculateTravelTime(robot.position, task.location) + task.estimated_duration))) {
            score *= 0.1;  // Task deadline too tight
        }
        
        // Add other capability considerations
        return std::max(0.0, score);
    }
    
    static constexpr int MAX_BIDDING_ITERATIONS = 20;
    static constexpr double BID_MULTIPLIER = 1.1;  // 10% markup
    static constexpr double COST_PER_METER = 0.5;  // Example cost per meter traveled
};
```

### Formation Control

#### Virtual Structure Formation
```cpp
class VirtualStructureFormation {
private:
    std::string robot_id_;
    int robot_index_;
    std::vector<geometry_msgs::msg::Point> formation_positions_;
    std::vector<std::string> swarm_member_ids_;
    geometry_msgs::msg::Point formation_center_;
    double formation_scale_;
    double k_position_;  // Proportional gain for position control
    
    struct FormationPoint {
        geometry_msgs::msg::Point local_offset;  // Offset from formation center
        double weight;  // Weight in formation (importance factor)
    };

public:
    VirtualStructureFormation(const std::string& id, 
                            const std::vector<std::string>& members,
                            const std::string& formation_type = "line") {
        robot_id_ = id;
        
        // Find robot index in swarm
        for (size_t i = 0; i < members.size(); i++) {
            if (members[i] == robot_id_) {
                robot_index_ = i;
                break;
            }
        }
        
        swarm_member_ids_ = members;
        formation_scale_ = 1.0;
        k_position_ = 2.0;
        
        // Initialize formation based on type
        initializeFormation(formation_type);
    }
    
    geometry_msgs::msg::Twist calculateFormationControl(
        const geometry_msgs::msg::Point& current_position,
        const geometry_msgs::msg::Point& current_velocity) {
        
        // Calculate desired formation position
        geometry_msgs::msg::Point desired_position = 
            calculateDesiredFormationPosition();
        
        // Calculate position error
        geometry_msgs::msg::Vector3 position_error;
        position_error.x = desired_position.x - current_position.x;
        position_error.y = desired_position.y - current_position.y;
        position_error.z = desired_position.z - current_position.z;
        
        // Calculate control command
        geometry_msgs::msg::Twist control_cmd;
        
        // Proportional control for position
        control_cmd.linear.x = k_position_ * position_error.x;
        control_cmd.linear.y = k_position_ * position_error.y;
        control_cmd.linear.z = k_position_ * position_error.z;
        
        // Add velocity damping
        control_cmd.linear.x -= 0.5 * current_velocity.x;
        control_cmd.linear.y -= 0.5 * current_velocity.y;
        control_cmd.linear.z -= 0.5 * current_velocity.z;
        
        // Limit velocity
        double max_vel = 0.5;  // 0.5 m/s max
        double vel_magnitude = std::sqrt(
            std::pow(control_cmd.linear.x, 2) + 
            std::pow(control_cmd.linear.y, 2) + 
            std::pow(control_cmd.linear.z, 2));
        
        if (vel_magnitude > max_vel) {
            control_cmd.linear.x *= max_vel / vel_magnitude;
            control_cmd.linear.y *= max_vel / vel_magnitude;
            control_cmd.linear.z *= max_vel / vel_magnitude;
        }
        
        return control_cmd;
    }
    
    void updateFormationParameters(const geometry_msgs::msg::Point& new_center,
                                  double new_scale) {
        formation_center_ = new_center;
        formation_scale_ = std::max(0.1, new_scale);  // Minimum scale
    }

private:
    void initializeFormation(const std::string& formation_type) {
        formation_positions_.clear();
        
        if (formation_type == "line") {
            initializeLineFormation();
        } else if (formation_type == "circle") {
            initializeCircleFormation();
        } else if (formation_type == "grid") {
            initializeGridFormation();
        } else if (formation_type == "v-shape") {
            initializeVShapeFormation();
        } else {
            // Default to single file line
            initializeLineFormation();
        }
    }
    
    void initializeLineFormation() {
        for (size_t i = 0; i < swarm_member_ids_.size(); i++) {
            FormationPoint fp;
            fp.local_offset.x = i * 0.5 * formation_scale_;  // 0.5m spacing
            fp.local_offset.y = 0.0;
            fp.local_offset.z = 0.0;
            fp.weight = 1.0;
            
            formation_positions_.push_back(fp.local_offset);
        }
    }
    
    void initializeCircleFormation() {
        double angle_increment = 2 * M_PI / swarm_member_ids_.size();
        
        for (size_t i = 0; i < swarm_member_ids_.size(); i++) {
            FormationPoint fp;
            double angle = i * angle_increment;
            fp.local_offset.x = std::cos(angle) * 1.0 * formation_scale_;
            fp.local_offset.y = std::sin(angle) * 1.0 * formation_scale_;
            fp.local_offset.z = 0.0;
            fp.weight = 1.0;
            
            formation_positions_.push_back(fp.local_offset);
        }
    }
    
    void initializeGridFormation() {
        int grid_size = std::ceil(std::sqrt(swarm_member_ids_.size()));
        
        for (size_t i = 0; i < swarm_member_ids_.size(); i++) {
            FormationPoint fp;
            int row = i / grid_size;
            int col = i % grid_size;
            
            fp.local_offset.x = col * 0.5 * formation_scale_;
            fp.local_offset.y = row * 0.5 * formation_scale_;
            fp.local_offset.z = 0.0;
            fp.weight = 1.0;
            
            formation_positions_.push_back(fp.local_offset);
        }
    }
    
    void initializeVShapeFormation() {
        // V-shaped formation
        for (size_t i = 0; i < swarm_member_ids_.size(); i++) {
            FormationPoint fp;
            if (i == 0) {
                // Leader at point of V
                fp.local_offset.x = 0.0;
                fp.local_offset.y = 0.0;
                fp.local_offset.z = 0.0;
            } else {
                // Others form V sides
                int side = (i - 1) % 2 == 0 ? -1 : 1;  // Alternate sides
                int position_on_side = (i - 1) / 2 + 1;  // Position along side
                
                fp.local_offset.x = position_on_side * 0.5 * formation_scale_;
                fp.local_offset.y = side * position_on_side * 0.3 * formation_scale_;
                fp.local_offset.z = 0.0;
            }
            fp.weight = 1.0;
            
            formation_positions_.push_back(fp.local_offset);
        }
    }
    
    geometry_msgs::msg::Point calculateDesiredFormationPosition() {
        geometry_msgs::msg::Point desired_pos;
        
        if (robot_index_ < formation_positions_.size()) {
            // Add formation offset to formation center
            desired_pos.x = formation_center_.x + formation_positions_[robot_index_].x;
            desired_pos.y = formation_center_.y + formation_positions_[robot_index_].y;
            desired_pos.z = formation_center_.z + formation_positions_[robot_index_].z;
        } else {
            // Robot not in formation - stay at formation center
            desired_pos = formation_center_;
        }
        
        return desired_pos;
    }
    
    geometry_msgs::msg::Point formation_center_{0, 0, 0};
};
```

## Swarm Behaviors and Algorithms

### Collective Behaviors

#### Foraging Behavior
```cpp
class SwarmForaging {
private:
    std::string robot_id_;
    bool has_food_;
    geometry_msgs::msg::Point carrying_location_;
    double food_quality_;
    std::vector<FoodSource> known_food_sources_;
    std::vector<PheromoneTrail> pheromone_trails_;
    
    struct FoodSource {
        geometry_msgs::msg::Point location;
        double quality;  // How much food is available
        double last_visited;
        bool depleted;
    };
    
    struct PheromoneTrail {
        geometry_msgs::msg::Point start;
        geometry_msgs::msg::Point end;
        double strength;
        rclcpp::Time timestamp;
    };

public:
    SwarmForaging(const std::string& id) : robot_id_(id), has_food_(false), food_quality_(0.0) {}
    
    geometry_msgs::msg::Twist forageBehavior(
        const geometry_msgs::msg::Point& current_pos,
        const std::vector<FoodSource>& sensed_food) {
        
        geometry_msgs::msg::Twist cmd;
        
        if (has_food_) {
            // Return to nest/base
            return navigateToNest(current_pos);
        } else {
            // Search for food
            if (!sensed_food.empty()) {
                // Food detected, go to it
                return navigateToFood(sensed_food[0].location, current_pos);
            } else {
                // No food detected, follow pheromone trails or random walk
                auto trail_direction = followPheromoneTrails();
                if (trail_direction.x != 0 || trail_direction.y != 0) {
                    // Follow pheromone trail
                    cmd.linear.x = trail_direction.x * 0.3;  // 0.3 m/s
                    cmd.linear.y = trail_direction.y * 0.3;
                    cmd.angular.z = calculateAvoidance(current_pos);  // Avoid obstacles
                } else {
                    // Random walk with bias toward unexplored areas
                    cmd = randomWalk(current_pos);
                }
            }
        }
        
        return cmd;
    }
    
    void depositPheromone(const geometry_msgs::msg::Point& start,
                         const geometry_msgs::msg::Point& end) {
        // Deposit pheromone trail to indicate food path
        PheromoneTrail trail;
        trail.start = start;
        trail.end = end;
        trail.strength = 1.0;
        trail.timestamp = this->now();
        
        pheromone_trails_.push_back(trail);
        
        // Publish pheromone information to swarm
        publishPheromoneUpdate(trail);
    }
    
    void updateFoodSources(const std::vector<FoodSource>& new_sources) {
        for (const auto& new_source : new_sources) {
            bool found = false;
            for (auto& known_source : known_food_sources_) {
                if (calculateDistance(known_source.location, new_source.location) < 0.5) {
                    // Same location, update quality
                    known_source.quality = new_source.quality;
                    known_source.last_visited = this->now().seconds();
                    known_source.depleted = new_source.depleted;
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                // New food source
                known_food_sources_.push_back(new_source);
            }
        }
    }

private:
    geometry_msgs::msg::Twist navigateToFood(const geometry_msgs::msg::Point& food_pos,
                                           const geometry_msgs::msg::Point& current_pos) {
        geometry_msgs::msg::Twist cmd;
        
        double dx = food_pos.x - current_pos.x;
        double dy = food_pos.y - current_pos.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        if (distance > 0.1) {  // 10cm threshold
            cmd.linear.x = dx / distance * 0.3;  // 0.3 m/s
            cmd.linear.y = dy / distance * 0.3;
            
            // Calculate angular velocity to face food
            double target_angle = std::atan2(dy, dx);
            cmd.angular.z = target_angle * 0.5;  // Proportional control
        }
        
        return cmd;
    }
    
    geometry_msgs::msg::Twist navigateToNest(const geometry_msgs::msg::Point& current_pos) {
        // Navigate back to nest/base location
        geometry_msgs::msg::Point nest_pos = getNestLocation();
        
        double dx = nest_pos.x - current_pos.x;
        double dy = nest_pos.y - current_pos.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        geometry_msgs::msg::Twist cmd;
        
        if (distance > 0.1) {
            cmd.linear.x = dx / distance * 0.2;  // Slower when carrying food
            cmd.linear.y = dy / distance * 0.2;
            
            // Deposit pheromone trail
            if (distance < 10.0) {  // Only deposit when close to nest
                depositPheromone(current_pos, nest_pos);
            }
        }
        
        return cmd;
    }
    
    geometry_msgs::msg::Point followPheromoneTrails() {
        // Calculate direction based on strongest pheromone trail
        geometry_msgs::msg::Point direction{0, 0, 0};
        double max_strength = 0.0;
        
        for (const auto& trail : pheromone_trails_) {
            // Apply pheromone decay
            double age = (this->now() - trail.timestamp).seconds();
            double decayed_strength = trail.strength * std::exp(-DECAY_RATE * age);
            
            if (decayed_strength > max_strength) {
                // Calculate direction vector from trail
                double dx = trail.end.x - trail.start.x;
                double dy = trail.end.y - trail.start.y;
                double mag = std::sqrt(dx*dx + dy*dy);
                
                if (mag > 0) {
                    direction.x = dx / mag;
                    direction.y = dy / mag;
                    max_strength = decayed_strength;
                }
            }
        }
        
        return direction;
    }
    
    geometry_msgs::msg::Twist randomWalk(const geometry_msgs::msg::Point& current_pos) {
        // Implement random walk with bias toward unexplored areas
        geometry_msgs::msg::Twist cmd;
        
        // Simple random walk
        cmd.linear.x = 0.2;  // Forward motion
        cmd.angular.z = (static_cast<double>(rand()) / RAND_MAX - 0.5) * 0.5;  // Random turn
        
        return cmd;
    }
    
    geometry_msgs::msg::Point getNestLocation() {
        // Return the location of the nest/base
        geometry_msgs::msg::Point nest;
        nest.x = 0.0;  // Example nest location
        nest.y = 0.0;
        nest.z = 0.0;
        return nest;
    }
    
    double calculateAvoidance(const geometry_msgs::msg::Point& current_pos) {
        // Simple obstacle avoidance (would use actual sensor data in implementation)
        // This is a placeholder for actual sensor processing
        return 0.0;  // Placeholder
    }
    
    void publishPheromoneUpdate(const PheromoneTrail& trail) {
        // Publish pheromone information to other swarm members
        auto msg = multirobot_msgs::msg::PheromoneUpdate();
        msg.header.stamp = this->now();
        msg.start_point = trail.start;
        msg.end_point = trail.end;
        msg.strength = trail.strength;
        msg.source_robot = robot_id_;
        
        pheromone_publisher_->publish(msg);
    }
    
    static constexpr double DECAY_RATE = 0.1;  // Pheromone decay rate
    
    rclcpp::Publisher<multirobot_msgs::msg::PheromoneUpdate>::SharedPtr pheromone_publisher_;
};
```

### Flocking Behavior Implementation

```cpp
class FlockingBehavior {
private:
    std::string robot_id_;
    double separation_distance_;
    double alignment_distance_;
    double cohesion_distance_;
    double max_velocity_;
    
    std::vector<RobotNeighbor> neighbors_;

    struct RobotNeighbor {
        std::string id;
        geometry_msgs::msg::Point position;
        geometry_msgs::msg::Vector3 velocity;
        double distance;
    };

public:
    FlockingBehavior(const std::string& id) 
        : robot_id_(id), separation_distance_(0.5), 
          alignment_distance_(1.5), cohesion_distance_(2.0), max_velocity_(0.3) {}
    
    geometry_msgs::msg::Twist calculateFlockingMotion(
        const geometry_msgs::msg::Point& current_pos,
        const geometry_msgs::msg::Vector3& current_vel,
        const std::vector<RobotState>& swarm_states) {
        
        // Update neighbor list
        updateNeighbors(current_pos, swarm_states);
        
        // Calculate flocking forces
        auto separation = calculateSeparation(current_pos);
        auto alignment = calculateAlignment(current_vel);
        auto cohesion = calculateCohesion(current_pos);
        
        // Combine forces with weights
        geometry_msgs::msg::Vector3 flocking_force;
        flocking_force.x = SEPARATION_WEIGHT * separation.x + 
                          ALIGNMENT_WEIGHT * alignment.x + 
                          COHESION_WEIGHT * cohesion.x;
        flocking_force.y = SEPARATION_WEIGHT * separation.y + 
                          ALIGNMENT_WEIGHT * alignment.y + 
                          COHESION_WEIGHT * cohesion.y;
        flocking_force.z = SEPARATION_WEIGHT * separation.z + 
                          ALIGNMENT_WEIGHT * alignment.z + 
                          COHESION_WEIGHT * cohesion.z;
        
        // Convert force to velocity command
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::clamp(flocking_force.x, -max_velocity_, max_velocity_);
        cmd.linear.y = std::clamp(flocking_force.y, -max_velocity_, max_velocity_);
        cmd.linear.z = std::clamp(flocking_force.z, -max_velocity_, max_velocity_);
        
        // Calculate angular velocity to align with movement direction
        if (std::abs(cmd.linear.x) > 0.01 || std::abs(cmd.linear.y) > 0.01) {
            double target_angle = std::atan2(cmd.linear.y, cmd.linear.x);
            double current_angle = getCurrentYaw(current_vel);
            cmd.angular.z = std::clamp(target_angle - current_angle, -1.0, 1.0);
        }
        
        return cmd;
    }

private:
    void updateNeighbors(const geometry_msgs::msg::Point& current_pos,
                        const std::vector<RobotState>& swarm_states) {
        neighbors_.clear();
        
        for (const auto& robot_state : swarm_states) {
            if (robot_state.id != robot_id_) {
                double distance = calculateDistance(current_pos, robot_state.position);
                
                if (distance < cohesion_distance_) {  // Only consider nearby robots
                    RobotNeighbor neighbor;
                    neighbor.id = robot_state.id;
                    neighbor.position = robot_state.position;
                    neighbor.velocity = robot_state.velocity;
                    neighbor.distance = distance;
                    
                    neighbors_.push_back(neighbor);
                }
            }
        }
    }
    
    geometry_msgs::msg::Vector3 calculateSeparation(const geometry_msgs::msg::Point& current_pos) {
        geometry_msgs::msg::Vector3 separation{0, 0, 0};
        int count = 0;
        
        for (const auto& neighbor : neighbors_) {
            if (neighbor.distance < separation_distance_ && neighbor.distance > 0) {
                // Calculate repulsion vector (away from neighbor)
                double dx = current_pos.x - neighbor.position.x;
                double dy = current_pos.y - neighbor.position.y;
                double dz = current_pos.z - neighbor.position.z;
                
                // Normalize and scale by inverse distance (closer = stronger repulsion)
                double dist = neighbor.distance;
                if (dist > 0.01) {  // Avoid division by zero
                    separation.x += (dx / dist) / dist;  // Inverse square law
                    separation.y += (dy / dist) / dist;
                    separation.z += (dz / dist) / dist;
                    count++;
                }
            }
        }
        
        if (count > 0) {
            separation.x /= count;
            separation.y /= count;
            separation.z /= count;
        }
        
        return separation;
    }
    
    geometry_msgs::msg::Vector3 calculateAlignment(const geometry_msgs::msg::Vector3& current_vel) {
        geometry_msgs::msg::Vector3 alignment{0, 0, 0};
        int count = 0;
        
        for (const auto& neighbor : neighbors_) {
            if (neighbor.distance < alignment_distance_) {
                alignment.x += neighbor.velocity.x;
                alignment.y += neighbor.velocity.y;
                alignment.z += neighbor.velocity.z;
                count++;
            }
        }
        
        if (count > 0) {
            alignment.x /= count;
            alignment.y /= count;
            alignment.z /= count;
            
            // Normalize to unit vector
            double magnitude = std::sqrt(alignment.x*alignment.x + 
                                       alignment.y*alignment.y + 
                                       alignment.z*alignment.z);
            if (magnitude > 0) {
                alignment.x /= magnitude;
                alignment.y /= magnitude;
                alignment.z /= magnitude;
            }
        }
        
        return alignment;
    }
    
    geometry_msgs::msg::Vector3 calculateCohesion(const geometry_msgs::msg::Point& current_pos) {
        geometry_msgs::msg::Point center_of_mass{0, 0, 0};
        int count = 0;
        
        for (const auto& neighbor : neighbors_) {
            if (neighbor.distance < cohesion_distance_) {
                center_of_mass.x += neighbor.position.x;
                center_of_mass.y += neighbor.position.y;
                center_of_mass.z += neighbor.position.z;
                count++;
            }
        }
        
        if (count > 0) {
            center_of_mass.x /= count;
            center_of_mass.y /= count;
            center_of_mass.z /= count;
            
            // Calculate attraction vector toward center of mass
            geometry_msgs::msg::Vector3 cohesion;
            cohesion.x = center_of_mass.x - current_pos.x;
            cohesion.y = center_of_mass.y - current_pos.y;
            cohesion.z = center_of_mass.z - current_pos.z;
            
            // Normalize
            double magnitude = std::sqrt(cohesion.x*cohesion.x + 
                                       cohesion.y*cohesion.y + 
                                       cohesion.z*cohesion.z);
            if (magnitude > 0) {
                cohesion.x /= magnitude;
                cohesion.y /= magnitude;
                cohesion.z /= magnitude;
            }
            
            return cohesion;
        }
        
        return {0, 0, 0};
    }
    
    double getCurrentYaw(const geometry_msgs::msg::Vector3& velocity) {
        // Calculate yaw from velocity vector
        return std::atan2(velocity.y, velocity.x);
    }
    
    static constexpr double SEPARATION_WEIGHT = 1.5;
    static constexpr double ALIGNMENT_WEIGHT = 1.0;
    static constexpr double COHESION_WEIGHT = 1.0;
};
```

## Multi-Robot Coordination Patterns

### Distributed Coordination

#### Leader-Follower Pattern
```cpp
class LeaderFollowerCoordinator {
private:
    std::string robot_id_;
    RobotRole role_;
    std::string leader_id_;
    geometry_msgs::msg::Point follower_offset_;
    double max_following_distance_;
    
    enum class RobotRole {
        LEADER,
        FOLLOWER
    };

public:
    LeaderFollowerCoordinator(const std::string& id, 
                            const std::string& leader_id = "")
        : robot_id_(id), leader_id_(leader_id) {
        
        if (leader_id.empty() || id == leader_id) {
            role_ = RobotRole::LEADER;
            // Leader doesn't follow anyone
        } else {
            role_ = RobotRole::FOLLOWER;
            // Default offset for follower (behind leader)
            follower_offset_.x = -1.0;  // 1 meter behind
            follower_offset_.y = 0.0;
            follower_offset_.z = 0.0;
            max_following_distance_ = 2.0;  // Max 2 meters from leader
        }
    }
    
    geometry_msgs::msg::Twist getMotionCommand(
        const geometry_msgs::msg::Point& current_pos,
        const geometry_msgs::msg::Point& leader_pos,
        const geometry_msgs::msg::Vector3& leader_vel) {
        
        if (role_ == RobotRole::LEADER) {
            // Leader follows its own path or commands
            return getLeaderCommand();
        } else {
            // Follower tries to maintain position relative to leader
            return getFollowerCommand(current_pos, leader_pos, leader_vel);
        }
    }
    
    void setFollowerOffset(const geometry_msgs::msg::Point& offset) {
        if (role_ == RobotRole::FOLLOWER) {
            follower_offset_ = offset;
        }
    }
    
    bool isLeader() const { return role_ == RobotRole::LEADER; }

private:
    geometry_msgs::msg::Twist getLeaderCommand() {
        // Leader behavior - could be path following, exploration, etc.
        geometry_msgs::msg::Twist cmd;
        
        // Example: Leader follows a predetermined path
        auto next_waypoint = getNextWaypoint();
        if (next_waypoint.has_value()) {
            cmd = navigateToPoint(next_waypoint.value());
        }
        
        return cmd;
    }
    
    geometry_msgs::msg::Twist getFollowerCommand(
        const geometry_msgs::msg::Point& current_pos,
        const geometry_msgs::msg::Point& leader_pos,
        const geometry_msgs::msg::Vector3& leader_vel) {
        
        geometry_msgs::msg::Twist cmd;
        
        // Calculate desired position relative to leader
        geometry_msgs::msg::Point desired_pos;
        desired_pos.x = leader_pos.x + follower_offset_.x;
        desired_pos.y = leader_pos.y + follower_offset_.y;
        desired_pos.z = leader_pos.z + follower_offset_.z;
        
        // Calculate error from desired position
        double dx = desired_pos.x - current_pos.x;
        double dy = desired_pos.y - current_pos.y;
        double dz = desired_pos.z - current_pos.z;
        
        double distance_error = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (distance_error > 0.1) {  // If significantly off position
            // Proportional control to reach desired position
            cmd.linear.x = dx * 0.5;
            cmd.linear.y = dy * 0.5;
            cmd.linear.z = dz * 0.5;
            
            // Add leader velocity to maintain formation during movement
            cmd.linear.x += leader_vel.x * 0.8;  // Follow leader's movement
            cmd.linear.y += leader_vel.y * 0.8;
            cmd.linear.z += leader_vel.z * 0.8;
            
            // Limit maximum velocity
            double vel_magnitude = std::sqrt(
                std::pow(cmd.linear.x, 2) + 
                std::pow(cmd.linear.y, 2) + 
                std::pow(cmd.linear.z, 2));
            
            if (vel_magnitude > max_velocity_) {
                cmd.linear.x *= max_velocity_ / vel_magnitude;
                cmd.linear.y *= max_velocity_ / vel_magnitude;
                cmd.linear.z *= max_velocity_ / vel_magnitude;
            }
        } else {
            // Close to desired position, match leader's velocity
            cmd.linear.x = leader_vel.x;
            cmd.linear.y = leader_vel.y;
            cmd.linear.z = leader_vel.z;
        }
        
        // Calculate angular velocity to face movement direction
        double target_angle = std::atan2(cmd.linear.y, cmd.linear.x);
        cmd.angular.z = target_angle * 0.5;
        
        return cmd;
    }
    
    std::optional<geometry_msgs::msg::Point> getNextWaypoint() {
        // Implementation to get next waypoint for leader
        // This would typically interface with path planning system
        return std::nullopt;  // Placeholder
    }
    
    geometry_msgs::msg::Twist navigateToPoint(const geometry_msgs::msg::Point& target) {
        // Simple navigation to target point
        geometry_msgs::msg::Twist cmd;
        
        // Calculate direction to target
        // Implementation would go here
        return cmd;  // Placeholder
    }
    
    static constexpr double MAX_VELOCITY = 0.3;  // m/s
};
```

### Consensus-Based Coordination

#### Distributed Consensus for Task Coordination
```cpp
class DistributedConsensusTaskCoordinator {
private:
    std::string robot_id_;
    std::vector<std::string> swarm_ids_;
    std::vector<TaskProposal> proposals_;
    std::vector<TaskAssignment> assignments_;
    std::map<std::string, double> robot_scores_;  // Performance scores
    
    struct TaskProposal {
        std::string task_id;
        std::string proposer_id;
        geometry_msgs::msg::Point task_location;
        TaskType task_type;
        double task_value;
        rclcpp::Time timestamp;
    };
    
    struct TaskAssignment {
        std::string task_id;
        std::string assigned_robot;
        rclcpp::Time assignment_time;
    };
    
    enum class TaskType {
        NAVIGATION, MANIPULATION, INSPECTION, TRANSPORT, OTHER
    };

public:
    DistributedConsensusTaskCoordinator(const std::string& id, 
                                      const std::vector<std::string>& swarm_ids)
        : robot_id_(id), swarm_ids_(swarm_ids) {
        
        // Initialize performance scores
        for (const auto& robot_id : swarm_ids_) {
            robot_scores_[robot_id] = 1.0;  // Start with neutral score
        }
    }
    
    TaskAssignment coordinateTaskAssignment(const TaskProposal& proposal) {
        // Add proposal to local list
        proposals_.push_back(proposal);
        
        // Start consensus process
        auto consensus_result = reachConsensusOnTaskAssignment(proposal);
        
        // Create assignment based on consensus
        TaskAssignment assignment;
        assignment.task_id = proposal.task_id;
        assignment.assigned_robot = consensus_result.selected_robot;
        assignment.assignment_time = this->now();
        
        assignments_.push_back(assignment);
        
        return assignment;
    }
    
    std::vector<TaskAssignment> getActiveAssignments() const {
        // Return assignments that are still active
        std::vector<TaskAssignment> active;
        auto now = this->now();
        
        for (const auto& assignment : assignments_) {
            if ((now - assignment.assignment_time).seconds() < ASSIGNMENT_TIMEOUT) {
                active.push_back(assignment);
            }
        }
        
        return active;
    }

private:
    struct ConsensusResult {
        std::string selected_robot;
        double confidence;
        std::vector<std::string> participating_robots;
    };
    
    ConsensusResult reachConsensusOnTaskAssignment(const TaskProposal& proposal) {
        ConsensusResult result;
        
        // Calculate cost for each robot to perform task
        std::vector<std::pair<std::string, double>> robot_costs;
        
        for (const auto& robot_id : swarm_ids_) {
            double cost = calculateTaskCost(robot_id, proposal);
            robot_costs.push_back({robot_id, cost});
        }
        
        // Sort by cost (lowest cost first)
        std::sort(robot_costs.begin(), robot_costs.end(),
                 [](const std::pair<std::string, double>& a, 
                    const std::pair<std::string, double>& b) {
                     return a.second < b.second;
                 });
        
        // Select robot with lowest cost
        result.selected_robot = robot_costs[0].first;
        result.confidence = 1.0 / (1.0 + robot_costs[0].second);  // Higher confidence for lower cost
        result.participating_robots = swarm_ids_;
        
        return result;
    }
    
    double calculateTaskCost(const std::string& robot_id, const TaskProposal& proposal) {
        // Calculate cost based on distance, capabilities, current load, etc.
        double distance_cost = getDistanceToRobot(robot_id, proposal.task_location) * DISTANCE_WEIGHT;
        double capability_cost = getCapabilityMismatch(robot_id, proposal.task_type) * CAPABILITY_WEIGHT;
        double load_cost = getCurrentLoad(robot_id) * LOAD_WEIGHT;
        double performance_cost = (1.0 / robot_scores_[robot_id]) * PERFORMANCE_WEIGHT;
        
        return distance_cost + capability_cost + load_cost + performance_cost;
    }
    
    double getDistanceToRobot(const std::string& robot_id, 
                             const geometry_msgs::msg::Point& task_location) {
        // Get current position of robot and calculate distance to task
        auto robot_pos = getCurrentRobotPosition(robot_id);
        return std::sqrt(std::pow(robot_pos.x - task_location.x, 2) + 
                        std::pow(robot_pos.y - task_location.y, 2) + 
                        std::pow(robot_pos.z - task_location.z, 2));
    }
    
    double getCapabilityMismatch(const std::string& robot_id, TaskType task_type) {
        // Check if robot has required capabilities for task type
        auto robot_caps = getRobotCapabilities(robot_id);
        
        switch (task_type) {
            case TaskType::MANIPULATION:
                return robot_caps.has_manipulator ? 0.0 : 10.0;
            case TaskType::TRANSPORT:
                return (robot_caps.has_manipulator && 
                       robot_caps.max_payload >= MIN_PAYLOAD) ? 0.0 : 10.0;
            case TaskType::INSPECTION:
                return robot_caps.has_appropriate_sensors ? 0.0 : 5.0;
            case TaskType::NAVIGATION:
                return 0.0;  // All robots can navigate
            default:
                return 1.0;  // Default penalty
        }
    }
    
    double getCurrentLoad(const std::string& robot_id) {
        // Calculate current task load for robot
        int task_count = 0;
        for (const auto& assignment : assignments_) {
            if (assignment.assigned_robot == robot_id) {
                task_count++;
            }
        }
        return task_count * 2.0;  // Penalty per assigned task
    }
    
    RobotPosition getCurrentRobotPosition(const std::string& robot_id) {
        // Get current position of specified robot
        // This would interface with swarm state management
        RobotPosition pos;
        pos.x = 0.0; pos.y = 0.0; pos.z = 0.0;  // Placeholder
        return pos;
    }
    
    RobotCapabilities getRobotCapabilities(const std::string& robot_id) {
        // Get capabilities of specified robot
        // This would interface with robot capability database
        RobotCapabilities caps;
        caps.has_manipulator = false;
        caps.has_appropriate_sensors = true;
        caps.max_payload = 0.0;
        return caps;  // Placeholder
    }
    
    struct RobotPosition {
        double x, y, z;
    };
    
    struct RobotCapabilities {
        bool has_manipulator;
        bool has_appropriate_sensors;
        double max_payload;
    };
    
    static constexpr double DISTANCE_WEIGHT = 1.0;
    static constexpr double CAPABILITY_WEIGHT = 5.0;
    static constexpr double LOAD_WEIGHT = 2.0;
    static constexpr double PERFORMANCE_WEIGHT = 1.0;
    static constexpr double ASSIGNMENT_TIMEOUT = 60.0;  // seconds
    static constexpr double MIN_PAYLOAD = 1.0;  // kg
};
```

## Performance Optimization for Swarms

### Communication Efficiency

#### Message Optimization
- **Compression**: Compress large data structures before transmission
- **Filtering**: Only transmit necessary information
- **Aggregation**: Combine multiple updates into single messages
- **Throttling**: Limit update frequency to necessary rates

#### Bandwidth Management
```cpp
class BandwidthManager {
private:
    std::map<std::string, double> robot_bandwidth_usage_;
    double total_bandwidth_limit_;
    double current_bandwidth_usage_;
    
public:
    BandwidthManager(double total_limit = 10.0)  // 10 Mbps limit
        : total_bandwidth_limit_(total_limit), current_bandwidth_usage_(0.0) {}
    
    bool canSend(const std::string& robot_id, double message_size_bytes, double frequency_hz) {
        double required_bandwidth = message_size_bytes * frequency_hz * 8 / 1e6;  // Convert to Mbps
        
        // Calculate current usage for this robot
        double robot_current_usage = robot_bandwidth_usage_[robot_id];
        
        // Check if adding this message would exceed limits
        if (robot_current_usage + required_bandwidth > getRobotLimit(robot_id)) {
            return false;
        }
        
        // Check if total swarm would exceed limits
        if (current_bandwidth_usage_ + required_bandwidth > total_bandwidth_limit_) {
            return false;
        }
        
        return true;
    }
    
    void recordTransmission(const std::string& robot_id, double message_size_bytes, double frequency_hz) {
        double bandwidth_used = message_size_bytes * frequency_hz * 8 / 1e6;  // Mbps
        
        robot_bandwidth_usage_[robot_id] += bandwidth_used;
        current_bandwidth_usage_ += bandwidth_used;
    }
    
    void optimizeMessages(std::vector<CommunicationMessage>& messages) {
        // Apply optimization strategies to reduce bandwidth usage
        for (auto& msg : messages) {
            if (msg.type == MessageType::SENSOR_DATA) {
                // Apply data compression to sensor messages
                msg.payload = compressSensorData(msg.payload);
            } else if (msg.type == MessageType::STATE_UPDATE) {
                // Apply delta compression to state messages
                msg.payload = deltaCompressState(msg.payload, msg.previous_payload);
            }
        }
    }

private:
    double getRobotLimit(const std::string& robot_id) {
        // Different robots might have different bandwidth limits
        // based on their communication hardware
        return 2.0;  // Default 2 Mbps per robot
    }
    
    std::vector<uint8_t> compressSensorData(const std::vector<uint8_t>& data) {
        // Apply compression algorithm to sensor data
        // This could be lossless or lossy compression depending on requirements
        return data;  // Placeholder
    }
    
    std::vector<uint8_t> deltaCompressState(const std::vector<uint8_t>& current_state,
                                           const std::vector<uint8_t>& previous_state) {
        // Only transmit changes from previous state
        // This reduces bandwidth for frequently updated state information
        return current_state;  // Placeholder
    }
    
    enum class MessageType {
        STATE_UPDATE, SENSOR_DATA, TASK_ASSIGNMENT, FORMATION_CONTROL, OTHER
    };
    
    struct CommunicationMessage {
        std::string source_robot;
        std::string destination_robot;
        MessageType type;
        std::vector<uint8_t> payload;
        std::vector<uint8_t> previous_payload;  // For delta compression
        rclcpp::Time timestamp;
    };
};
```

### Computational Efficiency

#### Distributed Computation
- **Task Division**: Divide computational tasks among swarm members
- **Load Balancing**: Distribute computational load evenly
- **Caching**: Cache frequently computed values
- **Approximation**: Use approximations for non-critical computations

#### Resource Sharing
```cpp
class DistributedComputationManager {
private:
    std::vector<ComputationTask> pending_tasks_;
    std::vector<RobotComputationCapability> robot_capabilities_;
    std::vector<RobotWorkload> robot_workloads_;
    
    struct ComputationTask {
        std::string id;
        ComputationType type;
        std::vector<uint8_t> input_data;
        std::function<void(const std::vector<uint8_t>&)> result_callback;
        double priority;
        rclcpp::Time deadline;
    };
    
    struct RobotComputationCapability {
        std::string robot_id;
        double cpu_capacity;      // FLOPS
        double gpu_capacity;      // FLOPS (if available)
        double available_memory;  // MB
        double network_bandwidth; // Mbps
        std::vector<ComputationType> supported_types;
    };
    
    struct RobotWorkload {
        std::string robot_id;
        int active_tasks;
        double cpu_utilization;
        double memory_utilization;
    };

public:
    std::string assignComputationTask(const ComputationTask& task) {
        // Find the most suitable robot for this task
        auto suitable_robots = findSuitableRobots(task);
        
        if (suitable_robots.empty()) {
            return "";  // No suitable robot found
        }
        
        // Select robot based on capability and current workload
        auto selected_robot = selectOptimalRobot(suitable_robots, task);
        
        if (!selected_robot.empty()) {
            // Assign task to robot
            assignTaskToRobot(selected_robot, task);
            
            // Update workload tracking
            updateRobotWorkload(selected_robot, task);
        }
        
        return selected_robot;
    }

private:
    std::vector<std::string> findSuitableRobots(const ComputationTask& task) {
        std::vector<std::string> suitable;
        
        for (const auto& robot_caps : robot_capabilities_) {
            // Check if robot supports this computation type
            bool supports_type = std::find(robot_caps.supported_types.begin(),
                                         robot_caps.supported_types.end(),
                                         task.type) != robot_caps.supported_types.end();
            
            if (supports_type) {
                // Estimate required resources
                double estimated_cpu = estimateCPUNeeds(task);
                double estimated_memory = estimateMemoryNeeds(task);
                
                // Check if robot has sufficient resources
                if (estimated_cpu < robot_caps.cpu_capacity * (1 - CPU_UTILIZATION_THRESHOLD) &&
                    estimated_memory < robot_caps.available_memory * (1 - MEMORY_THRESHOLD)) {
                    suitable.push_back(robot_caps.robot_id);
                }
            }
        }
        
        return suitable;
    }
    
    std::string selectOptimalRobot(const std::vector<std::string>& candidates,
                                  const ComputationTask& task) {
        // Select robot based on multiple factors:
        // - Current workload
        // - Computation capability
        // - Network connectivity
        // - Task priority
        
        std::string best_robot = candidates[0];
        double best_score = calculateAssignmentScore(candidates[0], task);
        
        for (size_t i = 1; i < candidates.size(); i++) {
            double score = calculateAssignmentScore(candidates[i], task);
            if (score > best_score) {
                best_score = score;
                best_robot = candidates[i];
            }
        }
        
        return best_robot;
    }
    
    double calculateAssignmentScore(const std::string& robot_id, 
                                   const ComputationTask& task) {
        // Calculate score based on multiple factors
        auto robot_caps = getRobotCapabilities(robot_id);
        auto robot_workload = getRobotWorkload(robot_id);
        
        double capability_score = 0.0;
        if (task.type == ComputationType::NEURAL_NETWORK) {
            capability_score = robot_caps.gpu_capacity > 0 ? 1.0 : 0.3;  // Prefer GPU-enabled robots
        } else {
            capability_score = robot_caps.cpu_capacity / 1e9;  // Normalize to GHz
        }
        
        double availability_score = (1.0 - robot_workload.cpu_utilization) * 
                                   (1.0 - robot_workload.active_tasks / MAX_TASKS_PER_ROBOT);
        
        double network_score = robot_caps.network_bandwidth / 100.0;  // Normalize to 100 Mbps
        
        // Weighted combination of factors
        return (capability_score * 0.5 + 
                availability_score * 0.3 + 
                network_score * 0.2) * task.priority;
    }
    
    enum class ComputationType {
        NEURAL_NETWORK, FILTERING, PLANNING, CONTROL, OTHER
    };
    
    static constexpr double CPU_UTILIZATION_THRESHOLD = 0.8;
    static constexpr double MEMORY_THRESHOLD = 0.8;
    static constexpr int MAX_TASKS_PER_ROBOT = 5;
};
```

## Safety and Coordination

### Swarm Safety Protocols

#### Collision Avoidance in Swarms
```cpp
class SwarmCollisionAvoidance {
private:
    std::string robot_id_;
    std::vector<RobotState> swarm_states_;
    double safe_distance_;
    double detection_radius_;
    
    struct RobotState {
        std::string id;
        geometry_msgs::msg::Point position;
        geometry_msgs::msg::Vector3 velocity;
        geometry_msgs::msg::Point size;  // Width, length, height
        rclcpp::Time timestamp;
    };

public:
    SwarmCollisionAvoidance(const std::string& id, double safe_dist = 0.5)
        : robot_id_(id), safe_distance_(safe_dist), detection_radius_(2.0) {}
    
    geometry_msgs::msg::Twist avoidCollisions(
        const geometry_msgs::msg::Point& current_pos,
        const geometry_msgs::msg::Vector3& current_vel,
        const std::vector<RobotState>& nearby_robots) {
        
        geometry_msgs::msg::Twist avoidance_cmd;
        
        // Check for potential collisions
        for (const auto& other_robot : nearby_robots) {
            if (other_robot.id == robot_id_) continue;  // Skip self
            
            double distance = calculateDistance(current_pos, other_robot.position);
            
            if (distance < safe_distance_) {
                // Potential collision detected, calculate avoidance vector
                double dx = current_pos.x - other_robot.position.x;
                double dy = current_pos.y - other_robot.position.y;
                double dz = current_pos.z - other_robot.position.z;
                
                // Normalize direction away from obstacle
                double norm = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (norm > 0.01) {  // Avoid division by zero
                    dx /= norm;
                    dy /= norm;
                    dz /= norm;
                    
                    // Apply stronger avoidance as distance decreases
                    double avoidance_strength = (safe_distance_ - distance) / safe_distance_;
                    
                    avoidance_cmd.linear.x += dx * avoidance_strength * AVOIDANCE_GAIN;
                    avoidance_cmd.linear.y += dy * avoidance_strength * AVOIDANCE_GAIN;
                    avoidance_cmd.linear.z += dz * avoidance_strength * AVOIDANCE_GAIN;
                }
            }
        }
        
        // Limit avoidance command to reasonable values
        avoidance_cmd.linear.x = std::clamp(avoidance_cmd.linear.x, -MAX_AVOIDANCE_VEL, MAX_AVOIDANCE_VEL);
        avoidance_cmd.linear.y = std::clamp(avoidance_cmd.linear.y, -MAX_AVOIDANCE_VEL, MAX_AVOIDANCE_VEL);
        avoidance_cmd.linear.z = std::clamp(avoidance_cmd.linear.z, -MAX_AVOIDANCE_VEL, MAX_AVOIDANCE_VEL);
        
        return avoidance_cmd;
    }
    
    std::vector<RobotState> filterNearbyRobots(const std::vector<RobotState>& all_robots) {
        std::vector<RobotState> nearby;
        
        for (const auto& robot : all_robots) {
            if (robot.id != robot_id_) {
                double distance = calculateDistance(getCurrentPosition(), robot.position);
                if (distance < detection_radius_) {
                    nearby.push_back(robot);
                }
            }
        }
        
        return nearby;
    }

private:
    double calculateDistance(const geometry_msgs::msg::Point& p1,
                           const geometry_msgs::msg::Point& p2) {
        return std::sqrt(std::pow(p1.x - p2.x, 2) + 
                        std::pow(p1.y - p2.y, 2) + 
                        std::pow(p1.z - p2.z, 2));
    }
    
    geometry_msgs::msg::Point getCurrentPosition() {
        // Get current robot position from localization
        geometry_msgs::msg::Point pos;
        // Implementation would get actual position
        return pos;  // Placeholder
    }
    
    static constexpr double AVOIDANCE_GAIN = 0.5;
    static constexpr double MAX_AVOIDANCE_VEL = 0.5;  // m/s
};
```

## Troubleshooting Common Issues

### Communication Issues

#### Network Partitions
- **Symptoms**: Robots lose communication with parts of swarm
- **Causes**: Network outages, communication range limitations, interference
- **Solutions**: Mesh networking, relay robots, opportunistic communication
- **Prevention**: Redundant communication paths, range monitoring

#### Message Loss
- **Symptoms**: Inconsistent swarm behavior, task failures
- **Causes**: Network congestion, packet loss, interference
- **Solutions**: Reliable communication protocols, message acknowledgment, redundancy
- **Monitoring**: Track message delivery rates and latencies

### Coordination Issues

#### Deadlocks
- **Symptoms**: Robots stop moving, waiting for each other
- **Causes**: Resource contention, circular dependencies
- **Solutions**: Deadlock detection/prevention, resource prioritization
- **Prevention**: Careful task allocation, formation planning

#### Race Conditions
- **Symptoms**: Inconsistent behavior, timing-dependent failures
- **Causes**: Concurrent access to shared resources
- **Solutions**: Synchronization mechanisms, mutual exclusion
- **Prevention**: Proper locking, message ordering

### Performance Issues

#### Scalability Problems
- **Symptoms**: Performance degrades with more robots
- **Causes**: Centralized coordination, broadcast storms
- **Solutions**: Distributed algorithms, hierarchical coordination
- **Monitoring**: Track performance metrics with swarm size

#### Resource Contention
- **Symptoms**: Slow response, task conflicts
- **Causes**: Insufficient resources, poor task allocation
- **Solutions**: Resource reservation, load balancing
- **Prevention**: Capacity planning, resource monitoring

## Future Developments

### Emerging Technologies

#### Swarm Intelligence Algorithms
- **Bio-inspired Approaches**: More sophisticated biological algorithms
- **Learning-based Coordination**: AI algorithms for emergent coordination
- **Adaptive Formation Control**: Formations that adapt to tasks and environments
- **Collective Decision Making**: Sophisticated consensus algorithms

#### Advanced Communication
- **5G Integration**: Ultra-low latency, high-bandwidth communication
- **Edge Computing**: Distributed computation and coordination
- **Quantum Communication**: Secure communication for critical swarms
- **Swarm Mesh Networks**: Self-organizing communication networks

### Integration Trends

#### AI-Enhanced Swarms
- **Predictive Coordination**: Predicting and anticipating swarm behaviors
- **Adaptive Learning**: Swarms that learn to coordinate better over time
- **Human-Swarm Interaction**: Natural interaction with human operators
- **Cross-Domain Integration**: Integration with other AI systems

## Conclusion

Swarm robotics and multi-robot coordination represent a powerful paradigm for Physical AI systems, enabling complex behaviors to emerge from the interaction of simple agents. The key to successful swarm robotics lies in understanding the appropriate communication patterns, coordination algorithms, and the trade-offs between centralized and decentralized approaches.

The implementation of swarm systems requires careful attention to communication efficiency, safety protocols, and the specific requirements of the robotic application. Modern swarm robotics systems leverage advanced algorithms for task allocation, formation control, and collective decision making while maintaining the robustness and scalability that make swarm approaches attractive.

As robotics applications become more complex and diverse, swarm robotics will continue to provide solutions for tasks that benefit from distributed intelligence and parallel execution. The integration of machine learning and AI techniques with swarm algorithms promises to create more adaptive and intelligent collective systems.

Understanding these swarm robotics concepts is essential for developing Physical AI systems that can operate effectively in scenarios where multiple robots working together can achieve goals that would be impossible for a single robot.

## Exercises

1. Implement a simple swarm system with 3-5 robots that demonstrates flocking behavior using the algorithms described in this chapter.
2. Design and implement a task allocation system for a multi-robot team that assigns tasks based on robot capabilities and current workload.
3. Create a safety system for swarm robotics that prevents collisions between multiple robots while maintaining swarm objectives.

## Further Reading

- Brambilla, M., et al. (2013). "Swarm robotics: a review from the swarm engineering perspective."
- Chen, J., et al. (2012). "A comprehensive survey of the bee colony optimization approach."
- Parker, L. E. (2008). "Distributed intelligence: Overview of the field and its application in multi-robot systems."
- Dorigo, M., et al. (2014). "Swarm robotics: A review of the existing work and future directions."
- Rubenstein, M., et al. (2014). "Programmable self-assembly in a thousand-robot swarm."