---
sidebar_label: Navigation Stacks
title: Navigation Stacks - ROS 2 Navigation System Configuration
description: Understanding ROS 2 navigation stacks including Navigation2, path planning, and autonomous navigation systems
keywords: [navigation, Navigation2, path planning, autonomous navigation, ROS 2, robotics, localization, mapping]
---

# 5.2 Navigation Stacks

## Introduction

Navigation in robotics refers to the capability of a robot to move from one location to another autonomously while avoiding obstacles and respecting environmental constraints. ROS 2's Navigation2 stack provides a comprehensive framework for implementing navigation systems, building upon the mapping capabilities developed through SLAM to enable robots to operate effectively in known or unknown environments.

The Navigation2 stack represents a significant advancement over the previous ROS 1 navigation stack, offering improved performance, modularity, and extensibility. It provides a standardized interface for various navigation components including localization, path planning, path execution, and obstacle avoidance. Understanding navigation stacks is essential for creating robots that can operate autonomously in real-world environments.

Navigation systems in Physical AI systems must handle the complexities of the real world, including dynamic obstacles, uncertain localization, and changing environmental conditions. The Navigation2 stack provides the tools and frameworks to address these challenges while maintaining modularity and extensibility.

## Navigation2 Architecture

### Core Components

The Navigation2 stack consists of several core components that work together to provide complete navigation functionality:

#### Navigation Server
The navigation server is the central coordinator of the navigation system, managing the overall navigation pipeline and coordinating between different components.

**Responsibilities**:
- **Goal Management**: Receiving and managing navigation goals
- **Pipeline Coordination**: Coordinating between different navigation components
- **Lifecycle Management**: Managing the lifecycle of navigation components
- **State Reporting**: Reporting navigation state to other systems

#### Local Planner (Controller)
The local planner, also known as the controller, is responsible for generating low-level velocity commands to follow a given path while avoiding obstacles.

**Key Functions**:
- **Path Following**: Following the global path with local adjustments
- **Obstacle Avoidance**: Adjusting path in real-time to avoid obstacles
- **Velocity Generation**: Generating appropriate velocity commands
- **Dynamic Window**: Considering robot dynamics and constraints

#### Global Planner
The global planner computes the overall path from the robot's current location to the goal location.

**Key Functions**:
- **Path Planning**: Computing optimal or feasible paths
- **Map Utilization**: Using costmap information for planning
- **Path Optimization**: Optimizing paths for various criteria
- **Replanning**: Computing new paths when needed

#### Localizer
The localizer determines the robot's position within a known map.

**Key Functions**:
- **Pose Estimation**: Estimating robot's position and orientation
- **Map Matching**: Matching sensor data to known map
- **Particle Filtering**: Using particle filters for robust localization
- **Multi-sensor Fusion**: Combining information from multiple sensors

### Navigation2 Plugins Architecture

Navigation2 uses a plugin-based architecture that allows for interchangeable components:

#### Planners
- **Global Planners**: Dijkstra, A*, Theta*, NavFn, etc.
- **Local Planners**: DWA, TEB, MPC, etc.
- **Trajectory Generators**: Various approaches to generating feasible trajectories

#### Controllers
- **Pure Pursuit**: Simple path-following algorithm
- **MPC**: Model Predictive Control for optimal control
- **PID**: Proportional-Integral-Derivative controllers
- **Adaptive Controllers**: Controllers that adapt to changing conditions

#### Behavior Trees
Navigation2 uses behavior trees for managing complex navigation behaviors:

**Core Behaviors**:
- **NavigateToPose**: Navigate to a specific pose
- **NavigateThroughPoses**: Navigate through multiple poses
- **FollowPath**: Follow a predefined path
- **Spin**: Rotate in place to localize
- **Wait**: Wait for a specified duration
- **Backup**: Back up to escape from tight spaces

### Costmap Integration

Navigation2 relies heavily on costmaps for representing the environment:

#### Costmap 2D
Costmap 2D provides a 2D representation of obstacles and free space:

**Layers**:
- **Static Layer**: Static map information
- **Obstacle Layer**: Dynamic obstacles from sensors
- **Inflation Layer**: Safety margins around obstacles
- **Voxel Layer**: 3D obstacle information projected to 2D
- **Range Layer**: Range sensor information

**Configuration Parameters**:
```yaml
# Example costmap configuration
local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: true
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      transform_tolerance: 0.5
      footprint: "[ [0.14, 0.14], [0.14, -0.14], [-0.14, -0.14], [-0.14, 0.14] ]"
      
      plugins: ["obstacle_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
```

## Global Path Planning

### Path Planning Algorithms

#### A* Algorithm
A* is a popular path planning algorithm that uses heuristics to find optimal paths efficiently:

**Characteristics**:
- **Optimality**: Finds optimal path if heuristic is admissible
- **Completeness**: Guaranteed to find a path if one exists
- **Heuristic**: Uses distance heuristic to guide search
- **Memory Usage**: Stores all explored nodes

**Implementation Considerations**:
- **Grid Resolution**: Balance between path quality and computation time
- **Heuristic Design**: Proper heuristic affects performance significantly
- **Tie Breaking**: Handling nodes with equal f-values
- **Any-angle Paths**: Techniques to find paths that aren't restricted to grid directions

#### Dijkstra's Algorithm
Dijkstra's algorithm finds shortest paths without using heuristics:

**Characteristics**:
- **Optimality**: Guarantees optimal path
- **Completeness**: Finds path if one exists
- **No Heuristic**: Explores in all directions equally
- **Computation**: More computationally expensive than A*

#### D* Lite
D* Lite is designed for dynamic environments where obstacles can appear or disappear:

**Characteristics**:
- **Dynamic**: Handles changing environments
- **Incremental**: Updates path without full recomputation
- **Replanning**: Efficient replanning when environment changes
- **Optimality**: Maintains optimality in dynamic environments

#### Sampling-Based Planners
Sampling-based planners like RRT (Rapidly-exploring Random Trees) are useful for high-dimensional spaces:

**RRT**:
- **High-Dimensional**: Effective in high-dimensional configuration spaces
- **Anytime**: Can return path at any time during execution
- **Probabilistic Completeness**: Guaranteed to find solution if it exists
- **Suboptimal**: Solutions are not necessarily optimal

**RRT***:
- **Optimality**: Approaches optimal solution over time
- **Asymptotic**: Asymptotically optimal
- **Convergence**: Converges to optimal path with time
- **Complexity**: More complex implementation than RRT

### Global Planner Implementation Example

```cpp
// Example global planner plugin implementation
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

class AdvancedGlobalPlanner : public nav2_core::GlobalPlanner
{
public:
    void configure(
        rclcpp_lifecycle::LifecycleNode::SharedPtr node,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
    {
        node_ = node;
        name_ = name;
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();
        
        // Initialize planner-specific parameters
        node_->declare_parameter(name_ + ".planner_frequency", 1.0);
        node_->declare_parameter(name_ + ".tolerance", 0.5);
        
        planner_frequency_ = node_->get_parameter(name_ + ".planner_frequency").as_double();
        tolerance_ = node_->get_parameter(name_ + ".tolerance").as_double();
    }

    void cleanup() override
    {
        // Clean up resources
    }

    void activate() override
    {
        // Activate the planner
        RCLCPP_INFO(node_->get_logger(), "Activating %s", name_.c_str());
    }

    void deactivate() override
    {
        // Deactivate the planner
        RCLCPP_INFO(node_->get_logger(), "Deactivating %s", name_.c_str());
    }

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override
    {
        nav_msgs::msg::Path path;
        
        // Check if start and goal are in valid map locations
        if (!isValidPose(start, *costmap_) || !isValidPose(goal, *costmap_)) {
            RCLCPP_WARN(node_->get_logger(), "Start or goal pose is invalid");
            return path;  // Return empty path
        }
        
        // Convert start and goal to map coordinates
        unsigned int start_x, start_y, goal_x, goal_y;
        costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_x, start_y);
        costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y);
        
        // Plan path using selected algorithm
        std::vector<std::pair<unsigned int, unsigned int>> path_indices;
        
        if (use_astar_) {
            path_indices = planAStar(start_x, start_y, goal_x, goal_y);
        } else if (use_rrt_) {
            path_indices = planRRT(start_x, start_y, goal_x, goal_y);
        } else {
            // Default to A* if no algorithm specified
            path_indices = planAStar(start_x, start_y, goal_x, goal_y);
        }
        
        // Convert indices back to world coordinates
        path.header.frame_id = costmap_ros_->getGlobalFrameID();
        path.header.stamp = node_->now();
        
        for (const auto& idx : path_indices) {
            double world_x, world_y;
            costmap_->mapToWorld(idx.first, idx.second, world_x, world_y);
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = world_x;
            pose.pose.position.y = world_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;  // Default orientation
            
            path.poses.push_back(pose);
        }
        
        // Smooth the path using cubic splines or other techniques
        if (smooth_path_) {
            path = smoothPath(path);
        }
        
        return path;
    }

private:
    std::vector<std::pair<unsigned int, unsigned int>> planAStar(
        unsigned int start_x, unsigned int start_y,
        unsigned int goal_x, unsigned int goal_y)
    {
        // Implementation of A* algorithm
        // This is a simplified version - real implementation would be more complex
        std::vector<std::pair<unsigned int, unsigned int>> path;
        
        // A* algorithm implementation would go here
        // - Initialize open and closed sets
        // - Add start node to open set
        // - While open set not empty:
        //   - Pick node with lowest f-score
        //   - If goal reached, reconstruct path
        //   - Move current node to closed set
        //   - Examine neighbors and update scores
        // - Return reconstructed path
        
        return path;  // Placeholder
    }
    
    std::vector<std::pair<unsigned int, unsigned int>> planRRT(
        unsigned int start_x, unsigned int start_y,
        unsigned int goal_x, unsigned int goal_y)
    {
        // Implementation of RRT algorithm
        std::vector<std::pair<unsigned int, unsigned int>> path;
        
        // RRT algorithm implementation would go here
        // - Initialize tree with start position
        // - While goal not reached:
        //   - Sample random point
        //   - Find nearest node in tree
        //   - Extend tree toward sampled point
        //   - Check for collisions
        // - Reconstruct path from start to goal
        // - Return path
        
        return path;  // Placeholder
    }
    
    nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& path)
    {
        // Implement path smoothing using cubic splines or other methods
        nav_msgs::msg::Path smoothed_path = path;
        
        // Path smoothing algorithm implementation
        // Could use cubic splines, Dubins curves, or other methods
        // Ensure smoothed path respects robot kinematics and constraints
        
        return smoothed_path;
    }
    
    bool isValidPose(
        const geometry_msgs::msg::PoseStamped& pose,
        const nav2_costmap_2d::Costmap2D& costmap)
    {
        unsigned int mx, my;
        if (!costmap.worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
            return false;  // Pose is outside map bounds
        }
        
        unsigned char cost = costmap.getCost(mx, my);
        return cost != nav2_costmap_2d::LETHAL_OBSTACLE && 
               cost != nav2_costmap_2d::UNKNOWN && 
               cost != nav2_costmap_2d::NO_INFORMATION;
    }
    
    // Member variables
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string name_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_costmap_2d::Costmap2D* costmap_;
    
    // Parameters
    double planner_frequency_;
    double tolerance_;
    bool use_astar_ = true;
    bool use_rrt_ = false;
    bool smooth_path_ = true;
};
```

## Local Path Planning and Control

### Local Planner Algorithms

#### Dynamic Window Approach (DWA)
DWA is a local planner that considers robot dynamics and generates trajectories in real-time:

**Principles**:
- **Dynamic Window**: Feasible velocity space considering robot dynamics
- **Trajectory Evaluation**: Evaluate trajectories based on goals and obstacles
- **Real-time**: Generates trajectories in real-time for obstacle avoidance
- **Kinematic Constraints**: Respects robot kinematic constraints

**Advantages**:
- Good obstacle avoidance
- Considers robot dynamics
- Real-time capable
- Simple implementation

**Disadvantages**:
- Short-sighted planning horizon
- May get stuck in local minima
- Requires careful parameter tuning
- Limited path optimality

#### Timed Elastic Band (TEB)
TEB is a trajectory optimization approach that creates elastic bands of poses:

**Characteristics**:
- **Trajectory Optimization**: Optimizes entire trajectory
- **Elastic Band**: Creates flexible trajectory that adapts to environment
- **Kinodynamic Constraints**: Considers both kinematic and dynamic constraints
- **Multi-objective**: Optimizes for multiple criteria simultaneously

**Optimization Criteria**:
- **Path Following**: Stay close to global path
- **Obstacle Avoidance**: Avoid collisions with obstacles
- **Kinematic Constraints**: Respect robot kinematics
- **Dynamic Constraints**: Respect robot dynamics
- **Smoothness**: Minimize jerk and curvature

#### Model Predictive Control (MPC)
MPC solves an optimization problem at each time step to determine optimal control actions:

**Features**:
- **Predictive**: Uses model to predict future behavior
- **Optimization**: Solves optimization problem at each step
- **Constraints**: Explicitly handles constraints
- **Adaptive**: Adjusts to changing conditions

**Implementation**:
- **Prediction Horizon**: Time window for prediction
- **Control Horizon**: Time window for control optimization
- **Cost Function**: Defines optimization objective
- **Constraints**: Kinematic, dynamic, and environmental constraints

### Local Planner Configuration

```yaml
# Example local planner configuration
local_planner:
  TrajectoryPlannerROS:
    ros__parameters:
      # Robot physical properties
      max_vel_x: 0.5
      min_vel_x: 0.1
      max_vel_theta: 1.0
      min_vel_theta: -1.0
      min_in_place_vel_theta: 0.4
      escape_vel: -0.1
      
      # Acceleration limits
      acc_lim_x: 2.5
      acc_lim_y: 0.0  # Differential drive robot
      acc_lim_theta: 3.2
      
      # Goal tolerance
      xy_goal_tolerance: 0.1
      yaw_goal_tolerance: 0.1
      
      # Path following parameters
      holonomic_robot: false
      meter_scoring: true
      
      # Obstacle avoidance
      occdist_scale: 0.01
      pdist_scale: 0.6
      gdist_scale: 0.8
      heading_lookahead: 0.325
      heading_scoring: false
      heading_scoring_timestep: 0.8
      dwa: true
      
      # Simulation parameters
      sim_time: 1.7
      sim_granularity: 0.025
      vx_samples: 3
      vy_samples: 10
      vtheta_samples: 20
      controller_frequency: 20.0
```

### Path Following Algorithms

#### Pure Pursuit
Pure pursuit is a simple path following algorithm that works well for differential drive robots:

**Principles**:
- **Lookahead Point**: Follows a point at a fixed distance ahead on the path
- **Steering Control**: Adjusts steering angle to follow the path
- **Simple**: Easy to understand and implement
- **Effective**: Works well for smooth paths

#### Follow-the-Carrot
Similar to pure pursuit but with a fixed lookahead distance:

**Characteristics**:
- **Fixed Lookahead**: Constant distance to lookahead point
- **Simple Implementation**: Very straightforward
- **Stability**: Generally stable but may oscillate
- **Limited Performance**: Not optimal for all path types

#### Stanley Controller
Stanley controller combines cross-track error correction with heading error correction:

**Approach**:
- **Cross-track Error**: Corrects for deviation from path
- **Heading Error**: Corrects for heading difference
- **Combined Control**: Uses both errors for control
- **Stability**: Good stability characteristics

## Localization Integration

### AMCL (Adaptive Monte Carlo Localization)

AMCL is the standard localization approach in ROS 2, using particle filters for robust pose estimation:

#### Core Concepts
- **Particle Filter**: Represents belief as a set of particles
- **Motion Model**: Predicts particle movement based on odometry
- **Sensor Model**: Updates particle weights based on sensor data
- **Resampling**: Maintains particle diversity

#### Configuration Parameters
```yaml
# AMCL configuration
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "differential"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.1
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan
```

### Localization Quality Assessment

#### Metrics
- **Position Error**: Difference between estimated and true position
- **Orientation Error**: Difference between estimated and true orientation
- **Consistency**: How well the estimated uncertainty matches actual error
- **Robustness**: Ability to recover from localization failures

#### Improvement Strategies
- **Multi-sensor Fusion**: Combine multiple sensors for better localization
- **Map Quality**: Use high-quality, detailed maps
- **Parameter Tuning**: Optimize localization parameters
- **Sensor Placement**: Optimize sensor placement for maximum information

## Navigation Behavior Trees

### Behavior Tree Concepts

Navigation2 uses behavior trees to manage complex navigation behaviors:

#### Core Concepts
- **Nodes**: Actions, conditions, decorators, and control nodes
- **Blackboards**: Shared memory for information exchange
- **Actions**: Atomic operations (move, rotate, wait)
- **Conditions**: State checks (obstacle detected, goal reached)

#### Navigation Behavior Tree Example
```xml
<!-- Example behavior tree for navigation -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <Sequence name="NavigateToPose">
      <Fallback name="NavigateWithRecovery">
        <PipelineSequence name="Navigate">
          <RecoveryNode name="ComputePathToPose" number_of_retries="1">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
            <ReinitializeGlobalLocalization/>
          </RecoveryNode>
          <RecoveryNode name="SmoothPath" number_of_retries="1">
            <SmoothPath input_path="{path}" output_path="{smoothed_path}" smoother_id="SimpleSmoother"/>
          </RecoveryNode>
          <RecoveryNode name="FollowPath" number_of_retries="1">
            <FollowPath path="{smoothed_path}" controller_id="FollowPath"/>
          </RecoveryNode>
        </PipelineSequence>
        <ReactiveSequence name="Recovery">
          <RecoveryNode name="Spin" number_of_retries="1">
            <Spin spin_dist="1.57"/>
          </RecoveryNode>
          <RecoveryNode name="Backup" number_of_retries="1">
            <Backup backup_dist="0.15" backup_speed="0.025"/>
          </RecoveryNode>
        </ReactiveSequence>
      </Fallback>
    </Sequence>
  </BehaviorTree>
</root>
```

### Custom Behavior Tree Nodes

Creating custom behavior tree nodes for specific navigation behaviors:

```cpp
// Example custom behavior tree node
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class NavigateToClosestPoint : public BT::ActionNodeBase
{
public:
    NavigateToClosestPoint(const std::string& name, const BT::NodeConfiguration& config)
        : BT::ActionNodeBase(name, config), node_(rclcpp::Node::make_shared("navigate_to_closest_point"))
    {
        // Initialize navigation client
        navigator_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            node_, "navigate_to_pose");
    }

    BT::NodeStatus tick() override
    {
        // Get input parameters
        geometry_msgs::msg::PoseStamped goal;
        if (!getInput<geometry_msgs::msg::PoseStamped>("closest_point", goal)) {
            RCLCPP_ERROR(node_->get_logger(), "Could not get closest_point input");
            return BT::NodeStatus::FAILURE;
        }

        // Send navigation goal
        auto goal_handle = sendNavigationGoal(goal);
        
        // Wait for result with timeout
        auto result = waitForResult(goal_handle, std::chrono::seconds(30));
        
        if (result == nav2_msgs::action::NavigateToPose::Result::SUCCEEDED) {
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<geometry_msgs::msg::PoseStamped>("closest_point", "Target pose to navigate to")
        };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigator_client_;

    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr 
    sendNavigationGoal(const geometry_msgs::msg::PoseStamped& goal)
    {
        // Implementation for sending navigation goal
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = goal;
        
        auto options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        options.result_callback = [this](const auto& result) {
            // Handle result
        };
        
        return navigator_client_->async_send_goal(goal_msg, options);
    }

    typename nav2_msgs::action::NavigateToPose::Result::SharedPtr 
    waitForResult(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle,
                  std::chrono::seconds timeout)
    {
        // Implementation for waiting for navigation result
        // This would use node_->wait_for_action_server() and other async operations
        return nullptr;  // Placeholder
    }
};
```

## Advanced Navigation Features

### Multi-floor Navigation

#### Map Management
- **Map Switching**: Switching between different floor maps
- **Elevator/Door Handling**: Managing transitions between floors
- **Localization Across Floors**: Maintaining localization during transitions
- **Path Planning Across Floors**: Planning paths that span multiple floors

#### Implementation Strategies
```cpp
// Multi-floor navigation example
class MultiFloorNavigator {
private:
    std::map<std::string, std::shared_ptr<nav2_costmap_2d::Costmap2DROS>> costmaps_;
    std::map<std::string, std::shared_ptr<AdvancedGlobalPlanner>> planners_;
    std::string current_floor_;
    
public:
    bool navigateBetweenFloors(const geometry_msgs::msg::PoseStamped& start,
                              const geometry_msgs::msg::PoseStamped& goal) {
        // Determine start and goal floors
        std::string start_floor = determineFloor(start.pose.position.z);
        std::string goal_floor = determineFloor(goal.pose.position.z);
        
        if (start_floor == goal_floor) {
            // Same floor - normal navigation
            return navigateWithinFloor(start, goal);
        } else {
            // Different floors - need to handle elevator/stairs
            return navigateBetweenFloors(start, goal, start_floor, goal_floor);
        }
    }
    
    bool navigateBetweenFloors(const geometry_msgs::msg::PoseStamped& start,
                              const geometry_msgs::msg::PoseStamped& goal,
                              const std::string& start_floor,
                              const std::string& goal_floor) {
        // Plan path to elevator/stairs on start floor
        auto elev_path = planToElevator(start, start_floor);
        if (elev_path.poses.empty()) {
            return false;
        }
        
        // Execute path to elevator
        if (!followPath(elev_path)) {
            return false;
        }
        
        // Wait for elevator/stairs
        if (!waitForElevator(goal_floor)) {
            return false;
        }
        
        // Update current floor
        current_floor_ = goal_floor;
        
        // Plan path from elevator to goal
        auto final_path = planFromElevator(goal, goal_floor);
        if (final_path.poses.empty()) {
            return false;
        }
        
        // Execute path to goal
        return followPath(final_path);
    }
    
private:
    std::string determineFloor(double z_position) {
        // Implementation to determine floor based on z-position
        // This could use a floor plan or elevation mapping
        return "floor_1";  // Placeholder
    }
    
    nav_msgs::msg::Path planToElevator(const geometry_msgs::msg::PoseStamped& start,
                                      const std::string& floor) {
        // Plan path from current position to elevator position on specified floor
        // Implementation would use the appropriate costmap and planner
        return nav_msgs::msg::Path();  // Placeholder
    }
    
    bool waitForElevator(const std::string& destination_floor) {
        // Wait for elevator to arrive and transport robot to destination floor
        // This might involve calling elevator, waiting, and detecting floor arrival
        return true;  // Placeholder
    }
    
    nav_msgs::msg::Path planFromElevator(const geometry_msgs::msg::PoseStamped& goal,
                                        const std::string& floor) {
        // Plan path from elevator position to goal on specified floor
        return nav_msgs::msg::Path();  // Placeholder
    }
};
```

### Dynamic Obstacle Avoidance

#### Real-time Obstacle Detection
- **Sensor Integration**: Combining data from multiple sensors
- **Prediction**: Predicting obstacle trajectories
- **Replanning**: Adjusting path based on dynamic obstacles
- **Recovery**: Handling situations where path becomes blocked

#### Collision Prediction and Avoidance
```cpp
// Dynamic obstacle avoidance implementation
class DynamicObstacleAvoider {
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    
    std::vector<ObstaclePrediction> predicted_obstacles_;
    double safety_margin_;
    double prediction_horizon_;
    
public:
    void processScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        // Detect and track obstacles
        auto obstacles = detectObstacles(scan);
        
        // Predict future positions
        predicted_obstacles_ = predictObstacleTrajectories(obstacles);
        
        // Update costmap with predicted obstacle positions
        updateCostmapWithPredictions();
    }
    
    bool isPathClear(const nav_msgs::msg::Path& path, double time_horizon) {
        for (double t = 0; t <= time_horizon; t += 0.1) {
            auto robot_pos = interpolatePath(path, t);
            
            for (const auto& pred_obs : predicted_obstacles_) {
                auto obs_pos = predictObstaclePosition(pred_obs, t);
                
                double dist = distance(robot_pos, obs_pos);
                if (dist < safety_margin_) {
                    return false;  // Path is not clear
                }
            }
        }
        
        return true;
    }
    
    geometry_msgs::msg::Twist generateAvoidanceCommand(
        const geometry_msgs::msg::PoseStamped& robot_pose,
        const nav_msgs::msg::Path& current_path) {
        
        geometry_msgs::msg::Twist cmd;
        
        // Check if current path is safe
        if (!isPathClear(current_path, prediction_horizon_)) {
            // Generate avoidance maneuver
            cmd = calculateAvoidanceManeuver(robot_pose, current_path);
        } else {
            // Follow current path normally
            cmd = followPath(robot_pose, current_path);
        }
        
        return cmd;
    }
    
private:
    struct ObstaclePrediction {
        geometry_msgs::msg::Point initial_position;
        geometry_msgs::msg::Vector3 velocity;
        ros::Time timestamp;
        double uncertainty;
    };
    
    std::vector<ObstaclePrediction> predictObstacleTrajectories(
        const std::vector<geometry_msgs::msg::Point>& obstacles) {
        
        std::vector<ObstaclePrediction> predictions;
        
        for (const auto& obs : obstacles) {
            ObstaclePrediction pred;
            pred.initial_position = obs;
            pred.velocity = estimateVelocity(obs);  // Implementation specific
            pred.timestamp = node_->now();
            pred.uncertainty = estimateUncertainty(obs);  // Implementation specific
            predictions.push_back(pred);
        }
        
        return predictions;
    }
    
    geometry_msgs::msg::Point predictObstaclePosition(
        const ObstaclePrediction& pred, double time_offset) {
        
        geometry_msgs::msg::Point future_pos;
        future_pos.x = pred.initial_position.x + pred.velocity.x * time_offset;
        future_pos.y = pred.initial_position.y + pred.velocity.y * time_offset;
        future_pos.z = pred.initial_position.z + pred.velocity.z * time_offset;
        
        return future_pos;
    }
};
```

## Navigation in Physical AI Systems

### Integration with Perception Systems

Navigation systems must integrate closely with perception systems:

#### Sensor Fusion for Navigation
- **Multi-modal Sensing**: Combining LiDAR, cameras, sonar
- **Uncertainty Management**: Handling uncertain sensor data
- **Dynamic Environment**: Adapting to changing environments
- **Robustness**: Maintaining navigation capability despite sensor failures

#### Map Updates
- **Real-time Updates**: Updating maps based on current sensor data
- **Change Detection**: Detecting changes in the environment
- **Map Consistency**: Maintaining consistency in dynamic maps
- **Memory Management**: Managing memory for large, dynamic maps

### Safety Considerations

#### Safety Architecture
- **Emergency Stop**: Immediate stop capability
- **Safe Velocities**: Velocity limits based on environment
- **Collision Avoidance**: Proactive collision prevention
- **Human Safety**: Special considerations for human environments

#### Safety Implementation
```cpp
// Safety implementation for navigation
class SafetyNavigator {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safety_cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr safety_scan_sub_;
    double safety_distance_;
    bool emergency_stop_active_;
    
public:
    void safetyCheck(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        // Check for obstacles within safety distance
        bool too_close = false;
        for (auto range : scan->ranges) {
            if (range < safety_distance_ && range > scan->range_min) {
                too_close = true;
                break;
            }
        }
        
        if (too_close) {
            // Emergency stop
            geometry_msgs::msg::Twist stop_cmd;
            stop_cmd.linear.x = 0.0;
            stop_cmd.angular.z = 0.0;
            safety_cmd_pub_->publish(stop_cmd);
            emergency_stop_active_ = true;
        } else if (emergency_stop_active_) {
            // Resume navigation
            emergency_stop_active_ = false;
        }
    }
    
    bool checkNavigationSafety(const geometry_msgs::msg::PoseStamped& goal) {
        // Check if navigation goal is in a safe area
        // This might check against safety zones in the map
        auto costmap = getCostmap();
        unsigned int mx, my;
        
        if (costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) {
            unsigned char cost = costmap->getCost(mx, my);
            return cost < nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;  // Not in safety zone
        }
        
        return false;  // Goal is outside map bounds
    }
};
```

### Performance Optimization

#### Computational Efficiency
- **Algorithm Selection**: Choosing appropriate algorithms for the task
- **Parameter Tuning**: Optimizing parameters for performance
- **Multi-threading**: Using parallel processing where possible
- **Hardware Acceleration**: Using GPUs or specialized hardware

#### Memory Management
- **Map Compression**: Compressing map data
- **Resolution Management**: Using appropriate resolution
- **Cache Management**: Caching frequently accessed data
- **Dynamic Allocation**: Managing memory allocation patterns

## Configuration and Tuning

### Parameter Optimization

#### Key Parameters
- **Velocity Limits**: Maximum linear and angular velocities
- **Acceleration Limits**: Maximum linear and angular accelerations
- **Goal Tolerance**: Distance and angle tolerances for goal achievement
- **Costmap Parameters**: Resolution, inflation radius, update rates

#### Tuning Process
1. **Start Conservative**: Begin with conservative parameters
2. **Incremental Adjustment**: Make small adjustments iteratively
3. **Test Thoroughly**: Test in various environments
4. **Document Changes**: Keep records of parameter changes

### Performance Evaluation

#### Metrics
- **Success Rate**: Percentage of successful navigation attempts
- **Path Efficiency**: Ratio of actual path length to optimal path
- **Execution Time**: Time to reach goal
- **Safety Incidents**: Number of safety-related events

#### Evaluation Tools
- **Navigation Metrics**: Built-in navigation evaluation tools
- **Simulation Testing**: Testing in simulated environments
- **Real-world Testing**: Testing in actual deployment environments
- **Statistical Analysis**: Analyzing performance over time

## Troubleshooting Common Issues

### Navigation Problems

#### Localization Issues
- **Symptoms**: Robot doesn't know where it is in the map
- **Causes**: Poor initial pose, incorrect map, sensor issues
- **Solutions**: Better initial localization, map updates, sensor calibration

#### Path Planning Issues
- **Symptoms**: Robot can't find path to goal
- **Causes**: Obstacles in the way, incorrect costmap, narrow passages
- **Solutions**: Costmap parameter tuning, path planning algorithm selection, environment modification

#### Path Execution Issues
- **Symptoms**: Robot doesn't follow path correctly
- **Causes**: Control parameters, wheel slippage, dynamic obstacles
- **Solutions**: Control parameter tuning, better odometry, obstacle avoidance

#### Recovery Issues
- **Symptoms**: Robot gets stuck and doesn't recover
- **Causes**: Inadequate recovery behaviors, sensor limitations
- **Solutions**: Recovery behavior tuning, additional sensors, manual intervention

### Sensor-Specific Issues

#### LiDAR Issues
- **Dynamic Objects**: LiDAR detects moving objects as permanent obstacles
- **Reflection Issues**: Shiny surfaces causing incorrect distance readings
- **Range Limitations**: Limited detection range affecting navigation
- **Solutions**: Proper costmap layer configuration, filtering, multiple sensors

#### Camera Issues
- **Lighting Changes**: Navigation performance affected by lighting
- **Feature Poverty**: Lack of visual features for visual SLAM
- **Motion Blur**: Fast motion causing blurry images
- **Solutions**: Robust visual algorithms, multiple sensor fusion, lighting considerations

## Future Developments

### Emerging Navigation Technologies

#### Learning-Based Navigation
- **Deep Reinforcement Learning**: Learning navigation policies
- **Imitation Learning**: Learning from expert demonstrations
- **Neural Path Planning**: Using neural networks for path planning
- **Adaptive Behavior**: Learning from experience

#### Multi-Agent Navigation
- **Coordination**: Coordinating multiple robots
- **Communication**: Sharing information between agents
- **Collision Avoidance**: Avoiding collisions between robots
- **Task Allocation**: Distributing navigation tasks

#### Semantic Navigation
- **Semantic Maps**: Maps with object and room labels
- **Goal Specification**: Natural language goal specification
- **Context Awareness**: Using semantic context for navigation
- **Human-Robot Interaction**: Natural navigation interaction

## Conclusion

Navigation systems are a critical component of Physical AI systems, enabling robots to operate autonomously in complex environments. The Navigation2 stack in ROS 2 provides a robust, modular framework for implementing navigation systems with various path planning, control, and localization approaches.

The success of navigation systems depends on proper integration with perception systems, careful parameter tuning, and consideration of environmental factors. Modern navigation systems must handle dynamic environments, ensure safety, and operate efficiently with limited computational resources.

As robotics applications become more complex and diverse, navigation systems must adapt to new challenges including multi-floor navigation, dynamic obstacle avoidance, and integration with higher-level AI systems. The modular architecture of Navigation2 allows for the integration of new algorithms and approaches as they develop.

Understanding navigation systems is essential for creating Physical AI systems that can operate effectively in real-world environments, bridging the gap between planning and physical action.

## Exercises

1. Configure and tune the Navigation2 stack for a specific robot platform, analyzing the impact of different parameters on navigation performance.
2. Implement a custom local planner plugin that incorporates specific robot constraints and capabilities.
3. Design a multi-floor navigation system that manages transitions between different floor maps.

## Further Reading

- Navigation2 Documentation: "Navigation2 Configuration Guide"
- Fox, D., Burgard, W., & Thrun, S. (1997). "The Dynamic Window Approach to Collision Avoidance."
- LaValle, S. M. (2006). "Planning Algorithms." Cambridge University Press.
- Siciliano, B., & Khatib, O. (Eds.). (2016). "Springer Handbook of Robotics."