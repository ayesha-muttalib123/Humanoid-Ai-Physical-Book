---
sidebar_label: Trajectory Generation and Execution
title: Trajectory Generation and Execution - Planning and Executing Robot Motion
description: Understanding trajectory generation and execution for robotics applications including path planning, motion control, and smooth motion execution
keywords: [trajectory, path planning, motion control, robotics, control, interpolation, spline, trajectory execution, planning, motion]
---

# 10.2 Trajectory Generation and Execution

## Introduction

Trajectory generation and execution form the critical bridge between high-level planning and low-level control in Physical AI systems. A trajectory represents the time-parameterized path that specifies position, velocity, and acceleration at each point in time, enabling robots to execute smooth, controlled movements that respect kinematic and dynamic constraints. In robotics applications, the quality of trajectory generation significantly impacts the robot's performance, safety, and ability to interact effectively with its environment.

The trajectory generation process involves converting abstract goals from navigation or manipulation planners into specific motion commands that can be executed by the robot's control system. This requires understanding both the robot's physical constraints and the environmental requirements. The trajectory execution component ensures that the planned trajectory is followed accurately while handling real-world disturbances and constraints.

This chapter explores the mathematical foundations of trajectory generation, various generation techniques, execution strategies, and integration with control systems. We'll cover both position-based trajectories for manipulators and velocity-based trajectories for mobile robots, with special attention to ensuring safety and smooth execution.

## Mathematical Foundations of Trajectories

### Trajectory Representation

A trajectory is typically represented as a function of time that defines the robot's state over time:

τ(t) = [x(t), y(t), z(t), θ(t), ẋ(t), ẏ(t), ż(t), θ̇(t), ẍ(t), ÿ(t), z̈(t), θ̈(t)]

Where the state includes position, velocity, and acceleration for each degree of freedom.

#### Parametric Representations
- **Time-based**: Position as a function of time (most common)
- **Path-based**: Position as a function of path parameter (s)
- **Phase-based**: Position as a function of phase in the motion cycle

#### Constraint Types
- **Kinematic**: Position, velocity, acceleration limits
- **Dynamic**: Force, torque, power constraints
- **Environmental**: Obstacle avoidance, safety zones
- **Task-specific**: Specific requirements for manipulation tasks

### Polynomial Trajectories

#### Cubic Polynomials

Cubic polynomials provide position and velocity continuity (C¹) and are commonly used for single-joint trajectories:

x(t) = a₀ + a₁t + a₂t² + a₃t³

For a trajectory between two points with specified initial and final conditions:
- x(t₀) = x₀, x'(t₀) = v₀
- x(t₁) = x₁, x'(t₁) = v₁

```cpp
class CubicTrajectorySegment {
private:
    double a0_, a1_, a2_, a3_;  // Polynomial coefficients
    double t0_, t1_;            // Start and end times
    double duration_;            // Total duration (t1 - t0)
    double x0_, x1_;            // Start and end positions
    double v0_, v1_;            // Start and end velocities

public:
    CubicTrajectorySegment(double start_time, double end_time,
                          double start_pos, double end_pos,
                          double start_vel, double end_vel) 
        : t0_(start_time), t1_(end_time), duration_(end_time - start_time),
          x0_(start_pos), x1_(end_pos), v0_(start_vel), v1_(end_vel) {
        
        // Solve for polynomial coefficients
        double dt = duration_;
        double dt_sq = dt * dt;
        double dt_cu = dt * dt_sq;
        
        a0_ = x0_;
        a1_ = v0_;
        a2_ = (3.0 * (x1_ - x0_) - 2.0 * v0_ * dt - v1_ * dt) / dt_sq;
        a3_ = (2.0 * (x0_ - x1_) + (v0_ + v1_) * dt) / dt_cu;
    }
    
    double getPosition(double t) const {
        if (t < t0_ || t > t1_) return 0.0;  // Beyond segment
        
        double tau = t - t0_;
        return a0_ + a1_*tau + a2_*tau*tau + a3_*tau*tau*tau;
    }
    
    double getVelocity(double t) const {
        if (t < t0_ || t > t1_) return 0.0;  // Beyond segment
        
        double tau = t - t0_;
        return a1_ + 2*a2_*tau + 3*a3_*tau*tau;
    }
    
    double getAcceleration(double t) const {
        if (t < t0_ || t > t1_) return 0.0;  // Beyond segment
        
        double tau = t - t0_;
        return 2*a2_ + 6*a3_*tau;
    }
    
    bool isFinished(double t) const {
        return t >= t1_;
    }
    
    double getDuration() const {
        return duration_;
    }
    
    double getEndTime() const {
        return t1_;
    }
};
```

#### Quintic Polynomials

Quintic polynomials provide position, velocity, and acceleration continuity (C²), making them suitable for applications requiring smooth acceleration profiles:

x(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵

```cpp
class QuinticTrajectorySegment {
private:
    double a0_, a1_, a2_, a3_, a4_, a5_;
    double t0_, t1_;
    double duration_;
    double x0_, x1_;  // Start and end positions
    double v0_, v1_;  // Start and end velocities
    double a0_, a1_;  // Start and end accelerations

public:
    QuinticTrajectorySegment(double start_time, double end_time,
                           double start_pos, double end_pos,
                           double start_vel, double end_vel,
                           double start_acc, double end_acc) 
        : t0_(start_time), t1_(end_time), duration_(end_time - start_time),
          x0_(start_pos), x1_(end_pos), v0_(start_vel), v1_(end_vel),
          a0_(start_acc), a1_(end_acc) {
        
        double x0 = start_pos;
        double x1 = end_pos;
        double v0 = start_vel;
        double v1 = end_vel;
        double a0 = start_acc;
        double a1 = end_acc;
        double T = duration_;
        
        // Solve quintic polynomial coefficients
        a0_ = x0;
        a1_ = v0;
        a2_ = a0 / 2.0;
        a3_ = (20*x1 - 20*x0 - (8*v1 + 12*v0)*T - (3*a0 - a1)*T*T) / (2*T*T*T);
        a4_ = (30*x0 - 30*x1 + (14*v1 + 16*v0)*T + (3*a0 - 2*a1)*T*T) / (2*T*T*T*T);
        a5_ = (12*x1 - 12*x0 - (6*v1 + 6*v0)*T - (a0 - a1)*T*T) / (2*T*T*T*T*T);
    }
    
    double getPosition(double t) const {
        if (t < t0_ || t > t1_) return 0.0;
        
        double tau = t - t0_;
        double tau_sq = tau * tau;
        double tau_cu = tau_sq * tau;
        double tau_4 = tau_cu * tau;
        double tau_5 = tau_4 * tau;
        
        return a0_ + a1_*tau + a2_*tau_sq + a3_*tau_cu + a4_*tau_4 + a5_*tau_5;
    }
    
    double getVelocity(double t) const {
        if (t < t0_ || t > t1_) return 0.0;
        
        double tau = t - t0_;
        double tau_sq = tau * tau;
        double tau_cu = tau_sq * tau;
        double tau_4 = tau_cu * tau;
        
        return a1_ + 2*a2_*tau + 3*a3_*tau_sq + 4*a4_*tau_cu + 5*a5_*tau_4;
    }
    
    double getAcceleration(double t) const {
        if (t < t0_ || t > t1_) return 0.0;
        
        double tau = t - t0_;
        double tau_sq = tau * tau;
        double tau_cu = tau_sq * tau;
        
        return 2*a2_ + 6*a3_*tau + 12*a4_*tau_sq + 20*a5_*tau_cu;
    }
    
    // Check if trajectory is kinematically feasible
    bool isKinematicallyFeasible(double max_vel, double max_acc, double max_jerk) const {
        // For quintic trajectories, we need to check the maximums analytically
        // or numerically over the time interval
        const int samples = 100;
        double dt = duration_ / samples;
        
        for (int i = 0; i <= samples; i++) {
            double time = t0_ + i * dt;
            double vel = getVelocity(time);
            double acc = getAcceleration(time);
            
            if (std::abs(vel) > max_vel || std::abs(acc) > max_acc) {
                return false;
            }
        }
        
        return true;
    }
};
```

### Spline-Based Trajectories

#### Cubic Splines

Cubic splines provide smooth interpolation through a sequence of waypoints:

```cpp
class CubicSplineTrajectory {
private:
    std::vector<CubicTrajectorySegment> segments_;
    std::vector<double> segment_times_;  // Cumulative time for each segment
    std::vector<double> waypoints_;
    std::vector<double> velocities_;
    
    double total_duration_;
    
public:
    CubicSplineTrajectory(const std::vector<double>& waypoints,
                         const std::vector<double>& times,
                         const std::vector<double>& velocities = {}) {
        
        if (waypoints.size() != times.size()) {
            throw std::invalid_argument("Waypoints and times must have same size");
        }
        
        waypoints_ = waypoints;
        segment_times_.resize(waypoints.size());
        
        for (size_t i = 0; i < waypoints.size() - 1; i++) {
            double start_pos = waypoints[i];
            double end_pos = waypoints[i + 1];
            double start_time = times[i];
            double end_time = times[i + 1];
            
            // If velocities not provided, use zero velocity at waypoints
            double start_vel = (i < velocities.size()) ? velocities[i] : 0.0;
            double end_vel = (i + 1 < velocities.size()) ? velocities[i + 1] : 0.0;
            
            segments_.emplace_back(start_time, end_time, start_pos, end_pos, start_vel, end_vel);
            segment_times_[i + 1] = (i == 0) ? end_time : segment_times_[i] + (end_time - start_time);
        }
        
        total_duration_ = segment_times_.empty() ? 0.0 : segment_times_.back();
    }
    
    double getPosition(double t) const {
        if (t < 0 || t > total_duration_) {
            return 0.0;
        }
        
        // Find which segment we're in
        size_t seg_idx = findSegmentIndex(t);
        if (seg_idx < segments_.size()) {
            return segments_[seg_idx].getPosition(t);
        }
        return 0.0;  // Default if not found
    }
    
    double getVelocity(double t) const {
        if (t < 0 || t > total_duration_) {
            return 0.0;
        }
        
        size_t seg_idx = findSegmentIndex(t);
        if (seg_idx < segments_.size()) {
            return segments_[seg_idx].getVelocity(t);
        }
        return 0.0;  // Default if not found
    }
    
    double getAcceleration(double t) const {
        if (t < 0 || t > total_duration_) {
            return 0.0;
        }
        
        size_t seg_idx = findSegmentIndex(t);
        if (seg_idx < segments_.size()) {
            return segments_[seg_idx].getAcceleration(t);
        }
        return 0.0;  // Default if not found
    }

private:
    size_t findSegmentIndex(double t) const {
        // Binary search to find segment
        auto it = std::upper_bound(segment_times_.begin(), segment_times_.end(), t);
        if (it == segment_times_.begin()) {
            return 0;
        }
        return std::distance(segment_times_.begin(), it) - 1;
    }
    
    // Method to compute velocities for spline using cubic spline interpolation
    std::vector<double> computeVelocitiesForSpline() {
        // Implement cubic spline velocity calculation using tridiagonal matrix algorithm
        // This would compute the appropriate velocities at each waypoint to ensure smoothness
        std::vector<double> computed_velocities(waypoints_.size(), 0.0);
        
        // For a natural cubic spline, we set the second derivative to zero at endpoints
        // Then solve the tridiagonal system to find velocities
        // Implementation would go here
        
        return computed_velocities;  // Placeholder
    }
};
```

#### Bézier Curves

Bézier curves are useful for generating smooth trajectories with control over the shape:

```cpp
class BezierTrajectory {
private:
    std::vector<Eigen::Vector3d> control_points_;
    double duration_;
    
public:
    BezierTrajectory(const std::vector<Eigen::Vector3d>& control_points, double duration)
        : control_points_(control_points), duration_(duration) {}
    
    Eigen::Vector3d getPosition(double t) const {
        if (control_points_.empty()) return Eigen::Vector3d::Zero();
        if (t < 0.0) t = 0.0;
        if (t > duration_) t = duration_;
        
        double u = t / duration_;  // Normalize time to [0,1]
        return computeBezier(u, control_points_);
    }
    
    Eigen::Vector3d getVelocity(double t) const {
        if (control_points_.size() < 2) return Eigen::Vector3d::Zero();
        
        double u = t / duration_;
        std::vector<Eigen::Vector3d> velocity_control_points = computeDerivativePoints();
        
        Eigen::Vector3d velocity = computeBezier(u, velocity_control_points);
        return velocity / duration_;  // Scale by time normalization
    }
    
    Eigen::Vector3d getAcceleration(double t) const {
        if (control_points_.size() < 3) return Eigen::Vector3d::Zero();
        
        double u = t / duration_;
        std::vector<Eigen::Vector3d> acceleration_control_points = computeSecondDerivativePoints();
        
        Eigen::Vector3d acceleration = computeBezier(u, acceleration_control_points);
        return acceleration / (duration_ * duration_);  // Scale by time normalization
    }

private:
    Eigen::Vector3d computeBezier(double u, const std::vector<Eigen::Vector3d>& points) const {
        if (points.size() == 1) {
            return points[0];
        }
        
        std::vector<Eigen::Vector3d> new_points;
        for (size_t i = 0; i < points.size() - 1; i++) {
            new_points.push_back((1.0 - u) * points[i] + u * points[i + 1]);
        }
        
        return computeBezier(u, new_points);
    }
    
    std::vector<Eigen::Vector3d> computeDerivativePoints() const {
        std::vector<Eigen::Vector3d> derivative_points;
        for (size_t i = 0; i < control_points_.size() - 1; i++) {
            derivative_points.push_back(
                (control_points_[i + 1] - control_points_[i]) * (control_points_.size() - 1)
            );
        }
        return derivative_points;
    }
    
    std::vector<Eigen::Vector3d> computeSecondDerivativePoints() const {
        auto first_deriv_points = computeDerivativePoints();
        std::vector<Eigen::Vector3d> second_deriv_points;
        for (size_t i = 0; i < first_deriv_points.size() - 1; i++) {
            second_deriv_points.push_back(
                (first_deriv_points[i + 1] - first_deriv_points[i]) * (first_deriv_points.size() - 1)
            );
        }
        return second_deriv_points;
    }
};
```

## Advanced Trajectory Generation

### Optimization-Based Trajectory Generation

#### Trajectory Optimization
```cpp
#include <Eigen/Dense>
#include <vector>

class TrajectoryOptimizer {
private:
    struct TrajectoryPoint {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
        rclcpp::Time timestamp;
    };
    
    std::vector<TrajectoryPoint> trajectory_points_;
    double time_step_;
    int num_points_;
    
    // Constraints
    std::vector<Eigen::Vector3d> obstacles_;
    std::vector<Eigen::Vector3d> forbidden_zones_;
    double min_clearance_;
    
    // Cost weights
    double smoothness_weight_;
    double obstacle_weight_;
    double velocity_weight_;
    double acceleration_weight_;
    double path_length_weight_;

public:
    TrajectoryOptimizer(double dt = 0.1, int points = 100, 
                       double clearance = 0.5)
        : time_step_(dt), num_points_(points), min_clearance_(clearance),
          smoothness_weight_(1.0), obstacle_weight_(10.0), 
          velocity_weight_(0.5), acceleration_weight_(0.2), 
          path_length_weight_(0.1) {}
    
    std::vector<TrajectoryPoint> optimizeTrajectory(
        const Eigen::Vector3d& start_pos,
        const Eigen::Vector3d& goal_pos,
        const std::vector<Eigen::Vector3d>& obstacles) {
        
        obstacles_ = obstacles;
        
        // Initialize trajectory with straight line
        trajectory_points_ = initializeTrajectory(start_pos, goal_pos);
        
        // Optimize trajectory using iterative approach
        for (int iter = 0; iter < MAX_OPTIMIZATION_ITERATIONS; iter++) {
            bool improved = optimizeIteration();
            if (!improved) {
                break;  // Convergence reached
            }
        }
        
        return trajectory_points_;
    }

private:
    std::vector<TrajectoryPoint> initializeTrajectory(
        const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
        
        std::vector<TrajectoryPoint> initial_trajectory(num_points_);
        
        // Create straight line trajectory
        Eigen::Vector3d direction = (goal - start) / num_points_;
        
        for (int i = 0; i < num_points_; i++) {
            double t = static_cast<double>(i) / num_points_;
            
            initial_trajectory[i].position = start + t * (goal - start);
            initial_trajectory[i].timestamp = 
                rclcpp::Time(0) + rclcpp::Duration::from_seconds(i * time_step_);
        }
        
        // Calculate velocities and accelerations
        for (int i = 1; i < num_points_; i++) {
            initial_trajectory[i].velocity = 
                (initial_trajectory[i].position - initial_trajectory[i-1].position) / time_step_;
        }
        
        for (int i = 1; i < num_points_ - 1; i++) {
            initial_trajectory[i].acceleration = 
                (initial_trajectory[i+1].velocity - initial_trajectory[i-1].velocity) / (2 * time_step_);
        }
        
        return initial_trajectory;
    }
    
    bool optimizeIteration() {
        bool improvement_made = false;
        
        for (int i = 1; i < num_points_ - 1; i++) {  // Skip start and end points
            // Calculate gradient of cost function
            Eigen::Vector3d gradient = calculateCostGradient(i);
            
            // Update trajectory point
            Eigen::Vector3d new_position = trajectory_points_[i].position - 
                                         OPTIMIZATION_STEP_SIZE * gradient;
            
            // Check if new position is better
            double old_cost = calculateCost(i);
            trajectory_points_[i].position = new_position;
            double new_cost = calculateCost(i);
            
            if (new_cost < old_cost) {
                // Update velocity and acceleration
                updateVelocitiesAndAccelerations(i);
                improvement_made = true;
            } else {
                // Revert change
                trajectory_points_[i].position = 
                    trajectory_points_[i].position + OPTIMIZATION_STEP_SIZE * gradient;
            }
        }
        
        return improvement_made;
    }
    
    Eigen::Vector3d calculateCostGradient(int point_idx) {
        Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
        
        // Gradient for smoothness (minimize acceleration)
        if (point_idx > 0 && point_idx < num_points_ - 1) {
            Eigen::Vector3d prev_pos = trajectory_points_[point_idx - 1].position;
            Eigen::Vector3d curr_pos = trajectory_points_[point_idx].position;
            Eigen::Vector3d next_pos = trajectory_points_[point_idx + 1].position;
            
            // Gradient of acceleration term: (next_pos - 2*curr_pos + prev_pos)
            Eigen::Vector3d acc_gradient = 
                smoothness_weight_ * 2.0 * (2 * curr_pos - prev_pos - next_pos);
            gradient += acc_gradient;
        }
        
        // Gradient for obstacle avoidance
        Eigen::Vector3d obs_gradient = calculateObstacleGradient(point_idx);
        gradient += obs_gradient;
        
        // Gradient for velocity constraints
        if (point_idx > 0 && point_idx < num_points_) {
            Eigen::Vector3d vel_gradient = 
                velocity_weight_ * (trajectory_points_[point_idx].velocity.norm() > MAX_VELOCITY ?
                                  trajectory_points_[point_idx].velocity.normalized() : Eigen::Vector3d::Zero());
            gradient += vel_gradient;
        }
        
        return gradient;
    }
    
    Eigen::Vector3d calculateObstacleGradient(int point_idx) {
        Eigen::Vector3d gradient = Eigen::Vector3d::Zero();
        
        Eigen::Vector3d current_pos = trajectory_points_[point_idx].position;
        
        for (const auto& obstacle : obstacles_) {
            Eigen::Vector3d to_obstacle = obstacle - current_pos;
            double distance = to_obstacle.norm();
            
            if (distance < OBSTACLE_AVOIDANCE_RADIUS) {
                // Repulsive force proportional to inverse distance
                double force_magnitude = obstacle_weight_ / (distance * distance + 1e-6);
                Eigen::Vector3d force_direction = to_obstacle.normalized();
                
                gradient += force_magnitude * force_direction;
            }
        }
        
        return gradient;
    }
    
    double calculateCost(int point_idx) {
        double cost = 0.0;
        
        // Path length cost
        if (point_idx > 0) {
            double dist = (trajectory_points_[point_idx].position - 
                          trajectory_points_[point_idx-1].position).norm();
            cost += path_length_weight_ * dist;
        }
        
        // Velocity cost
        if (trajectory_points_[point_idx].velocity.norm() > MAX_VELOCITY) {
            cost += velocity_weight_ * 
                   (trajectory_points_[point_idx].velocity.norm() - MAX_VELOCITY);
        }
        
        // Acceleration cost
        if (trajectory_points_[point_idx].acceleration.norm() > MAX_ACCELERATION) {
            cost += acceleration_weight_ * 
                   (trajectory_points_[point_idx].acceleration.norm() - MAX_ACCELERATION);
        }
        
        // Obstacle cost
        for (const auto& obstacle : obstacles_) {
            double distance = (trajectory_points_[point_idx].position - obstacle).norm();
            if (distance < min_clearance_) {
                cost += obstacle_weight_ * (min_clearance_ - distance);
            }
        }
        
        // Smoothness cost (based on jerk or higher derivatives)
        if (point_idx > 0 && point_idx < num_points_ - 1) {
            Eigen::Vector3d jerk = (trajectory_points_[point_idx+1].acceleration - 
                                   2*trajectory_points_[point_idx].acceleration + 
                                   trajectory_points_[point_idx-1].acceleration) / (time_step_ * time_step_);
            cost += smoothness_weight_ * jerk.squaredNorm();
        }
        
        return cost;
    }
    
    void updateVelocitiesAndAccelerations(int point_idx) {
        // Update velocity and acceleration for the changed point and neighbors
        if (point_idx > 0 && point_idx < num_points_) {
            trajectory_points_[point_idx].velocity = 
                (trajectory_points_[point_idx].position - 
                 trajectory_points_[point_idx-1].position) / time_step_;
        }
        
        if (point_idx > 0 && point_idx < num_points_ - 1) {
            trajectory_points_[point_idx].acceleration = 
                (trajectory_points_[point_idx+1].position - 
                 2*trajectory_points_[point_idx].position + 
                 trajectory_points_[point_idx-1].position) / (time_step_ * time_step_);
        }
        
        // Update neighbors as well
        if (point_idx > 1) {
            trajectory_points_[point_idx-1].velocity = 
                (trajectory_points_[point_idx].position - 
                 trajectory_points_[point_idx-2].position) / time_step_;
        }
        
        if (point_idx < num_points_ - 2) {
            trajectory_points_[point_idx+1].velocity = 
                (trajectory_points_[point_idx+2].position - 
                 trajectory_points_[point_idx].position) / time_step_;
        }
    }
    
    static constexpr double MAX_VELOCITY = 1.0;      // m/s
    static constexpr double MAX_ACCELERATION = 2.0;  // m/s²
    static constexpr double OBSTACLE_AVOIDANCE_RADIUS = 2.0;
    static constexpr double OPTIMIZATION_STEP_SIZE = 0.01;
    static constexpr int MAX_OPTIMIZATION_ITERATIONS = 1000;
};
```

### Model Predictive Control (MPC) for Trajectory Generation

MPC generates trajectories by solving an optimization problem at each time step:

```cpp
class MPCTrajectoryGenerator {
private:
    size_t prediction_horizon_;
    size_t control_horizon_;
    double dt_;
    
    // System matrices (A: state transition, B: control, C: output)
    Eigen::MatrixXd A_, B_, C_;
    Eigen::VectorXd state_;
    
    // Cost matrices
    Eigen::MatrixXd Q_;  // State cost
    Eigen::MatrixXd R_;  // Control cost
    Eigen::MatrixXd Qf_; // Terminal cost
    
    // Constraints
    Eigen::VectorXd u_min_, u_max_;
    Eigen::VectorXd x_min_, x_max_;
    
    // Optimization parameters
    double max_iterations_;
    double tolerance_;

public:
    MPCTrajectoryGenerator(size_t pred_horizon, size_t ctrl_horizon, double time_step)
        : prediction_horizon_(pred_horizon), control_horizon_(ctrl_horizon), 
          dt_(time_step), max_iterations_(100), tolerance_(1e-6) {
        
        // Initialize with default robot model (e.g., double integrator)
        initializeRobotModel();
    }
    
    std::vector<TrajectoryPoint> generateTrajectory(
        const Eigen::VectorXd& current_state,
        const std::vector<Eigen::VectorXd>& reference_trajectory) {
        
        state_ = current_state;
        
        // Build prediction matrices
        Eigen::MatrixXd Phi = buildPredictionMatrix();
        Eigen::MatrixXd Gamma = buildControlMatrix();
        
        // Formulate QP problem
        auto [H, f, A_ineq, b_ineq] = formulateQP(Phi, Gamma, reference_trajectory);
        
        // Solve QP problem
        Eigen::VectorXd optimal_controls = solveQP(H, f, A_ineq, b_ineq);
        
        // Simulate trajectory with optimal controls
        std::vector<TrajectoryPoint> simulated_trajectory = 
            simulateTrajectory(current_state, optimal_controls);
        
        return simulated_trajectory;
    }

private:
    void initializeRobotModel() {
        // Example: Double integrator model for a 2D point mass robot
        // State: [x, y, vx, vy]
        A_ = Eigen::MatrixXd::Identity(4, 4);
        A_(0, 2) = dt_;  // x += vx*dt
        A_(1, 3) = dt_;  // y += vy*dt
        
        B_ = Eigen::MatrixXd::Zero(4, 2);
        B_(2, 0) = dt_;  // vx += ax*dt
        B_(3, 1) = dt_;  // vy += ay*dt
        
        C_ = Eigen::MatrixXd::Identity(4, 4);  // Direct state output
        
        // Cost matrices (Q penalizes state error, R penalizes control effort)
        Q_ = Eigen::MatrixXd::Identity(4, 4);
        Q_(0, 0) = 10.0;  // Higher penalty for position error
        Q_(1, 1) = 10.0;
        Qf_ = Q_;  // Same terminal cost
        R_ = Eigen::MatrixXd::Identity(2, 2) * 0.1;  // Lower penalty for control effort
        
        // Initialize state
        state_ = Eigen::VectorXd::Zero(4);
        
        // Initialize constraints
        u_min_ = Eigen::VectorXd::Constant(2, -1.0);  // Min control values
        u_max_ = Eigen::VectorXd::Constant(2, 1.0);   // Max control values
        x_min_ = Eigen::VectorXd::Constant(4, -100.0); // Min state values
        x_max_ = Eigen::VectorXd::Constant(4, 100.0);  // Max state values
    }
    
    Eigen::MatrixXd buildPredictionMatrix() {
        // Build state prediction matrix: X = Phi*x0 + Gamma*U
        size_t nx = A_.rows();
        
        Eigen::MatrixXd Phi = Eigen::MatrixXd::Zero(prediction_horizon_ * nx, nx);
        
        // Fill prediction matrix
        Eigen::MatrixXd Ak = Eigen::MatrixXd::Identity(nx, nx);
        for (size_t k = 0; k < prediction_horizon_; k++) {
            if (k > 0) {
                Ak = Ak * A_;
            }
            Phi.block(k * nx, 0, nx, nx) = Ak;
        }
        
        return Phi;
    }
    
    Eigen::MatrixXd buildControlMatrix() {
        // Build control-to-state matrix: X = Phi*x0 + Gamma*U
        size_t nx = A_.rows();
        size_t nu = B_.cols();
        
        Eigen::MatrixXd Gamma = Eigen::MatrixXd::Zero(prediction_horizon_ * nx, 
                                                     control_horizon_ * nu);
        
        for (size_t i = 0; i < prediction_horizon_; i++) {
            for (size_t j = 0; j <= i && j < control_horizon_; j++) {
                Eigen::MatrixXd Ak = (i == j) ? Eigen::MatrixXd::Identity(nx, nx) : 
                                   matrixPower(A_, i - j);
                Eigen::MatrixXd block = Ak * B_;
                
                Gamma.block(i * nx, j * nu, nx, nu) = block;
            }
        }
        
        return Gamma;
    }
    
    Eigen::MatrixXd matrixPower(const Eigen::MatrixXd& A, int n) {
        if (n == 0) return Eigen::MatrixXd::Identity(nx, nx);
        if (n == 1) return A;
        
        Eigen::MatrixXd result = A;
        for (int i = 1; i < n; i++) {
            result = result * A;
        }
        return result;
    }
    
    std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::MatrixXd, Eigen::VectorXd>
    formulateQP(const Eigen::MatrixXd& Phi, 
                const Eigen::MatrixXd& Gamma, 
                const std::vector<Eigen::VectorXd>& ref_traj) {
        
        size_t N = prediction_horizon_;
        size_t nu = control_horizon_ * B_.cols();  // Total control variables
        size_t nx = A_.rows();  // State dimension
        
        // Quadratic cost matrix H: 1/2 * U^T * H * U + f^T * U
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(nu, nu);
        Eigen::VectorXd f = Eigen::VectorXd::Zero(nu);
        
        // For simplicity, using diagonal Q and R matrices
        Eigen::MatrixXd QQ = Eigen::MatrixXd::Zero(N * nx, N * nx);
        for (size_t i = 0; i < N; i++) {
            QQ.block(i * nx, i * nx, nx, nx) = Q_;
        }
        
        Eigen::MatrixXd RR = Eigen::MatrixXd::Zero(control_horizon_ * B_.cols(), 
                                                  control_horizon_ * B_.cols());
        for (size_t i = 0; i < control_horizon_; i++) {
            RR.block(i * B_.cols(), i * B_.cols(), B_.cols(), B_.cols()) = R_;
        }
        
        // Hessian: H = Gamma^T * QQ * Gamma + RR
        H = Gamma.transpose() * QQ * Gamma + RR;
        
        // Linear term: f = -Gamma^T * QQ * Phi * x0
        Eigen::VectorXd x0_extended = Eigen::VectorXd::Zero(N * nx);
        for (size_t i = 0; i < nx; i++) {
            x0_extended[i] = state_[i];
        }
        
        f = -Gamma.transpose() * QQ * Phi * x0_extended;
        
        // Constraints: A_ineq * U <= b_ineq
        size_t constraint_count = 2 * (control_horizon_ * B_.cols() + N * nx);  // Control and state constraints
        Eigen::MatrixXd A_ineq = Eigen::MatrixXd::Zero(constraint_count, nu);
        Eigen::VectorXd b_ineq = Eigen::VectorXd::Zero(constraint_count);
        
        // Control constraints: u_min <= U <= u_max
        for (size_t i = 0; i < control_horizon_; i++) {
            for (size_t j = 0; j < B_.cols(); j++) {
                // Upper bound: U_i_j <= u_max_j
                A_ineq(i * B_.cols() + j, i * B_.cols() + j) = 1.0;
                b_ineq(i * B_.cols() + j) = u_max_[j];
                
                // Lower bound: -U_i_j <= -u_min_j
                A_ineq((control_horizon_ * B_.cols()) + i * B_.cols() + j, i * B_.cols() + j) = -1.0;
                b_ineq((control_horizon_ * B_.cols()) + i * B_.cols() + j) = -u_min_[j];
            }
        }
        
        // State constraints would be more complex to formulate
        // This would involve the relationship between states and controls through system dynamics
        
        return std::make_tuple(H, f, A_ineq, b_ineq);
    }
    
    Eigen::VectorXd solveQP(const Eigen::MatrixXd& H, 
                           const Eigen::VectorXd& f,
                           const Eigen::MatrixXd& A_ineq, 
                           const Eigen::VectorXd& b_ineq) {
        
        // In practice, this would use a QP solver like OSQP or qpOASES
        // For this example, returning a zero vector as a placeholder
        return Eigen::VectorXd::Zero(H.cols());
    }
    
    std::vector<TrajectoryPoint> simulateTrajectory(
        const Eigen::VectorXd& initial_state,
        const Eigen::VectorXd& control_sequence) {
        
        std::vector<TrajectoryPoint> trajectory;
        Eigen::VectorXd current_state = initial_state;
        
        for (size_t i = 0; i < prediction_horizon_; i++) {
            TrajectoryPoint point;
            point.position = {current_state[0], current_state[1], 0.0};
            point.velocity = {current_state[2], current_state[3], 0.0};
            
            // Calculate acceleration from control input
            if (i < control_sequence.size()/2) {  // Assuming 2D control inputs
                point.acceleration = {control_sequence[i*2], control_sequence[i*2+1], 0.0};
            } else {
                point.acceleration = {0.0, 0.0, 0.0};
            }
            
            point.timestamp = rclcpp::Time(0) + 
                             rclcpp::Duration::from_seconds(i * dt_);
            
            trajectory.push_back(point);
            
            // Update state with dynamics
            if (i < control_sequence.size()/2) {
                Eigen::VectorXd control_input(2);
                control_input << control_sequence[i*2], control_sequence[i*2+1];
                
                current_state = A_ * current_state + B_ * control_input;
            } else {
                current_state = A_ * current_state;  // No control input
            }
        }
        
        return trajectory;
    }
    
    struct TrajectoryPoint {
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
        rclcpp::Time timestamp;
    };
};
```

## Trajectory Execution and Control

### Trajectory Tracking Controllers

#### PID-Based Tracking
```cpp
class TrajectoryTracker {
private:
    std::unique_ptr<RobotController> robot_controller_;
    std::vector<TrajectoryPoint> current_trajectory_;
    size_t current_waypoint_index_;
    rclcpp::Time trajectory_start_time_;
    
    // PID controllers for each dimension
    std::vector<PIDController> position_controllers_;
    std::vector<PIDController> velocity_controllers_;
    
    // Tracking parameters
    double tracking_error_threshold_;
    double velocity_threshold_;
    
    // Performance metrics
    std::vector<TrackingMetrics> tracking_history_;
    size_t max_tracking_history_;

public:
    TrajectoryTracker(double tracking_threshold = 0.1, size_t history_size = 1000)
        : tracking_error_threshold_(tracking_threshold), 
          max_tracking_history_(history_size), 
          current_waypoint_index_(0) {
        
        robot_controller_ = std::make_unique<RobotController>();
        
        // Initialize PID controllers for each DOF
        for (int i = 0; i < NUM_ROBOT_DOF; i++) {
            position_controllers_.emplace_back(100.0, 10.0, 15.0);  // Kp, Ki, Kd
            velocity_controllers_.emplace_back(50.0, 5.0, 8.0);     // Kp, Ki, Kd for velocity
        }
    }
    
    ControlCommand trackTrajectory(const RobotState& current_state, 
                                  const rclcpp::Time& current_time) {
        
        if (current_trajectory_.empty()) {
            return generateStopCommand();  // No trajectory to track
        }
        
        // Get reference state for current time
        auto reference_state = getReferenceState(current_time);
        
        // Calculate tracking errors
        auto position_error = calculatePositionError(current_state, reference_state);
        auto velocity_error = calculateVelocityError(current_state, reference_state);
        
        // Generate control commands using PID
        ControlCommand control_cmd;
        for (size_t i = 0; i < position_controllers_.size(); i++) {
            double pos_control = position_controllers_[i].compute(position_error[i], 0.01);
            double vel_control = velocity_controllers_[i].compute(velocity_error[i], 0.01);
            
            // Combine position and velocity control
            control_cmd.efforts[i] = pos_control + vel_control;
            
            // Apply safety limits
            control_cmd.efforts[i] = std::clamp(control_cmd.efforts[i], 
                                              -MAX_JOINT_EFFORT, MAX_JOINT_EFFORT);
        }
        
        // Update performance metrics
        updateTrackingMetrics(position_error, velocity_error, current_time);
        
        // Check if trajectory is complete
        if (isTrajectoryComplete(current_time)) {
            control_cmd = generateStopCommand();
        }
        
        return control_cmd;
    }
    
    void setCurrentTrajectory(const std::vector<TrajectoryPoint>& trajectory) {
        current_trajectory_ = trajectory;
        current_waypoint_index_ = 0;
        trajectory_start_time_ = this->now();
    }
    
    bool isTrajectoryComplete(const rclcpp::Time& current_time) {
        return current_waypoint_index_ >= current_trajectory_.size() - 1;
    }
    
    std::vector<double> getTrackingErrorHistory() {
        std::vector<double> errors;
        for (const auto& metric : tracking_history_) {
            errors.push_back(metric.position_error_magnitude);
        }
        return errors;
    }

private:
    TrajectoryPoint getReferenceState(const rclcpp::Time& current_time) {
        // Interpolate reference state at current time
        double elapsed_time = (current_time - trajectory_start_time_).seconds();
        
        if (current_trajectory_.empty()) {
            return TrajectoryPoint();  // Return zero state if no trajectory
        }
        
        if (elapsed_time <= 0.0) {
            return current_trajectory_.front();
        }
        
        if (elapsed_time >= current_trajectory_.back().timestamp.seconds()) {
            return current_trajectory_.back();
        }
        
        // Find appropriate segment for interpolation
        for (size_t i = 0; i < current_trajectory_.size() - 1; i++) {
            if (elapsed_time >= current_trajectory_[i].timestamp.seconds() &&
                elapsed_time <= current_trajectory_[i+1].timestamp.seconds()) {
                
                // Linear interpolation between trajectory points
                double t1 = current_trajectory_[i].timestamp.seconds();
                double t2 = current_trajectory_[i+1].timestamp.seconds();
                
                double alpha = (elapsed_time - t1) / (t2 - t1);
                
                TrajectoryPoint interpolated;
                interpolated.position = current_trajectory_[i].position * (1.0 - alpha) + 
                                       current_trajectory_[i+1].position * alpha;
                interpolated.velocity = current_trajectory_[i].velocity * (1.0 - alpha) + 
                                       current_trajectory_[i+1].velocity * alpha;
                interpolated.acceleration = current_trajectory_[i].acceleration * (1.0 - alpha) + 
                                           current_trajectory_[i+1].acceleration * alpha;
                interpolated.timestamp = current_time;
                
                current_waypoint_index_ = i;
                
                return interpolated;
            }
        }
        
        // If no segment found, return last point
        return current_trajectory_.back();
    }
    
    std::vector<double> calculatePositionError(const RobotState& actual, 
                                             const TrajectoryPoint& reference) {
        std::vector<double> errors(NUM_ROBOT_DOF);
        
        for (size_t i = 0; i < errors.size(); i++) {
            errors[i] = reference.position[i] - actual.positions[i];
        }
        
        return errors;
    }
    
    std::vector<double> calculateVelocityError(const RobotState& actual, 
                                             const TrajectoryPoint& reference) {
        std::vector<double> errors(NUM_ROBOT_DOF);
        
        for (size_t i = 0; i < errors.size(); i++) {
            errors[i] = reference.velocity[i] - actual.velocities[i];
        }
        
        return errors;
    }
    
    void updateTrackingMetrics(const std::vector<double>& pos_errors,
                              const std::vector<double>& vel_errors,
                              const rclcpp::Time& current_time) {
        TrackingMetrics metrics;
        metrics.timestamp = current_time;
        
        // Calculate error magnitudes
        double pos_error_sq_sum = 0.0;
        double vel_error_sq_sum = 0.0;
        
        for (double err : pos_errors) {
            pos_error_sq_sum += err * err;
        }
        for (double err : vel_errors) {
            vel_error_sq_sum += err * err;
        }
        
        metrics.position_error_magnitude = std::sqrt(pos_error_sq_sum);
        metrics.velocity_error_magnitude = std::sqrt(vel_error_sq_sum);
        
        // Add to history
        if (tracking_history_.size() >= max_tracking_history_) {
            tracking_history_.erase(tracking_history_.begin());
        }
        tracking_history_.push_back(metrics);
    }
    
    ControlCommand generateStopCommand() {
        ControlCommand stop_cmd;
        stop_cmd.efforts.resize(NUM_ROBOT_DOF, 0.0);  // Zero efforts to stop
        stop_cmd.velocities.resize(NUM_ROBOT_DOF, 0.0);
        stop_cmd.positions.clear();  // Clear desired positions to maintain current
        return stop_cmd;
    }
    
    struct TrackingMetrics {
        rclcpp::Time timestamp;
        double position_error_magnitude;
        double velocity_error_magnitude;
        double acceleration_error_magnitude;
    };
    
    static constexpr int NUM_ROBOT_DOF = 6;  // Example for 6-DOF manipulator
    static constexpr double MAX_JOINT_EFFORT = 100.0;  // N-m for joints
};
```

### Advanced Trajectory Execution

#### Feedforward Control Integration
```cpp
class AdvancedTrajectoryTracker {
private:
    std::vector<PIDController> feedback_controllers_;
    std::vector<FeedforwardController> feedforward_controllers_;
    std::vector<TrajectoryPoint> current_trajectory_;
    size_t current_waypoint_index_;
    
    // Feedforward control for dynamics compensation
    std::unique_ptr<RobotDynamicsModel> dynamics_model_;
    
    // Adaptive control components
    std::unique_ptr<ParameterEstimator> parameter_estimator_;
    std::unique_ptr<AdaptiveController> adaptive_controller_;

public:
    AdvancedTrajectoryTracker() {
        // Initialize controllers for each DOF
        for (int i = 0; i < NUM_ROBOT_DOF; i++) {
            feedback_controllers_.emplace_back(100.0, 10.0, 15.0);
            feedforward_controllers_.emplace_back();
        }
        
        dynamics_model_ = std::make_unique<RobotDynamicsModel>();
        parameter_estimator_ = std::make_unique<ParameterEstimator>();
        adaptive_controller_ = std::make_unique<AdaptiveController>();
    }
    
    ControlCommand trackTrajectoryAdvanced(const RobotState& current_state,
                                          const TrajectoryPoint& reference_state,
                                          const TrajectoryPoint& next_reference_state,
                                          double dt) {
        
        ControlCommand control_cmd;
        control_cmd.efforts.resize(NUM_ROBOT_DOF);
        
        for (size_t i = 0; i < NUM_ROBOT_DOF; i++) {
            // Feedback control (PID)
            double position_error = reference_state.position[i] - current_state.position[i];
            double velocity_error = reference_state.velocity[i] - current_state.velocity[i];
            
            double feedback_effort = feedback_controllers_[i].compute(
                position_error, velocity_error, dt);
            
            // Feedforward control (dynamics compensation)
            double feedforward_effort = calculateFeedforwardEffort(
                reference_state, next_reference_state, i);
            
            // Adaptive control component (if available)
            double adaptive_effort = adaptive_controller_->getAdaptiveEffort(
                i, position_error, velocity_error, current_state);
            
            // Combine all control components
            control_cmd.efforts[i] = feedback_effort + feedforward_effort + adaptive_effort;
            
            // Apply safety limits
            control_cmd.efforts[i] = std::clamp(control_cmd.efforts[i], 
                                              -MAX_JOINT_EFFORT, MAX_JOINT_EFFORT);
        }
        
        // Update parameter estimates for adaptive control
        parameter_estimator_->updateEstimates(current_state, control_cmd);
        
        return control_cmd;
    }

private:
    double calculateFeedforwardEffort(const TrajectoryPoint& ref_state,
                                    const TrajectoryPoint& next_ref_state,
                                    size_t joint_idx) {
        
        // Calculate desired acceleration from trajectory
        double desired_acc = (next_ref_state.velocity[joint_idx] - 
                             ref_state.velocity[joint_idx]) / 0.01;  // Assuming 10ms dt
        
        // Use dynamics model to calculate required effort for desired acceleration
        auto joint_state = JointState{
            .position = ref_state.position[joint_idx],
            .velocity = ref_state.velocity[joint_idx],
            .acceleration = desired_acc
        };
        
        // Calculate required effort using inverse dynamics
        double required_effort = dynamics_model_->calculateRequiredEffort(
            joint_idx, joint_state);
        
        return required_effort;
    }
    
    struct JointState {
        double position;
        double velocity;
        double acceleration;
    };
    
    struct RobotDynamicsModel {
        double calculateRequiredEffort(size_t joint_idx, const JointState& state) {
            // Calculate required effort using robot dynamics (e.g., inverse dynamics model)
            // This would implement the inverse dynamics equations for the robot
            // τ = M(q)q̈ + C(q, q̇)q̇ + g(q) + F(q̇)
            // where M is mass matrix, C is Coriolis forces, g is gravity, F is friction
            
            double mass_matrix_effort = calculateMassMatrixEffort(joint_idx, state.acceleration);
            double coriolis_effort = calculateCoriolisEffort(joint_idx, state.position, state.velocity);
            double gravity_effort = calculateGravityEffort(joint_idx, state.position);
            double friction_effort = calculateFrictionEffort(joint_idx, state.velocity);
            
            return mass_matrix_effort + coriolis_effort + gravity_effort + friction_effort;
        }
        
        double calculateMassMatrixEffort(size_t joint_idx, double acceleration) {
            // Calculate effort needed for acceleration based on mass matrix
            return 0.0;  // Placeholder - implementation would depend on specific robot
        }
        
        double calculateCoriolisEffort(size_t joint_idx, double position, double velocity) {
            // Calculate effort needed for Coriolis forces
            return 0.0;  // Placeholder
        }
        
        double calculateGravityEffort(size_t joint_idx, double position) {
            // Calculate effort needed to counteract gravity
            return 0.0;  // Placeholder
        }
        
        double calculateFrictionEffort(size_t joint_idx, double velocity) {
            // Calculate effort needed to overcome friction
            return 0.0;  // Placeholder
        }
    };
    
    struct ParameterEstimator {
        void updateEstimates(const RobotState& state, const ControlCommand& command) {
            // Update estimates of robot parameters (mass, friction, etc.)
            // using adaptive control techniques
        }
        
        std::vector<double> getEstimatedParameters() {
            // Return current parameter estimates
            return {};  // Placeholder
        }
    };
    
    struct AdaptiveController {
        double getAdaptiveEffort(size_t joint_idx, 
                               double position_error,
                               double velocity_error,
                               const RobotState& state) {
            // Calculate adaptive control component based on parameter estimates
            // and tracking errors
            return 0.0;  // Placeholder
        }
    };
};
```

## Integration with Navigation and Planning

### Path Planning to Trajectory Conversion

#### Path Smoothing and Parametrization
```cpp
class PathToTrajectoryConverter {
private:
    std::unique_ptr<PathPlanner> path_planner_;
    std::unique_ptr<TrajectoryOptimizer> trajectory_optimizer_;
    
    // Path smoothing parameters
    double smoothing_weight_;
    double smoothness_weight_;
    double obstacle_avoidance_weight_;
    
    // Velocity profile generation
    std::unique_ptr<VelocityProfileGenerator> velocity_generator_;

public:
    PathToTrajectoryConverter() 
        : smoothing_weight_(0.1), smoothness_weight_(0.8), 
          obstacle_avoidance_weight_(0.1) {
        
        path_planner_ = std::make_unique<OMPLPathPlanner>();
        trajectory_optimizer_ = std::make_unique<TrajectoryOptimizer>();
        velocity_generator_ = std::make_unique<TrapezoidalVelocityProfile>();
    }
    
    std::vector<TrajectoryPoint> convertPathToTrajectory(
        const std::vector<geometry_msgs::msg::Point>& path,
        const RobotConstraints& constraints,
        double execution_time) {
        
        std::vector<TrajectoryPoint> trajectory;
        
        if (path.size() < 2) {
            return trajectory;  // Invalid path
        }
        
        // Smooth the path to make it more suitable for execution
        auto smoothed_path = smoothPath(path, constraints);
        
        // Generate velocity profile along the path
        auto velocity_profile = velocity_generator_->generateProfile(
            smoothed_path, constraints);
        
        // Parametrize path with time to create trajectory
        trajectory = parametrizePathToTrajectory(smoothed_path, 
                                              velocity_profile, 
                                              execution_time);
        
        // Optimize trajectory for dynamic feasibility
        trajectory = trajectory_optimizer_->optimizeTrajectory(
            trajectory, constraints);
        
        return trajectory;
    }

private:
    std::vector<geometry_msgs::msg::Point> smoothPath(
        const std::vector<geometry_msgs::msg::Point>& path,
        const RobotConstraints& constraints) {
        
        if (path.size() < 3) return path;  // Not enough points to smooth
        
        std::vector<geometry_msgs::msg::Point> smoothed_path = path;
        
        // Apply iterative smoothing (similar to gradient descent on path)
        for (int iteration = 0; iteration < 100; iteration++) {
            for (size_t i = 1; i < smoothed_path.size() - 1; i++) {
                geometry_msgs::msg::Point orig = smoothed_path[i];
                
                // Calculate smoothing direction (balance between path following and smoothness)
                geometry_msgs::msg::Point smooth_direction;
                smooth_direction.x = smoothing_weight_ * 
                    (smoothed_path[i-1].x + smoothed_path[i+1].x - 2 * smoothed_path[i].x);
                smooth_direction.y = smoothing_weight_ * 
                    (smoothed_path[i-1].y + smoothed_path[i+1].y - 2 * smoothed_path[i].y);
                smooth_direction.z = smoothing_weight_ * 
                    (smoothed_path[i-1].z + smoothed_path[i+1].z - 2 * smoothed_path[i].z);
                
                // Add obstacle avoidance
                geometry_msgs::msg::Point obstacle_avoidance = 
                    calculateObstacleAvoidance(smoothed_path[i]);
                
                // Update path point
                smoothed_path[i].x += smoothness_weight_ * smooth_direction.x + 
                                    obstacle_avoidance_weight_ * obstacle_avoidance.x;
                smoothed_path[i].y += smoothness_weight_ * smooth_direction.y + 
                                    obstacle_avoidance_weight_ * obstacle_avoidance.y;
                smoothed_path[i].z += smoothness_weight_ * smooth_direction.z + 
                                    obstacle_avoidance_weight_ * obstacle_avoidance.z;
                
                // Ensure path stays within bounds
                smoothed_path[i] = enforcePathConstraints(smoothed_path[i], constraints);
            }
        }
        
        return smoothed_path;
    }
    
    geometry_msgs::msg::Point calculateObstacleAvoidance(
        const geometry_msgs::msg::Point& point) {
        
        geometry_msgs::msg::Point avoidance_force;
        
        // Calculate repulsive forces from nearby obstacles
        for (const auto& obstacle : environment_obstacles_) {
            geometry_msgs::msg::Point to_obstacle;
            to_obstacle.x = obstacle.x - point.x;
            to_obstacle.y = obstacle.y - point.y;
            to_obstacle.z = obstacle.z - point.z;
            
            double distance = std::sqrt(to_obstacle.x*to_obstacle.x + 
                                      to_obstacle.y*to_obstacle.y + 
                                      to_obstacle.z*to_obstacle.z);
            
            if (distance < OBSTACLE_AVOIDANCE_RADIUS) {
                // Calculate repulsive force
                double force_magnitude = (OBSTACLE_AVOIDANCE_RADIUS - distance) * 
                                        OBSTACLE_REPULSION_STRENGTH;
                
                if (distance > 0.01) {  // Avoid division by zero
                    avoidance_force.x += force_magnitude * to_obstacle.x / distance;
                    avoidance_force.y += force_magnitude * to_obstacle.y / distance;
                    avoidance_force.z += force_magnitude * to_obstacle.z / distance;
                }
            }
        }
        
        return avoidance_force;
    }
    
    std::vector<TrajectoryPoint> parametrizePathToTrajectory(
        const std::vector<geometry_msgs::msg::Point>& path,
        const std::vector<double>& velocities,
        double total_time) {
        
        std::vector<TrajectoryPoint> trajectory;
        
        if (path.empty() || velocities.empty()) return trajectory;
        
        // Calculate time intervals based on path length and velocities
        std::vector<double> time_intervals = calculateTimeIntervals(path, velocities);
        
        // Accumulate time and create trajectory points
        double accumulated_time = 0.0;
        
        for (size_t i = 0; i < path.size(); i++) {
            TrajectoryPoint point;
            point.position = path[i];
            point.timestamp = rclcpp::Time(0) + 
                             rclcpp::Duration::from_seconds(accumulated_time);
            
            if (i < velocities.size()) {
                point.velocity = calculateVelocityVector(path, i, velocities[i]);
            }
            
            if (i > 0 && i < path.size() - 1) {
                point.acceleration = calculateAccelerationVector(path, time_intervals, i);
            }
            
            trajectory.push_back(point);
            
            if (i < time_intervals.size()) {
                accumulated_time += time_intervals[i];
            }
        }
        
        return trajectory;
    }
    
    std::vector<double> calculateTimeIntervals(
        const std::vector<geometry_msgs::msg::Point>& path,
        const std::vector<double>& velocities) {
        
        std::vector<double> intervals;
        
        for (size_t i = 0; i < path.size() - 1; i++) {
            // Calculate distance between consecutive points
            double dx = path[i+1].x - path[i].x;
            double dy = path[i+1].y - path[i].y;
            double dz = path[i+1].z - path[i].z;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            // Calculate time based on average velocity
            double avg_velocity = (velocities[i] + velocities[i+1]) / 2.0;
            if (avg_velocity > 0.01) {  // Avoid division by zero
                intervals.push_back(distance / avg_velocity);
            } else {
                intervals.push_back(1.0);  // Default to 1 second if velocity is zero
            }
        }
        
        return intervals;
    }
    
    Eigen::Vector3d calculateVelocityVector(const std::vector<geometry_msgs::msg::Point>& path,
                                          size_t idx, double magnitude) {
        if (idx == 0) {
            if (path.size() > 1) {
                double dx = path[1].x - path[0].x;
                double dy = path[1].y - path[0].y;
                double dz = path[1].z - path[0].z;
                
                double norm = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (norm > 0.01) {
                    return {magnitude * dx/norm, magnitude * dy/norm, magnitude * dz/norm};
                }
            }
        } else if (idx == path.size() - 1) {
            if (path.size() > 1) {
                double dx = path[idx].x - path[idx-1].x;
                double dy = path[idx].y - path[idx-1].y;
                double dz = path[idx].z - path[idx-1].z;
                
                double norm = std::sqrt(dx*dx + dy*dy + dz*dz);
                if (norm > 0.01) {
                    return {magnitude * dx/norm, magnitude * dy/norm, magnitude * dz/norm};
                }
            }
        } else {
            // Use average of forward and backward directions
            double dx1 = path[idx+1].x - path[idx].x;
            double dy1 = path[idx+1].y - path[idx].y;
            double dz1 = path[idx+1].z - path[idx].z;
            
            double dx2 = path[idx].x - path[idx-1].x;
            double dy2 = path[idx].y - path[idx-1].y;
            double dz2 = path[idx].z - path[idx-1].z;
            
            double dx_avg = (dx1 + dx2) / 2.0;
            double dy_avg = (dy1 + dy2) / 2.0;
            double dz_avg = (dz1 + dz2) / 2.0;
            
            double norm = std::sqrt(dx_avg*dx_avg + dy_avg*dy_avg + dz_avg*dz_avg);
            if (norm > 0.01) {
                return {magnitude * dx_avg/norm, magnitude * dy_avg/norm, magnitude * dz_avg/norm};
            }
        }
        
        return {0.0, 0.0, 0.0};  // Zero velocity if cannot calculate
    }
    
    Eigen::Vector3d calculateAccelerationVector(const std::vector<geometry_msgs::msg::Point>& path,
                                              const std::vector<double>& time_intervals,
                                              size_t idx) {
        if (idx == 0 || idx >= path.size() - 1) {
            return {0.0, 0.0, 0.0};  // No acceleration at endpoints
        }
        
        // Calculate velocity at current and next points
        auto vel_curr = calculateVelocityVector(path, idx, 1.0);  // Use unit magnitude
        auto vel_next = calculateVelocityVector(path, idx+1, 1.0);
        
        // Calculate acceleration (change in velocity over time)
        double dt = (time_intervals[idx] + time_intervals[idx-1]) / 2.0;  // Average time step
        
        if (dt > 0.001) {  // Avoid division by zero
            return (vel_next - vel_curr) / dt;
        }
        
        return {0.0, 0.0, 0.0};
    }
    
    geometry_msgs::msg::Point enforcePathConstraints(
        const geometry_msgs::msg::Point& point,
        const RobotConstraints& constraints) {
        
        geometry_msgs::msg::Point constrained_point = point;
        
        // Apply position constraints
        constrained_point.x = std::clamp(constrained_point.x, 
                                       constraints.min_position.x, 
                                       constraints.max_position.x);
        constrained_point.y = std::clamp(constrained_point.y, 
                                       constraints.min_position.y, 
                                       constraints.max_position.y);
        constrained_point.z = std::clamp(constrained_point.z, 
                                       constraints.min_position.z, 
                                       constraints.max_position.z);
        
        return constrained_point;
    }
    
    std::vector<geometry_msgs::msg::Point> environment_obstacles_;
    
    static constexpr double OBSTACLE_AVOIDANCE_RADIUS = 1.0;
    static constexpr double OBSTACLE_REPULSION_STRENGTH = 5.0;
};
```

### Dynamic Trajectory Replanning

#### Reactive Trajectory Adjustment
```cpp
class DynamicTrajectoryAdjuster {
private:
    std::vector<TrajectoryPoint> current_trajectory_;
    std::vector<TrajectoryPoint> planned_trajectory_;
    std::unique_ptr<PathPlanner> replanner_;
    std::unique_ptr<CollisionDetector> collision_detector_;
    std::unique_ptr<TrajectoryOptimizer> optimizer_;
    
    // Replanning triggers
    double collision_detection_distance_;
    double replanning_frequency_;
    rclcpp::Time last_replanning_time_;
    
    // Trajectory tracking state
    size_t current_waypoint_idx_;
    double tracking_error_threshold_;
    bool replanning_needed_;

public:
    DynamicTrajectoryAdjuster(double collision_dist = 0.5, double replan_freq = 0.5)
        : collision_detection_distance_(collision_dist), 
          replanning_frequency_(replan_freq), 
          tracking_error_threshold_(0.2), 
          replanning_needed_(false) {
        
        replanner_ = std::make_unique<DynamicReplanner>();
        collision_detector_ = std::make_unique<CollisionDetector>();
        optimizer_ = std::make_unique<TrajectoryOptimizer>();
    }
    
    std::vector<TrajectoryPoint> adjustTrajectory(
        const RobotState& current_state,
        const std::vector<geometry_msgs::msg::Point>& obstacles,
        const geometry_msgs::msg::Point& goal) {
        
        // Check if replanning is needed
        if (needsReplanning(current_state, obstacles)) {
            // Generate new path avoiding obstacles
            auto new_path = replanner_->planPath(current_state.pose.position, goal, obstacles);
            
            // Convert path to trajectory
            auto new_trajectory = pathToTrajectoryConverter_->convertPathToTrajectory(
                new_path, getRobotConstraints(), ESTIMATED_EXECUTION_TIME);
            
            // Update current trajectory
            current_trajectory_ = new_trajectory;
            last_replanning_time_ = this->now();
        }
        
        // Update current trajectory based on tracking performance
        auto updated_trajectory = updateTrajectoryBasedOnTracking(
            current_trajectory_, current_state);
        
        return updated_trajectory;
    }

private:
    bool needsReplanning(const RobotState& current_state,
                       const std::vector<geometry_msgs::msg::Point>& obstacles) {
        
        // Check if sufficient time has passed since last replanning
        if ((this->now() - last_replanning_time_).seconds() < 1.0/replanning_frequency_) {
            return false;
        }
        
        // Check if there are new obstacles in the path
        for (const auto& obstacle : obstacles) {
            double distance_to_robot = std::sqrt(
                std::pow(obstacle.x - current_state.pose.position.x, 2) +
                std::pow(obstacle.y - current_state.pose.position.y, 2) +
                std::pow(obstacle.z - current_state.pose.position.z, 2));
            
            if (distance_to_robot < collision_detection_distance_) {
                // Check if obstacle is in path of planned trajectory
                for (size_t i = current_waypoint_idx_; i < planned_trajectory_.size(); i++) {
                    double distance_to_path = std::sqrt(
                        std::pow(obstacle.x - planned_trajectory_[i].position.x, 2) +
                        std::pow(obstacle.y - planned_trajectory_[i].position.y, 2) +
                        std::pow(obstacle.z - planned_trajectory_[i].position.z, 2));
                    
                    if (distance_to_path < collision_detection_distance_) {
                        return true;  // Obstacle in planned path
                    }
                }
            }
        }
        
        // Check if tracking error is too high
        double error_magnitude = calculateTrackingError(current_state);
        if (error_magnitude > tracking_error_threshold_) {
            return true;  // Significant deviation from planned trajectory
        }
        
        // Check if goal has changed significantly
        if (hasGoalChangedSignificantly()) {
            return true;
        }
        
        return false;  // No replanning needed
    }
    
    std::vector<TrajectoryPoint> updateTrajectoryBasedOnTracking(
        const std::vector<TrajectoryPoint>& original_trajectory,
        const RobotState& current_state) {
        
        std::vector<TrajectoryPoint> updated_trajectory = original_trajectory;
        
        // Adjust trajectory based on current tracking performance
        // This could involve: 
        // - Adjusting timing based on actual progress
        // - Modifying path based on environmental changes
        // - Smoothing trajectory based on tracking errors
        
        size_t current_idx = findClosestTrajectoryPoint(current_state.pose.position);
        
        if (current_idx < updated_trajectory.size()) {
            // Calculate tracking error at current point
            double dx = updated_trajectory[current_idx].position.x - current_state.pose.position.x;
            double dy = updated_trajectory[current_idx].position.y - current_state.pose.position.y;
            double dz = updated_trajectory[current_idx].position.z - current_state.pose.position.z;
            
            double tracking_error = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            // If tracking error is significant, adjust future trajectory points
            if (tracking_error > POSITION_ERROR_THRESHOLD) {
                for (size_t i = current_idx + 1; i < updated_trajectory.size(); i++) {
                    // Apply correction based on current error
                    double correction_factor = std::exp(-(i - current_idx) * CORRECTION_DECAY);
                    updated_trajectory[i].position.x -= dx * correction_factor * CORRECTION_GAIN;
                    updated_trajectory[i].position.y -= dy * correction_factor * CORRECTION_GAIN;
                    updated_trajectory[i].position.z -= dz * correction_factor * CORRECTION_GAIN;
                }
            }
        }
        
        return updated_trajectory;
    }
    
    size_t findClosestTrajectoryPoint(const geometry_msgs::msg::Point& current_position) {
        size_t closest_idx = 0;
        double min_distance = std::numeric_limits<double>::max();
        
        for (size_t i = current_waypoint_idx_; i < current_trajectory_.size(); i++) {
            double dx = current_trajectory_[i].position.x - current_position.x;
            double dy = current_trajectory_[i].position.y - current_position.y;
            double dz = current_trajectory_[i].position.z - current_position.z;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            if (distance < min_distance) {
                min_distance = distance;
                closest_idx = i;
            }
        }
        
        return closest_idx;
    }
    
    double calculateTrackingError(const RobotState& current_state) {
        size_t current_idx = findClosestTrajectoryPoint(current_state.pose.position);
        
        if (current_idx < current_trajectory_.size()) {
            double dx = current_trajectory_[current_idx].position.x - current_state.pose.position.x;
            double dy = current_trajectory_[current_idx].position.y - current_state.pose.position.y;
            double dz = current_trajectory_[current_idx].position.z - current_state.pose.position.z;
            
            return std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        
        return 0.0;  // Default if trajectory is empty
    }
    
    bool hasGoalChangedSignificantly() {
        // Implementation would check if goal has changed significantly
        // since the current trajectory was planned
        return false;  // Placeholder
    }
    
    struct RobotConstraints {
        geometry_msgs::msg::Point max_velocity;
        geometry_msgs::msg::Point max_acceleration;
        geometry_msgs::msg::Point max_jerk;
        geometry_msgs::msg::Point min_position;
        geometry_msgs::msg::Point max_position;
    };
    
    static constexpr double POSITION_ERROR_THRESHOLD = 0.5;  // meters
    static constexpr double CORRECTION_GAIN = 0.1;
    static constexpr double CORRECTION_DECAY = 0.05;
    static constexpr double ESTIMATED_EXECUTION_TIME = 60.0;  // seconds
};
```

## Performance Optimization

### Real-time Performance Considerations

#### Computational Efficiency
- **Algorithm Selection**: Choose algorithms appropriate for real-time requirements
- **Optimization**: Optimize critical path algorithms for speed
- **Parallel Processing**: Use multi-threading where appropriate
- **Hardware Acceleration**: Leverage GPUs for computationally intensive tasks

#### Memory Management
- **Pre-allocated Buffers**: Avoid dynamic allocation in real-time loops
- **Memory Pools**: Reuse memory for frequent allocations
- **Cache Optimization**: Organize data for efficient cache access
- **Efficient Data Structures**: Use appropriate data structures for performance

#### Example: Efficient Trajectory Tracking
```cpp
class RealTimeTrajectoryTracker {
private:
    // Pre-allocated memory for efficiency
    std::vector<double> position_errors_buffer_;
    std::vector<double> velocity_errors_buffer_;
    std::vector<double> control_outputs_buffer_;
    std::vector<TrajectoryPoint> pre_allocated_trajectory_points_;
    
    // Fixed-size sliding window for performance tracking
    std::vector<double> tracking_error_history_;
    std::vector<double> execution_time_history_;
    size_t max_history_size_;
    size_t history_index_;
    
    // Optimized PID controllers (with pre-computed coefficients)
    struct OptimizedPID {
        double kp, ki, kd;
        double last_error;
        double integral;
        double output;
        double min_output, max_output;
        
        double compute(double error, double dt) {
            // Proportional term
            double p_term = kp * error;
            
            // Integral term (with anti-windup)
            integral += error * dt;
            integral = std::clamp(integral, min_output/kd, max_output/kd);
            double i_term = ki * integral;
            
            // Derivative term
            double derivative = (error - last_error) / dt;
            double d_term = kd * derivative;
            
            // Total output
            output = std::clamp(p_term + i_term + d_term, min_output, max_output);
            
            last_error = error;
            
            return output;
        }
    };
    
    std::vector<OptimizedPID> optimized_pid_controllers_;
    
    // Performance metrics
    double avg_execution_time_;
    double max_execution_time_;
    double min_execution_time_;
    double execution_time_variance_;

public:
    RealTimeTrajectoryTracker(size_t max_history = 1000) 
        : max_history_size_(max_history), history_index_(0),
          avg_execution_time_(0.0), max_execution_time_(0.0), 
          min_execution_time_(std::numeric_limits<double>::max()),
          execution_time_variance_(0.0) {
        
        // Pre-allocate all necessary buffers
        position_errors_buffer_.resize(NUM_ROBOT_DOF, 0.0);
        velocity_errors_buffer_.resize(NUM_ROBOT_DOF, 0.0);
        control_outputs_buffer_.resize(NUM_ROBOT_DOF, 0.0);
        tracking_error_history_.resize(max_history_size_);
        execution_time_history_.resize(max_history_size_);
        
        // Initialize PID controllers with pre-allocated memory
        for (size_t i = 0; i < NUM_ROBOT_DOF; i++) {
            OptimizedPID pid;
            pid.kp = 100.0;
            pid.ki = 10.0;
            pid.kd = 15.0;
            pid.last_error = 0.0;
            pid.integral = 0.0;
            pid.output = 0.0;
            pid.min_output = -MAX_JOINT_EFFORT;
            pid.max_output = MAX_JOINT_EFFORT;
            
            optimized_pid_controllers_.push_back(pid);
        }
    }
    
    ControlCommand trackTrajectoryRealTime(const RobotState& current_state,
                                          const rclcpp::Time& current_time,
                                          double dt) {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Get reference state for current time
        auto reference_state = getReferenceState(current_time);
        
        // Calculate errors efficiently
        for (size_t i = 0; i < NUM_ROBOT_DOF; i++) {
            position_errors_buffer_[i] = reference_state.position[i] - current_state.position[i];
            velocity_errors_buffer_[i] = reference_state.velocity[i] - current_state.velocity[i];
        }
        
        // Compute control outputs
        for (size_t i = 0; i < NUM_ROBOT_DOF; i++) {
            control_outputs_buffer_[i] = optimized_pid_controllers_[i].compute(
                position_errors_buffer_[i], dt);
        }
        
        // Package results
        ControlCommand control_cmd;
        control_cmd.efforts = control_outputs_buffer_;
        control_cmd.timestamp = current_time;
        
        // Track performance
        auto end_time = std::chrono::high_resolution_clock::now();
        double execution_time = std::chrono::duration<double, std::milli>(
            end_time - start_time).count();
        
        updatePerformanceMetrics(execution_time);
        
        return control_cmd;
    }
    
    void updatePerformanceMetrics(double execution_time_ms) {
        // Update execution time history
        execution_time_history_[history_index_] = execution_time_ms;
        
        // Update min/max tracking
        max_execution_time_ = std::max(max_execution_time_, execution_time_ms);
        min_execution_time_ = std::min(min_execution_time_, execution_time_ms);
        
        // Update average
        if (tracking_performance_count_ == 0) {
            avg_execution_time_ = execution_time_ms;
        } else {
            avg_execution_time_ = (avg_execution_time_ * tracking_performance_count_ + execution_time_ms) / 
                                 (tracking_performance_count_ + 1);
        }
        tracking_performance_count_++;
        
        // Update index for circular buffer
        history_index_ = (history_index_ + 1) % max_history_size_;
        
        // Check if we're meeting real-time requirements
        if (execution_time_ms > MAX_ALLOWED_EXECUTION_TIME) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                1000,  // 1 second throttle
                "Trajectory tracking exceeded maximum execution time: %f ms (max: %f ms)",
                execution_time_ms, MAX_ALLOWED_EXECUTION_TIME);
        }
    }
    
    bool isMeetingRealTimeRequirements() const {
        return avg_execution_time_ < TARGET_REAL_TIME_EXECUTION_MS;
    }
    
    double getAverageExecutionTime() const {
        return avg_execution_time_;
    }
    
    double getMaxExecutionTime() const {
        return max_execution_time_;
    }

private:
    size_t tracking_performance_count_ = 0;
    
    static constexpr size_t NUM_ROBOT_DOF = 6;
    static constexpr double MAX_JOINT_EFFORT = 100.0;
    static constexpr double TARGET_REAL_TIME_EXECUTION_MS = 5.0;  // 5ms for real-time control
    static constexpr double MAX_ALLOWED_EXECUTION_TIME = 10.0;     // 10ms maximum allowed
};
```

## Safety Considerations

### Safe Trajectory Execution

#### Safety Architecture
```cpp
class SafeTrajectoryExecutor {
private:
    std::unique_ptr<TrajectoryTracker> trajectory_tracker_;
    std::unique_ptr<SafetyMonitor> safety_monitor_;
    std::unique_ptr<CollisionDetector> collision_detector_;
    std::unique_ptr<EmergencyStop> emergency_stop_;
    
    // Safety constraints
    std::vector<SafetyConstraint> safety_constraints_;
    double safety_margin_;
    double max_velocity_limit_;
    double max_acceleration_limit_;
    
    // Trajectory validation
    std::unique_ptr<TrajectoryValidator> trajectory_validator_;

public:
    SafeTrajectoryExecutor(double safety_margin = 0.3) 
        : safety_margin_(safety_margin), 
          max_velocity_limit_(1.0), max_acceleration_limit_(2.0) {
        
        trajectory_tracker_ = std::make_unique<TrajectoryTracker>();
        safety_monitor_ = std::make_unique<SafetyMonitor>();
        collision_detector_ = std::make_unique<CollisionDetector>();
        emergency_stop_ = std::make_unique<EmergencyStopSystem>();
        trajectory_validator_ = std::make_unique<TrajectoryValidator>();
        
        // Initialize safety constraints
        initializeSafetyConstraints();
    }
    
    ControlCommand executeSafeTrajectory(const RobotState& current_state,
                                        const std::vector<TrajectoryPoint>& trajectory,
                                        const rclcpp::Time& current_time) {
        
        // Validate trajectory against safety constraints
        auto validation_result = trajectory_validator_->validate(trajectory);
        if (!validation_result.safe) {
            RCLCPP_ERROR(this->get_logger(), 
                        "Trajectory validation failed: %s", 
                        validation_result.reason.c_str());
            
            // Return safe stop command
            return generateSafeStopCommand();
        }
        
        // Check for immediate safety violations
        auto safety_status = safety_monitor_->check(current_state);
        if (!safety_status.safe) {
            RCLCPP_WARN(this->get_logger(), 
                       "Immediate safety violation detected, triggering safety response");
            
            return generateSafetyResponse(current_state, safety_status);
        }
        
        // Check for potential collisions along trajectory
        auto collision_risk = checkTrajectoryCollisionRisk(current_state, trajectory);
        if (collision_risk.high_risk) {
            RCLCPP_WARN(this->get_logger(), 
                       "High collision risk detected in trajectory, adjusting execution");
            
            return generateAvoidanceCommand(current_state, trajectory, collision_risk);
        }
        
        // Execute normal trajectory tracking
        auto normal_command = trajectory_tracker_->trackTrajectory(current_state, current_time);
        
        // Apply safety limits to the command
        auto safe_command = applySafetyLimits(normal_command, current_state);
        
        return safe_command;
    }
    
    void initializeSafetyConstraints() {
        // Define safety constraints for different aspects of robot operation
        safety_constraints_.push_back(CollisionConstraint(safety_margin_));
        safety_constraints_.push_back(VelocityConstraint(max_velocity_limit_));
        safety_constraints_.push_back(AccelerationConstraint(max_acceleration_limit_));
        safety_constraints_.push_back(JointLimitConstraint());
        safety_constraints_.push_back(PowerConstraint());
        safety_constraints_.push_back(HumanSafetyConstraint());
    }

private:
    struct ValidationResult {
        bool safe;
        std::string reason;
        std::vector<SafetyViolation> violations;
    };
    
    struct CollisionRisk {
        bool high_risk;
        double min_distance;
        geometry_msgs::msg::Point nearest_obstacle;
        size_t nearest_waypoint_idx;
    };
    
    ValidationResult validateTrajectory(const std::vector<TrajectoryPoint>& trajectory) {
        ValidationResult result;
        result.safe = true;
        result.violations.clear();
        
        for (size_t i = 0; i < trajectory.size(); i++) {
            // Check velocity constraints
            if (trajectory[i].velocity.norm() > max_velocity_limit_) {
                result.safe = false;
                result.reason = "Trajectory exceeds maximum velocity limits";
                SafetyViolation violation;
                violation.type = "VELOCITY_EXCEEDED";
                violation.details = "Waypoint " + std::to_string(i) + " velocity exceeds limit";
                violation.timestamp = trajectory[i].timestamp;
                result.violations.push_back(violation);
            }
            
            // Check acceleration constraints
            if (i > 0) {
                Eigen::Vector3d acceleration = 
                    (trajectory[i].velocity - trajectory[i-1].velocity) / 
                    std::max(0.01, (trajectory[i].timestamp - trajectory[i-1].timestamp).seconds());
                
                if (acceleration.norm() > max_acceleration_limit_) {
                    result.safe = false;
                    result.reason = "Trajectory exceeds maximum acceleration limits";
                    SafetyViolation violation;
                    violation.type = "ACCELERATION_EXCEEDED";
                    violation.details = "Waypoint " + std::to_string(i) + " acceleration exceeds limit";
                    violation.timestamp = trajectory[i].timestamp;
                    result.violations.push_back(violation);
                }
            }
            
            // Check joint limit constraints (if applicable)
            // This would require inverse kinematics to determine joint positions
        }
        
        return result;
    }
    
    CollisionRisk checkTrajectoryCollisionRisk(const RobotState& current_state,
                                             const std::vector<TrajectoryPoint>& trajectory) {
        CollisionRisk risk;
        risk.high_risk = false;
        risk.min_distance = std::numeric_limits<double>::max();
        
        auto current_position = current_state.pose.position;
        auto obstacles = collision_detector_->getEnvironmentObstacles();
        
        // Check upcoming trajectory points for collision risk
        for (size_t i = current_waypoint_idx_; i < trajectory.size(); i++) {
            for (const auto& obstacle : obstacles) {
                double distance = std::sqrt(
                    std::pow(trajectory[i].position.x - obstacle.x, 2) +
                    std::pow(trajectory[i].position.y - obstacle.y, 2) +
                    std::pow(trajectory[i].position.z - obstacle.z, 2));
                
                if (distance < risk.min_distance) {
                    risk.min_distance = distance;
                    risk.nearest_obstacle = obstacle;
                    risk.nearest_waypoint_idx = i;
                    
                    if (distance < safety_margin_) {
                        risk.high_risk = true;
                        return risk;  // Return immediately if high risk detected
                    }
                }
            }
        }
        
        return risk;
    }
    
    ControlCommand generateAvoidanceCommand(const RobotState& current_state,
                                          const std::vector<TrajectoryPoint>& trajectory,
                                          const CollisionRisk& risk) {
        // Generate command that avoids the detected obstacle
        ControlCommand avoidance_cmd;
        
        // Calculate avoidance direction (away from nearest obstacle)
        double dx = current_state.pose.position.x - risk.nearest_obstacle.x;
        double dy = current_state.pose.position.y - risk.nearest_obstacle.y;
        double dz = current_state.pose.position.z - risk.nearest_obstacle.z;
        
        double norm = std::sqrt(dx*dx + dy*dy + dz*dz);
        if (norm > 0.01) {  // Avoid division by zero
            // Normalize direction away from obstacle
            dx /= norm;
            dy /= norm;
            dz /= norm;
            
            // Scale by safety factor
            avoidance_cmd.linear_velocity.x = dx * AVOIDANCE_VELOCITY_FACTOR;
            avoidance_cmd.linear_velocity.y = dy * AVOIDANCE_VELOCITY_FACTOR;
            avoidance_cmd.linear_velocity.z = dz * AVOIDANCE_VELOCITY_FACTOR;
        }
        
        return avoidance_cmd;
    }
    
    ControlCommand applySafetyLimits(const ControlCommand& command,
                                   const RobotState& state) {
        ControlCommand limited_command = command;
        
        // Apply velocity limits
        double vel_magnitude = std::sqrt(
            command.linear_velocity.x * command.linear_velocity.x +
            command.linear_velocity.y * command.linear_velocity.y +
            command.linear_velocity.z * command.linear_velocity.z);
        
        if (vel_magnitude > max_velocity_limit_) {
            double scale = max_velocity_limit_ / vel_magnitude;
            limited_command.linear_velocity.x *= scale;
            limited_command.linear_velocity.y *= scale;
            limited_command.linear_velocity.z *= scale;
        }
        
        // Apply effort limits
        for (size_t i = 0; i < limited_command.efforts.size(); i++) {
            limited_command.efforts[i] = std::clamp(
                limited_command.efforts[i],
                -MAX_JOINT_EFFORT,
                MAX_JOINT_EFFORT);
        }
        
        return limited_command;
    }
    
    ControlCommand generateSafeStopCommand() {
        ControlCommand stop_cmd;
        stop_cmd.linear_velocity = {0.0, 0.0, 0.0};
        stop_cmd.angular_velocity = {0.0, 0.0, 0.0};
        stop_cmd.efforts = std::vector<double>(NUM_ROBOT_DOF, 0.0);
        return stop_cmd;
    }
    
    ControlCommand generateSafetyResponse(const RobotState& current_state,
                                        const SafetyStatus& status) {
        // Generate appropriate safety response based on violation type
        if (status.violation_type == "COLLISION_IMMEDIATE") {
            return generateEmergencyStop();
        } else if (status.violation_type == "JOINT_LIMIT_APPROACHING") {
            return generateJointLimitAvoidance(current_state);
        } else if (status.violation_type == "VELOCITY_EXCEEDED") {
            return generateVelocityReduction();
        } else {
            return generateSafeStopCommand();
        }
    }
    
    ControlCommand generateEmergencyStop() {
        // Trigger emergency stop mechanism
        emergency_stop_->activate();
        return generateSafeStopCommand();
    }
    
    struct SafetyConstraint {
        virtual bool check(const RobotState& state, const std::vector<TrajectoryPoint>& trajectory) = 0;
        virtual std::string getViolationReason() = 0;
    };
    
    struct CollisionConstraint : public SafetyConstraint {
        double min_distance;
        
        CollisionConstraint(double distance) : min_distance(distance) {}
        
        bool check(const RobotState& state, const std::vector<TrajectoryPoint>& trajectory) override {
            // Implementation would check for collisions
            return true;  // Placeholder
        }
        
        std::string getViolationReason() override {
            return "Collision detected";
        }
    };
    
    struct SafetyViolation {
        std::string type;
        std::string details;
        rclcpp::Time timestamp;
    };
    
    static constexpr double AVOIDANCE_VELOCITY_FACTOR = 0.3;  // m/s
    static constexpr double MAX_JOINT_EFFORT = 100.0;        // N-m
    static constexpr int NUM_ROBOT_DOF = 6;
};
```

## Troubleshooting Common Issues

### Trajectory Generation Issues

#### Discontinuous Trajectories
- **Symptoms**: Robot motion jerks between waypoints, high accelerations
- **Causes**: Poor interpolation, insufficient derivatives, velocity discontinuities
- **Solutions**: Use higher-order polynomials, ensure C² continuity, add intermediate points

#### Overshooting
- **Symptoms**: Robot passes target position before settling
- **Causes**: High control gains, inadequate feedforward, dynamic mismatches
- **Solutions**: Reduce gains, add feedforward control, improve dynamic model

#### Drifting Trajectories
- **Symptoms**: Robot gradually deviates from planned path
- **Causes**: Inadequate feedback, sensor drift, model inaccuracies
- **Solutions**: Improve feedback control, add integral action, sensor fusion

### Execution Issues

#### Timing Problems
- **Symptoms**: Irregular execution, missed control deadlines
- **Causes**: Inadequate computational resources, poor scheduling, blocking operations
- **Solutions**: Optimize algorithms, improve scheduling, use real-time OS

#### Instability
- **Symptoms**: Oscillatory behavior, divergence from trajectory
- **Causes**: High gains, sensor noise, actuator delays
- **Solutions**: Retune controllers, add filtering, reduce gains

#### Poor Tracking Performance
- **Symptoms**: Large tracking errors, inability to follow trajectory
- **Causes**: Inadequate control authority, dynamic constraints, model errors
- **Solutions**: Increase control authority, add feedforward, improve models

### Integration Issues

#### Perception-Action Timing
- **Symptoms**: Delays between perception and action, inconsistent behavior
- **Causes**: Processing delays, communication delays, buffering
- **Solutions**: Pipeline processing, reduce computation, optimize communication

#### State Estimation Problems
- **Symptoms**: Inaccurate state estimates, poor trajectory tracking
- **Causes**: Sensor noise, model errors, integration drift
- **Solutions**: Sensor fusion, Kalman filtering, proper calibration

## Best Practices

### Trajectory Design Best Practices

#### Smoothness Requirements
- **Continuity**: Ensure C² continuity for smooth motion
- **Bounded Acceleration**: Limit acceleration to prevent jerky motion
- **Velocity Profiles**: Use smooth velocity profiles (trapezoidal, s-curve)
- **Dynamic Feasibility**: Verify trajectories respect robot dynamics

#### Safety Considerations
- **Conservative Planning**: Plan with safety margins
- **Obstacle Avoidance**: Include adequate obstacle clearance
- **Emergency Stops**: Design trajectories with emergency stop capabilities
- **Human Safety**: Consider human safety zones and behavior

### Performance Best Practices

#### Computational Efficiency
- **Pre-computation**: Pre-compute trajectory parameters when possible
- **Optimized Algorithms**: Use efficient algorithms for real-time execution
- **Memory Management**: Minimize dynamic allocation in critical loops
- **Parallel Processing**: Use multi-threading for non-critical computations

#### Validation and Testing
- **Simulation First**: Test trajectories in simulation before real execution
- **Gradual Deployment**: Start with simple trajectories, increase complexity
- **Continuous Monitoring**: Monitor trajectory execution in real-time
- **Performance Metrics**: Track execution performance metrics

## Future Developments

### Advanced Trajectory Generation

#### Learning-Based Trajectory Generation
- **Neural Networks**: Using neural networks for trajectory generation
- **Reinforcement Learning**: Learning optimal trajectories through interaction
- **Imitation Learning**: Learning from expert demonstrations
- **Generative Models**: Creating diverse trajectories for different scenarios

#### AI-Enhanced Control
- **Adaptive Control**: Controllers that adjust parameters based on performance
- **Predictive Control**: Using AI predictions for proactive control
- **Multi-Modal Control**: Combining multiple sensory modalities
- **Human-Robot Collaboration**: AI for collaborative control

### Integration Approaches

#### AI-Physical Integration
- **Direct Perception-Action**: Eliminating intermediate representations
- **Continuous Learning**: Robots that learn continuously from experience
- **Adaptive Behavior**: Behavior that adapts to changing conditions
- **Self-Improvement**: Systems that optimize themselves over time

#### Human-Robot Integration
- **Natural Interaction**: More intuitive human-robot interfaces
- **Collaborative Learning**: Robots learning from human demonstrations
- **Shared Autonomy**: Humans and robots sharing control authority
- **Social Integration**: Robots as social actors in human environments

## Conclusion

Trajectory generation and execution form the backbone of effective Physical AI systems, enabling robots to move precisely and safely through their environments. The integration of sophisticated generation algorithms with robust execution frameworks ensures that robots can follow complex paths while respecting their physical constraints and safety requirements.

Modern trajectory systems must balance multiple competing objectives: smoothness, efficiency, safety, and real-time performance. The success of these systems depends on careful attention to the mathematical foundations, computational efficiency, and safety considerations that govern physical robot operation.

As robotics systems become more sophisticated and operate in more diverse environments, the importance of advanced trajectory generation and execution techniques continues to grow. These systems must handle the challenges of real-world operation while providing the sophisticated behaviors needed for effective human-robot interaction and autonomous operation.

Understanding these techniques and their implementation is essential for creating Physical AI systems that can operate effectively in the real world, bridging the gap between computational planning and physical execution.

## Exercises

1. Implement a complete trajectory tracking system for a robot arm that includes path planning, trajectory generation, and PID control with feedforward compensation.
2. Design and implement a trajectory optimization system that balances smoothness, obstacle avoidance, and execution efficiency.
3. Create a safe trajectory execution system that validates trajectories against safety constraints before execution and handles dynamic replanning.

## Further Reading

- Siciliano, B., & Khatib, O. (Eds.). (2016). "Springer Handbook of Robotics." Springer.
- Spong, M.W., Hutchinson, S., & Vidyasagar, M. (2006). "Robot Modeling and Control." Wiley.
- LaValle, S. M. (2006). "Planning Algorithms." Cambridge University Press.
- Kelly, A. (2006). "Mobile Robotics: Mathematics, Models, and Methods." Cambridge University Press.
- Corke, P. (2017). "Robotics, Vision and Control: Fundamental Algorithms in MATLAB." Springer.
- Research Papers: "Trajectory Optimization for Robotics" and "Safe Motion Planning in Dynamic Environments."