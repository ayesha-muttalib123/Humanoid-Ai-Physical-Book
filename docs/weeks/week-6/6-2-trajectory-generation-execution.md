---
sidebar_label: Trajectory Generation and Execution
title: Trajectory Generation and Execution - Planning and Executing Robot Motion
description: Understanding trajectory generation and execution for robotics applications including path planning, motion control, and smooth motion execution
keywords: [trajectory, path planning, motion control, robotics, control, interpolation, spline, trajectory execution]
---

# 6.2 Trajectory Generation and Execution

## Introduction

Trajectory generation is a fundamental aspect of robotics that involves planning smooth, feasible paths for robots to follow while respecting kinematic and dynamic constraints. The trajectory represents the time-parameterized path that specifies position, velocity, and acceleration at each point in time. Proper trajectory generation and execution are essential for smooth, safe, and efficient robot operation.

In Physical AI systems, trajectory generation bridges the gap between high-level planning and low-level control. It translates abstract goals from navigation or manipulation planners into specific motion commands that can be executed by the robot's control system. The quality of trajectory generation significantly impacts the robot's performance, safety, and ability to interact effectively with its environment.

This chapter explores the mathematical foundations of trajectory generation, various generation techniques, execution strategies, and integration with control systems. We'll cover both position-based trajectories for manipulators and velocity-based trajectories for mobile robots.

## Mathematical Foundations

### Interpolation Methods

#### Polynomial Interpolation

Polynomial interpolation is commonly used for trajectory generation due to its mathematical simplicity and smoothness properties.

##### Cubic Polynomials

Cubic polynomials provide position and velocity continuity (C¹) and are commonly used for single-joint trajectories:

x(t) = a₀ + a₁t + a₂t² + a₃t³

For a trajectory between two points with specified initial and final conditions:
- x(t₀) = x₀, x'(t₀) = v₀
- x(t₁) = x₁, x'(t₁) = v₁

The coefficients can be solved as:
- a₀ = x₀
- a₁ = v₀
- a₂ = (3(x₁ - x₀) - 2v₀(t₁ - t₀) - v₁(t₁ - t₀)) / (t₁ - t₀)²
- a₃ = (2(x₀ - x₁) + (v₀ + v₁)(t₁ - t₀)) / (t₁ - t₀)³

```cpp
class CubicTrajectorySegment {
private:
    double a0_, a1_, a2_, a3_;  // Polynomial coefficients
    double t0_, t1_;            // Start and end times
    double x0_, x1_;            // Start and end positions
    double v0_, v1_;            // Start and end velocities

public:
    CubicTrajectorySegment(double start_time, double end_time,
                          double start_pos, double end_pos,
                          double start_vel, double end_vel) 
        : t0_(start_time), t1_(end_time), x0_(start_pos), x1_(end_pos),
          v0_(start_vel), v1_(end_vel) {
        
        double dt = t1_ - t0_;
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
        return t1_ - t0_;
    }
    
    double getEndTime() const {
        return t1_;
    }
};
```

##### Quintic Polynomials

Quintic polynomials provide position, velocity, and acceleration continuity (C²), making them suitable for applications requiring smooth acceleration profiles:

x(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵

For boundary conditions:
- x(t₀) = x₀, x'(t₀) = v₀, x''(t₀) = a₀
- x(t₁) = x₁, x'(t₁) = v₁, x''(t₁) = a₁

The coefficients can be solved with the additional acceleration constraints.

```cpp
class QuinticTrajectorySegment {
private:
    double a0_, a1_, a2_, a3_, a4_, a5_;
    double t0_, t1_;
    double duration_;

public:
    QuinticTrajectorySegment(double start_time, double end_time,
                            double start_pos, double end_pos,
                            double start_vel, double end_vel,
                            double start_acc, double end_acc) 
        : t0_(start_time), t1_(end_time), duration_(end_time - start_time) {
        
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
    double total_duration_;
    
public:
    CubicSplineTrajectory(const std::vector<double>& waypoints,
                         const std::vector<double>& times,
                         const std::vector<double>& velocities = {}) {
        
        if (waypoints.size() != times.size()) {
            throw std::invalid_argument("Waypoints and times must have same size");
        }
        
        total_duration_ = 0.0;
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
            segment_times_[i + 1] = total_duration_ + (end_time - start_time);
            total_duration_ = segment_times_[i + 1];
        }
    }
    
    double getPosition(double t) const {
        if (t < 0 || t > total_duration_) {
            return 0.0;  // Or handle boundary conditions differently
        }
        
        // Find which segment we're in
        size_t seg_idx = findSegmentIndex(t);
        return segments_[seg_idx].getPosition(t);
    }
    
    double getVelocity(double t) const {
        if (t < 0 || t > total_duration_) {
            return 0.0;
        }
        
        size_t seg_idx = findSegmentIndex(t);
        return segments_[seg_idx].getVelocity(t);
    }
    
    double getAcceleration(double t) const {
        if (t < 0 || t > total_duration_) {
            return 0.0;
        }
        
        size_t seg_idx = findSegmentIndex(t);
        return segments_[seg_idx].getAcceleration(t);
    }

private:
    size_t findSegmentIndex(double t) const {
        for (size_t i = 0; i < segment_times_.size() - 1; i++) {
            if (t >= segment_times_[i] && t <= segment_times_[i + 1]) {
                return i;
            }
        }
        return segments_.size() - 1;  // Return last segment if time is at the end
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

### Path Parametrization

Path parametrization involves assigning time values to points along a geometric path:

#### Velocity-Profile Based Parametrization
```cpp
class PathParametrizer {
public:
    struct Waypoint {
        Eigen::Vector3d position;
        double time;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
    };
    
    std::vector<Waypoint> parametrizePath(const std::vector<Eigen::Vector3d>& path,
                                         double max_velocity,
                                         double max_acceleration,
                                         double start_velocity = 0.0,
                                         double end_velocity = 0.0) {
        
        std::vector<Waypoint> parametrized_path;
        if (path.size() < 2) return parametrized_path;
        
        parametrized_path.resize(path.size());
        
        // Set positions
        for (size_t i = 0; i < path.size(); i++) {
            parametrized_path[i].position = path[i];
        }
        
        // Calculate distances between consecutive points
        std::vector<double> distances(path.size() - 1);
        for (size_t i = 0; i < path.size() - 1; i++) {
            distances[i] = (path[i + 1] - path[i]).norm();
        }
        
        // Calculate velocity profile using trapezoidal velocity profile
        std::vector<double> velocities = calculateTrapezoidalVelocities(distances, 
                                                                      max_velocity, 
                                                                      max_acceleration,
                                                                      start_velocity, 
                                                                      end_velocity);
        
        // Calculate time stamps based on velocity profile
        std::vector<double> times = calculateTimeStamps(velocities, distances);
        
        // Assign times to waypoints
        for (size_t i = 0; i < parametrized_path.size(); i++) {
            parametrized_path[i].time = (i < times.size()) ? times[i] : times.back();
        }
        
        // Calculate accelerations
        calculateAccelerations(parametrized_path, distances);
        
        return parametrized_path;
    }

private:
    std::vector<double> calculateTrapezoidalVelocities(
        const std::vector<double>& distances,
        double max_vel, 
        double max_acc,
        double start_vel, 
        double end_vel) {
        
        std::vector<double> velocities(distances.size() + 1);
        velocities[0] = start_vel;
        
        // Calculate acceleration/deceleration distances
        double accel_dist = (max_vel * max_vel - start_vel * start_vel) / (2 * max_acc);
        double decel_dist = (max_vel * max_vel - end_vel * end_vel) / (2 * max_acc);
        
        // For each segment, calculate maximum possible velocity
        for (size_t i = 0; i < distances.size(); i++) {
            // Calculate if we can reach max velocity in this segment
            double total_required_dist = accel_dist + decel_dist;
            
            if (distances[i] >= total_required_dist) {
                // We can reach max velocity
                velocities[i + 1] = max_vel;
            } else {
                // We cannot reach max velocity - calculate achievable max
                double achievable_max_vel = std::sqrt(
                    (start_vel * start_vel + 2 * max_acc * distances[i] - end_vel * end_vel) / 2.0
                );
                velocities[i + 1] = std::min(achievable_max_vel, max_vel);
            }
        }
        
        return velocities;
    }
    
    std::vector<double> calculateTimeStamps(const std::vector<double>& velocities,
                                           const std::vector<double>& distances) {
        std::vector<double> times(velocities.size());
        times[0] = 0.0;
        
        for (size_t i = 0; i < distances.size(); i++) {
            // Approximate time for segment based on average velocity
            double avg_vel = (velocities[i] + velocities[i + 1]) / 2.0;
            if (avg_vel > 0.001) {  // Avoid division by zero
                double dt = distances[i] / avg_vel;
                times[i + 1] = times[i] + dt;
            } else {
                times[i + 1] = times[i];  // No motion
            }
        }
        
        return times;
    }
    
    void calculateAccelerations(std::vector<Waypoint>& path,
                               const std::vector<double>& distances) {
        // Calculate accelerations based on velocity changes
        for (size_t i = 1; i < path.size() - 1; i++) {
            double dt = path[i + 1].time - path[i - 1].time;
            if (dt > 0.001) {
                double dv = (path[i + 1].velocity.norm() - path[i - 1].velocity.norm());
                double acceleration_mag = dv / dt;
                
                // Calculate acceleration direction
                Eigen::Vector3d direction = (path[i + 1].position - path[i - 1].position).normalized();
                path[i].acceleration = acceleration_mag * direction;
            }
        }
        
        // Set boundary accelerations
        if (path.size() >= 2) {
            path[0].acceleration = Eigen::Vector3d::Zero();
            path.back().acceleration = Eigen::Vector3d::Zero();
        }
    }
};
```

## Trajectory Types and Applications

### Joint Space Trajectories

Joint space trajectories specify desired joint positions over time:

#### Point-to-Point Motion
```cpp
class JointSpaceTrajectory {
private:
    std::vector<std::vector<double>> joint_positions_;  // [time][joint]
    std::vector<std::vector<double>> joint_velocities_;
    std::vector<std::vector<double>> joint_accelerations_;
    std::vector<double> time_stamps_;
    
    std::vector<std::unique_ptr<CubicSplineTrajectory>> joint_trajectories_;
    
public:
    JointSpaceTrajectory() = default;
    
    void generatePointToPointTrajectory(
        const std::vector<double>& start_positions,
        const std::vector<double>& end_positions,
        double duration,
        double start_velocity = 0.0,
        double end_velocity = 0.0) {
        
        joint_trajectories_.clear();
        
        // Generate trajectory for each joint independently
        for (size_t i = 0; i < start_positions.size(); i++) {
            std::vector<double> waypoints = {start_positions[i], end_positions[i]};
            std::vector<double> times = {0.0, duration};
            std::vector<double> velocities = {start_velocity, end_velocity};
            
            joint_trajectories_.push_back(
                std::make_unique<CubicSplineTrajectory>(waypoints, times, velocities));
        }
        
        // Pre-sample trajectory for real-time execution
        double dt = 0.01;  // 10ms sampling
        int num_samples = static_cast<int>(duration / dt) + 1;
        
        joint_positions_.resize(num_samples);
        joint_velocities_.resize(num_samples);
        joint_accelerations_.resize(num_samples);
        time_stamps_.resize(num_samples);
        
        for (int i = 0; i < num_samples; i++) {
            double t = i * dt;
            time_stamps_[i] = t;
            
            joint_positions_[i].resize(start_positions.size());
            joint_velocities_[i].resize(start_positions.size());
            joint_accelerations_[i].resize(start_positions.size());
            
            for (size_t j = 0; j < start_positions.size(); j++) {
                joint_positions_[i][j] = joint_trajectories_[j]->getPosition(t);
                joint_velocities_[i][j] = joint_trajectories_[j]->getVelocity(t);
                joint_accelerations_[i][j] = joint_trajectories_[j]->getAcceleration(t);
            }
        }
    }
    
    std::vector<double> getJointPositions(double t) const {
        // Find the two closest samples
        auto it = std::lower_bound(time_stamps_.begin(), time_stamps_.end(), t);
        
        if (it == time_stamps_.end()) {
            // Return the last sample
            return joint_positions_.back();
        }
        
        size_t idx = std::distance(time_stamps_.begin(), it);
        
        if (idx == 0) {
            // Return the first sample
            return joint_positions_.front();
        }
        
        // Linear interpolation between samples
        double t1 = time_stamps_[idx - 1];
        double t2 = time_stamps_[idx];
        double alpha = (t - t1) / (t2 - t1);
        
        std::vector<double> interpolated_positions(joint_positions_[0].size());
        
        for (size_t i = 0; i < joint_positions_[0].size(); i++) {
            interpolated_positions[i] = 
                (1 - alpha) * joint_positions_[idx - 1][i] + 
                alpha * joint_positions_[idx][i];
        }
        
        return interpolated_positions;
    }
    
    std::vector<double> getJointVelocities(double t) const {
        // Similar interpolation for velocities
        auto it = std::lower_bound(time_stamps_.begin(), time_stamps_.end(), t);
        
        if (it == time_stamps_.end() || it == time_stamps_.begin()) {
            return (it == time_stamps_.end()) ? joint_velocities_.back() : joint_velocities_.front();
        }
        
        size_t idx = std::distance(time_stamps_.begin(), it);
        double t1 = time_stamps_[idx - 1];
        double t2 = time_stamps_[idx];
        double alpha = (t - t1) / (t2 - t1);
        
        std::vector<double> interpolated_velocities(joint_velocities_[0].size());
        
        for (size_t i = 0; i < joint_velocities_[0].size(); i++) {
            interpolated_velocities[i] = 
                (1 - alpha) * joint_velocities_[idx - 1][i] + 
                alpha * joint_velocities_[idx][i];
        }
        
        return interpolated_velocities;
    }
};
```

### Cartesian Space Trajectories

Cartesian trajectories specify end-effector position and orientation over time:

#### Cartesian Path Generation
```cpp
class CartesianTrajectory {
private:
    std::vector<Eigen::Vector3d> positions_;
    std::vector<Eigen::Quaterniond> orientations_;
    std::vector<double> time_stamps_;
    std::unique_ptr<BezierTrajectory> position_trajectory_;
    std::unique_ptr<SLERPInterpolator> orientation_trajectory_;
    
public:
    CartesianTrajectory() = default;
    
    void generateCartesianTrajectory(
        const std::vector<Eigen::Vector3d>& waypoints,
        const std::vector<Eigen::Quaterniond>& orientations,
        double duration) {
        
        // Generate position trajectory using Bezier curves
        position_trajectory_ = std::make_unique<BezierTrajectory>(waypoints, duration);
        
        // Generate orientation trajectory using SLERP
        orientations_ = orientations;
        
        // Pre-sample for real-time execution
        double dt = 0.01;  // 10ms sampling
        int num_samples = static_cast<int>(duration / dt) + 1;
        
        positions_.resize(num_samples);
        orientations_.resize(num_samples);
        time_stamps_.resize(num_samples);
        
        for (int i = 0; i < num_samples; i++) {
            double t = i * dt;
            time_stamps_[i] = t;
            
            positions_[i] = position_trajectory_->getPosition(t);
            
            // Interpolate orientation using SLERP
            double u = t / duration;
            orientations_[i] = interpolateOrientation(u);
        }
    }
    
    geometry_msgs::msg::PoseStamped getPose(double t) const {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = rclcpp::Clock().now();
        pose.header.frame_id = "base_link";
        
        // Find appropriate sample
        auto it = std::lower_bound(time_stamps_.begin(), time_stamps_.end(), t);
        size_t idx = std::distance(time_stamps_.begin(), it);
        
        if (idx == 0) {
            idx = 1;  // Ensure we have a valid index for interpolation
        } else if (idx >= time_stamps_.size()) {
            idx = time_stamps_.size() - 1;  // Use last sample
        }
        
        // Interpolate position and orientation
        double t1 = time_stamps_[idx - 1];
        double t2 = time_stamps_[idx];
        double alpha = (t - t1) / (t2 - t1);
        
        Eigen::Vector3d interp_pos = (1 - alpha) * positions_[idx - 1] + alpha * positions_[idx];
        
        // Convert to ROS message
        pose.pose.position.x = interp_pos.x();
        pose.pose.position.y = interp_pos.y();
        pose.pose.position.z = interp_pos.z();
        
        Eigen::Quaterniond quat = (1 - alpha) * orientations_[idx - 1] + alpha * orientations_[idx];
        quat.normalize();
        
        pose.pose.orientation.x = quat.x();
        pose.pose.orientation.y = quat.y();
        pose.pose.orientation.z = quat.z();
        pose.pose.orientation.w = quat.w();
        
        return pose;
    }

private:
    Eigen::Quaterniond interpolateOrientation(double u) const {
        if (orientations_.size() < 2) {
            return orientations_.empty() ? Eigen::Quaterniond::Identity() : orientations_[0];
        }
        
        // Find which segment we're in
        int segment = static_cast<int>(u * (orientations_.size() - 1));
        double local_u = (u * (orientations_.size() - 1)) - segment;
        
        if (segment >= orientations_.size() - 1) {
            return orientations_.back();
        }
        
        // SLERP between adjacent orientations
        return orientations_[segment].slerp(local_u, orientations_[segment + 1]);
    }
    
    // Helper class for SLERP interpolation
    class SLERPInterpolator {
    public:
        static Eigen::Quaterniond slerp(const Eigen::Quaterniond& q1,
                                      const Eigen::Quaterniond& q2,
                                      double t) {
            return q1.slerp(t, q2);
        }
    };
};
```

### Mobile Robot Trajectories

Mobile robot trajectories often specify velocity profiles rather than position profiles:

#### Velocity Trajectory Generation
```cpp
class MobileRobotTrajectory {
private:
    std::vector<double> linear_velocities_;
    std::vector<double> angular_velocities_;
    std::vector<double> time_stamps_;
    double wheel_separation_;
    double max_linear_vel_;
    double max_angular_vel_;
    
public:
    MobileRobotTrajectory(double wheel_sep, double max_lin, double max_ang)
        : wheel_separation_(wheel_sep), max_linear_vel_(max_lin), max_angular_vel_(max_ang) {}
    
    void generateVelocityTrajectory(
        const geometry_msgs::msg::PoseStamped& start_pose,
        const geometry_msgs::msg::PoseStamped& goal_pose,
        double duration) {
        
        // Calculate required linear and angular displacements
        double dx = goal_pose.pose.position.x - start_pose.pose.position.x;
        double dy = goal_pose.pose.position.y - start_pose.pose.position.y;
        double linear_dist = std::sqrt(dx*dx + dy*dy);
        
        // Calculate required rotation
        double start_yaw = getYawFromQuaternion(start_pose.pose.orientation);
        double goal_yaw = getYawFromQuaternion(goal_pose.pose.orientation);
        double angular_dist = normalizeAngle(goal_yaw - start_yaw);
        
        // Generate trapezoidal velocity profile
        generateTrapezoidalProfile(linear_dist, angular_dist, duration);
    }
    
    void generateArcTrajectory(
        const geometry_msgs::msg::PoseStamped& start_pose,
        const geometry_msgs::msg::PoseStamped& goal_pose,
        double arc_radius,
        double duration) {
        
        // Calculate arc parameters
        double dx = goal_pose.pose.position.x - start_pose.pose.position.x;
        double dy = goal_pose.pose.position.y - start_pose.pose.position.y;
        double linear_dist = std::sqrt(dx*dx + dy*dy);
        
        // Calculate required angular displacement for arc
        double angular_dist = linear_dist / arc_radius;  // s = rθ
        
        // Generate velocity profile for arc motion
        generateTrapezoidalProfile(linear_dist, angular_dist, duration);
    }
    
    std::pair<double, double> getVelocities(double t) const {
        if (time_stamps_.empty()) return {0.0, 0.0};
        
        // Find appropriate sample
        auto it = std::lower_bound(time_stamps_.begin(), time_stamps_.end(), t);
        size_t idx = std::distance(time_stamps_.begin(), it);
        
        if (idx == 0) {
            return {linear_velocities_[0], angular_velocities_[0]};
        } else if (idx >= time_stamps_.size()) {
            return {linear_velocities_.back(), angular_velocities_.back()};
        }
        
        // Interpolate between samples
        double t1 = time_stamps_[idx - 1];
        double t2 = time_stamps_[idx];
        double alpha = (t - t1) / (t2 - t1);
        
        double lin_vel = (1 - alpha) * linear_velocities_[idx - 1] + 
                         alpha * linear_velocities_[idx];
        double ang_vel = (1 - alpha) * angular_velocities_[idx - 1] + 
                         alpha * angular_velocities_[idx];
        
        return {lin_vel, ang_vel};
    }

private:
    void generateTrapezoidalProfile(double linear_dist, double angular_dist, double duration) {
        // Calculate required average velocities
        double avg_linear_vel = linear_dist / duration;
        double avg_angular_vel = angular_dist / duration;
        
        // Limit velocities to maximum values
        double target_linear_vel = std::min(avg_linear_vel * 1.5, max_linear_vel_);  // Allow for acceleration
        double target_angular_vel = std::min(avg_angular_vel * 1.5, max_angular_vel_);
        
        // Calculate acceleration time to reach target velocity
        double linear_accel_time = target_linear_vel / 1.0;  // Assume 1.0 m/s² acceleration
        double angular_accel_time = target_angular_vel / 1.0;  // Assume 1.0 rad/s² acceleration
        
        double accel_time = std::max(linear_accel_time, angular_accel_time);
        
        // Calculate if we can reach target velocity
        double linear_dist_at_target = target_linear_vel * (duration - 2 * accel_time);
        double angular_dist_at_target = target_angular_vel * (duration - 2 * accel_time);
        
        if (linear_dist_at_target < 0 || angular_dist_at_target < 0) {
            // We can't reach target velocity - adjust accordingly
            double reduced_duration = duration / 2.0;  // Use triangular profile instead
            target_linear_vel = linear_dist / reduced_duration;
            target_angular_vel = angular_dist / reduced_duration;
            accel_time = duration / 2.0;
        }
        
        // Sample trajectory at regular intervals
        double dt = 0.01;  // 10ms sampling
        int num_samples = static_cast<int>(duration / dt) + 1;
        
        linear_velocities_.resize(num_samples);
        angular_velocities_.resize(num_samples);
        time_stamps_.resize(num_samples);
        
        for (int i = 0; i < num_samples; i++) {
            double t = i * dt;
            time_stamps_[i] = t;
            
            // Generate trapezoidal velocity profile
            if (t < accel_time) {
                // Acceleration phase
                linear_velocities_[i] = (t / accel_time) * target_linear_vel;
                angular_velocities_[i] = (t / accel_time) * target_angular_vel;
            } else if (t < duration - accel_time) {
                // Constant velocity phase
                linear_velocities_[i] = target_linear_vel;
                angular_velocities_[i] = target_angular_vel;
            } else {
                // Deceleration phase
                double decel_t = t - (duration - accel_time);
                linear_velocities_[i] = target_linear_vel * (1 - decel_t / accel_time);
                angular_velocities_[i] = target_angular_vel * (1 - decel_t / accel_time);
            }
        }
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
};
```

## Trajectory Execution

### Trajectory Tracking Controllers

#### PID-Based Tracking
```cpp
class TrajectoryTracker {
private:
    std::unique_ptr<JointSpaceTrajectory> trajectory_;
    std::vector<PIDController> position_controllers_;
    std::vector<PIDController> velocity_controllers_;
    double tracking_dt_;
    
public:
    TrajectoryTracker(const std::vector<double>& kp, 
                     const std::vector<double>& ki, 
                     const std::vector<double>& kd,
                     double dt = 0.01)
        : tracking_dt_(dt) {
        
        // Initialize PID controllers for each joint
        for (size_t i = 0; i < kp.size(); i++) {
            position_controllers_.emplace_back(kp[i], ki[i], kd[i]);
            velocity_controllers_.emplace_back(kp[i]*0.1, ki[i]*0.1, kd[i]*0.1);  // Scaled gains for velocity
        }
    }
    
    std::vector<double> computeControlEfforts(
        const std::vector<double>& current_positions,
        const std::vector<double>& current_velocities,
        double current_time) {
        
        if (!trajectory_) {
            return std::vector<double>(current_positions.size(), 0.0);
        }
        
        // Get desired state from trajectory
        auto desired_positions = trajectory_->getJointPositions(current_time);
        auto desired_velocities = trajectory_->getJointVelocities(current_time);
        
        std::vector<double> control_efforts(current_positions.size());
        
        for (size_t i = 0; i < current_positions.size(); i++) {
            // Position error
            double pos_error = desired_positions[i] - current_positions[i];
            
            // Velocity error
            double vel_error = desired_velocities[i] - current_velocities[i];
            
            // Compute control effort using feedforward + feedback
            double pos_effort = position_controllers_[i].compute(pos_error, tracking_dt_);
            double vel_effort = velocity_controllers_[i].compute(vel_error, tracking_dt_);
            
            // Combine efforts (feedforward term would come from trajectory acceleration)
            control_efforts[i] = pos_effort + vel_effort;
        }
        
        return control_efforts;
    }
    
    void setTrajectory(std::unique_ptr<JointSpaceTrajectory> traj) {
        trajectory_ = std::move(traj);
    }
    
    bool isTrajectoryComplete(double current_time) const {
        if (!trajectory_) return true;
        
        // Check if current time is beyond trajectory duration
        return current_time >= trajectory_->getDuration();
    }
};
```

### Advanced Execution Strategies

#### Model Predictive Control for Trajectory Tracking
```cpp
class MPCBasedTrajectoryTracker {
private:
    Eigen::MatrixXd A_, B_, C_;  // System matrices
    Eigen::VectorXd Q_, R_;     // Cost matrices
    size_t prediction_horizon_;
    size_t control_horizon_;
    double dt_;
    
    // Current state
    Eigen::VectorXd state_;
    
    // Reference trajectory
    std::vector<Eigen::VectorXd> reference_trajectory_;
    
public:
    MPCBasedTrajectoryTracker(size_t pred_horizon, size_t ctrl_horizon, 
                            const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                            const Eigen::VectorXd& Q, const Eigen::VectorXd& R,
                            double dt)
        : prediction_horizon_(pred_horizon), control_horizon_(ctrl_horizon),
          A_(A), B_(B), Q_(Q), R_(R), dt_(dt) {
        
        state_ = Eigen::VectorXd::Zero(A_.rows());
    }
    
    std::vector<double> computeMPCControl(
        const std::vector<double>& current_state,
        const std::vector<std::vector<double>>& reference_traj,
        double current_time) {
        
        // Convert to Eigen format
        Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(
            current_state.data(), current_state.size());
        
        // Set current state
        state_ = x;
        
        // Build reference trajectory for prediction horizon
        std::vector<Eigen::VectorXd> local_ref;
        for (size_t i = 0; i < prediction_horizon_ && 
             (current_time + i) < reference_traj.size(); i++) {
            local_ref.push_back(Eigen::Map<const Eigen::VectorXd>(
                reference_traj[current_time + i].data(), 
                reference_traj[current_time + i].size()));
        }
        
        // Solve MPC optimization problem
        Eigen::VectorXd optimal_control = solveMPC(x, local_ref);
        
        // Return first control action (receding horizon)
        std::vector<double> control_output(optimal_control.size());
        for (int i = 0; i < optimal_control.size(); i++) {
            control_output[i] = optimal_control[i];
        }
        
        return control_output;
    }

private:
    Eigen::VectorXd solveMPC(const Eigen::VectorXd& current_state,
                            const std::vector<Eigen::VectorXd>& reference_traj) {
        
        // Build prediction matrices
        Eigen::MatrixXd PHI = buildPredictionMatrix();
        Eigen::MatrixXd GAMMA = buildControlMatrix();
        
        // Build reference trajectory vector
        Eigen::VectorXd ref_vec = buildReferenceVector(reference_traj);
        
        // Formulate quadratic programming problem
        // min 0.5 * u^T * H * u + f^T * u
        // subject to constraints
        
        Eigen::MatrixXd H = GAMMA.transpose() * buildQDiagonal() * GAMMA + 
                           buildRDiagonal();
        Eigen::VectorXd f = -(ref_vec - PHI * current_state).transpose() * 
                           buildQDiagonal() * GAMMA;
        
        // Solve optimization problem (simplified - would use actual QP solver)
        Eigen::VectorXd optimal_controls = H.ldlt().solve(f);
        
        // Return first control in sequence (receding horizon)
        Eigen::VectorXd first_controls = optimal_controls.head(B_.cols());
        
        return first_controls;
    }
    
    Eigen::MatrixXd buildPredictionMatrix() {
        // Build state prediction matrix over prediction horizon
        Eigen::MatrixXd PHI(A_.rows() * prediction_horizon_, A_.rows());
        
        Eigen::MatrixXd Ak = Eigen::MatrixXd::Identity(A_.rows(), A_.cols());
        for (size_t k = 0; k < prediction_horizon_; k++) {
            if (k > 0) {
                Ak = Ak * A_;
            }
            PHI.block(k * A_.rows(), 0, A_.rows(), A_.cols()) = Ak;
        }
        
        return PHI;
    }
    
    Eigen::MatrixXd buildControlMatrix() {
        // Build control-to-state mapping matrix
        Eigen::MatrixXd Gamma(A_.rows() * prediction_horizon_, 
                             B_.cols() * control_horizon_);
        
        for (size_t i = 0; i < prediction_horizon_; i++) {
            for (size_t j = 0; j < control_horizon_; j++) {
                if (i >= j) {
                    Eigen::MatrixXd Ak = matrixPower(A_, i - j);
                    Gamma.block(i * A_.rows(), j * B_.cols(), A_.rows(), B_.cols()) = 
                        Ak * B_;
                }
            }
        }
        
        return Gamma;
    }
    
    Eigen::MatrixXd buildQDiagonal() {
        // Build diagonal Q matrix repeated for prediction horizon
        Eigen::MatrixXd Q_diag = Eigen::MatrixXd::Zero(
            A_.rows() * prediction_horizon_, A_.rows() * prediction_horizon_);
        
        for (size_t i = 0; i < prediction_horizon_; i++) {
            Q_diag.block(i * A_.rows(), i * A_.rows(), A_.rows(), A_.cols()) = 
                Q_.asDiagonal();
        }
        
        return Q_diag;
    }
    
    Eigen::MatrixXd buildRDiagonal() {
        // Build diagonal R matrix repeated for control horizon
        Eigen::MatrixXd R_diag = Eigen::MatrixXd::Zero(
            B_.cols() * control_horizon_, B_.cols() * control_horizon_);
        
        for (size_t i = 0; i < control_horizon_; i++) {
            R_diag.block(i * B_.cols(), i * B_.cols(), B_.cols(), B_.cols()) = 
                R_.asDiagonal();
        }
        
        return R_diag;
    }
    
    Eigen::MatrixXd matrixPower(const Eigen::MatrixXd& A, int n) {
        if (n == 0) return Eigen::MatrixXd::Identity(A.rows(), A.cols());
        if (n == 1) return A;
        
        Eigen::MatrixXd result = A;
        for (int i = 1; i < n; i++) {
            result = result * A;
        }
        return result;
    }
    
    Eigen::VectorXd buildReferenceVector(const std::vector<Eigen::VectorXd>& ref_traj) {
        Eigen::VectorXd ref_vec(A_.rows() * prediction_horizon_);
        
        for (size_t i = 0; i < ref_traj.size() && i < prediction_horizon_; i++) {
            ref_vec.segment(i * A_.rows(), A_.rows()) = ref_traj[i];
        }
        
        // Fill remaining with last reference if needed
        for (size_t i = ref_traj.size(); i < prediction_horizon_; i++) {
            ref_vec.segment(i * A_.rows(), A_.rows()) = ref_traj.back();
        }
        
        return ref_vec;
    }
};
```

### Trajectory Monitoring and Safety

#### Safety Checks During Execution
```cpp
class TrajectorySafetyMonitor {
private:
    double position_tolerance_;
    double velocity_tolerance_;
    double effort_tolerance_;
    double max_deviation_from_path_;
    
    std::vector<double> joint_limits_min_;
    std::vector<double> joint_limits_max_;
    
    std::vector<double> previous_positions_;
    double max_velocity_change_rate_;  // Jerk limit
    
public:
    TrajectorySafetyMonitor(double pos_tol = 0.1, 
                          double vel_tol = 0.5, 
                          double eff_tol = 50.0,
                          double path_dev_tol = 0.5)
        : position_tolerance_(pos_tol), velocity_tolerance_(vel_tol),
          effort_tolerance_(eff_tol), max_deviation_from_path_(path_dev_tol),
          max_velocity_change_rate_(10.0) {}  // 10 rad/s² jerk limit
    
    bool isSafe(const std::vector<double>& current_positions,
                const std::vector<double>& current_velocities,
                const std::vector<double>& current_efforts,
                const std::vector<double>& desired_positions,
                const std::vector<double>& desired_velocities,
                double dt) {
        
        // Check position tracking accuracy
        for (size_t i = 0; i < current_positions.size(); i++) {
            double pos_error = std::abs(current_positions[i] - desired_positions[i]);
            if (pos_error > position_tolerance_) {
                RCLCPP_WARN(rclcpp::get_logger("trajectory_safety"), 
                           "Position error exceeded tolerance for joint %zu: %f > %f", 
                           i, pos_error, position_tolerance_);
                return false;
            }
        }
        
        // Check velocity limits
        for (size_t i = 0; i < current_velocities.size(); i++) {
            if (std::abs(current_velocities[i]) > velocity_tolerance_) {
                RCLCPP_WARN(rclcpp::get_logger("trajectory_safety"), 
                           "Velocity exceeded limit for joint %zu: %f > %f", 
                           i, std::abs(current_velocities[i]), velocity_tolerance_);
                return false;
            }
            
            // Check jerk limits (rate of velocity change)
            if (!previous_positions_.empty()) {
                double current_vel = current_velocities[i];
                double prev_vel = (current_positions[i] - previous_positions_[i]) / dt;
                double jerk = std::abs(current_vel - prev_vel) / dt;
                
                if (jerk > max_velocity_change_rate_) {
                    RCLCPP_WARN(rclcpp::get_logger("trajectory_safety"), 
                               "Jerk exceeded limit for joint %zu: %f > %f", 
                               i, jerk, max_velocity_change_rate_);
                    return false;
                }
            }
        }
        
        // Check effort limits
        for (size_t i = 0; i < current_efforts.size(); i++) {
            if (std::abs(current_efforts[i]) > effort_tolerance_) {
                RCLCPP_WARN(rclcpp::get_logger("trajectory_safety"), 
                           "Effort exceeded limit for joint %zu: %f > %f", 
                           i, std::abs(current_efforts[i]), effort_tolerance_);
                return false;
            }
        }
        
        // Check joint limits
        for (size_t i = 0; i < current_positions.size(); i++) {
            if (current_positions[i] < joint_limits_min_[i] || 
                current_positions[i] > joint_limits_max_[i]) {
                RCLCPP_WARN(rclcpp::get_logger("trajectory_safety"), 
                           "Joint %zu exceeded limits: %f not in [%f, %f]", 
                           i, current_positions[i], joint_limits_min_[i], joint_limits_max_[i]);
                return false;
            }
        }
        
        // Update previous positions for next iteration
        previous_positions_ = current_positions;
        
        return true;
    }
    
    void setJointLimits(const std::vector<double>& min_limits,
                       const std::vector<double>& max_limits) {
        joint_limits_min_ = min_limits;
        joint_limits_max_ = max_limits;
    }
    
    void setTolerances(double pos_tol, double vel_tol, double eff_tol) {
        position_tolerance_ = pos_tol;
        velocity_tolerance_ = vel_tol;
        effort_tolerance_ = eff_tol;
    }
};
```

## Integration with Physical AI Systems

### ROS 2 Trajectory Integration

#### Trajectory Message Types
```cpp
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "control_msgs/msg/follow_joint_trajectory_action.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class TrajectoryExecutionNode : public rclcpp::Node {
private:
    rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr action_server_;
    std::vector<std::string> joint_names_;
    std::vector<std::unique_ptr<TrajectoryTracker>> trackers_;
    std::unique_ptr<TrajectorySafetyMonitor> safety_monitor_;
    
    // Feedback publisher for action server
    rclcpp::TimerBase::SharedPtr feedback_timer_;
    rclcpp_action::GoalHandle<control_msgs::action::FollowJointTrajectory>::SharedPtr current_goal_;
    
public:
    TrajectoryExecutionNode() : Node("trajectory_execution_node") {
        // Initialize joint names from parameters
        this->declare_parameter("joint_names", std::vector<std::string>{});
        joint_names_ = this->get_parameter("joint_names").as_string_array();
        
        // Initialize trackers for each joint
        std::vector<double> kp(joint_names_.size(), 100.0);
        std::vector<double> ki(joint_names_.size(), 0.1);
        std::vector<double> kd(joint_names_.size(), 10.0);
        
        for (size_t i = 0; i < joint_names_.size(); i++) {
            trackers_.push_back(
                std::make_unique<TrajectoryTracker>(
                    kp, ki, kd, 0.01  // 100Hz control rate
                )
            );
        }
        
        safety_monitor_ = std::make_unique<TrajectorySafetyMonitor>();
        
        // Create action server
        action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "follow_joint_trajectory",
            std::bind(&TrajectoryExecutionNode::handleGoal, this, 
                     std::placeholders::_1, std::placeholders::_2),
            std::bind(&TrajectoryExecutionNode::handleCancel, this, 
                     std::placeholders::_1),
            std::bind(&TrajectoryExecutionNode::handleAccepted, this, 
                     std::placeholders::_1)
        );
        
        // Timer for control loop (100 Hz)
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TrajectoryExecutionNode::controlLoop, this)
        );
    }

private:
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received trajectory goal");
        
        // Validate trajectory
        if (!validateTrajectory(*goal)) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handleAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
    {
        // This needs to be run in a separate thread to avoid blocking
        using namespace std::placeholders;
        std::thread{std::bind(&TrajectoryExecutionNode::execute, this, _1), goal_handle}.detach();
    }
    
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing trajectory...");
        
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<control_msgs::action::FollowJointTrajectory::Feedback>();
        auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
        
        // Convert ROS trajectory to internal format
        auto internal_trajectory = convertToInternalTrajectory(goal->trajectory);
        
        // Set trajectory for tracker
        for (auto& tracker : trackers_) {
            tracker->setTrajectory(std::make_unique<JointSpaceTrajectory>(internal_trajectory));
        }
        
        rclcpp::Rate rate(100);  // 100 Hz control rate
        auto start_time = this->now();
        
        while (rclcpp::ok()) {
            auto current_time = this->now() - start_time;
            double t = current_time.seconds();
            
            // Get current robot state (from joint state publisher)
            auto current_state = getCurrentRobotState();
            
            // Check if goal was canceled
            if (goal_handle->is_canceling()) {
                // Stop robot and set result
                stopRobot();
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Trajectory was canceled");
                return;
            }
            
            // Compute control efforts
            std::vector<double> control_efforts = computeControlEfforts(current_state, t);
            
            // Check safety
            if (!safety_monitor_->isSafe(
                    current_state.positions, 
                    current_state.velocities, 
                    control_efforts,
                    getCurrentDesiredPositions(t),
                    getCurrentDesiredVelocities(t),
                    0.01)) {  // dt = 0.01s
                
                stopRobot();
                result->error_code = control_msgs::action::FollowJointTrajectory::Result::PATH_TOLERANCE_VIOLATED;
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(), "Safety violation during trajectory execution");
                return;
            }
            
            // Publish feedback
            updateFeedback(feedback, current_state, t);
            goal_handle->publish_feedback(feedback);
            
            // Check if trajectory is complete
            if (t >= internal_trajectory.getDuration()) {
                // Wait for robot to settle
                if (isRobotSettled(current_state, getCurrentDesiredPositions(t))) {
                    result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Trajectory execution completed successfully");
                    return;
                }
            }
            
            rate.sleep();
        }
    }
    
    void controlLoop() {
        // This is the main control loop that runs at 100Hz
        if (!current_goal_ || current_goal_->is_executing()) {
            return;  // No active goal or goal is being handled by action server
        }
        
        // Get current state from hardware interface
        auto current_state = getCurrentRobotState();
        
        // Get desired state from trajectory
        auto desired_state = getDesiredStateFromTrajectory(this->now());
        
        // Compute control commands
        auto control_commands = computeJointControlEfforts(current_state, desired_state);
        
        // Send commands to hardware
        sendCommandsToHardware(control_commands);
        
        // Publish feedback if needed
        publishControlFeedback(current_state, desired_state);
    }
    
    bool validateTrajectory(const control_msgs::action::FollowJointTrajectory::Goal& goal) {
        // Validate that joint names match
        if (goal.trajectory.joint_names != joint_names_) {
            RCLCPP_ERROR(this->get_logger(), "Joint names in trajectory do not match robot joint names");
            return false;
        }
        
        // Validate trajectory timing
        if (goal.trajectory.points.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory has no points");
            return false;
        }
        
        // Check that times are increasing
        builtin_interfaces::msg::Duration prev_time;
        for (const auto& point : goal.trajectory.points) {
            if (point.time_from_start.sec < prev_time.sec || 
                (point.time_from_start.sec == prev_time.sec && 
                 point.time_from_start.nanosec <= prev_time.nanosec)) {
                RCLCPP_ERROR(this->get_logger(), "Trajectory times are not increasing");
                return false;
            }
            prev_time = point.time_from_start;
        }
        
        // Validate joint limits (if provided)
        if (!goal.path_tolerance.empty()) {
            for (size_t i = 0; i < goal.path_tolerance.size(); i++) {
                if (goal.path_tolerance[i].name != joint_names_[i]) {
                    RCLCPP_ERROR(this->get_logger(), "Path tolerance joint name mismatch");
                    return false;
                }
            }
        }
        
        return true;
    }
    
    std::vector<double> computeControlEfforts(const RobotState& state, double t) {
        std::vector<double> efforts;
        
        for (size_t i = 0; i < trackers_.size(); i++) {
            // Get desired state for this joint from trajectory
            double desired_pos = getDesiredJointPosition(i, t);
            double desired_vel = getDesiredJointVelocity(i, t);
            
            // Compute control effort using tracker
            double effort = trackers_[i]->computeEffort(
                state.positions[i], state.velocities[i], 
                desired_pos, desired_vel, 0.01  // dt
            );
            
            efforts.push_back(effort);
        }
        
        return efforts;
    }
    
    void stopRobot() {
        // Send zero efforts to all joints
        std::vector<double> zero_efforts(joint_names_.size(), 0.0);
        sendCommandsToHardware(zero_efforts);
    }
    
    bool isRobotSettled(const RobotState& current_state, 
                       const std::vector<double>& desired_positions) {
        const double position_tolerance = 0.01;  // 1cm tolerance
        const double velocity_tolerance = 0.05;  // 0.05 rad/s tolerance
        
        for (size_t i = 0; i < current_state.positions.size(); i++) {
            if (std::abs(current_state.positions[i] - desired_positions[i]) > position_tolerance ||
                std::abs(current_state.velocities[i]) > velocity_tolerance) {
                return false;
            }
        }
        return true;
    }
};
```

### Trajectory Generation Integration

#### Trajectory Generator Node
```cpp
class TrajectoryGeneratorNode : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    
    std::unique_ptr<PathParametrizer> path_parametrizer_;
    std::unique_ptr<CartesianTrajectory> cartesian_generator_;
    
    // Robot kinematic model for IK
    std::unique_ptr<kinematics::KinematicsBase> kinematics_solver_;
    
public:
    TrajectoryGeneratorNode() : Node("trajectory_generator") {
        // Initialize publishers and subscribers
        trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "generated_trajectory", 10);
            
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10,
            std::bind(&TrajectoryGeneratorNode::poseCallback, this, std::placeholders::_1));
            
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "global_plan", 10,
            std::bind(&TrajectoryGeneratorNode::pathCallback, this, std::placeholders::_1));
            
        path_parametrizer_ = std::make_unique<PathParametrizer>();
        cartesian_generator_ = std::make_unique<CartesianTrajectory>();
    }
    
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Generate trajectory to reach the target pose
        auto current_pose = getCurrentPose();
        
        // Create simple straight-line trajectory
        std::vector<Eigen::Vector3d> waypoints = {
            Eigen::Vector3d(current_pose.pose.position.x, 
                           current_pose.pose.position.y, 
                           current_pose.pose.position.z),
            Eigen::Vector3d(msg->pose.position.x, 
                           msg->pose.position.y, 
                           msg->pose.position.z)
        };
        
        // Convert orientation to quaternion
        Eigen::Quaterniond start_quat(current_pose.pose.orientation.w,
                                    current_pose.pose.orientation.x,
                                    current_pose.pose.orientation.y,
                                    current_pose.pose.orientation.z);
        Eigen::Quaterniond target_quat(msg->pose.orientation.w,
                                     msg->pose.orientation.x,
                                     msg->pose.orientation.y,
                                     msg->pose.orientation.z);
        
        std::vector<Eigen::Quaterniond> orientations = {start_quat, target_quat};
        
        // Generate Cartesian trajectory
        cartesian_generator_->generateCartesianTrajectory(waypoints, orientations, 5.0);  // 5 seconds
        
        // Convert to joint space trajectory using inverse kinematics
        auto joint_trajectory = convertToJointTrajectory(cartesian_generator_.get());
        
        // Publish the trajectory
        trajectory_publisher_->publish(joint_trajectory);
    }
    
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
        // Generate trajectory from path waypoints
        std::vector<Eigen::Vector3d> positions;
        std::vector<Eigen::Quaterniond> orientations;
        
        for (const auto& pose : msg->poses) {
            positions.emplace_back(pose.pose.position.x, 
                                  pose.pose.position.y, 
                                  pose.pose.position.z);
                                  
            Eigen::Quaterniond quat(pose.pose.orientation.w,
                                  pose.pose.orientation.x,
                                  pose.pose.orientation.y,
                                  pose.pose.orientation.z);
            orientations.push_back(quat);
        }
        
        // Parametrize path with velocity limits
        auto parametrized_path = path_parametrizer_->parametrizePath(
            positions, 0.5, 1.0);  // max velocity: 0.5 m/s, max acc: 1.0 m/s²
        
        // Generate trajectory with proper timing
        auto trajectory = generatePathFollowingTrajectory(parametrized_path);
        
        // Publish trajectory
        trajectory_publisher_->publish(trajectory);
    }

private:
    trajectory_msgs::msg::JointTrajectory convertToJointTrajectory(
        CartesianTrajectory* cartesian_traj) {
        
        trajectory_msgs::msg::JointTrajectory joint_trajectory;
        joint_trajectory.joint_names = getJointNames();  // Get from robot description
        
        // Sample the Cartesian trajectory at regular intervals
        double dt = 0.01;  // 100Hz sampling
        double duration = cartesian_traj->getDuration();
        int num_points = static_cast<int>(duration / dt) + 1;
        
        joint_trajectory.points.resize(num_points);
        
        for (int i = 0; i < num_points; i++) {
            double t = i * dt;
            
            // Get Cartesian pose at time t
            auto pose = cartesian_traj->getPose(t);
            
            // Convert to joint space using inverse kinematics
            std::vector<double> joint_positions = inverseKinematics(pose.pose);
            
            // Set trajectory point
            joint_trajectory.points[i].positions = joint_positions;
            joint_trajectory.points[i].time_from_start.sec = static_cast<int32_t>(t);
            joint_trajectory.points[i].time_from_start.nanosec = 
                static_cast<uint32_t>((t - std::floor(t)) * 1e9);
        }
        
        return joint_trajectory;
    }
    
    trajectory_msgs::msg::JointTrajectory generatePathFollowingTrajectory(
        const std::vector<PathParametrizer::Waypoint>& parametrized_path) {
        
        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = getJointNames();
        
        // Convert each parametrized waypoint to joint space
        traj.points.reserve(parametrized_path.size());
        
        for (const auto& wp : parametrized_path) {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            
            // Convert Cartesian position to joint space
            geometry_msgs::msg::Pose pose;
            pose.position.x = wp.position.x();
            pose.position.y = wp.position.y();
            pose.position.z = wp.position.z();
            
            // Use simple IK to convert to joint space
            auto joint_pos = inverseKinematics(pose);
            point.positions = joint_pos;
            
            // Set timing
            point.time_from_start.sec = static_cast<int32_t>(wp.time);
            point.time_from_start.nanosec = 
                static_cast<uint32_t>((wp.time - std::floor(wp.time)) * 1e9);
            
            traj.points.push_back(point);
        }
        
        return traj;
    }
    
    std::vector<double> inverseKinematics(const geometry_msgs::msg::Pose& pose) {
        // Placeholder - would use actual IK solver
        // In practice, this would call a kinematics service or use a kinematics library
        std::vector<double> joint_angles;
        
        // Example: simple 2D IK for a 2-DOF arm
        double x = pose.position.x;
        double y = pose.position.y;
        double l1 = 0.5;  // Link 1 length
        double l2 = 0.5;  // Link 2 length
        
        double r = std::sqrt(x*x + y*y);
        if (r > l1 + l2) {
            // Target unreachable, return joint angles pointing toward target
            double angle = std::atan2(y, x);
            joint_angles = {angle, 0.0};
        } else {
            // Calculate joint angles using inverse kinematics
            double cos_theta2 = (r*r - l1*l1 - l2*l2) / (2*l1*l2);
            double sin_theta2 = std::sqrt(1 - cos_theta2*cos_theta2);
            double theta2 = std::atan2(sin_theta2, cos_theta2);
            
            double k1 = l1 + l2 * cos_theta2;
            double k2 = l2 * sin_theta2;
            double theta1 = std::atan2(y, x) - std::atan2(k2, k1);
            
            joint_angles = {theta1, theta2};
        }
        
        return joint_angles;
    }
    
    std::vector<std::string> getJointNames() {
        // Return the joint names for the robot
        // In practice, this would be loaded from robot description
        return {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    }
    
    geometry_msgs::msg::PoseStamped getCurrentPose() {
        // Get current robot pose (would use TF or robot state)
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.orientation.w = 1.0;  // Identity quaternion
        return pose;
    }
};
```

## Performance Optimization

### Real-time Performance

#### Control Loop Optimization
- **Fixed Time Steps**: Use consistent control loop timing
- **Minimal Computation**: Keep control computations lightweight
- **Memory Efficiency**: Avoid dynamic allocation in control loops
- **Cache Optimization**: Organize data for efficient memory access

#### Trajectory Generation Optimization
- **Pre-computation**: Compute trajectories offline when possible
- **Simplification**: Use simplified models for real-time computation
- **Sampling Rate**: Balance accuracy with computational requirements
- **Interpolation**: Use efficient interpolation methods

### Memory Management

#### Efficient Data Structures
- **Pre-allocated Buffers**: Avoid dynamic allocation in real-time loops
- **Memory Pools**: Reuse memory for frequent allocations
- **Cache-Friendly Layout**: Organize data for efficient access patterns
- **Batch Processing**: Process multiple points at once when possible

## Troubleshooting Common Issues

### Trajectory Generation Issues

#### Discontinuities
- **Symptoms**: Jerky motion, high accelerations, control instability
- **Causes**: Poor interpolation, velocity jumps, acceleration discontinuities
- **Solutions**: Use higher-order polynomials, ensure continuity, smooth transitions

#### Overshooting
- **Symptoms**: Robot passes target position before settling
- **Causes**: High gains, improper trajectory timing, insufficient damping
- **Solutions**: Reduce gains, add derivative action, improve trajectory planning

#### Tracking Errors
- **Symptoms**: Robot doesn't follow trajectory accurately
- **Causes**: Insufficient control gains, model inaccuracies, disturbances
- **Solutions**: Tune controller, improve system identification, add feedforward

### Execution Issues

#### Timing Problems
- **Symptoms**: Irregular motion, inconsistent velocity profiles
- **Causes**: Control loop jitter, communication delays, system load
- **Solutions**: Real-time scheduling, priority configuration, dedicated threads

#### Safety Violations
- **Symptoms**: Trajectory execution stops due to safety limits
- **Causes**: Aggressive trajectories, sensor noise, model errors
- **Solutions**: Adjust safety limits, improve trajectory planning, enhance filtering

## Future Developments

### Advanced Trajectory Generation

#### Learning-Based Trajectory Generation
- **Neural Networks**: Using NNs to generate optimal trajectories
- **Reinforcement Learning**: Learning from experience to improve trajectory generation
- **Imitation Learning**: Learning from expert demonstrations
- **Adaptive Generation**: Adjusting trajectories based on environment

#### Predictive Trajectory Generation
- **Environmental Prediction**: Anticipating environmental changes
- **Human Intent Prediction**: Predicting human behavior for collaborative tasks
- **Obstacle Prediction**: Predicting moving obstacle trajectories
- **Dynamic Replanning**: Adjusting trajectories based on predictions

### Integration with AI Systems

#### AI-Enhanced Control
- **Perception-Action Integration**: Direct integration with perception systems
- **Predictive Control**: Using AI predictions for proactive control
- **Adaptive Behavior**: Controllers that adapt to changing conditions
- **Learning from Demonstration**: Teaching robots new trajectories through demonstration

## Conclusion

Trajectory generation and execution form the bridge between high-level planning and low-level control in Physical AI systems. Proper trajectory generation ensures that robots can move smoothly and safely while achieving their objectives. The integration of trajectory generation with control systems and safety monitoring creates robust robotic systems capable of operating effectively in real-world environments.

Understanding the mathematical foundations of trajectory generation, the various interpolation techniques, and the integration with control systems is essential for creating effective Physical AI applications. The choice of trajectory generation approach depends on the specific requirements of the application, including accuracy needs, computational constraints, and safety requirements.

As robotics systems become more sophisticated, trajectory generation will continue to evolve with new techniques for handling complex environments, dynamic obstacles, and collaborative tasks. The integration of AI techniques with traditional trajectory generation methods promises to create more adaptive and intelligent robotic systems.

The implementation strategies discussed in this chapter provide the foundation for creating trajectory generation and execution systems that can handle the complexities of real-world robotic applications while maintaining safety and performance requirements.

## Exercises

1. Implement a trajectory generator that creates smooth paths for a 6-DOF manipulator using cubic splines.
2. Design and implement a trajectory tracking controller for a mobile robot following a path with obstacle avoidance.
3. Create a safety monitoring system that validates trajectory execution in real-time and prevents unsafe robot behaviors.

## Further Reading

- Siciliano, B., & Khatib, O. (Eds.). (2016). "Springer Handbook of Robotics." Springer.
- Spong, M.W., Hutchinson, S., & Vidyasagar, M. (2006). "Robot Modeling and Control." Wiley.
- Murray, R.M., Li, Z.X., & Sastry, S.S. (1994). "A Mathematical Introduction to Robotic Manipulation." CRC Press.
- Slotine, J.J.E., & Li, W. (1991). "Applied Nonlinear Control." Prentice Hall.
- Kelly, A. (2006). "Mobile Robotics: Mathematics, Models, and Methods." Cambridge University Press.