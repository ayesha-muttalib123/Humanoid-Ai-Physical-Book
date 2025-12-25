---
sidebar_label: Whole-Body Control
title: Whole-Body Control - Coordinated Control of Entire Robot Bodies
description: Understanding whole-body control techniques for coordinating entire robot bodies including humanoid robots and manipulators
keywords: [whole body control, humanoid robotics, manipulation, coordination, robotics, control, balance, dynamics]
---

# 6.4 Whole-Body Control

## Introduction

Whole-body control is a sophisticated approach to controlling robots with many degrees of freedom (DOF), such as humanoid robots or multi-arm manipulator systems. Unlike traditional joint-level control, whole-body control considers the entire robot system simultaneously, taking into account the dynamic coupling between joints, the robot's center of mass, and its interaction with the environment. This approach is essential for Physical AI systems that must perform complex, coordinated tasks while maintaining stability and respecting physical constraints.

In whole-body control, multiple tasks are prioritized and executed simultaneously, such as maintaining balance while performing manipulation tasks. The controller must handle the complex dynamics of the entire system, including the effects of gravity, Coriolis forces, and external contacts. This requires advanced mathematical tools from mechanics, optimization, and control theory.

The integration of whole-body control with other Physical AI components like perception, planning, and navigation creates robots capable of performing complex tasks in real-world environments. Understanding whole-body control is crucial for developing humanoid robots, mobile manipulators, and other complex robotic systems.

## Mathematical Foundations

### Rigid Body Dynamics

The foundation of whole-body control lies in understanding the dynamics of rigid body systems:

#### Newton-Euler Equations
For a rigid body, the Newton-Euler equations describe the relationship between forces and motion:

**Translation**: F = m * a_cm
**Rotation**: τ = I * α + ω × (I * ω)

Where:
- F: External force
- m: Mass
- a_cm: Acceleration of center of mass
- τ: Torque
- I: Inertia tensor
- α: Angular acceleration
- ω: Angular velocity

#### Lagrangian Formulation

For multi-body systems, the Lagrangian formulation is more appropriate:

**Configuration Vector**: q ∈ ℝ^n (n = number of DOF)
**Lagrangian**: L = T - V (kinetic energy - potential energy)

**Euler-Lagrange Equation**:
d/dt(∂L/∂q̇) - ∂L/∂q = τ

This results in the standard form:
M(q)q̈ + C(q,q̇)q̇ + g(q) = τ + J^T(q)f

Where:
- M(q): Mass matrix
- C(q,q̇): Coriolis and centrifugal forces
- g(q): Gravity forces
- τ: Joint torques
- J(q): Jacobian matrix
- f: External forces

### Kinematic Chains and Jacobians

#### Forward Kinematics
Forward kinematics maps joint positions to end-effector positions:

x = fk(q)

Where x is the end-effector pose and q is the joint configuration.

#### Jacobian Matrix
The Jacobian relates joint velocities to end-effector velocities:

ẋ = J(q)q̇

For multiple end-effectors, we have multiple Jacobians:
- **J_ee**: Jacobian for end-effector
- **J_com**: Jacobian for center of mass
- **J_contact**: Jacobian for contact points

#### Example Implementation
```cpp
class KinematicModel {
private:
    std::unique_ptr<RobotModel> robot_model_;
    Eigen::MatrixXd jacobian_;
    Eigen::VectorXd joint_positions_;
    
public:
    KinematicModel(const std::string& urdf_path) {
        robot_model_ = std::make_unique<RobotModel>(urdf_path);
        jacobian_.resize(6, robot_model_->getDOF());  // 6 DOF for position/orientation
    }
    
    void updateKinematics(const std::vector<double>& joint_pos) {
        joint_positions_ = Eigen::Map<const Eigen::VectorXd>(joint_pos.data(), joint_pos.size());
        
        // Update robot model with current joint positions
        robot_model_->update(joint_positions_);
        
        // Calculate Jacobian for end-effector
        calculateJacobian("end_effector_link", jacobian_);
        
        // Calculate center of mass Jacobian
        calculateCoMJacobian(com_jacobian_);
    }
    
    Eigen::MatrixXd getJacobian(const std::string& link_name) {
        Eigen::MatrixXd jac(6, joint_positions_.size());
        robot_model_->getJacobian(link_name, joint_positions_, jac);
        return jac;
    }
    
    Eigen::MatrixXd getCoMJacobian() {
        Eigen::MatrixXd com_jac(3, joint_positions_.size());
        robot_model_->getCoMJacobian(joint_positions_, com_jac);
        return com_jac;
    }
    
    Eigen::VectorXd getCenterOfMass() {
        return robot_model_->getCenterOfMass(joint_positions_);
    }
    
    std::vector<Eigen::Vector3d> getContactPoints() {
        // Get all contact points for current configuration
        return robot_model_->getContactPoints(joint_positions_);
    }
    
    double getMass() {
        return robot_model_->getTotalMass();
    }

private:
    void calculateJacobian(const std::string& link_name, Eigen::MatrixXd& jac) {
        // Calculate 6xN Jacobian (position + orientation) for the specified link
        // Implementation depends on specific kinematic library used
        // This would typically use geometric or analytical methods
    }
    
    void calculateCoMJacobian(Eigen::MatrixXd& com_jac) {
        // Calculate 3xN Jacobian for center of mass
        // This involves computing partial derivatives of CoM position
        // with respect to each joint angle
    }
};
```

### Task Space Formulation

#### Task Functions
Tasks in whole-body control are typically formulated as:

e_task = f_task(x) - f_desired

Where:
- e_task: Task error
- f_task: Current task function value
- f_desired: Desired task function value

#### Common Tasks
- **End-effector position**: e_pos = x_ee - x_desired
- **End-effector orientation**: e_orient = R_ee - R_desired
- **Center of mass**: e_com = com_pos - com_desired
- **Balance**: e_balance = zmp_actual - zmp_desired
- **Joint limits**: e_limits = joint_pos - joint_limits

## Control Frameworks

### Operational Space Control (OSC)

Operational space control is a framework for controlling robots in task space rather than joint space.

#### Mathematical Formulation
The operational space control law is:

τ = J^T * F_task + τ_null

Where:
- τ: Joint torques
- J: Task Jacobian
- F_task: Task space forces
- τ_null: Null-space torques for secondary tasks

The task space force is computed as:
F_task = M_task * (ẍ_desired + K_d * ė + K_p * e) - h_task

Where:
- M_task: Task space inertia
- K_d, K_p: Damping and stiffness gains
- h_task: Bias forces in task space

#### Implementation Example
```cpp
class OperationalSpaceController {
private:
    std::unique_ptr<KinematicModel> kin_model_;
    std::vector<std::unique_ptr<Task>> tasks_;
    std::vector<double> joint_positions_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_torques_;
    
    // Control parameters
    double kp_pos_, kd_pos_;
    double kp_orient_, kd_orient_;
    
public:
    OperationalSpaceController(const std::string& urdf_path) 
        : kin_model_(std::make_unique<KinematicModel>(urdf_path)),
          kp_pos_(100.0), kd_pos_(20.0),
          kp_orient_(10.0), kd_orient_(5.0) {}
    
    std::vector<double> computeControlEfforts(
        const std::vector<double>& desired_positions,
        const std::vector<double>& desired_velocities,
        const std::vector<double>& desired_accelerations) {
        
        // Update kinematic model
        kin_model_->updateKinematics(joint_positions_);
        
        // Calculate mass matrix
        Eigen::MatrixXd M = calculateMassMatrix(joint_positions_);
        
        // Calculate Coriolis and gravity terms
        Eigen::VectorXd Cg = calculateCoriolisAndGravity(joint_positions_, joint_velocities_);
        
        // Initialize joint torques
        Eigen::VectorXd tau = Eigen::VectorXd::Zero(joint_positions_.size());
        
        // Process each task in order of priority
        for (auto& task : tasks_) {
            // Calculate task Jacobian
            Eigen::MatrixXd J_task = task->getJacobian();
            
            // Calculate task error
            Eigen::VectorXd error = task->getError();
            Eigen::VectorXd error_dot = task->getErrorDerivative();
            Eigen::VectorXd error_ddot = task->getErrorSecondDerivative();
            
            // Calculate task space inertia
            Eigen::MatrixXd lambda_inv = J_task * M.inverse() * J_task.transpose();
            Eigen::MatrixXd lambda = lambda_inv.inverse();
            
            // Calculate task space forces
            Eigen::VectorXd F_task = lambda * (desired_accelerations + 
                                            kp_pos_ * error + 
                                            kd_pos_ * error_dot) - 
                                   task->getBiasForces();
            
            // Calculate joint torques for this task
            Eigen::VectorXd tau_task = J_task.transpose() * F_task;
            
            // Add to total torques
            tau += tau_task;
            
            // Update null-space projector for lower-priority tasks
            Eigen::MatrixXd N = Eigen::MatrixXd::Identity(M.rows(), M.cols()) - 
                               M.inverse() * J_task.transpose() * lambda * J_task;
            task->setNullSpaceProjector(N);
        }
        
        // Add gravity compensation
        tau += Cg;
        
        return std::vector<double>(tau.data(), tau.data() + tau.size());
    }
    
    void addTask(std::unique_ptr<Task> task) {
        tasks_.push_back(std::move(task));
    }

private:
    Eigen::MatrixXd calculateMassMatrix(const std::vector<double>& q) {
        // Calculate mass matrix using composite rigid body algorithm
        // This would use the robot model to compute the mass matrix
        return Eigen::MatrixXd();  // Placeholder
    }
    
    Eigen::VectorXd calculateCoriolisAndGravity(
        const std::vector<double>& q, 
        const std::vector<double>& qdot) {
        // Calculate Coriolis and gravity terms
        // This would use the robot model to compute these terms
        return Eigen::VectorXd();  // Placeholder
    }
    
    void updateTaskPriorities() {
        // Update task priorities based on current situation
        // Higher priority tasks are handled first
        std::sort(tasks_.begin(), tasks_.end(), 
                 [](const std::unique_ptr<Task>& a, const std::unique_ptr<Task>& b) {
                     return a->getPriority() > b->getPriority();
                 });
    }
};
```

### Hierarchical Control

Hierarchical control organizes tasks in a priority hierarchy, where higher-priority tasks are fulfilled exactly while lower-priority tasks are fulfilled only if possible.

#### Priority-Based Task Execution
```cpp
class HierarchicalController {
private:
    struct TaskHierarchy {
        std::vector<std::unique_ptr<Task>> primary_tasks;      // Priority 1 (highest)
        std::vector<std::unique_ptr<Task>> secondary_tasks;   // Priority 2
        std::vector<std::unique_ptr<Task>> tertiary_tasks;    // Priority 3 (lowest)
        
        // Null-space projectors for each priority level
        std::vector<Eigen::MatrixXd> null_space_projectors;
    };
    
    TaskHierarchy task_hierarchy_;
    Eigen::MatrixXd mass_matrix_;
    Eigen::VectorXd coriolis_gravity_forces_;
    
public:
    std::vector<double> computeHierarchicalControl() {
        Eigen::VectorXd total_tau = Eigen::VectorXd::Zero(joint_positions_.size());
        Eigen::MatrixXd current_projector = Eigen::MatrixXd::Identity(
            joint_positions_.size(), joint_positions_.size());
        
        // Process primary tasks (highest priority)
        for (auto& task : task_hierarchy_.primary_tasks) {
            auto [tau_task, next_projector] = computeTaskControl(
                task.get(), current_projector);
            total_tau += tau_task;
            current_projector = next_projector;
        }
        
        // Process secondary tasks in null space of primary tasks
        for (auto& task : task_hierarchy_.secondary_tasks) {
            auto [tau_task, next_projector] = computeTaskControl(
                task.get(), current_projector);
            total_tau += tau_task;
            current_projector = next_projector;
        }
        
        // Process tertiary tasks in null space of higher priority tasks
        for (auto& task : task_hierarchy_.tertiary_tasks) {
            auto [tau_task, next_projector] = computeTaskControl(
                task.get(), current_projector);
            total_tau += tau_task;
            current_projector = next_projector;
        }
        
        return std::vector<double>(total_tau.data(), 
                                  total_tau.data() + total_tau.size());
    }

private:
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> computeTaskControl(
        Task* task, const Eigen::MatrixXd& projector) {
        
        // Get task Jacobian
        Eigen::MatrixXd J_task = task->getJacobian();
        
        // Apply null-space projector
        Eigen::MatrixXd J_proj = J_task * projector;
        
        // Calculate task error
        Eigen::VectorXd error = task->getError();
        Eigen::VectorXd error_dot = task->getErrorDerivative();
        
        // Calculate projected mass matrix in task space
        Eigen::MatrixXd M_task_inv = J_proj * mass_matrix_.inverse() * J_proj.transpose();
        Eigen::MatrixXd M_task = M_task_inv.inverse();
        
        // Calculate task space acceleration
        Eigen::VectorXd x_ddot = task->getDesiredAcceleration() + 
                                kp_ * error + kd_ * error_dot;
        
        // Calculate task space force
        Eigen::VectorXd F_task = M_task * x_ddot - task->getBiasForces();
        
        // Calculate joint torques for this task
        Eigen::VectorXd tau_task = projector.transpose() * J_proj.transpose() * F_task;
        
        // Calculate new null-space projector
        Eigen::MatrixXd N_task = projector - 
                                projector.transpose() * J_proj.transpose() * 
                                M_task_inv * J_proj * projector;
        
        return {tau_task, N_task};
    }
};
```

### Quadratic Programming (QP) Based Control

QP-based control formulates the whole-body control problem as an optimization problem:

#### Mathematical Formulation
```
minimize: ||Ax - b||² + λ||x||²
subject to: Cx ≤ d
           Ex = f
```

Where:
- x: Optimization variables (joint accelerations, contact forces, etc.)
- A, b: Quadratic cost terms
- C, d: Inequality constraints
- E, f: Equality constraints
- λ: Regularization parameter

#### Implementation Example
```cpp
#include <qpOASES.hpp>

class QPWholeBodyController {
private:
    // Optimization variables
    int n_variables_;  // Joint accelerations + contact forces
    int n_constraints_;
    
    // Cost function matrices
    Eigen::MatrixXd H_;  // Hessian matrix
    Eigen::VectorXd g_;  // Linear term in cost
    
    // Constraint matrices
    Eigen::MatrixXd A_;  // Constraint matrix
    Eigen::VectorXd lb_;  // Lower bounds
    Eigen::VectorXd ub_;  // Upper bounds
    
    // QP solver
    qpOASES::SQProblem qp_problem_;
    
public:
    QPWholeBodyController(int n_joints, int n_contacts) 
        : n_variables_(n_joints + 3*n_contacts),  // 3 DOF per contact point
          n_constraints_(0),
          qp_problem_(n_variables_, n_constraints_) {
        
        H_.resize(n_variables_, n_variables_);
        g_.resize(n_variables_);
        A_.resize(n_constraints_, n_variables_);
        lb_.resize(n_constraints_);
        ub_.resize(n_constraints_);
    }
    
    std::vector<double> computeControl(const RobotState& state, 
                                     const TaskSet& tasks) {
        
        // Formulate the QP problem
        formulateProblem(state, tasks);
        
        // Solve the QP problem
        qpOASES::real_t* H_qp = convertToQPMatrices(H_);
        qpOASES::real_t* g_qp = convertToQPVector(g_);
        qpOASES::real_t* A_qp = convertToQPMatrices(A_);
        qpOASES::real_t* lb_qp = convertToQPVector(lb_);
        qpOASES::real_t* ub_qp = convertToQPVector(ub_);
        
        // Setup QP problem
        int max_iter = 1000;
        qp_problem_.init(H_qp, g_qp, A_qp, NULL, NULL, lb_qp, ub_qp, max_iter);
        
        // Solve QP
        qpOASES::real_t x_opt[n_variables_];
        qp_problem_.getPrimalSolution(x_opt);
        
        // Extract joint accelerations from solution
        std::vector<double> joint_accelerations(n_variables_);
        for (int i = 0; i < n_variables_; i++) {
            joint_accelerations[i] = x_opt[i];
        }
        
        return joint_accelerations;
    }

private:
    void formulateProblem(const RobotState& state, const TaskSet& tasks) {
        // Construct cost function
        constructCostFunction(state, tasks);
        
        // Construct constraints
        constructConstraints(state, tasks);
        
        // Initialize QP problem
        qp_problem_.init(H_.data(), g_.data(), A_.data(), 
                        lb_.data(), ub_.data(), 
                        n_variables_, n_constraints_);
    }
    
    void constructCostFunction(const RobotState& state, const TaskSet& tasks) {
        // Task tracking terms
        for (const auto& task : tasks) {
            // Add terms to minimize task errors
            // ||J*qddot - (desired_acceleration - bias_terms)||^2
        }
        
        // Regularization terms
        // ||qddot||^2 to smooth joint accelerations
        // ||tau||^2 to minimize control effort
        // ||f_contact||^2 to minimize contact forces
    }
    
    void constructConstraints(const RobotState& state, const TaskSet& tasks) {
        // Joint limits
        // Joint velocity limits
        // Joint torque limits
        // Contact stability constraints (friction cones)
        // Balance constraints (ZMP, COM within support polygon)
        // Dynamic balance constraints (momentum rate bounds)
    }
    
    std::vector<double> computeJointTorques(const std::vector<double>& joint_accelerations,
                                          const RobotState& state) {
        // Convert accelerations to torques using inverse dynamics
        // tau = M(q)*qddot + C(q,qdot)*qdot + g(q) + J^T*f_external
        return {};
    }
    
    std::vector<double> computeContactForces(const std::vector<double>& solution) {
        // Extract contact forces from QP solution
        // Contact forces are typically at the end of the optimization variable vector
        std::vector<double> contact_forces;
        size_t start_idx = joint_positions_.size();  // Skip joint accelerations
        
        for (size_t i = start_idx; i < solution.size(); i++) {
            contact_forces.push_back(solution[i]);
        }
        
        return contact_forces;
    }
};
```

## Balance and Locomotion Control

### Center of Mass (CoM) Control

Maintaining balance is crucial for legged robots, requiring careful control of the center of mass position and momentum.

#### Zero Moment Point (ZMP)
The ZMP is a crucial concept in balance control:

ZMP_x = (Σ(F_z * x - M_y)) / ΣF_z
ZMP_y = (Σ(F_z * y + M_x)) / ΣF_z

Where F_z is vertical force, M_x and M_y are moments about x and y axes.

#### Balance Control Implementation
```cpp
class BalanceController {
private:
    std::unique_ptr<KinematicModel> robot_model_;
    std::vector<ContactPoint> contact_points_;
    
    // Balance control parameters
    double zmp_kp_, zmp_kd_;
    double com_kp_, com_kd_;
    
    // Desired ZMP trajectory
    std::vector<Eigen::Vector2d> zmp_trajectory_;
    
    // Support polygon
    SupportPolygon support_polygon_;
    
public:
    BalanceController(std::unique_ptr<KinematicModel> model)
        : robot_model_(std::move(model)), 
          zmp_kp_(100.0), zmp_kd_(20.0),
          com_kp_(50.0), com_kd_(10.0) {}
    
    std::vector<double> computeBalanceControl(const RobotState& state,
                                             const std::vector<double>& desired_com) {
        
        // Calculate current ZMP
        auto current_zmp = calculateCurrentZMP(state);
        
        // Calculate desired ZMP (typically inside support polygon)
        auto desired_zmp = calculateDesiredZMP(state);
        
        // Calculate ZMP error
        Eigen::Vector2d zmp_error = desired_zmp - current_zmp;
        
        // Calculate CoM error
        Eigen::Vector3d current_com = robot_model_->getCenterOfMass(state.joint_positions);
        Eigen::Vector3d com_error = desired_com - current_com.head<3>();
        
        // Compute balance control torques
        Eigen::VectorXd balance_torques = computeBalanceTorques(zmp_error, com_error, state);
        
        return std::vector<double>(balance_torques.data(), 
                                  balance_torques.data() + balance_torques.size());
    }

private:
    Eigen::Vector2d calculateCurrentZMP(const RobotState& state) {
        // Calculate ZMP from current forces and moments
        double total_fz = 0.0;
        double moment_x = 0.0, moment_y = 0.0;
        
        // Sum forces and moments from all contact points
        for (const auto& contact : contact_points_) {
            auto force = getContactForce(contact);
            auto moment = getContactMoment(contact);
            
            total_fz += force.z();
            moment_x += contact.position.x() * force.z() - moment.y();
            moment_y += contact.position.y() * force.z() + moment.x();
        }
        
        // Calculate ZMP position
        Eigen::Vector2d zmp;
        if (total_fz > 0.1) {  // Avoid division by zero
            zmp.x() = moment_x / total_fz;
            zmp.y() = moment_y / total_fz;
        } else {
            zmp = Eigen::Vector2d::Zero();
        }
        
        return zmp;
    }
    
    Eigen::Vector2d calculateDesiredZMP(const RobotState& state) {
        // Calculate desired ZMP based on support polygon
        // Typically inside the convex hull of contact points
        Eigen::Vector2d desired_zmp;
        
        // For single support, keep ZMP near center of foot
        // For double support, keep ZMP within support polygon
        desired_zmp = support_polygon_.getCenter();
        
        // Add some margin from polygon edges for stability
        desired_zmp = support_polygon_.getInteriorPoint(desired_zmp, 0.05);  // 5cm margin
        
        return desired_zmp;
    }
    
    Eigen::VectorXd computeBalanceTorques(const Eigen::Vector2d& zmp_error,
                                        const Eigen::Vector3d& com_error,
                                        const RobotState& state) {
        
        // Use operational space control for balance
        Eigen::MatrixXd com_jacobian = robot_model_->getCoMJacobian();
        Eigen::MatrixXd zmp_jacobian = calculateZMPJacobian();
        
        // Calculate control gains
        Eigen::Vector2d zmp_control = zmp_kp_ * zmp_error + zmp_kd_ * zmp_error_derivative_;
        Eigen::Vector3d com_control = com_kp_ * com_error.head<3>() + 
                                     com_kd_ * com_error_derivative_.head<3>();
        
        // Combine ZMP and CoM control
        Eigen::VectorXd task_control(5);  // 3 for CoM, 2 for ZMP
        task_control << com_control, zmp_control;
        
        // Calculate Jacobian matrix for combined task
        Eigen::MatrixXd combined_jacobian(5, state.joint_positions.size());
        combined_jacobian.topRows(3) = com_jacobian.topRows(3);
        combined_jacobian.bottomRows(2) = zmp_jacobian;
        
        // Compute joint torques
        Eigen::VectorXd tau = combined_jacobian.transpose() * task_control;
        
        return tau;
    }
    
    Eigen::MatrixXd calculateZMPJacobian() {
        // Calculate Jacobian relating joint velocities to ZMP position
        // This involves complex derivatives of force and moment relationships
        return Eigen::MatrixXd();  // Placeholder
    }
    
    void updateSupportPolygon(const std::vector<ContactPoint>& contacts) {
        // Update support polygon based on current contact points
        support_polygon_.update(contacts);
    }
};
```

### Locomotion Patterns

#### Walking Gaits
- **Static Gait**: Stable at all times, CoM within support polygon
- **Dynamic Gait**: Momentarily unstable, relies on momentum control
- **Periodic Gait**: Repeating pattern of foot placement
- **Adaptive Gait**: Adjusts to terrain and disturbances

#### Example: Inverted Pendulum Model
```cpp
class InvertedPendulumController {
private:
    double com_height_;  // Height of center of mass
    double gravity_;     // Gravity constant
    double omega_;       // Natural frequency of inverted pendulum
    
public:
    InvertedPendulumController(double com_height) 
        : com_height_(com_height), gravity_(9.81) {
        omega_ = std::sqrt(gravity_ / com_height_);
    }
    
    std::vector<double> computeFootPlacement(const RobotState& state,
                                           const Eigen::Vector2d& desired_velocity) {
        // Calculate capture point for stopping
        Eigen::Vector2d current_com_pos = getCurrentCoMPosition(state);
        Eigen::Vector2d current_com_vel = getCurrentCoMVelocity(state);
        
        Eigen::Vector2d capture_point = current_com_pos + current_com_vel / omega_;
        
        // Calculate desired foot placement based on capture point
        Eigen::Vector2d desired_foot_pos = calculateFootPlacement(capture_point, desired_velocity);
        
        return convertToJointAngles(desired_foot_pos, state);
    }

private:
    Eigen::Vector2d calculateFootPlacement(const Eigen::Vector2d& capture_point,
                                         const Eigen::Vector2d& desired_velocity) {
        // Calculate where to place foot to achieve desired motion
        // This involves solving the inverted pendulum equations
        
        Eigen::Vector2d foot_pos = capture_point;
        
        // Adjust for desired velocity
        if (desired_velocity.norm() > 0.01) {
            // For forward motion, place foot ahead of capture point
            double step_time = 0.5;  // Half cycle time
            foot_pos += desired_velocity * step_time;
        }
        
        // Keep within reasonable bounds
        foot_pos.x() = std::clamp(foot_pos.x(), -0.3, 0.3);  // 30cm step limits
        foot_pos.y() = std::clamp(foot_pos.y(), -0.15, 0.15);  // 15cm lateral limits
        
        return foot_pos;
    }
    
    std::vector<double> convertToJointAngles(const Eigen::Vector2d& foot_position,
                                           const RobotState& state) {
        // Convert desired foot position to joint angles using inverse kinematics
        // This would use the robot's kinematic model to solve for joint angles
        return {};
    }
    
    Eigen::Vector2d getCurrentCoMPosition(const RobotState& state) {
        // Get current center of mass position
        return robot_model_->getCenterOfMass(state.joint_positions).head<2>();
    }
    
    Eigen::Vector2d getCurrentCoMVelocity(const RobotState& state) {
        // Get current center of mass velocity
        // This might be estimated from joint velocities
        return Eigen::Vector2d::Zero();
    }
};
```

## Manipulation Control

### Multi-Task Manipulation

Controlling multiple manipulation tasks simultaneously requires careful coordination:

#### Bimanual Manipulation
```cpp
class BimanualController {
private:
    std::unique_ptr<KinematicModel> robot_model_;
    std::unique_ptr<OperationalSpaceController> osc_;
    
    // Left and right end-effector tasks
    std::unique_ptr<Task> left_hand_task_;
    std::unique_ptr<Task> right_hand_task_;
    std::unique_ptr<Task> bimanual_task_;
    
    // CoM balance task (secondary)
    std::unique_ptr<Task> balance_task_;
    
public:
    BimanualController(const std::string& urdf_path) {
        robot_model_ = std::make_unique<KinematicModel>(urdf_path);
        osc_ = std::make_unique<OperationalSpaceController>(urdf_path);
        
        // Initialize tasks
        left_hand_task_ = std::make_unique<EndEffectorTask>("left_hand");
        right_hand_task_ = std::make_unique<EndEffectorTask>("right_hand");
        bimanual_task_ = std::make_unique<BimanualTask>("bimanual");
        balance_task_ = std::make_unique<BalanceTask>("balance");
        
        // Set task priorities
        left_hand_task_->setPriority(1);
        right_hand_task_->setPriority(1);
        bimanual_task_->setPriority(2);  // Lower priority than individual hands
        balance_task_->setPriority(3);   // Lowest priority - balance after manipulation
    }
    
    std::vector<double> computeBimanualControl(
        const std::vector<double>& desired_left_pose,
        const std::vector<double>& desired_right_pose,
        const std::vector<double>& desired_bimanual_constraint) {
        
        // Update individual hand tasks
        left_hand_task_->setDesiredPose(desired_left_pose);
        right_hand_task_->setDesiredPose(desired_right_pose);
        
        // Update bimanual constraint task
        bimanual_task_->setDesiredConstraint(desired_bimanual_constraint);
        
        // Update balance task
        balance_task_->setDesiredCoM(getDesiredCoMPosition());
        
        // Compute control using hierarchical framework
        auto control_torques = osc_->computeHierarchicalControl(
            {left_hand_task_, right_hand_task_, bimanual_task_, balance_task_});
        
        return control_torques;
    }

private:
    class BimanualTask : public Task {
    private:
        std::string left_ee_name_;
        std::string right_ee_name_;
        std::vector<double> desired_constraint_;
        
    public:
        BimanualTask(const std::string& name, 
                    const std::string& left_ee, 
                    const std::string& right_ee)
            : Task(name), left_ee_name_(left_ee), right_ee_name_(right_ee) {}
        
        void setDesiredConstraint(const std::vector<double>& constraint) {
            desired_constraint_ = constraint;
        }
        
        Eigen::VectorXd getError() override {
            // Calculate error based on relationship between both end-effectors
            auto left_pos = getEndEffectorPosition(left_ee_name_);
            auto right_pos = getEndEffectorPosition(right_ee_name_);
            
            // Example: maintain fixed distance between hands
            double current_distance = (left_pos - right_pos).norm();
            double desired_distance = desired_constraint_[0];
            
            Eigen::VectorXd error(1);
            error[0] = current_distance - desired_distance;
            
            return error;
        }
        
        Eigen::MatrixXd getJacobian() override {
            // Calculate combined Jacobian for both end-effectors
            auto left_jac = getEndEffectorJacobian(left_ee_name_);
            auto right_jac = getEndEffectorJacobian(right_ee_name_);
            
            // For distance constraint, Jacobian is the derivative of distance
            // with respect to joint angles
            auto left_pos = getEndEffectorPosition(left_ee_name_);
            auto right_pos = getEndEffectorPosition(right_ee_name_);
            Eigen::Vector3d direction = (left_pos - right_pos).normalized();
            
            Eigen::MatrixXd combined_jac(1, left_jac.cols());
            combined_jac.row(0) = direction.transpose() * (left_jac - right_jac);
            
            return combined_jac;
        }
    };
};
```

### Grasp and Manipulation

#### Force Control for Grasping
```cpp
class GraspController {
private:
    std::unique_ptr<ForceController> finger_force_controller_;
    std::unique_ptr<PositionController> grasp_position_controller_;
    std::unique_ptr<ComplianceController> grasp_compliance_controller_;
    
    // Grasp parameters
    double target_grasp_force_;
    double grasp_tolerance_;
    double slip_detection_threshold_;
    
    // Finger positions and forces
    std::vector<double> finger_positions_;
    std::vector<double> finger_forces_;
    std::vector<double> target_finger_forces_;
    
public:
    GraspController(double target_force, double tolerance) 
        : target_grasp_force_(target_force), grasp_tolerance_(tolerance) {
        
        // Initialize controllers
        finger_force_controller_ = std::make_unique<ForceController>();
        grasp_position_controller_ = std::make_unique<PositionController>();
        grasp_compliance_controller_ = std::make_unique<ComplianceController>();
    }
    
    bool executeGrasp(const std::vector<double>& object_pose) {
        // Approach object
        if (!approachObject(object_pose)) {
            return false;
        }
        
        // Close fingers with force control
        if (!closeWithForceControl()) {
            return false;
        }
        
        // Verify grasp stability
        return verifyGrasp();
    }
    
    bool executeRelease() {
        // Open fingers with position control
        std::vector<double> open_positions = getOpenFingerPositions();
        return moveFingersToPosition(open_positions);
    }

private:
    bool approachObject(const std::vector<double>& object_pose) {
        // Move gripper to approach position
        std::vector<double> approach_pose = object_pose;
        approach_pose[2] += 0.05;  // 5cm above object
        
        return moveToPose(approach_pose);
    }
    
    bool closeWithForceControl() {
        // Initialize target forces for each finger
        for (size_t i = 0; i < finger_positions_.size(); i++) {
            target_finger_forces_[i] = target_grasp_force_ / finger_positions_.size();
        }
        
        // Use force control to close fingers
        double dt = 0.01;  // 100 Hz
        rclcpp::Rate rate(1.0/dt);
        
        while (rclcpp::ok()) {
            // Read current forces
            auto current_forces = getCurrentFingerForces();
            
            // Check if target force reached
            bool force_reached = true;
            for (size_t i = 0; i < current_forces.size(); i++) {
                if (std::abs(current_forces[i] - target_finger_forces_[i]) > grasp_tolerance_) {
                    force_reached = false;
                    break;
                }
            }
            
            if (force_reached) {
                return true;  // Grasp successful
            }
            
            // Adjust finger positions based on force feedback
            std::vector<double> position_adjustments = computeForceBasedAdjustments(
                current_forces, target_finger_forces_);
            
            for (size_t i = 0; i < finger_positions_.size(); i++) {
                finger_positions_[i] += position_adjustments[i] * dt;
            }
            
            // Send position commands
            sendFingerPositionCommands(finger_positions_);
            
            rate.sleep();
        }
        
        return false;  // Timeout
    }
    
    std::vector<double> computeForceBasedAdjustments(
        const std::vector<double>& current_forces,
        const std::vector<double>& target_forces) {
        
        // Compute position adjustments based on force errors
        std::vector<double> adjustments(current_forces.size());
        
        for (size_t i = 0; i < current_forces.size(); i++) {
            double force_error = target_forces[i] - current_forces[i];
            
            // Use PD control for force regulation
            double adjustment = kp_force_ * force_error + 
                               kd_force_ * (force_error - previous_force_errors_[i]);
            
            // Limit adjustment magnitude
            adjustments[i] = std::clamp(adjustment, -max_adjustment_, max_adjustment_);
            
            previous_force_errors_[i] = force_error;
        }
        
        return adjustments;
    }
    
    bool verifyGrasp() {
        // Check if grasp is stable
        auto current_forces = getCurrentFingerForces();
        
        // Check for slip detection
        if (detectSlip(current_forces)) {
            RCLCPP_WARN(rclcpp::get_logger("grasp_controller"), "Slip detected during grasp");
            return false;
        }
        
        // Check if forces are within expected range
        for (double force : current_forces) {
            if (force < target_grasp_force_ * 0.8 || force > target_grasp_force_ * 1.2) {
                RCLCPP_WARN(rclcpp::get_logger("grasp_controller"), 
                           "Grasp force outside expected range");
                return false;
            }
        }
        
        return true;  // Grasp verified as stable
    }
    
    bool detectSlip(const std::vector<double>& forces) {
        // Detect slip based on force changes
        // Implementation depends on specific sensors and gripper type
        return false;  // Placeholder
    }
};
```

## Integration with Physical AI Systems

### Perception-Action Integration

Whole-body control systems must integrate with perception systems to operate effectively in real environments:

#### Visual Servoing Integration
```cpp
class VisualServoController {
private:
    std::unique_ptr<CameraInterface> camera_;
    std::unique_ptr<FeatureDetector> feature_detector_;
    std::unique_ptr<WholeBodyController> wb_controller_;
    
    // Visual servoing parameters
    Eigen::Matrix<double, 6, 6> visual_servo_gains_;
    double feature_threshold_;
    
    // Object tracking
    std::vector<Eigen::Vector2d> tracked_features_;
    std::vector<Eigen::Vector3d> object_positions_;
    
public:
    VisualServoController(std::unique_ptr<CameraInterface> cam,
                         std::unique_ptr<WholeBodyController> wb_ctrl)
        : camera_(std::move(cam)), wb_controller_(std::move(wb_ctrl)),
          feature_threshold_(5.0) {  // 5 pixel threshold
        
        // Initialize visual servo gains
        visual_servo_gains_.setZero();
        visual_servo_gains_.diagonal() << 1.0, 1.0, 0.5, 0.1, 0.1, 0.1;  // [x, y, z, roll, pitch, yaw]
    }
    
    std::vector<double> computeVisualServoControl(
        const std::vector<Eigen::Vector2d>& target_features,
        const std::vector<Eigen::Vector3d>& target_positions) {
        
        // Detect current features in camera image
        auto current_features = detectFeatures(camera_->getImage());
        
        // Calculate feature errors
        auto feature_errors = calculateFeatureErrors(current_features, target_features);
        
        // Calculate image Jacobian (relating feature motion to camera motion)
        auto image_jacobian = calculateImageJacobian(current_features);
        
        // Convert visual errors to Cartesian space
        Eigen::Vector6d cartesian_error = image_jacobian.inverse() * feature_errors;
        
        // Compute visual servo control
        Eigen::Vector6d visual_control = -visual_servo_gains_ * cartesian_error;
        
        // Integrate with whole-body control
        auto wb_control = wb_controller_->computeControl();
        
        // Blend visual servoing with whole-body control
        auto blended_control = blendControls(visual_control, wb_control);
        
        return blended_control;
    }

private:
    Eigen::VectorXd calculateFeatureErrors(
        const std::vector<Eigen::Vector2d>& current_features,
        const std::vector<Eigen::Vector2d>& target_features) {
        
        if (current_features.size() != target_features.size()) {
            throw std::runtime_error("Feature count mismatch in visual servoing");
        }
        
        Eigen::VectorXd errors(2 * current_features.size());
        
        for (size_t i = 0; i < current_features.size(); i++) {
            Eigen::Vector2d error = current_features[i] - target_features[i];
            errors.segment<2>(2*i) = error;
        }
        
        return errors;
    }
    
    Eigen::MatrixXd calculateImageJacobian(const std::vector<Eigen::Vector2d>& features) {
        // Calculate image Jacobian relating feature velocities to camera velocities
        // This depends on the camera model and feature types
        
        size_t n_features = features.size();
        Eigen::MatrixXd jacobian(2 * n_features, 6);  // 2D feature errors to 6D camera motion
        
        for (size_t i = 0; i < n_features; i++) {
            // Calculate image Jacobian for this feature
            auto Ji = calculateSingleFeatureJacobian(features[i]);
            jacobian.block(2*i, 0, 2, 6) = Ji;
        }
        
        return jacobian;
    }
    
    Eigen::Matrix<double, 2, 6> calculateSingleFeatureJacobian(const Eigen::Vector2d& feature) {
        // Calculate image Jacobian for a single feature point
        // This is camera-dependent and requires depth information
        
        double u = feature.x();
        double v = feature.y();
        double z = getFeatureDepth(feature);  // Requires depth information
        
        // Standard image Jacobian for a point feature
        Eigen::Matrix<double, 2, 6> jacobian;
        jacobian << 1/z, 0, -u/z, -u*v, (1+u*u), -v,
                    0, 1/z, -v/z, -(1+v*v), u*v, u;
        
        return jacobian;
    }
    
    double getFeatureDepth(const Eigen::Vector2d& feature) {
        // Get depth for the feature (from depth camera, stereo, or estimation)
        // Implementation depends on available depth information
        return 1.0;  // Placeholder
    }
    
    std::vector<double> blendControls(const Eigen::Vector6d& visual_control,
                                    const std::vector<double>& wb_control) {
        // Blend visual servoing control with whole-body control
        // This might involve priority-based blending or optimization-based blending
        
        std::vector<double> blended_control = wb_control;
        
        // Example: Add visual servoing as a task in the whole-body controller
        // This would be done by adding the visual control as a constraint or task
        
        return blended_control;
    }
};
```

### Integration with Navigation

#### Navigation-Aware Manipulation
```cpp
class NavigationAwareController {
private:
    std::unique_ptr<NavigationSystem> navigation_system_;
    std::unique_ptr<ManipulationController> manipulation_controller_;
    std::unique_ptr<WholeBodyController> whole_body_controller_;
    
    // Navigation constraints
    std::vector<Eigen::Vector3d> navigation_waypoints_;
    double navigation_priority_;
    double manipulation_priority_;
    
    // Collision avoidance integration
    std::unique_ptr<CollisionAvoidanceSystem> collision_avoider_;
    
public:
    NavigationAwareController(
        std::unique_ptr<NavigationSystem> nav_sys,
        std::unique_ptr<ManipulationController> manip_ctrl,
        std::unique_ptr<WholeBodyController> wb_ctrl)
        : navigation_system_(std::move(nav_sys)),
          manipulation_controller_(std::move(manip_ctrl)),
          whole_body_controller_(std::move(wb_ctrl)),
          navigation_priority_(0.8), manipulation_priority_(0.2) {
        
        collision_avoider_ = std::make_unique<CollisionAvoidanceSystem>();
    }
    
    std::vector<double> computeIntegratedControl(
        const geometry_msgs::msg::PoseStamped& goal_pose,
        const ManipulationTask& manipulation_task) {
        
        // Get navigation plan to goal
        auto nav_plan = navigation_system_->getPlanToGoal(goal_pose);
        
        // Get manipulation plan
        auto manip_plan = manipulation_controller_->getPlan(manipulation_task);
        
        // Check for conflicts between navigation and manipulation
        auto conflict_resolution = resolveConflicts(nav_plan, manip_plan);
        
        // Compute whole-body control that respects both navigation and manipulation goals
        auto wb_control = whole_body_controller_->computeIntegratedControl(
            nav_plan.current_waypoint, 
            manip_plan.current_goal,
            conflict_resolution.constraints);
        
        return wb_control;
    }

private:
    struct ConflictResolution {
        std::vector<Eigen::Vector3d> safe_positions;
        std::vector<Eigen::Vector3d> safe_orientations;
        std::vector<double> timing_constraints;
        std::vector<Eigen::VectorXd> priority_weights;
    };
    
    ConflictResolution resolveConflicts(const NavigationPlan& nav_plan,
                                      const ManipulationPlan& manip_plan) {
        
        ConflictResolution resolution;
        
        // Check if manipulator pose would interfere with navigation
        for (size_t i = 0; i < nav_plan.waypoints.size(); i++) {
            auto robot_pose = nav_plan.waypoints[i];
            auto manip_pose = manip_plan.goals[i];
            
            // Check if manipulator configuration would cause collision
            if (wouldCauseCollision(robot_pose, manip_pose)) {
                // Generate alternative manipulator configuration
                auto alternative_manip = collision_avoider_->findSafeConfiguration(
                    robot_pose, manip_pose);
                
                resolution.safe_positions.push_back(alternative_manip.position);
                resolution.safe_orientations.push_back(alternative_manip.orientation);
            } else {
                resolution.safe_positions.push_back(manip_pose.position);
                resolution.safe_orientations.push_back(manip_pose.orientation);
            }
        }
        
        // Calculate priority weights based on current task importance
        double nav_importance = calculateNavigationImportance(nav_plan);
        double manip_importance = calculateManipulationImportance(manip_plan);
        
        resolution.priority_weights.push_back(
            Eigen::VectorXd::Constant(6, nav_importance));
        resolution.priority_weights.push_back(
            Eigen::VectorXd::Constant(6, manip_importance));
        
        return resolution;
    }
    
    bool wouldCauseCollision(const Eigen::Vector3d& robot_pose,
                           const ManipulatorPose& manip_pose) {
        // Check if this manipulator configuration would cause collision
        // with navigation path or obstacles
        return false;  // Placeholder
    }
    
    double calculateNavigationImportance(const NavigationPlan& plan) {
        // Calculate importance based on navigation urgency, safety, etc.
        if (plan.isEmergency()) return 1.0;
        if (plan.isAvoidingObstacle()) return 0.8;
        return navigation_priority_;
    }
    
    double calculateManipulationImportance(const ManipulationPlan& plan) {
        // Calculate importance based on manipulation requirements
        if (plan.isCriticalOperation()) return 1.0;
        if (plan.isDelicateOperation()) return 0.9;
        return manipulation_priority_;
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

#### Example Optimized Control Loop
```cpp
class RealTimeWholeBodyController {
private:
    // Pre-allocated memory for computations
    Eigen::VectorXd joint_torques_buffer_;
    Eigen::MatrixXd mass_matrix_buffer_;
    Eigen::VectorXd coriolis_gravity_buffer_;
    
    // Fixed-size data structures
    std::vector<double> precomputed_jacobians_;
    std::vector<double> precomputed_inertias_;
    
    // Timing information
    rclcpp::Time last_update_time_;
    double control_frequency_;
    
public:
    RealTimeWholeBodyController(size_t num_joints, double frequency)
        : control_frequency_(frequency) {
        
        joint_torques_buffer_.resize(num_joints);
        mass_matrix_buffer_.resize(num_joints, num_joints);
        coriolis_gravity_buffer_.resize(num_joints);
        
        // Initialize with zeros to avoid dynamic allocation during control
        joint_torques_buffer_.setZero();
        mass_matrix_buffer_.setZero();
        coriolis_gravity_buffer_.setZero();
    }
    
    std::vector<double> computeControl(const RobotState& state, double dt) {
        // Verify timing constraints
        if (dt > 1.5 / control_frequency_) {
            RCLCPP_WARN_THROTTLE(
                rclcpp::get_logger("rt_wbc"), 
                *rclcpp::get_clock(), 
                1000,  // Throttle to 1s
                "Control loop timing violated: dt=%f, expected=%f", 
                dt, 1.0/control_frequency_);
        }
        
        // Perform control computation using pre-allocated buffers
        computeControlOptimized(state, joint_torques_buffer_);
        
        // Convert to std::vector (minimal allocation)
        std::vector<double> result(joint_torques_buffer_.size());
        Eigen::Map<Eigen::VectorXd>(result.data(), result.size()) = joint_torques_buffer_;
        
        return result;
    }

private:
    void computeControlOptimized(const RobotState& state, Eigen::VectorXd& output) {
        // Use pre-allocated buffers and optimized algorithms
        // Avoid dynamic memory allocation during control
        
        // Calculate mass matrix using optimized algorithm
        calculateMassMatrixOptimized(state.joint_positions, mass_matrix_buffer_);
        
        // Calculate Coriolis and gravity terms
        calculateCgTermsOptimized(state.joint_positions, 
                                state.joint_velocities, 
                                coriolis_gravity_buffer_);
        
        // Process each task using optimized methods
        for (const auto& task : tasks_) {
            processTaskOptimized(task, output);
        }
        
        // Add gravity compensation
        output += coriolis_gravity_buffer_;
    }
    
    void calculateMassMatrixOptimized(const std::vector<double>& positions,
                                    Eigen::MatrixXd& output) {
        // Use optimized algorithm for mass matrix calculation
        // Possibly using pre-computed terms or lookup tables
    }
    
    void processTaskOptimized(const Task& task, Eigen::VectorXd& output) {
        // Use optimized task processing
        // Avoid temporary object creation
        // Use efficient matrix operations
    }
};
```

### Parallel Processing

#### Multi-threaded Control Architecture
```cpp
#include <thread>
#include <future>

class ParallelController {
private:
    std::vector<std::thread> control_threads_;
    std::vector<std::future<std::vector<double>>> control_futures_;
    
    // Task distribution
    std::vector<std::vector<Task>> thread_tasks_;
    std::vector<std::mutex> thread_mutexes_;
    
    // Shared state
    RobotState shared_state_;
    std::mutex state_mutex_;
    
public:
    ParallelController(size_t num_threads, const std::vector<Task>& all_tasks) {
        control_threads_.resize(num_threads);
        thread_tasks_.resize(num_threads);
        thread_mutexes_.resize(num_threads);
        
        // Distribute tasks among threads
        distributeTasks(all_tasks);
    }
    
    std::vector<double> computeParallelControl(const RobotState& state) {
        // Update shared state
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            shared_state_ = state;
        }
        
        // Start computation on all threads
        for (size_t i = 0; i < control_threads_.size(); i++) {
            control_futures_[i] = std::async(std::launch::async, 
                                           &ParallelController::computeThreadControl, 
                                           this, i, state);
        }
        
        // Collect results from all threads
        std::vector<std::vector<double>> thread_results(control_threads_.size());
        for (size_t i = 0; i < control_threads_.size(); i++) {
            thread_results[i] = control_futures_[i].get();
        }
        
        // Combine results
        auto combined_result = combineThreadResults(thread_results);
        
        return combined_result;
    }

private:
    void distributeTasks(const std::vector<Task>& all_tasks) {
        // Distribute tasks among threads based on computational complexity
        // and dependencies
        size_t tasks_per_thread = all_tasks.size() / control_threads_.size();
        
        for (size_t i = 0; i < all_tasks.size(); i++) {
            size_t thread_idx = i % control_threads_.size();
            thread_tasks_[thread_idx].push_back(all_tasks[i]);
        }
    }
    
    std::vector<double> computeThreadControl(size_t thread_idx, const RobotState& state) {
        std::vector<double> thread_control;
        
        for (const auto& task : thread_tasks_[thread_idx]) {
            auto task_control = computeTaskControl(task, state);
            thread_control.insert(thread_control.end(), 
                                task_control.begin(), 
                                task_control.end());
        }
        
        return thread_control;
    }
    
    std::vector<double> combineThreadResults(const std::vector<std::vector<double>>& results) {
        // Combine results from different threads according to priority hierarchy
        // This might involve solving a centralized optimization problem
        // or using a decentralized combination method
        return {};
    }
};
```

## Troubleshooting Common Issues

### Control Problems

#### Instability and Oscillations
- **Symptoms**: Robot exhibits oscillatory behavior, control effort fluctuates wildly
- **Causes**: High gains, numerical instability, sensor noise, actuator limitations
- **Solutions**: Reduce gains, add filtering, improve numerical methods, check hardware

#### Poor Tracking Performance
- **Symptoms**: Robot doesn't follow desired trajectories accurately
- **Causes**: Model inaccuracies, insufficient control authority, external disturbances
- **Solutions**: Improve system identification, increase gains appropriately, add feedforward

#### Singularity Issues
- **Symptoms**: Control becomes erratic near singular configurations
- **Causes**: Jacobian becomes ill-conditioned
- **Solutions**: Use damped least squares, avoid singular configurations, add null-space tasks

#### Drift and Bias
- **Symptoms**: Gradual deviation from desired behavior
- **Causes**: Sensor bias, unmodeled dynamics, accumulated errors
- **Solutions**: Add integral action, calibrate sensors, improve models

### Implementation Challenges

#### Numerical Issues
- **Matrix Inversion**: Use SVD or QR decomposition instead of direct inversion
- **Condition Numbers**: Monitor condition numbers of matrices
- **Precision**: Use appropriate numerical precision for applications
- **Stability**: Verify numerical stability of control algorithms

#### Integration Problems
- **Timing**: Ensure proper timing synchronization between components
- **Coordinate Systems**: Verify coordinate system consistency
- **Units**: Check unit consistency across all components
- **Calibration**: Verify all calibration parameters are correct

## Safety Considerations

### Safety Architecture

#### Multi-layer Safety System
- **High-Level**: Task-level safety checking
- **Mid-Level**: Trajectory-level safety checking
- **Low-Level**: Joint-level safety checking
- **Hardware**: Physical safety limits and emergency stops

#### Safety Controllers
```cpp
class SafetyController {
private:
    std::unique_ptr<WholeBodyController> main_controller_;
    std::unique_ptr<EmergencyStopController> emergency_controller_;
    std::unique_ptr<LimitController> limit_controller_;
    
    // Safety parameters
    double max_velocity_limit_;
    double max_acceleration_limit_;
    double max_effort_limit_;
    double max_position_limit_;
    
    // Safety monitoring
    std::vector<SafetyMonitor> safety_monitors_;
    
public:
    SafetyController(std::unique_ptr<WholeBodyController> main_ctrl)
        : main_controller_(std::move(main_ctrl)),
          max_velocity_limit_(2.0), max_acceleration_limit_(5.0),
          max_effort_limit_(100.0), max_position_limit_(M_PI) {
        
        // Initialize safety monitors
        safety_monitors_.push_back(JointLimitMonitor());
        safety_monitors_.push_back(VelocityLimitMonitor());
        safety_monitors_.push_back(EffortLimitMonitor());
        safety_monitors_.push_back(CollisionMonitor());
    }
    
    std::vector<double> computeSafeControl(const RobotState& state,
                                         const ControlRequest& request) {
        // Compute main control
        auto main_control = main_controller_->computeControl(state, request);
        
        // Check safety constraints
        auto safety_status = checkSafetyConstraints(state, main_control);
        
        if (safety_status.all_safe) {
            return main_control;
        } else {
            // Apply safety corrections
            auto safe_control = applySafetyCorrections(main_control, safety_status);
            return safe_control;
        }
    }

private:
    struct SafetyStatus {
        bool all_safe;
        std::vector<bool> joint_limits_violated;
        std::vector<bool> velocity_limits_violated;
        std::vector<bool> effort_limits_violated;
        bool collision_detected;
        bool balance_lost;
    };
    
    SafetyStatus checkSafetyConstraints(const RobotState& state,
                                       const std::vector<double>& control) {
        SafetyStatus status;
        status.all_safe = true;
        
        // Check joint limits
        for (size_t i = 0; i < state.joint_positions.size(); i++) {
            if (std::abs(state.joint_positions[i]) > max_position_limit_) {
                status.joint_limits_violated[i] = true;
                status.all_safe = false;
            }
        }
        
        // Check velocity limits
        for (size_t i = 0; i < state.joint_velocities.size(); i++) {
            if (std::abs(state.joint_velocities[i]) > max_velocity_limit_) {
                status.velocity_limits_violated[i] = true;
                status.all_safe = false;
            }
        }
        
        // Check effort limits
        for (size_t i = 0; i < control.size(); i++) {
            if (std::abs(control[i]) > max_effort_limit_) {
                status.effort_limits_violated[i] = true;
                status.all_safe = false;
            }
        }
        
        // Check for collisions using environment model
        status.collision_detected = checkCollision(state);
        if (status.collision_detected) {
            status.all_safe = false;
        }
        
        // Check balance using ZMP or CoM
        status.balance_lost = !checkBalance(state);
        if (status.balance_lost) {
            status.all_safe = false;
        }
        
        return status;
    }
    
    std::vector<double> applySafetyCorrections(const std::vector<double>& control,
                                             const SafetyStatus& status) {
        auto corrected_control = control;
        
        // Apply joint limit corrections
        for (size_t i = 0; i < corrected_control.size(); i++) {
            if (status.joint_limits_violated[i]) {
                corrected_control[i] = applyJointLimitCorrection(i, corrected_control[i]);
            }
            
            if (status.velocity_limits_violated[i]) {
                corrected_control[i] = applyVelocityLimitCorrection(i, corrected_control[i]);
            }
            
            if (status.effort_limits_violated[i]) {
                corrected_control[i] = std::clamp(corrected_control[i], 
                                                -max_effort_limit_, 
                                                max_effort_limit_);
            }
        }
        
        // If collision detected, apply avoidance
        if (status.collision_detected) {
            auto avoidance_control = computeCollisionAvoidanceControl();
            corrected_control = blendControls(corrected_control, avoidance_control, 0.7, 0.3);
        }
        
        // If balance lost, apply balance recovery
        if (status.balance_lost) {
            auto balance_control = computeBalanceRecoveryControl();
            corrected_control = blendControls(corrected_control, balance_control, 0.3, 0.7);
        }
        
        return corrected_control;
    }
};
```

## Future Developments

### Emerging Control Technologies

#### Learning-Based Control
- **Adaptive Control**: Controllers that adjust parameters based on performance
- **Neural Network Controllers**: Learning controllers that adapt to system dynamics
- **Reinforcement Learning**: Controllers that learn optimal behaviors through interaction
- **Imitation Learning**: Controllers that learn from expert demonstrations

#### AI-Enhanced Control
- **Predictive Control**: Using AI predictions for proactive control
- **Context-Aware Control**: Controllers that adapt to environmental context
- **Multi-Modal Control**: Controllers that integrate multiple sensory modalities
- **Human-Robot Collaboration**: Control systems for human-robot interaction

### Advanced Integration Approaches

#### Perception-Action Integration
- **Direct Perception-Action Coupling**: Eliminating intermediate representations
- **Event-Based Control**: Control based on sensory events rather than continuous signals
- **Neuromorphic Control**: Inspired by biological neural control systems
- **Bio-Inspired Control**: Control strategies inspired by biological systems

## Conclusion

Whole-body control is essential for creating sophisticated Physical AI systems that can perform complex, coordinated tasks while maintaining stability and respecting physical constraints. The integration of multiple control objectives in a hierarchical or optimization-based framework enables robots to operate effectively in real-world environments where multiple tasks must be performed simultaneously.

The mathematical foundations of whole-body control, including rigid body dynamics, kinematic chains, and operational space control, provide the theoretical basis for implementing effective control systems. Understanding these foundations is crucial for designing controllers that can handle the complex dynamics of multi-DOF robots.

Modern whole-body control implementations leverage advanced techniques like hierarchical control, optimization-based control, and integration with perception systems to create robust and adaptive robotic behaviors. The choice of control approach depends on the specific requirements of the application, including performance needs, computational constraints, and safety requirements.

As robotics applications become more complex and operate in more dynamic environments, whole-body control systems must continue to evolve with improvements in computational efficiency, adaptability, and integration with AI systems. The future of whole-body control lies in the integration of traditional control methods with learning-based approaches that can adapt to new situations and improve performance over time.

Understanding these control techniques and their implementation is essential for creating Physical AI systems that can operate effectively in the real world, performing complex tasks while maintaining safety and stability.

## Exercises

1. Implement a whole-body controller for a simple manipulator that simultaneously controls end-effector position and maintains joint limit constraints.
2. Design and implement a balance controller for a bipedal robot using the inverted pendulum model.
3. Create a multi-task control system that performs manipulation while maintaining robot balance.

## Further Reading

- Sentis, L. (2010). "Compliant Control of Whole-Body Multi-Contact Behaviors in Walking Humanoid Robots." PhD Thesis, Stanford University.
- Khatib, O. (1987). "A unified approach for motion and force control of robot manipulators: The operational space formulation." IEEE Journal on Robotics and Automation.
- Featherstone, R. (2008). "Rigid Body Dynamics Algorithms." Springer.
- Siciliano, B., & Khatib, O. (Eds.). (2016). "Springer Handbook of Robotics." Springer.
- Park, H. J., & Khatib, O. (2016). "Whole-body articulated dynamics for multi-contact motion planning and control."