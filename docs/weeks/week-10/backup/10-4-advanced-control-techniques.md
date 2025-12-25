---
sidebar_label: Advanced Control Techniques
title: Advanced Control Techniques - Modern Control Methods for Robotics
description: Understanding advanced control techniques for robotics including PID, model predictive control, adaptive control, and robust control methods
keywords: [control systems, PID, MPC, adaptive control, robust control, robotics, feedback control, trajectory tracking]
---

# 10.4 Advanced Control Techniques

## Introduction

Advanced control techniques form the backbone of modern robotics systems, enabling precise, stable, and adaptive behavior in complex physical environments. While basic control methods like PID controllers provide the foundation, advanced control techniques are essential for handling the complexities of real-world robotic applications including non-linear dynamics, external disturbances, and multi-variable systems. This chapter explores modern control methodologies that enable robots to operate effectively in dynamic, uncertain environments.

The field of control engineering has evolved significantly with the advent of powerful computational platforms and sophisticated algorithms. Modern robotics applications demand controllers that can handle high-dimensional systems, adapt to changing conditions, and maintain stability in the presence of uncertainties. Advanced control techniques provide the mathematical frameworks and practical implementations needed to achieve these goals.

This chapter covers a range of advanced control methodologies from classical approaches like PID and state-space control to modern techniques such as Model Predictive Control (MPC), adaptive control, and robust control. We'll examine the theoretical foundations, practical implementation considerations, and integration strategies for deploying these techniques in real robotic systems.

## Classical Control Methods

### PID Controllers

Proportional-Integral-Derivative (PID) controllers remain the most widely used control strategy in robotics due to their simplicity, effectiveness, and intuitive understanding. A PID controller calculates the control output based on the weighted sum of the error, its integral, and its derivative.

#### PID Mathematical Formulation
The continuous-time PID controller is defined as:
```
u(t) = Kp * e(t) + Ki * ∫e(τ)dτ + Kd * de(t)/dt
```

Where:
- `u(t)` is the control output
- `e(t)` is the error (reference - actual)
- `Kp`, `Ki`, `Kd` are the proportional, integral, and derivative gains respectively

#### Discrete-Time PID Implementation
```cpp
class PIDController {
private:
    double kp_, ki_, kd_;           // Controller gains
    double setpoint_;               // Desired value
    double previous_error_;         // Error from previous step
    double integral_;               // Accumulated error
    double derivative_;             // Rate of error change
    double output_limits_[2];      // Output limits [min, max]
    double integral_limits_[2];    // Integral limits [min, max]
    double last_derivative_calc_time_;  // For filtering derivative

public:
    PIDController(double kp, double ki, double kd) 
        : kp_(kp), ki_(ki), kd_(kd), setpoint_(0.0), previous_error_(0.0), 
          integral_(0.0), derivative_(0.0) {
        
        // Initialize limits
        output_limits_[0] = -std::numeric_limits<double>::max();
        output_limits_[1] = std::numeric_limits<double>::max();
        integral_limits_[0] = -std::numeric_limits<double>::max();
        integral_limits_[1] = std::numeric_limits<double>::max();
    }
    
    double compute(double process_variable, double dt) {
        // Calculate error
        double error = setpoint_ - process_variable;
        
        // Proportional term
        double proportional = kp_ * error;
        
        // Integral term with anti-windup
        integral_ += error * dt;
        integral_ = clamp(integral_, integral_limits_[0], integral_limits_[1]);
        double integral = ki_ * integral_;
        
        // Derivative term with filtering to reduce noise
        derivative_ = (error - previous_error_) / dt;
        double derivative = kd_ * derivative_;
        
        // Calculate total output
        double output = proportional + integral + derivative;
        output = clamp(output, output_limits_[0], output_limits_[1]);
        
        // Store values for next iteration
        previous_error_ = error;
        
        return output;
    }
    
    void setSetpoint(double setpoint) { setpoint_ = setpoint; }
    void setGains(double kp, double ki, double kd) { 
        kp_ = kp; ki_ = ki; kd_ = kd; 
    }
    void setOutputLimits(double min, double max) { 
        output_limits_[0] = min; output_limits_[1] = max; 
    }
    void setIntegralLimits(double min, double max) { 
        integral_limits_[0] = min; integral_limits_[1] = max; 
    }
    void reset() { 
        integral_ = 0.0; 
        previous_error_ = 0.0; 
        derivative_ = 0.0; 
    }

private:
    double clamp(double value, double min, double max) {
        return std::max(min, std::min(max, value));
    }
};
```

#### PID Tuning Methods

##### Ziegler-Nichols Method
The Ziegler-Nichols method provides empirical rules for PID tuning:

```cpp
class PIDTuner {
public:
    struct ZieglerNicholsParams {
        double ku;  // Ultimate gain
        double tu;  // Ultimate period
    };
    
    static PIDGains tuneZieglerNichols(const ZieglerNicholsParams& params, 
                                      TuningRule rule = TuningRule::CLASSIC_PID) {
        PIDGains gains;
        
        switch (rule) {
            case TuningRule::CLASSIC_PID:
                gains.kp = 0.6 * params.ku;
                gains.ki = gains.kp / (0.5 * params.tu);
                gains.kd = gains.kp * 0.125 * params.tu;
                break;
                
            case TuningRule::PESSEN_INTEGRAL:
                gains.kp = 0.7 * params.ku;
                gains.ki = gains.kp / (0.4 * params.tu);
                gains.kd = gains.kp * 0.15 * params.tu;
                break;
                
            case TuningRule::SOME_OVERSHOOT:
                gains.kp = 0.33 * params.ku;
                gains.ki = gains.kp / (0.5 * params.tu);
                gains.kd = gains.kp * 0.33 * params.tu;
                break;
                
            case TuningRule::NO_OVERSHOOT:
                gains.kp = 0.2 * params.ku;
                gains.ki = gains.kp / (0.5 * params.tu);
                gains.kd = gains.kp * 0.33 * params.tu;
                break;
        }
        
        return gains;
    }
    
    static PIDGains tuneCohenCoon(const SystemParameters& params) {
        // Cohen-Coon tuning method for first-order systems with delay
        PIDGains gains;
        
        // Assuming first-order system: G(s) = K * e^(-Ls) / (Ts + 1)
        // Where K is gain, L is delay, T is time constant
        double K = params.gain;
        double T = params.time_constant;
        double L = params.delay;
        
        double tau = T / L;
        gains.kp = (1.35 / K) * (tau + 0.185);
        gains.ki = gains.kp / (2.5 * L);
        gains.kd = gains.kp * 0.37 * L;
        
        return gains;
    }

private:
    struct PIDGains {
        double kp, ki, kd;
    };
    
    enum class TuningRule {
        CLASSIC_PID,
        PESSEN_INTEGRAL,
        SOME_OVERSHOOT,
        NO_OVERSHOOT
    };
    
    struct SystemParameters {
        double gain;
        double time_constant;
        double delay;
    };
};
```

##### Advanced PID Variants

```cpp
class AdvancedPIDController {
private:
    PIDController base_pid_;
    double derivative_filter_coeff_;
    bool derivative_on_measurement_;
    bool integral_windup_protection_;
    std::function<double(double)> feedforward_function_;

public:
    AdvancedPIDController(double kp, double ki, double kd)
        : base_pid_(kp, ki, kd), 
          derivative_filter_coeff_(0.1),
          derivative_on_measurement_(false),
          integral_windup_protection_(true) {}
    
    double compute(double process_variable, double setpoint, 
                  double dt, double feedforward = 0.0) {
        
        base_pid_.setSetpoint(setpoint);
        
        // Apply feedforward if provided
        double feedforward_output = feedforward_function_ ? 
                                   feedforward_function_(feedforward) : feedforward;
        
        // Compute PID output
        double pid_output = base_pid_.compute(process_variable, dt);
        
        // Add feedforward to PID output
        double total_output = pid_output + feedforward_output;
        
        return total_output;
    }
    
    // Set feedforward function
    void setFeedforwardFunction(std::function<double(double)> func) {
        feedforward_function_ = func;
    }
    
    // Enable derivative filtering
    void setDerivativeFilter(double coeff) {
        derivative_filter_coeff_ = coeff;
    }
    
    // Enable derivative on measurement (for smoother response)
    void setDerivativeOnMeasurement(bool enable) {
        derivative_on_measurement_ = enable;
    }
};
```

### State-Space Control

State-space representation provides a more general framework for control system design, particularly useful for multi-input, multi-output (MIMO) systems.

#### Linear State-Space Model
```
ẋ(t) = A*x(t) + B*u(t)
y(t) = C*x(t) + D*u(t)
```

Where:
- `x` is the state vector
- `u` is the input vector
- `y` is the output vector
- `A`, `B`, `C`, `D` are system matrices

```cpp
#include <Eigen/Dense>

class StateSpaceController {
private:
    Eigen::MatrixXd A_, B_, C_, D_;  // System matrices
    Eigen::MatrixXd K_;              // Feedback gain matrix
    Eigen::VectorXd state_;          // Current state
    Eigen::VectorXd reference_;      // Reference state
    bool state_feedback_enabled_;

public:
    StateSpaceController(const Eigen::MatrixXd& A, 
                        const Eigen::MatrixXd& B,
                        const Eigen::MatrixXd& C,
                        const Eigen::MatrixXd& D)
        : A_(A), B_(B), C_(C), D_(D), state_feedback_enabled_(true) {
        
        // Initialize state vector
        state_ = Eigen::VectorXd::Zero(A.rows());
        reference_ = Eigen::VectorXd::Zero(A.rows());
        
        // Initialize gain matrix (zeros initially)
        K_ = Eigen::MatrixXd::Zero(B.cols(), A.cols());
    }
    
    Eigen::VectorXd compute(const Eigen::VectorXd& output, 
                          const Eigen::VectorXd& reference, 
                          double dt) {
        
        reference_ = reference;
        
        // Calculate state error
        Eigen::VectorXd error = reference_ - state_;
        
        // Compute control input using state feedback
        Eigen::VectorXd control_input = K_ * error;
        
        // Update state using system dynamics
        Eigen::VectorXd state_derivative = A_ * state_ + B_ * control_input;
        state_ += state_derivative * dt;
        
        return control_input;
    }
    
    // Set feedback gains
    void setGains(const Eigen::MatrixXd& K) {
        if (K.rows() == B_.cols() && K.cols() == A_.cols()) {
            K_ = K;
        }
    }
    
    // Compute LQR gains
    void computeLQRGains(const Eigen::MatrixXd& Q, 
                        const Eigen::MatrixXd& R,
                        const Eigen::MatrixXd& A_continuous,
                        const Eigen::MatrixXd& B_continuous) {
        
        // Solve the Algebraic Riccati Equation for LQR
        // This is a simplified version - in practice would use a numerical solver
        Eigen::MatrixXd P = solveRiccatiEquation(A_continuous, B_continuous, Q, R);
        
        // LQR gain: K = R^(-1) * B^T * P
        Eigen::MatrixXd K = R.inverse() * B_continuous.transpose() * P;
        K_ = K;
    }

private:
    Eigen::MatrixXd solveRiccatiEquation(const Eigen::MatrixXd& A, 
                                       const Eigen::MatrixXd& B,
                                       const Eigen::MatrixXd& Q,
                                       const Eigen::MatrixXd& R) {
        // In practice, this would use a numerical method like Schur decomposition
        // For now, returning a placeholder
        return Q;  // Placeholder - implement proper Riccati solver
    }
};

// Linear Quadratic Regulator (LQR) controller
class LQRController {
private:
    StateSpaceController state_space_controller_;
    Eigen::MatrixXd Q_;  // State cost matrix
    Eigen::MatrixXd R_;  // Control cost matrix
    Eigen::MatrixXd P_;  // Solution to Riccati equation

public:
    LQRController(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                 const Eigen::MatrixXd& C, const Eigen::MatrixXd& D,
                 const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R)
        : state_space_controller_(A, B, C, D), Q_(Q), R_(R) {
        
        // Compute optimal LQR gains
        computeOptimalGains();
    }
    
    Eigen::VectorXd compute(const Eigen::VectorXd& state,
                          const Eigen::VectorXd& reference,
                          double dt) {
        
        // For LQR, we want to minimize: J = ∫[x^T*Q*x + u^T*R*u]dt
        // The optimal control is u = -K*x where K is the LQR gain
        
        Eigen::VectorXd error = reference - state;
        Eigen::VectorXd control_input = -state_space_controller_.getGains() * error;
        
        return control_input;
    }

private:
    void computeOptimalGains() {
        // Solve the continuous-time algebraic Riccati equation:
        // A^T*P + P*A - P*B*R^(-1)*B^T*P + Q = 0
        P_ = solveCARE(state_space_controller_.getA(), 
                      state_space_controller_.getB(), Q_, R_);
        
        // Compute optimal gain: K = R^(-1)*B^T*P
        Eigen::MatrixXd K = R_.inverse() * state_space_controller_.getB().transpose() * P_;
        state_space_controller_.setGains(K);
    }
    
    Eigen::MatrixXd solveCARE(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                            const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
        // Implementation would use numerical methods
        // This is a placeholder implementation
        return Q;  // Placeholder
    }
};
```

## Modern Control Techniques

### Model Predictive Control (MPC)

Model Predictive Control (MPC) is an advanced control technique that uses a model of the system to predict future behavior and optimize control actions over a finite horizon. MPC is particularly effective for systems with constraints and multi-variable interactions.

#### MPC Mathematical Framework
MPC solves the following optimization problem at each time step:

```
minimize: ∑(k=0 to N-1) [x(k)ᵀQx(k) + u(k)ᵀRu(k)] + x(N)ᵀP x(N)
subject to: x(k+1) = Ax(k) + Bu(k)
            x_min ≤ x(k) ≤ x_max
            u_min ≤ u(k) ≤ u_max
```

```cpp
#include <Eigen/Dense>
#include <vector>

class ModelPredictiveController {
private:
    // System matrices
    Eigen::MatrixXd A_, B_, C_, Q_, R_, P_;
    int prediction_horizon_;
    int control_horizon_;
    double dt_;
    
    // Constraints
    Eigen::VectorXd x_min_, x_max_;
    Eigen::VectorXd u_min_, u_max_;
    
    // Prediction matrices
    Eigen::MatrixXd Phi_;  // State prediction matrix
    Eigen::MatrixXd Gamma_;  // Input-to-state prediction matrix
    
    // Current state
    Eigen::VectorXd current_state_;
    
    // Quadratic programming solver (simplified)
    std::unique_ptr<QPSolver> qp_solver_;

public:
    ModelPredictiveController(const Eigen::MatrixXd& A, 
                            const Eigen::MatrixXd& B,
                            const Eigen::MatrixXd& C,
                            int pred_horizon, 
                            int ctrl_horizon,
                            double dt)
        : A_(A), B_(B), C_(C), prediction_horizon_(pred_horizon), 
          control_horizon_(ctrl_horizon), dt_(dt) {
        
        // Initialize with identity matrices
        Q_ = Eigen::MatrixXd::Identity(A.rows(), A.cols());
        R_ = Eigen::MatrixXd::Identity(B.cols(), B.cols());
        P_ = Q_;  // Terminal cost same as state cost
        
        current_state_ = Eigen::VectorXd::Zero(A.rows());
        
        // Initialize constraints (no constraints initially)
        x_min_ = -Eigen::VectorXd::Ones(A.rows()) * std::numeric_limits<double>::max();
        x_max_ = Eigen::VectorXd::Ones(A.rows()) * std::numeric_limits<double>::max();
        u_min_ = -Eigen::VectorXd::Ones(B.cols()) * std::numeric_limits<double>::max();
        u_max_ = Eigen::VectorXd::Ones(B.cols()) * std::numeric_limits<double>::max();
        
        // Build prediction matrices
        buildPredictionMatrices();
    }
    
    Eigen::VectorXd compute(const Eigen::VectorXd& reference_trajectory,
                          const Eigen::VectorXd& current_state) {
        
        current_state_ = current_state;
        
        // Formulate QP problem: minimize (1/2)x'Hx + f'x
        // subject to A_ineq * x <= b_ineq
        auto [H, f, A_ineq, b_ineq] = formulateQPProblem(reference_trajectory);
        
        // Solve QP problem
        Eigen::VectorXd solution = solveQP(H, f, A_ineq, b_ineq);
        
        // Extract first control input from solution
        int control_dim = B_.cols();
        Eigen::VectorXd control_input = solution.head(control_dim);
        
        return control_input;
    }
    
    void setCostMatrices(const Eigen::MatrixXd& Q, 
                        const Eigen::MatrixXd& R, 
                        const Eigen::MatrixXd& P) {
        Q_ = Q;
        R_ = R;
        P_ = P;
    }
    
    void setStateConstraints(const Eigen::VectorXd& x_min, 
                           const Eigen::VectorXd& x_max) {
        x_min_ = x_min;
        x_max_ = x_max;
    }
    
    void setControlConstraints(const Eigen::VectorXd& u_min, 
                             const Eigen::VectorXd& u_max) {
        u_min_ = u_min;
        u_max_ = u_max;
    }

private:
    void buildPredictionMatrices() {
        int n = A_.rows();  // State dimension
        int m = B_.cols();  // Control dimension
        
        // Build state prediction matrix: X = Φ*x0 + Γ*U
        Phi_ = Eigen::MatrixXd::Zero(n * prediction_horizon_, n);
        Gamma_ = Eigen::MatrixXd::Zero(n * prediction_horizon_, m * control_horizon_);
        
        // Fill prediction matrix Φ
        Eigen::MatrixXd Ak = Eigen::MatrixXd::Identity(n, n);
        for (int k = 0; k < prediction_horizon_; k++) {
            if (k > 0) {
                Ak = Ak * A_;
            }
            Phi_.block(k * n, 0, n, n) = Ak;
        }
        
        // Fill control matrix Γ
        for (int i = 0; i < prediction_horizon_; i++) {
            for (int j = 0; j <= i && j < control_horizon_; j++) {
                Eigen::MatrixXd Aij = (i == j) ? 
                    Eigen::MatrixXd::Identity(n, n) : 
                    matrixPower(A_, i - j);
                Eigen::MatrixXd block = Aij * B_;
                
                Gamma_.block(i * n, j * m, n, m) = block;
            }
        }
    }
    
    std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::MatrixXd, Eigen::VectorXd>
    formulateQPProblem(const Eigen::VectorXd& reference_trajectory) {
        
        int n = A_.rows();  // State dimension
        int m = B_.cols();  // Control dimension
        int N = prediction_horizon_;
        int M = control_horizon_;
        
        // Total optimization variables: M * m control inputs
        int total_vars = M * m;
        
        // Cost function: J = (1/2) * U' * H * U + f' * U + constant
        // where U = [u(0); u(1); ...; u(M-1)]
        
        // State cost matrix (diagonal blocks)
        Eigen::MatrixXd Q_block = Eigen::MatrixXd::Zero(N * n, N * n);
        for (int i = 0; i < N; i++) {
            Q_block.block(i * n, i * n, n, n) = Q_;
        }
        
        // Terminal cost
        Eigen::MatrixXd Qf_block = Eigen::MatrixXd::Zero(n, n);
        Qf_block = P_;
        
        // Control cost matrix
        Eigen::MatrixXd R_block = Eigen::MatrixXd::Zero(M * m, M * m);
        for (int i = 0; i < M; i++) {
            R_block.block(i * m, i * m, m, m) = R_;
        }
        
        // Hessian matrix
        Eigen::MatrixXd H = Gamma_.transpose() * Q_block * Gamma_ + R_block;
        
        // Linear term (assuming reference tracking)
        Eigen::VectorXd x_ref_vec = reshapeReferenceTrajectory(reference_trajectory);
        Eigen::VectorXd linear_term = Gamma_.transpose() * Q_block * x_ref_vec;
        
        // Add term from current state
        Eigen::VectorXd current_state_expanded = expandStateVector(current_state_);
        linear_term += Gamma_.transpose() * Q_block * Phi_ * current_state_expanded;
        
        // f vector (linear term in cost function)
        Eigen::VectorXd f = -linear_term;
        
        // Constraints: A_ineq * U <= b_ineq
        // This includes state and control constraints
        
        // For simplicity, we'll implement basic box constraints
        // In practice, this would be more complex with prediction-based constraints
        int constraint_count = 2 * (M * m + N * n);  // Control and state constraints
        Eigen::MatrixXd A_ineq = Eigen::MatrixXd::Zero(constraint_count, total_vars);
        Eigen::VectorXd b_ineq = Eigen::VectorXd::Zero(constraint_count);
        
        // Control constraints: u_min <= u <= u_max
        for (int i = 0; i < M; i++) {
            for (int j = 0; j < m; j++) {
                // Upper bound: u[i*m + j] <= u_max[j]
                A_ineq(i * m + j, i * m + j) = 1.0;
                b_ineq(i * m + j) = u_max_[j];
                
                // Lower bound: -u[i*m + j] <= -u_min[j]
                A_ineq(M * m + i * m + j, i * m + j) = -1.0;
                b_ineq(M * m + i * m + j) = -u_min_[j];
            }
        }
        
        return std::make_tuple(H, f, A_ineq, b_ineq);
    }
    
    Eigen::VectorXd solveQP(const Eigen::MatrixXd& H, 
                           const Eigen::VectorXd& f,
                           const Eigen::MatrixXd& A_ineq, 
                           const Eigen::VectorXd& b_ineq) {
        
        // In practice, this would use a QP solver like OSQP, qpOASES, or Gurobi
        // For this implementation, we'll return a zero vector as a placeholder
        // A real implementation would call the actual QP solver
        
        int num_controls = B_.cols() * control_horizon_;
        Eigen::VectorXd solution = Eigen::VectorXd::Zero(num_controls);
        
        // Placeholder implementation - in real system, would solve the QP problem
        // using a proper QP solver
        
        return solution;
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
    
    Eigen::VectorXd reshapeReferenceTrajectory(const Eigen::VectorXd& ref_traj) {
        // Reshape reference trajectory to match prediction horizon
        int n = A_.rows();
        Eigen::VectorXd reshaped = Eigen::VectorXd::Zero(prediction_horizon_ * n);
        
        for (int i = 0; i < prediction_horizon_; i++) {
            int ref_idx = std::min(i, static_cast<int>(ref_traj.size() / n) - 1);
            reshaped.segment(i * n, n) = ref_traj.segment(ref_idx * n, n);
        }
        
        return reshaped;
    }
    
    Eigen::VectorXd expandStateVector(const Eigen::VectorXd& state) {
        // Expand single state vector to prediction horizon length
        int n = A_.rows();
        Eigen::VectorXd expanded = Eigen::VectorXd::Zero(prediction_horizon_ * n);
        
        for (int i = 0; i < prediction_horizon_; i++) {
            expanded.segment(i * n, n) = state;
        }
        
        return expanded;
    }
    
    class QPSolver {
    public:
        virtual Eigen::VectorXd solve(const Eigen::MatrixXd& H,
                                    const Eigen::VectorXd& f,
                                    const Eigen::MatrixXd& A_ineq,
                                    const Eigen::VectorXd& b_ineq) = 0;
    };
};

// Nonlinear MPC implementation
class NonlinearMPC {
private:
    // System dynamics function
    std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> dynamics_func_;
    
    // Cost function
    std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)> stage_cost_func_;
    std::function<double(const Eigen::VectorXd&)> terminal_cost_func_;
    
    int prediction_horizon_;
    double dt_;
    
    // Constraints
    std::function<bool(const Eigen::VectorXd&)> state_constraint_func_;
    std::function<bool(const Eigen::VectorXd&)> control_constraint_func_;

public:
    NonlinearMPC(std::function<Eigen::VectorXd(const Eigen::VectorXd&, const Eigen::VectorXd&)> dynamics,
                 std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)> stage_cost,
                 std::function<double(const Eigen::VectorXd&)> terminal_cost,
                 int horizon, double dt)
        : dynamics_func_(dynamics), stage_cost_func_(stage_cost), 
          terminal_cost_func_(terminal_cost), prediction_horizon_(horizon), dt_(dt) {}
    
    Eigen::VectorXd compute(const Eigen::VectorXd& reference_trajectory,
                          const Eigen::VectorXd& current_state) {
        
        // For nonlinear MPC, we typically use numerical optimization
        // This is a simplified implementation using iterative linearization
        
        // Linearize around current trajectory
        auto [A_lin, B_lin] = linearizeSystem(current_state, Eigen::VectorXd::Zero(B_.cols()));
        
        // Create linear MPC with linearized model
        ModelPredictiveController linear_mpc(A_lin, B_lin, Eigen::MatrixXd::Identity(A_lin.rows(), A_lin.rows()),
                                           prediction_horizon_, prediction_horizon_/2, dt_);
        
        // Solve linearized problem
        return linear_mpc.compute(reference_trajectory, current_state);
    }

private:
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd> linearizeSystem(
        const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
        
        int nx = x.size();
        int nu = u.size();
        
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nx, nx);
        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(nx, nu);
        
        double eps = 1e-8;
        
        // Numerical differentiation for A matrix
        for (int i = 0; i < nx; i++) {
            Eigen::VectorXd x_plus = x;
            Eigen::VectorXd x_minus = x;
            x_plus[i] += eps;
            x_minus[i] -= eps;
            
            Eigen::VectorXd f_plus = dynamics_func_(x_plus, u);
            Eigen::VectorXd f_minus = dynamics_func_(x_minus, u);
            
            A.col(i) = (f_plus - f_minus) / (2 * eps);
        }
        
        // Numerical differentiation for B matrix
        for (int i = 0; i < nu; i++) {
            Eigen::VectorXd u_plus = u;
            Eigen::VectorXd u_minus = u;
            u_plus[i] += eps;
            u_minus[i] -= eps;
            
            Eigen::VectorXd f_plus = dynamics_func_(x, u_plus);
            Eigen::VectorXd f_minus = dynamics_func_(x, u_minus);
            
            B.col(i) = (f_plus - f_minus) / (2 * eps);
        }
        
        return std::make_pair(A, B);
    }
    
    Eigen::MatrixXd A_, B_;  // Linearized system matrices
};
```

### Adaptive Control

Adaptive control systems adjust their parameters in real-time to accommodate changes in system dynamics or uncertainties in the model.

#### Model Reference Adaptive Control (MRAC)
```cpp
class ModelReferenceAdaptiveController {
private:
    // Reference model
    Eigen::MatrixXd Am_;  // Reference model matrix
    Eigen::MatrixXd Br_;  // Reference model input matrix
    
    // Adaptive parameters
    Eigen::MatrixXd Kx_;  // State feedback gain
    Eigen::MatrixXd Kr_;  // Reference gain
    Eigen::VectorXd Theta_;  // Adaptive parameter vector
    
    // Learning rates
    double gamma_x_;
    double gamma_r_;
    
    // Current state and reference
    Eigen::VectorXd state_;
    Eigen::VectorXd reference_state_;
    Eigen::VectorXd output_error_;
    
    // Parameter bounds
    Eigen::VectorXd theta_min_;
    Eigen::VectorXd theta_max_;

public:
    ModelReferenceAdaptiveController(const Eigen::MatrixXd& Am, 
                                   const Eigen::MatrixXd& Br,
                                   double gamma_x = 0.1, 
                                   double gamma_r = 0.1)
        : Am_(Am), Br_(Br), gamma_x_(gamma_x), gamma_r_(gamma_r) {
        
        int nx = Am.rows();
        int nu = Br.cols();
        
        // Initialize adaptive parameters
        Kx_ = Eigen::MatrixXd::Zero(nu, nx);
        Kr_ = Eigen::MatrixXd::Zero(nu, nx);
        Theta_ = Eigen::VectorXd::Zero(nx * nu + nx * nu);  // Combined parameters
        
        state_ = Eigen::VectorXd::Zero(nx);
        reference_state_ = Eigen::VectorXd::Zero(nx);
        output_error_ = Eigen::VectorXd::Zero(nx);
        
        // Initialize parameter bounds
        theta_min_ = -Eigen::VectorXd::Ones(Theta_.size()) * 100.0;
        theta_max_ = Eigen::VectorXd::Ones(Theta_.size()) * 100.0;
    }
    
    Eigen::VectorXd compute(const Eigen::VectorXd& system_output,
                          const Eigen::VectorXd& reference_model_output,
                          const Eigen::VectorXd& reference_input,
                          double dt) {
        
        // Calculate output error
        output_error_ = reference_model_output - system_output;
        
        // Update adaptive parameters using gradient descent
        updateParameters(system_output, reference_input, dt);
        
        // Calculate control input
        Eigen::VectorXd control_input = calculateControlInput(system_output, reference_input);
        
        return control_input;
    }

private:
    void updateParameters(const Eigen::VectorXd& y, 
                         const Eigen::VectorXd& r, 
                         double dt) {
        
        // Separate parameter vector into Kx and Kr components
        int nx = Am_.rows();
        int nu = Br_.cols();
        
        Eigen::VectorXd Kx_vec = Theta_.head(nx * nu);
        Eigen::VectorXd Kr_vec = Theta_.tail(nx * nu);
        
        // Reshape to matrices
        Eigen::Map<Eigen::MatrixXd> Kx(Kx_vec.data(), nu, nx);
        Eigen::Map<Eigen::MatrixXd> Kr(Kr_vec.data(), nu, nx);
        
        // Calculate parameter update using MIT rule
        Eigen::VectorXd phi_x = y;  // Regression vector for state
        Eigen::VectorXd phi_r = r;  // Regression vector for reference
        
        // Update laws
        Eigen::VectorXd dKx = gamma_x_ * output_error_ * phi_x.transpose() * dt;
        Eigen::VectorXd dKr = gamma_r_ * output_error_ * phi_r.transpose() * dt;
        
        // Update parameter vectors
        Kx_vec += Eigen::Map<Eigen::VectorXd>(dKx.data(), dKx.size());
        Kr_vec += Eigen::Map<Eigen::VectorXd>(dKr.data(), dKr.size());
        
        // Apply parameter bounds
        Kx_vec = applyBounds(Kx_vec, theta_min_.head(nx * nu), theta_max_.head(nx * nu));
        Kr_vec = applyBounds(Kr_vec, theta_min_.tail(nx * nu), theta_max_.tail(nx * nu));
        
        // Update combined parameter vector
        Theta_.head(nx * nu) = Kx_vec;
        Theta_.tail(nx * nu) = Kr_vec;
    }
    
    Eigen::VectorXd calculateControlInput(const Eigen::VectorXd& y, 
                                       const Eigen::VectorXd& r) {
        
        int nx = Am_.rows();
        int nu = Br_.cols();
        
        // Extract parameter matrices
        Eigen::VectorXd Kx_vec = Theta_.head(nx * nu);
        Eigen::VectorXd Kr_vec = Theta_.tail(nx * nu);
        
        Eigen::Map<Eigen::MatrixXd> Kx(Kx_vec.data(), nu, nx);
        Eigen::Map<Eigen::MatrixXd> Kr(Kr_vec.data(), nu, nx);
        
        // Calculate control input: u = -Kx*y + Kr*r
        Eigen::VectorXd control_input = -Kx * y + Kr * r;
        
        return control_input;
    }
    
    Eigen::VectorXd applyBounds(const Eigen::VectorXd& params,
                              const Eigen::VectorXd& min_vals,
                              const Eigen::VectorXd& max_vals) {
        
        Eigen::VectorXd bounded = params;
        for (int i = 0; i < params.size(); i++) {
            bounded[i] = std::max(min_vals[i], std::min(max_vals[i], params[i]));
        }
        return bounded;
    }
};

// Self-organizing adaptive controller
class SelfOrganizingAdaptiveController {
private:
    // Radial Basis Function Network for function approximation
    std::vector<RBFNeuron> rbf_neurons_;
    Eigen::VectorXd weights_;
    int num_neurons_;
    double sigma_;  // RBF width parameter
    
    // Adaptive law parameters
    double learning_rate_;
    double normalization_constant_;
    
    // System state
    Eigen::VectorXd state_history_;
    Eigen::VectorXd control_history_;
    int history_size_;

public:
    SelfOrganizingAdaptiveController(int num_neurons, double sigma, double learning_rate)
        : num_neurons_(num_neurons), sigma_(sigma), learning_rate_(learning_rate),
          normalization_constant_(1.0), history_size_(10) {
        
        // Initialize RBF neurons with random centers
        for (int i = 0; i < num_neurons_; i++) {
            RBFNeuron neuron;
            neuron.center = Eigen::VectorXd::Random(2);  // Assuming 2D state
            rbf_neurons_.push_back(neuron);
        }
        
        // Initialize weights
        weights_ = Eigen::VectorXd::Zero(num_neurons_);
        
        // Initialize history
        state_history_ = Eigen::VectorXd::Zero(history_size_ * 2);  // 2D state
        control_history_ = Eigen::VectorXd::Zero(history_size_);
    }
    
    double compute(const Eigen::VectorXd& state, 
                  const double desired_output,
                  double dt) {
        
        // Calculate RBF network output
        Eigen::VectorXd rbf_outputs(num_neurons_);
        for (int i = 0; i < num_neurons_; i++) {
            rbf_outputs[i] = calculateRBFOutput(rbf_neurons_[i], state);
        }
        
        // Calculate network output
        double network_output = weights_.dot(rbf_outputs);
        
        // Calculate error
        double error = desired_output - network_output;
        
        // Update weights using delta rule
        Eigen::VectorXd delta_weights = learning_rate_ * error * rbf_outputs;
        weights_ += delta_weights * dt;
        
        // Calculate control output
        double control_output = network_output;
        
        // Update history
        updateHistory(state, control_output);
        
        return control_output;
    }

private:
    struct RBFNeuron {
        Eigen::VectorXd center;
        double output;
    };
    
    double calculateRBFOutput(const RBFNeuron& neuron, const Eigen::VectorXd& input) {
        // Gaussian RBF: exp(-||x - c||^2 / (2*sigma^2))
        double diff_norm = (input - neuron.center).norm();
        return std::exp(-diff_norm * diff_norm / (2 * sigma_ * sigma_));
    }
    
    void updateHistory(const Eigen::VectorXd& state, double control) {
        // Shift history and add new values
        for (int i = 0; i < history_size_ - 1; i++) {
            state_history_.segment(i * 2, 2) = state_history_.segment((i + 1) * 2, 2);
            control_history_[i] = control_history_[i + 1];
        }
        
        state_history_.segment((history_size_ - 1) * 2, 2) = state;
        control_history_[history_size_ - 1] = control;
    }
};
```

### Robust Control

Robust control techniques ensure system stability and performance in the presence of uncertainties and disturbances.

#### H-infinity Control
```cpp
class HInfinityController {
private:
    // System matrices in LFT (Linear Fractional Transformation) form
    Eigen::MatrixXd A_, B1_, B2_, C1_, C2_, D11_, D12_, D21_, D22_;
    
    // Controller matrices
    Eigen::MatrixXd Ak_, Bk_, Ck_, Dk_;
    
    // H-infinity performance level
    double gamma_;
    
    // Synthesis parameters
    double gamma_tolerance_;
    int max_iterations_;

public:
    HInfinityController(double gamma = 1.0)
        : gamma_(gamma), gamma_tolerance_(1e-6), max_iterations_(100) {}
    
    bool synthesizeController(const Eigen::MatrixXd& A, 
                            const Eigen::MatrixXd& B1,
                            const Eigen::MatrixXd& B2,
                            const Eigen::MatrixXd& C1,
                            const Eigen::MatrixXd& C2,
                            const Eigen::MatrixXd& D11,
                            const Eigen::MatrixXd& D12,
                            const Eigen::MatrixXd& D21,
                            const Eigen::MatrixXd& D22) {
        
        A_ = A; B1_ = B1; B2_ = B2; C1_ = C1; C2_ = C2;
        D11_ = D11; D12_ = D12; D21_ = D21; D22_ = D22;
        
        // Solve H-infinity control problem
        // This involves solving two coupled Riccati equations
        return solveHInfinityControl();
    }
    
    Eigen::VectorXd compute(const Eigen::VectorXd& measurement,
                          const Eigen::VectorXd& state) {
        
        // Implement controller dynamics: ẋk = Ak*xk + Bk*y
        // u = Ck*xk + Dk*y
        Eigen::VectorXd control_output = Ck_ * state + Dk_ * measurement;
        
        return control_output;
    }

private:
    bool solveHInfinityControl() {
        // H-infinity control synthesis involves solving:
        // 1. State feedback gain that minimizes ||Tzw||∞ < γ
        // 2. Using Riccati equations or LMI (Linear Matrix Inequalities)
        
        // This is a simplified placeholder implementation
        // In practice, would use sophisticated numerical methods
        
        try {
            // Solve the H-infinity Riccati equations
            Eigen::MatrixXd X = solveHInfinityRiccati();
            
            // Calculate controller matrices
            calculateControllerMatrices(X);
            
            return true;
        } catch (const std::exception& e) {
            std::cerr << "H-infinity synthesis failed: " << e.what() << std::endl;
            return false;
        }
    }
    
    Eigen::MatrixXd solveHInfinityRiccati() {
        // Solve the H-infinity algebraic Riccati equation
        // A'X + XA - XBR^(-1)B'X + Q = 0 (simplified form)
        
        // This is a placeholder - real implementation would use
        // numerical methods to solve the coupled Riccati equations
        int n = A_.rows();
        return Eigen::MatrixXd::Identity(n, n);  // Placeholder
    }
    
    void calculateControllerMatrices(const Eigen::MatrixXd& X) {
        // Calculate controller matrices based on solution X
        // This is a simplified representation
        int n = A_.rows();
        int m = B2_.cols();
        int p = C2_.rows();
        
        // Placeholder controller synthesis
        Ak_ = Eigen::MatrixXd::Zero(n, n);
        Bk_ = Eigen::MatrixXd::Zero(n, p);
        Ck_ = Eigen::MatrixXd::Zero(m, n);
        Dk_ = Eigen::MatrixXd::Zero(m, p);
    }
};

// Sliding Mode Control (SMC) - Robust control technique
class SlidingModeController {
private:
    // Sliding surface parameters
    Eigen::VectorXd S_;  // Sliding surface coefficients
    double lambda_;      // Sliding surface slope
    
    // Control parameters
    double K_;           // Switching gain
    double phi_;         // Boundary layer thickness
    double eta_;         // Robustness parameter
    
    // System parameters
    double dt_;
    Eigen::VectorXd state_prev_;
    bool boundary_layer_active_;

public:
    SlidingModeController(const Eigen::VectorXd& surface_params, 
                         double lambda, double K, double phi, double eta, double dt)
        : S_(surface_params), lambda_(lambda), K_(K), phi_(phi), eta_(eta), dt_(dt) {
        
        state_prev_ = Eigen::VectorXd::Zero(surface_params.size());
        boundary_layer_active_ = true;
    }
    
    double compute(const Eigen::VectorXd& state, 
                  const Eigen::VectorXd& reference = Eigen::VectorXd()) {
        
        // Calculate sliding surface
        double s = calculateSlidingSurface(state);
        
        // Calculate control law
        double control = calculateControlLaw(s, state);
        
        // Update previous state
        state_prev_ = state;
        
        return control;
    }

private:
    double calculateSlidingSurface(const Eigen::VectorXd& state) {
        // Sliding surface: s = σ'(x - x_ref) + λ(x - x_ref)
        // For simplicity, assuming first-order sliding surface
        if (state_prev_.size() == state.size()) {
            Eigen::VectorXd error = state - state_prev_;
            return S_.dot(error) + lambda_ * S_.dot(state);
        }
        return S_.dot(state);  // Simplified case
    }
    
    double calculateControlLaw(double s, const Eigen::VectorXd& state) {
        double control = 0.0;
        
        if (boundary_layer_active_ && std::abs(s) <= phi_) {
            // Continuous approximation in boundary layer
            control = -K_ * s / phi_ * getSign(s);
        } else {
            // Discontinuous switching control
            control = -K_ * getSign(s) - eta_ * s;  // Add linear term for smoothness
        }
        
        return control;
    }
    
    double getSign(double x) {
        if (x > 0) return 1.0;
        else if (x < 0) return -1.0;
        else return 0.0;
    }
};
```

## Specialized Control Techniques

### Optimal Control

#### Linear Quadratic Gaussian (LQG) Control
```cpp
class LQGController {
private:
    // LQR controller component
    LQRController lqr_controller_;
    
    // Kalman filter for state estimation
    KalmanFilter kalman_filter_;
    
    // System matrices
    Eigen::MatrixXd A_, B_, C_, Q_, R_, Qv_, Qw_;  // Qv: process noise, Qw: measurement noise

public:
    LQGController(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                 const Eigen::MatrixXd& C, const Eigen::MatrixXd& Q,
                 const Eigen::MatrixXd& R, const Eigen::MatrixXd& Qv,
                 const Eigen::MatrixXd& Qw)
        : A_(A), B_(B), C_(C), Q_(Q), R_(R), Qv_(Qv), Qw_(Qw) {
        
        // Initialize LQR controller
        lqr_controller_ = LQRController(A, B, C, Q, R);
        
        // Initialize Kalman filter
        kalman_filter_ = KalmanFilter(A, B, C, Qv, Qw);
    }
    
    Eigen::VectorXd compute(const Eigen::VectorXd& measurement,
                          const Eigen::VectorXd& reference,
                          double dt) {
        
        // Estimate state using Kalman filter
        Eigen::VectorXd estimated_state = kalman_filter_.update(measurement, dt);
        
        // Compute control using LQR with estimated state
        Eigen::VectorXd control_input = lqr_controller_.compute(estimated_state, reference, dt);
        
        return control_input;
    }
    
    Eigen::VectorXd getEstimatedState() const {
        return kalman_filter_.getState();
    }

private:
    class KalmanFilter {
    private:
        Eigen::MatrixXd A_, B_, C_;
        Eigen::MatrixXd Q_, R_;  // Process and measurement noise covariance
        Eigen::VectorXd x_hat_;  // State estimate
        Eigen::MatrixXd P_;      // Error covariance matrix

    public:
        KalmanFilter(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B,
                    const Eigen::MatrixXd& C, const Eigen::MatrixXd& Q,
                    const Eigen::MatrixXd& R)
            : A_(A), B_(B), C_(C), Q_(Q), R_(R) {
            
            x_hat_ = Eigen::VectorXd::Zero(A.rows());
            P_ = Eigen::MatrixXd::Identity(A.rows(), A.rows());
        }
        
        Eigen::VectorXd update(const Eigen::VectorXd& measurement, double dt) {
            // Prediction step
            Eigen::VectorXd x_pred = A_ * x_hat_;
            Eigen::MatrixXd P_pred = A_ * P_ * A_.transpose() + Q_;
            
            // Kalman gain
            Eigen::MatrixXd S = C_ * P_pred * C_.transpose() + R_;
            Eigen::MatrixXd K = P_pred * C_.transpose() * S.inverse();
            
            // Update step
            Eigen::VectorXd innovation = measurement - C_ * x_pred;
            x_hat_ = x_pred + K * innovation;
            P_ = (Eigen::MatrixXd::Identity(P_.rows(), P_.cols()) - K * C_) * P_pred;
            
            return x_hat_;
        }
        
        Eigen::VectorXd getState() const { return x_hat_; }
    };
};
```

### Learning-Based Control

#### Reinforcement Learning for Control
```cpp
class RLBasedController {
private:
    // Neural network for policy or value function
    std::unique_ptr<NeuralNetwork> policy_network_;
    std::unique_ptr<NeuralNetwork> value_network_;
    
    // Experience replay buffer
    std::deque<Experience> replay_buffer_;
    size_t buffer_capacity_;
    
    // Learning parameters
    double learning_rate_;
    double discount_factor_;
    double exploration_rate_;
    double target_update_rate_;
    
    // Current policy parameters
    std::vector<double> policy_params_;
    std::vector<double> target_params_;

public:
    RLBasedController(size_t state_dim, size_t action_dim, 
                     size_t buffer_size = 10000)
        : buffer_capacity_(buffer_size), learning_rate_(0.001),
          discount_factor_(0.99), exploration_rate_(0.1),
          target_update_rate_(0.005) {
        
        // Initialize neural networks
        policy_network_ = std::make_unique<DeepNeuralNetwork>(
            std::vector<int>{static_cast<int>(state_dim), 64, 64, static_cast<int>(action_dim)});
        value_network_ = std::make_unique<DeepNeuralNetwork>(
            std::vector<int>{static_cast<int>(state_dim + action_dim), 64, 64, 1});
        
        // Initialize parameters
        policy_params_ = policy_network_->getParameters();
        target_params_ = policy_params_;
    }
    
    Eigen::VectorXd compute(const Eigen::VectorXd& state) {
        // Add exploration noise or use policy network
        if (static_cast<double>(std::rand()) / RAND_MAX < exploration_rate_) {
            // Exploration: random action
            return Eigen::VectorXd::Random(state.size());  // Simplified
        } else {
            // Exploitation: use learned policy
            return policy_network_->forward(state);
        }
    }
    
    void update(const Experience& experience) {
        // Store experience in replay buffer
        replay_buffer_.push_back(experience);
        if (replay_buffer_.size() > buffer_capacity_) {
            replay_buffer_.pop_front();
        }
        
        // Sample batch from replay buffer
        auto batch = sampleBatch(BATCH_SIZE);
        
        // Update networks using batch
        updateNetworks(batch);
        
        // Soft update target networks
        updateTargetNetworks();
    }

private:
    struct Experience {
        Eigen::VectorXd state;
        Eigen::VectorXd action;
        double reward;
        Eigen::VectorXd next_state;
        bool done;
    };
    
    std::vector<Experience> sampleBatch(size_t batch_size) {
        std::vector<Experience> batch;
        if (replay_buffer_.size() < batch_size) {
            batch.assign(replay_buffer_.begin(), replay_buffer_.end());
        } else {
            // Random sampling
            std::sample(replay_buffer_.begin(), replay_buffer_.end(),
                       std::back_inserter(batch), batch_size,
                       std::mt19937{std::random_device{}()});
        }
        return batch;
    }
    
    void updateNetworks(const std::vector<Experience>& batch) {
        // Implement policy gradient or Q-learning update
        // This is a simplified placeholder
        
        for (const auto& exp : batch) {
            // Calculate target for value network
            double target = exp.reward;
            if (!exp.done) {
                // Use target network for next state value
                auto next_action = getTargetAction(exp.next_state);
                double next_value = getValue(exp.next_state, next_action);
                target += discount_factor_ * next_value;
            }
            
            // Update value network
            updateValueNetwork(exp.state, exp.action, target);
            
            // Update policy network using policy gradient
            updatePolicyNetwork(exp.state, exp.action);
        }
    }
    
    void updateValueNetwork(const Eigen::VectorXd& state, 
                           const Eigen::VectorXd& action,
                           double target) {
        // Calculate value network loss and update parameters
        // Implementation would use gradient descent
    }
    
    void updatePolicyNetwork(const Eigen::VectorXd& state, 
                            const Eigen::VectorXd& action) {
        // Calculate policy gradient and update parameters
        // Implementation would use policy gradient methods
    }
    
    void updateTargetNetworks() {
        // Soft update target network parameters
        for (size_t i = 0; i < target_params_.size(); i++) {
            target_params_[i] = target_update_rate_ * policy_params_[i] + 
                               (1 - target_update_rate_) * target_params_[i];
        }
    }
    
    Eigen::VectorXd getTargetAction(const Eigen::VectorXd& state) {
        // Get action from target policy
        return Eigen::VectorXd();  // Placeholder
    }
    
    double getValue(const Eigen::VectorXd& state, const Eigen::VectorXd& action) {
        // Get value from target network
        return 0.0;  // Placeholder
    }
    
    static constexpr size_t BATCH_SIZE = 32;
    
    class NeuralNetwork {
    public:
        virtual Eigen::VectorXd forward(const Eigen::VectorXd& input) = 0;
        virtual std::vector<double> getParameters() = 0;
        virtual void setParameters(const std::vector<double>& params) = 0;
    };
    
    class DeepNeuralNetwork : public NeuralNetwork {
    private:
        std::vector<int> layer_sizes_;
        std::vector<Eigen::MatrixXd> weights_;
        std::vector<Eigen::VectorXd> biases_;

    public:
        DeepNeuralNetwork(const std::vector<int>& layer_sizes) : layer_sizes_(layer_sizes) {
            // Initialize weights and biases randomly
            for (size_t i = 0; i < layer_sizes_.size() - 1; i++) {
                weights_.push_back(Eigen::MatrixXd::Random(layer_sizes_[i+1], layer_sizes_[i]));
                biases_.push_back(Eigen::VectorXd::Random(layer_sizes_[i+1]));
            }
        }
        
        Eigen::VectorXd forward(const Eigen::VectorXd& input) override {
            Eigen::VectorXd output = input;
            
            for (size_t i = 0; i < weights_.size(); i++) {
                output = weights_[i] * output + biases_[i];
                // Apply activation function (ReLU)
                output = output.array().max(0.0);
            }
            
            return output;
        }
        
        std::vector<double> getParameters() override {
            std::vector<double> params;
            
            for (const auto& w : weights_) {
                for (int i = 0; i < w.size(); i++) {
                    params.push_back(w.data()[i]);
                }
            }
            
            for (const auto& b : biases_) {
                for (int i = 0; i < b.size(); i++) {
                    params.push_back(b.data()[i]);
                }
            }
            
            return params;
        }
        
        void setParameters(const std::vector<double>& params) override {
            size_t idx = 0;
            
            for (auto& w : weights_) {
                for (int i = 0; i < w.size(); i++) {
                    w.data()[i] = params[idx++];
                }
            }
            
            for (auto& b : biases_) {
                for (int i = 0; i < b.size(); i++) {
                    b.data()[i] = params[idx++];
                }
            }
        }
    };
};
```

## Control System Integration

### Multi-Loop Control Architecture

```cpp
class MultiLoopController {
private:
    // Hierarchical control structure
    std::unique_ptr<PositionController> position_controller_;
    std::unique_ptr<VelocityController> velocity_controller_;
    std::unique_ptr<ForceController> force_controller_;
    
    // Coordination manager
    std::unique_ptr<CoordinationManager> coordination_manager_;
    
    // Safety system
    std::unique_ptr<SafetyController> safety_controller_;

public:
    MultiLoopController() {
        position_controller_ = std::make_unique<PIDController>(100.0, 10.0, 15.0);
        velocity_controller_ = std::make_unique<PIDController>(50.0, 5.0, 8.0);
        force_controller_ = std::make_unique<PIDController>(200.0, 20.0, 30.0);
        coordination_manager_ = std::make_unique<LoopCoordinationManager>();
        safety_controller_ = std::make_unique<SafetyLimitController>();
    }
    
    ControlOutput compute(const SystemState& state, 
                         const ControlReference& reference,
                         double dt) {
        
        ControlOutput output;
        
        // Compute force control (innermost loop)
        if (reference.force_control_enabled) {
            output.force = force_controller_->compute(
                state.force, reference.force, dt);
        }
        
        // Compute velocity control (middle loop)
        if (reference.velocity_control_enabled) {
            // Include force feedback in velocity control
            double velocity_reference = reference.velocity;
            if (reference.force_control_enabled) {
                velocity_reference += output.force * FORCE_TO_VELOCITY_GAIN;
            }
            
            output.velocity = velocity_controller_->compute(
                state.velocity, velocity_reference, dt);
        }
        
        // Compute position control (outer loop)
        if (reference.position_control_enabled) {
            // Include velocity feedback in position control
            double position_reference = reference.position;
            if (reference.velocity_control_enabled) {
                position_reference += output.velocity * VELOCITY_TO_POSITION_GAIN;
            }
            
            output.position = position_controller_->compute(
                state.position, position_reference, dt);
        }
        
        // Apply safety limits
        output = safety_controller_->applyLimits(output, state);
        
        // Coordinate loops to prevent conflicts
        output = coordination_manager_->coordinateLoops(output, state);
        
        return output;
    }

private:
    struct SystemState {
        double position;
        double velocity;
        double acceleration;
        double force;
        double current;
        double temperature;
    };
    
    struct ControlReference {
        double position;
        double velocity;
        double acceleration;
        double force;
        bool position_control_enabled;
        bool velocity_control_enabled;
        bool force_control_enabled;
    };
    
    struct ControlOutput {
        double position;
        double velocity;
        double force;
        double current;
        double effort;
    };
    
    class LoopCoordinationManager {
    public:
        ControlOutput coordinateLoops(const ControlOutput& raw_output,
                                    const SystemState& state) {
            
            ControlOutput coordinated_output = raw_output;
            
            // Prevent conflicts between loops
            // For example, if force loop is saturated, adjust position loop reference
            if (std::abs(raw_output.force) > MAX_FORCE_LIMIT * 0.95) {
                // Reduce position control effort when force is near limits
                coordinated_output.position *= 0.8;  // Reduce effort
            }
            
            // Check for velocity saturation
            if (std::abs(raw_output.velocity) > MAX_VELOCITY_LIMIT * 0.95) {
                coordinated_output.position *= 0.9;  // Reduce position command
            }
            
            return coordinated_output;
        }

    private:
        static constexpr double MAX_FORCE_LIMIT = 100.0;
        static constexpr double MAX_VELOCITY_LIMIT = 1.0;
    };
    
    class SafetyLimitController {
    public:
        ControlOutput applyLimits(const ControlOutput& raw_output,
                                const SystemState& state) {
            
            ControlOutput limited_output = raw_output;
            
            // Apply force limits
            limited_output.force = std::clamp(limited_output.force, 
                                            -MAX_FORCE_LIMIT, MAX_FORCE_LIMIT);
            
            // Apply velocity limits
            limited_output.velocity = std::clamp(limited_output.velocity,
                                              -MAX_VELOCITY_LIMIT, MAX_VELOCITY_LIMIT);
            
            // Apply position limits
            limited_output.position = std::clamp(limited_output.position,
                                              -MAX_POSITION_LIMIT, MAX_POSITION_LIMIT);
            
            // Check for safety violations
            if (state.temperature > MAX_TEMPERATURE) {
                // Reduce all control efforts for safety
                limited_output.force *= 0.5;
                limited_output.velocity *= 0.5;
                limited_output.position *= 0.5;
            }
            
            return limited_output;
        }

    private:
        static constexpr double MAX_FORCE_LIMIT = 100.0;
        static constexpr double MAX_VELOCITY_LIMIT = 1.0;
        static constexpr double MAX_POSITION_LIMIT = 10.0;
        static constexpr double MAX_TEMPERATURE = 80.0;  // Celsius
    };
    
    static constexpr double FORCE_TO_VELOCITY_GAIN = 0.1;
    static constexpr double VELOCITY_TO_POSITION_GAIN = 0.05;
};
```

## Implementation Considerations

### Real-time Implementation

#### Control Loop Timing
```cpp
class RealTimeController {
private:
    rclcpp::TimerBase::SharedPtr control_timer_;
    std::chrono::high_resolution_clock::time_point last_execution_time_;
    std::vector<double> execution_times_;
    size_t max_execution_history_;
    
    // Control loop parameters
    double nominal_period_;
    double max_execution_time_;
    std::mutex control_mutex_;

public:
    RealTimeController(double frequency_hz, size_t max_history = 1000)
        : nominal_period_(1.0 / frequency_hz), max_execution_history_(max_history) {
        
        max_execution_time_ = nominal_period_ * 0.8;  // 80% of period for safety
        
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(nominal_period_ * 1000)),
            std::bind(&RealTimeController::controlLoop, this));
    }
    
    void controlLoop() {
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Lock to prevent concurrent execution
        std::lock_guard<std::mutex> lock(control_mutex_);
        
        // Execute control algorithm
        executeControlAlgorithm();
        
        // Record execution time
        auto end_time = std::chrono::high_resolution_clock::now();
        double execution_time = std::chrono::duration<double, std::milli>(
            end_time - start_time).count();
        
        execution_times_.push_back(execution_time);
        if (execution_times_.size() > max_execution_history_) {
            execution_times_.erase(execution_times_.begin());
        }
        
        // Check for timing violations
        if (execution_time > max_execution_time_ * 1000) {
            RCLCPP_WARN(this->get_logger(),
                       "Control loop exceeded timing deadline: %f ms (max: %f ms)",
                       execution_time, max_execution_time_ * 1000);
        }
        
        // Update last execution time for monitoring
        last_execution_time_ = end_time;
    }
    
    double getAverageExecutionTime() const {
        if (execution_times_.empty()) return 0.0;
        
        double sum = 0.0;
        for (double time : execution_times_) {
            sum += time;
        }
        return sum / execution_times_.size();
    }
    
    double getMaxExecutionTime() const {
        if (execution_times_.empty()) return 0.0;
        
        return *std::max_element(execution_times_.begin(), execution_times_.end());
    }

private:
    void executeControlAlgorithm() {
        // Implement the main control algorithm here
        // This could be any of the control techniques discussed
    }
};
```

### Performance Optimization

#### Efficient Matrix Operations
```cpp
class OptimizedControlMath {
public:
    // Optimized matrix-vector multiplication for control applications
    static Eigen::VectorXd fastMultiply(const Eigen::MatrixXd& A, 
                                      const Eigen::VectorXd& x) {
        
        // For small matrices, use unrolled loops for better performance
        if (A.rows() <= 4 && A.cols() <= 4) {
            return unrolledMultiply(A, x);
        }
        
        // For larger matrices, use Eigen's optimized routines
        return A * x;
    }
    
    // Specialized function for 3x3 matrices (common in robotics)
    static Eigen::Vector3d multiply3x3(const Eigen::Matrix3d& A, 
                                     const Eigen::Vector3d& x) {
        
        Eigen::Vector3d result;
        result[0] = A(0,0)*x[0] + A(0,1)*x[1] + A(0,2)*x[2];
        result[1] = A(1,0)*x[0] + A(1,1)*x[1] + A(1,2)*x[2];
        result[2] = A(2,0)*x[0] + A(2,1)*x[1] + A(2,2)*x[2];
        
        return result;
    }
    
    // Inverse of 3x3 matrix (common in robotics)
    static Eigen::Matrix3d inverse3x3(const Eigen::Matrix3d& A) {
        // Calculate determinant
        double det = A(0,0)*(A(1,1)*A(2,2) - A(1,2)*A(2,1)) -
                    A(0,1)*(A(1,0)*A(2,2) - A(1,2)*A(2,0)) +
                    A(0,2)*(A(1,0)*A(2,1) - A(1,1)*A(2,0));
        
        if (std::abs(det) < 1e-9) {
            // Matrix is singular, return identity as fallback
            return Eigen::Matrix3d::Identity();
        }
        
        Eigen::Matrix3d inv;
        inv(0,0) = (A(1,1)*A(2,2) - A(1,2)*A(2,1)) / det;
        inv(0,1) = (A(0,2)*A(2,1) - A(0,1)*A(2,2)) / det;
        inv(0,2) = (A(0,1)*A(1,2) - A(0,2)*A(1,1)) / det;
        inv(1,0) = (A(1,2)*A(2,0) - A(1,0)*A(2,2)) / det;
        inv(1,1) = (A(0,0)*A(2,2) - A(0,2)*A(2,0)) / det;
        inv(1,2) = (A(0,2)*A(1,0) - A(0,0)*A(1,2)) / det;
        inv(2,0) = (A(1,0)*A(2,1) - A(1,1)*A(2,0)) / det;
        inv(2,1) = (A(0,1)*A(2,0) - A(0,0)*A(2,1)) / det;
        inv(2,2) = (A(0,0)*A(1,1) - A(0,1)*A(1,0)) / det;
        
        return inv;
    }

private:
    static Eigen::VectorXd unrolledMultiply(const Eigen::MatrixXd& A, 
                                          const Eigen::VectorXd& x) {
        
        int rows = A.rows();
        int cols = A.cols();
        Eigen::VectorXd result = Eigen::VectorXd::Zero(rows);
        
        // Unroll inner loop for better performance
        for (int i = 0; i < rows; i++) {
            double sum = 0.0;
            for (int j = 0; j < cols; j++) {
                sum += A(i, j) * x(j);
            }
            result(i) = sum;
        }
        
        return result;
    }
};
```

## Safety and Reliability

### Fault-Tolerant Control

```cpp
class FaultTolerantController {
private:
    std::vector<std::unique_ptr<ControllerInterface>> controllers_;
    std::vector<ComponentHealth> component_healths_;
    
    // Fault detection and isolation
    std::unique_ptr<FaultDetector> fault_detector_;
    std::unique_ptr<ControllerSelector> controller_selector_;
    
    // Reconfiguration manager
    std::unique_ptr<ReconfigurationManager> reconfiguration_manager_;

public:
    FaultTolerantController() {
        fault_detector_ = std::make_unique<ModelBasedFaultDetector>();
        controller_selector_ = std::make_unique<PriorityBasedControllerSelector>();
        reconfiguration_manager_ = std::make_unique<ReconfigurationManager>();
    }
    
    ControlOutput compute(const SystemState& state,
                         const ControlReference& reference,
                         double dt) {
        
        // Detect faults
        auto fault_status = fault_detector_->check(state);
        
        // Select appropriate controller based on fault status
        auto active_controller = controller_selector_->select(
            controllers_, fault_status);
        
        // Compute control output
        auto output = active_controller->compute(state, reference, dt);
        
        // If faults detected, trigger reconfiguration
        if (fault_status.has_faults) {
            reconfiguration_manager_->reconfigure(controllers_, fault_status);
        }
        
        return output;
    }
    
    void addController(std::unique_ptr<ControllerInterface> controller,
                      ControllerPriority priority) {
        
        controllers_.push_back(std::move(controller));
        component_healths_.push_back({priority, ControllerHealth::HEALTHY});
    }

private:
    struct ComponentHealth {
        ControllerPriority priority;
        ControllerHealth health;
        rclcpp::Time last_check;
    };
    
    enum class ControllerHealth {
        HEALTHY,
        DEGRADED,
        FAULTY,
        FAILED
    };
    
    enum class ControllerPriority {
        PRIMARY,
        SECONDARY,
        TERTIARY,
        EMERGENCY
    };
    
    class ControllerInterface {
    public:
        virtual ControlOutput compute(const SystemState& state,
                                   const ControlReference& reference,
                                   double dt) = 0;
        virtual bool isHealthy() = 0;
    };
    
    class ModelBasedFaultDetector {
    public:
        FaultStatus check(const SystemState& state) {
            FaultStatus status;
            status.has_faults = false;
            
            // Implement model-based fault detection
            // Compare actual behavior with expected behavior
            
            return status;
        }
    };
    
    struct FaultStatus {
        bool has_faults;
        std::vector<FaultType> detected_faults;
        std::vector<int> affected_components;
    };
    
    enum class FaultType {
        SENSOR_FAULT,
        ACTUATOR_FAULT,
        PARAMETER_DRIFT,
        EXTERNAL_DISTURBANCE
    };
};
```

## Troubleshooting Common Control Issues

### Stability Problems

#### Oscillation and Instability
- **Symptoms**: System oscillates around setpoint, diverges from desired behavior
- **Causes**: High gains, phase lag, resonance, modeling errors
- **Solutions**: Reduce gains, add filtering, redesign controller, improve modeling
- **Tools**: Bode plots, root locus, Nyquist diagrams

#### Tuning Guidelines
```cpp
class ControlTuningAssistant {
public:
    static PIDGains tuneForStability(const SystemCharacteristics& sys) {
        PIDGains gains;
        
        // Conservative tuning for stability
        gains.kp = 0.3 * sys.estimated_gain;
        gains.ki = gains.kp / (2.0 * sys.time_constant);
        gains.kd = gains.kp * 0.1 * sys.time_constant;
        
        return gains;
    }
    
    static PIDGains tuneForPerformance(const SystemCharacteristics& sys) {
        PIDGains gains;
        
        // Aggressive tuning for performance (less stable)
        gains.kp = 0.6 * sys.estimated_gain;
        gains.ki = gains.kp / (0.5 * sys.time_constant);
        gains.kd = gains.kp * 0.125 * sys.time_constant;
        
        return gains;
    }
    
    static bool checkStabilityMargins(const SystemCharacteristics& sys,
                                    const PIDGains& gains) {
        
        // Calculate gain and phase margins
        double gain_margin = calculateGainMargin(sys, gains);
        double phase_margin = calculatePhaseMargin(sys, gains);
        
        return gain_margin > MIN_GAIN_MARGIN && phase_margin > MIN_PHASE_MARGIN;
    }

private:
    struct SystemCharacteristics {
        double estimated_gain;
        double time_constant;
        double delay;
        double damping_ratio;
        double natural_frequency;
    };
    
    struct PIDGains {
        double kp, ki, kd;
    };
    
    static double calculateGainMargin(const SystemCharacteristics& sys, 
                                    const PIDGains& gains) {
        // Calculate gain margin from frequency response
        // Implementation would use control theory
        return 6.0;  // Placeholder
    }
    
    static double calculatePhaseMargin(const SystemCharacteristics& sys,
                                    const PIDGains& gains) {
        // Calculate phase margin from frequency response
        // Implementation would use control theory
        return 45.0;  // Placeholder (degrees)
    }
    
    static constexpr double MIN_GAIN_MARGIN = 3.0;      // Times
    static constexpr double MIN_PHASE_MARGIN = 30.0;    // Degrees
};
```

### Performance Issues

#### Slow Response
- **Symptoms**: System takes too long to reach setpoint, sluggish behavior
- **Causes**: Low gains, large time constants, integrator windup
- **Solutions**: Increase gains, add feedforward, anti-windup, redesign controller

#### Steady-State Error
- **Symptoms**: System doesn't reach exact setpoint, persistent error
- **Causes**: Insufficient integral action, external disturbances, modeling errors
- **Solutions**: Add integral action, increase integral gain, add feedforward

### Integration Issues

#### Sampling Rate Problems
- **Symptoms**: Aliasing, instability, poor performance
- **Causes**: Inadequate sampling rate, mismatched sensor/control rates
- **Solutions**: Follow Nyquist criterion, proper filtering, rate conversion

#### Quantization Effects
- **Symptoms**: Limit cycles, reduced precision, nonlinear behavior
- **Causes**: Limited resolution in sensors/actuators
- **Solutions**: Dithering, higher resolution, sigma-delta modulation

## Best Practices

### Controller Design Principles

#### System Understanding
- **Modeling**: Develop accurate mathematical models of system dynamics
- **Identification**: Use system identification techniques to determine parameters
- **Validation**: Validate models against real system behavior
- **Uncertainty**: Account for model uncertainties and disturbances

#### Robust Design
- **Margins**: Design with adequate stability margins
- **Constraints**: Consider actuator and sensor limitations
- **Disturbances**: Account for external disturbances and noise
- **Variations**: Handle parameter variations and aging effects

### Implementation Guidelines

#### Software Considerations
- **Real-time**: Ensure deterministic execution and timing
- **Precision**: Use appropriate numerical precision
- **Overflow**: Prevent numerical overflow and underflow
- **Filtering**: Apply appropriate signal filtering

#### Safety Considerations
- **Limits**: Implement hard and soft limits
- **Monitoring**: Continuously monitor system health
- **Fallbacks**: Provide safe fallback behaviors
- **Testing**: Thoroughly test with fault injection

### Performance Optimization

#### Computational Efficiency
- **Algorithms**: Choose computationally efficient algorithms
- **Data Structures**: Use appropriate data structures
- **Memory**: Minimize memory allocations in control loops
- **Caching**: Cache expensive computations when possible

#### Tuning Process
- **Methodical**: Use systematic tuning procedures
- **Validation**: Validate performance under various conditions
- **Documentation**: Document tuning process and parameters
- **Adaptation**: Consider adaptive tuning methods

## Future Developments

### Emerging Control Techniques

#### AI-Enhanced Control
- **Neural Network Controllers**: Learning-based control approaches
- **Reinforcement Learning**: Adaptive control through interaction
- **Fuzzy Logic**: Handling uncertainty in control systems
- **Genetic Algorithms**: Optimization-based controller design

#### Distributed Control
- **Multi-Agent Systems**: Coordination of multiple controllers
- **Networked Control**: Control over communication networks
- **Cloud Control**: Offloading computation to cloud resources
- **Edge Computing**: Local processing with global coordination

### Advanced Integration Approaches

#### Hybrid Control Systems
- **Switched Systems**: Controllers that switch between modes
- **Hierarchical Control**: Multi-level control architectures
- **Cooperative Control**: Multiple controllers working together
- **Adaptive Architectures**: Self-reconfiguring control systems

## Conclusion

Advanced control techniques are essential for creating sophisticated Physical AI systems that can operate effectively in complex, dynamic environments. From classical PID controllers to modern model predictive control and learning-based approaches, these techniques provide the mathematical frameworks and practical implementations needed to achieve precise, stable, and adaptive robot behavior.

The selection and implementation of control techniques must consider the specific requirements of the robotic application, including stability, performance, robustness, and computational constraints. Modern control systems often combine multiple techniques in hierarchical or cooperative architectures to achieve the best possible performance.

As robotics systems become more sophisticated and operate in more diverse environments, the importance of advanced control techniques continues to grow. The integration of AI and machine learning with classical control methods opens new possibilities for adaptive, intelligent control systems that can learn and improve their performance over time.

Understanding these advanced control techniques and their proper implementation is crucial for developing Physical AI systems that can operate effectively in the real world, bridging the gap between theoretical control design and practical robotic applications.

## Exercises

1. Implement a PID controller for a robot joint and tune it using Ziegler-Nichols method.
2. Design and implement an MPC controller for robot navigation with obstacle avoidance.
3. Create an adaptive control system that adjusts parameters based on changing robot dynamics.

## Further Reading

- Franklin, G. F., Powell, J. D., & Emami-Naeini, A. (2019). "Feedback Control of Dynamic Systems." Pearson.
- Ogata, K. (2010). "Modern Control Engineering." Prentice Hall.
- Rawlings, J. B., & Mayne, D. Q. (2009). "Model Predictive Control: Theory and Design." Nob Hill Publishing.
- Åström, K. J., & Murray, R. M. (2010). "Feedback Systems: An Introduction for Scientists and Engineers." Princeton University Press.
- Siciliano, B., & Khatib, O. (Eds.). (2016). "Springer Handbook of Robotics." Springer.