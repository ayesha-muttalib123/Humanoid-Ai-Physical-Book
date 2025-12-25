---
sidebar_label: ROS 2 Parameter System
title: ROS 2 Parameter System - Runtime Configuration Management
description: Understanding the ROS 2 parameter system for runtime configuration management and dynamic parameter updates
keywords: [ROS 2, parameters, configuration, runtime, dynamic, robotics, parameter server]
---

# 4.3 ROS 2 Parameter System

## Introduction

The ROS 2 parameter system provides a unified approach to runtime configuration management, allowing nodes to be configured dynamically without requiring recompilation or restart. Unlike ROS 1's centralized parameter server, ROS 2 implements a distributed parameter system where each node maintains its own parameters, accessible through standardized interfaces. This decentralized approach improves system robustness, reduces single points of failure, and enables more flexible configuration strategies.

The parameter system is fundamental to creating adaptable robotic systems that can be tuned for different environments, operational modes, or user requirements. Parameters in ROS 2 are strongly typed, support validation, and can be dynamically updated during runtime, making them suitable for configuring everything from basic operational settings to complex algorithm parameters.

## Parameter System Architecture

### Core Concepts

#### Parameter Types
ROS 2 supports several parameter types:

- **BOOL**: Boolean values (true/false)
- **INTEGER**: Integer values (64-bit signed)
- **DOUBLE**: Floating-point values (64-bit)
- **STRING**: String values
- **BYTE_ARRAY**: Array of bytes
- **BOOL_ARRAY**: Array of boolean values
- **INTEGER_ARRAY**: Array of integer values
- **DOUBLE_ARRAY**: Array of floating-point values
- **STRING_ARRAY**: Array of string values

#### Parameter Interface
Each ROS 2 node implements the parameter interface through:
- **Parameter Declaration**: Registering parameters with types and constraints
- **Parameter Access**: Getting and setting parameter values
- **Parameter Callbacks**: Responding to parameter changes
- **Parameter Descriptors**: Providing metadata about parameters

### Distributed Design

Unlike ROS 1's centralized parameter server, ROS 2 uses a distributed approach:

#### Node-Centric Parameters
- Each node maintains its own parameter set
- Parameters are stored locally within each node
- No central parameter server to become a bottleneck or failure point
- Enables independent parameter management per node

#### Parameter Discovery
- Parameters are discoverable through introspection
- Tools like `ros2 param` can query parameters from running nodes
- Parameter names follow hierarchical naming conventions

#### Parameter Services
- Each node provides parameter-related services:
  - `get_parameters`: Retrieve parameter values
  - `set_parameters`: Set parameter values
  - `list_parameters`: List available parameters
  - `describe_parameters`: Get parameter descriptors
  - `set_parameters_atomically`: Atomically set multiple parameters

## Parameter Declaration and Usage

### Basic Parameter Declaration

#### C++ Implementation
```cpp
#include "rclcpp/rclcpp.hpp"

class ParameterNode : public rclcpp::Node
{
public:
  ParameterNode() : Node("parameter_node")
  {
    // Declare parameters with default values
    this->declare_parameter("param_int", 42);
    this->declare_parameter("param_double", 3.14);
    this->declare_parameter("param_string", "default_value");
    this->declare_parameter("param_bool", true);
    
    // Declare parameters with type enforcement
    this->declare_parameter("strict_param", rclcpp::PARAMETER_INTEGER);
    
    // Declare parameters with default values and get immediately
    auto param_int = this->get_parameter("param_int").as_int();
    RCLCPP_INFO(this->get_logger(), "param_int: %d", param_int);
  }
};
```

#### Python Implementation
```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')
        
        # Declare parameters with default values
        self.declare_parameter('param_int', 42)
        self.declare_parameter('param_double', 3.14)
        self.declare_parameter('param_string', 'default_value')
        self.declare_parameter('param_bool', True)
        
        # Declare parameter with type enforcement
        self.declare_parameter('strict_param', Parameter.Type.INTEGER)
        
        # Get parameter values
        param_int = self.get_parameter('param_int').value
        self.get_logger().info(f'param_int: {param_int}')
```

### Parameter Descriptors

Parameters can be declared with descriptors that provide additional constraints and metadata:

#### C++ Implementation
```cpp
#include "rclcpp/rclcpp.hpp"

class ParameterNode : public rclcpp::Node
{
public:
  ParameterNode() : Node("parameter_node_with_descriptors")
  {
    // Integer parameter with range constraints
    rcl_interfaces::msg::ParameterDescriptor int_descriptor;
    int_descriptor.description = "An integer parameter with range constraints";
    int_descriptor.integer_range.resize(1);
    int_descriptor.integer_range[0].from_value = 0;
    int_descriptor.integer_range[0].to_value = 100;
    int_descriptor.integer_range[0].step = 1;
    
    this->declare_parameter("bounded_int", 50, int_descriptor);
    
    // Double parameter with floating-point constraints
    rcl_interfaces::msg::ParameterDescriptor double_descriptor;
    double_descriptor.description = "A double parameter with bounds";
    double_descriptor.floating_point_range.resize(1);
    double_descriptor.floating_point_range[0].from_value = 0.0;
    double_descriptor.floating_point_range[0].to_value = 10.0;
    double_descriptor.floating_point_range[0].step = 0.1;
    
    this->declare_parameter("bounded_double", 5.0, double_descriptor);
    
    // String parameter with allowed values
    rcl_interfaces::msg::ParameterDescriptor string_descriptor;
    string_descriptor.description = "A string parameter with allowed values";
    string_descriptor.additional_constraints = "Valid values are: 'option1', 'option2', 'option3'";
    
    this->declare_parameter("allowed_string", "option1", string_descriptor);
    
    // Read-only parameter
    rcl_interfaces::msg::ParameterDescriptor readonly_descriptor;
    readonly_descriptor.description = "This parameter cannot be changed at runtime";
    readonly_descriptor.read_only = true;
    
    this->declare_parameter("readonly_param", "fixed_value", readonly_descriptor);
  }
};
```

#### Python Implementation
```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, FloatingPointRange

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node_with_descriptors')
        
        # Integer parameter with range constraints
        int_descriptor = ParameterDescriptor()
        int_descriptor.description = "An integer parameter with range constraints"
        int_descriptor.integer_range = [
            IntegerRange(from_value=0, to_value=100, step=1)
        ]
        
        self.declare_parameter('bounded_int', 50, int_descriptor)
        
        # Double parameter with floating-point constraints
        double_descriptor = ParameterDescriptor()
        double_descriptor.description = "A double parameter with bounds"
        double_descriptor.floating_point_range = [
            FloatingPointRange(from_value=0.0, to_value=10.0, step=0.1)
        ]
        
        self.declare_parameter('bounded_double', 5.0, double_descriptor)
        
        # Read-only parameter
        readonly_descriptor = ParameterDescriptor()
        readonly_descriptor.description = "This parameter cannot be changed at runtime"
        readonly_descriptor.read_only = True
        
        self.declare_parameter('readonly_param', 'fixed_value', readonly_descriptor)
```

## Parameter Access Patterns

### Getting Parameters

#### Single Parameter Access
```cpp
// C++ - Get parameter with default fallback
auto param_value = this->get_parameter("param_name").as_int();

// C++ - Get parameter with fallback to default
auto param_with_default = this->get_parameter_or<int>("param_name", 42);

// C++ - Check if parameter exists before getting
if (this->has_parameter("param_name")) {
  auto value = this->get_parameter("param_name").as_double();
}
```

```python
# Python - Get parameter value
param_value = self.get_parameter('param_name').value

# Python - Get parameter with default fallback
param_with_default = self.get_parameter_or('param_name', 42)

# Python - Check if parameter exists
if self.has_parameter('param_name'):
    value = self.get_parameter('param_name').value
```

#### Multiple Parameter Access
```cpp
// C++ - Get multiple parameters
std::vector<std::string> param_names = {"param1", "param2", "param3"};
auto param_values = this->get_parameters(param_names);
```

```python
# Python - Get multiple parameters
param_names = ['param1', 'param2', 'param3']
param_values = self.get_parameters(param_names)
```

### Setting Parameters

#### Runtime Parameter Setting
```cpp
// C++ - Set parameter at runtime
auto result = this->set_parameter(rclcpp::Parameter("param_name", 123));
if (!result.successful) {
  RCLCPP_WARN(this->get_logger(), "Failed to set parameter: %s", result.reason.c_str());
}

// C++ - Set multiple parameters atomically
std::vector<rclcpp::Parameter> params;
params.emplace_back("param1", 1);
params.emplace_back("param2", 2.0);
params.emplace_back("param3", "three");

auto results = this->set_parameters_atomically(params);
```

```python
# Python - Set parameter at runtime
result = self.set_parameter(Parameter('param_name', Parameter.Type.INTEGER, 123))
if not result.successful:
    self.get_logger().warn(f'Failed to set parameter: {result.reason}')

# Python - Set multiple parameters atomically
params = [
    Parameter('param1', Parameter.Type.INTEGER, 1),
    Parameter('param2', Parameter.Type.DOUBLE, 2.0),
    Parameter('param3', Parameter.Type.STRING, 'three')
]
results = self.set_parameters_atomically(params)
```

## Parameter Callbacks and Validation

### Parameter Change Callbacks

Nodes can register callbacks to be notified when parameters change:

#### C++ Implementation
```cpp
#include "rclcpp/rclcpp.hpp"

class ParameterCallbackNode : public rclcpp::Node
{
public:
  ParameterCallbackNode() : Node("parameter_callback_node")
  {
    this->declare_parameter("threshold", 10.0);
    this->declare_parameter("algorithm", "default");
    
    // Register parameter callback
    this->set_on_parameters_set_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) 
      {
        auto result = rcl_interfaces::msg::SetParametersResult();
        result.successful = true;
        
        for (const auto & parameter : parameters) {
          if (parameter.get_name() == "threshold") {
            if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
              result.successful = false;
              result.reason = "Threshold must be a double value";
              return result;
            }
            
            if (parameter.as_double() < 0.0 || parameter.as_double() > 100.0) {
              result.successful = false;
              result.reason = "Threshold must be between 0.0 and 100.0";
              return result;
            }
            
            RCLCPP_INFO(this->get_logger(), 
                       "Threshold parameter changed to: %f", 
                       parameter.as_double());
          }
          
          if (parameter.get_name() == "algorithm") {
            std::string value = parameter.as_string();
            if (value != "default" && value != "advanced" && value != "legacy") {
              result.successful = false;
              result.reason = "Algorithm must be 'default', 'advanced', or 'legacy'";
              return result;
            }
            
            RCLCPP_INFO(this->get_logger(), 
                       "Algorithm parameter changed to: %s", 
                       value.c_str());
          }
        }
        
        return result;
      });
  }
};
```

#### Python Implementation
```python
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

class ParameterCallbackNode(Node):
    def __init__(self):
        super().__init__('parameter_callback_node')
        
        self.declare_parameter('threshold', 10.0)
        self.declare_parameter('algorithm', 'default')
        
        # Register parameter callback
        self.add_on_set_parameters_callback(self.parameter_callback)
    
    def parameter_callback(self, parameters):
        result = SetParametersResult()
        result.successful = True
        
        for parameter in parameters:
            if parameter.name == 'threshold':
                if parameter.type_ != Parameter.Type.DOUBLE:
                    result.successful = False
                    result.reason = 'Threshold must be a double value'
                    return result
                
                if parameter.value < 0.0 or parameter.value > 100.0:
                    result.successful = False
                    result.reason = 'Threshold must be between 0.0 and 100.0'
                    return result
                
                self.get_logger().info(f'Threshold parameter changed to: {parameter.value}')
            
            elif parameter.name == 'algorithm':
                if parameter.value not in ['default', 'advanced', 'legacy']:
                    result.successful = False
                    result.reason = "Algorithm must be 'default', 'advanced', or 'legacy'"
                    return result
                
                self.get_logger().info(f'Algorithm parameter changed to: {parameter.value}')
        
        return result
```

### Parameter Validation

Advanced validation techniques for ensuring parameter integrity:

```cpp
// C++ - Complex validation example
rcl_interfaces::msg::SetParametersResult validate_and_apply_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;
  
  // Validate parameter relationships
  double min_val = 0.0;
  double max_val = 100.0;
  
  for (const auto & param : parameters) {
    if (param.get_name() == "min_threshold") {
      min_val = param.as_double();
    } else if (param.get_name() == "max_threshold") {
      max_val = param.as_double();
    }
  }
  
  // Validate that min < max
  if (min_val >= max_val) {
    result.successful = false;
    result.reason = "min_threshold must be less than max_threshold";
    return result;
  }
  
  // Apply parameter-specific validations
  for (const auto & param : parameters) {
    if (param.get_name() == "control_gain") {
      if (param.as_double() <= 0.0) {
        result.successful = false;
        result.reason = "Control gain must be positive";
        return result;
      }
    }
    
    if (param.get_name() == "sampling_frequency") {
      if (param.as_integer() <= 0) {
        result.successful = false;
        result.reason = "Sampling frequency must be positive";
        return result;
      }
      
      // Validate against hardware constraints
      if (param.as_integer() > MAX_SAMPLING_FREQUENCY) {
        result.successful = false;
        result.reason = "Sampling frequency exceeds hardware limit";
        return result;
      }
    }
  }
  
  return result;
}
```

## Parameter Loading and Configuration

### YAML Parameter Files

Parameters can be loaded from YAML files, making configuration management easier:

#### Example YAML Configuration
```yaml
# config/robot_params.yaml
parameter_node:
  ros__parameters:
    # Basic parameters
    robot_name: "my_robot"
    use_sim_time: false
    
    # Navigation parameters
    navigation:
      max_linear_velocity: 0.5
      max_angular_velocity: 1.0
      goal_tolerance: 0.1
      rotation_tolerance: 0.1
    
    # Sensor parameters
    sensors:
      lidar:
        range_min: 0.1
        range_max: 30.0
        angle_min: -3.14
        angle_max: 3.14
        update_rate: 10.0
      
      camera:
        width: 640
        height: 480
        fps: 30.0
        compression: "jpeg"
    
    # Control parameters
    control:
      kp: 1.0
      ki: 0.1
      kd: 0.05
      max_effort: 100.0
```

#### Loading Parameters in Launch Files
```python
# launch/robot_with_params.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare configuration file argument
    config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_config'),
            'config',
            'robot_params.yaml'
        ]),
        description='Path to parameter configuration file'
    )
    
    # Node with parameter file
    robot_node = Node(
        package='my_robot_package',
        executable='robot_node',
        name='robot_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {'robot_name': 'configured_robot'},  # Override specific parameter
            {'use_sim_time': True}  # Additional parameter
        ],
        # Additional configuration
        remappings=[
            ('/original_topic', '/remapped_topic')
        ]
    )
    
    return LaunchDescription([
        config_file,
        robot_node
    ])
```

### Parameter Composition

Combining multiple parameter sources with precedence:

```python
# param_composition_demo.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Multiple parameter sources with different precedence
    default_config = PathJoinSubstitution([
        FindPackageShare('my_robot_config'),
        'config',
        'default_params.yaml'
    ])
    
    hardware_config = DeclareLaunchArgument(
        'hardware_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_config'),
            'config',
            'hardware_specific.yaml'
        ]),
        description='Hardware-specific parameter overrides'
    )
    
    runtime_config = DeclareLaunchArgument(
        'runtime_config',
        default_value='',
        description='Runtime parameter overrides (empty if none)'
    )
    
    # Node with multiple parameter sources
    # Later sources override earlier ones
    robot_node = Node(
        package='my_robot_package',
        executable='robot_node',
        name='robot_node',
        parameters=[
            default_config,                    # Base configuration
            LaunchConfiguration('hardware_config'),  # Hardware overrides
            LaunchConfiguration('runtime_config'),   # Runtime overrides
            {'use_sim_time': False},         # Final overrides
        ]
    )
    
    return LaunchDescription([
        hardware_config,
        runtime_config,
        robot_node
    ])
```

## Parameter Tools and Utilities

### Command Line Tools

ROS 2 provides several command-line tools for parameter management:

#### Basic Parameter Operations
```bash
# List parameters for a node
ros2 param list /node_name

# Get a specific parameter
ros2 param get /node_name param_name

# Set a parameter
ros2 param set /node_name param_name param_value

# Get all parameters and save to file
ros2 param dump /node_name > node_params.yaml

# Load parameters from file to node
ros2 param load /node_name node_params.yaml
```

#### Parameter Information
```bash
# Get detailed information about parameters
ros2 param describe /node_name param_name

# Get parameter types
ros2 param types /node_name
```

### Programmatic Parameter Tools

Creating custom parameter management tools:

```python
# parameter_manager.py
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
from rcl_interfaces.msg import Parameter, ParameterValue
import yaml

class ParameterManager(Node):
    def __init__(self):
        super().__init__('parameter_manager')
        
        # Create clients for parameter services
        self.get_params_client = self.create_client(
            GetParameters, 
            'get_parameters'
        )
        self.set_params_client = self.create_client(
            SetParameters, 
            'set_parameters'
        )
        
        # Timer for periodic parameter sync
        self.timer = self.create_timer(10.0, self.sync_parameters)
    
    def sync_parameters_to_file(self, node_name, file_path):
        """Sync node parameters to a YAML file"""
        # This would implement the logic to get parameters from a node
        # and save them to a YAML file
        pass
    
    def load_parameters_from_file(self, node_name, file_path):
        """Load parameters from a YAML file to a node"""
        with open(file_path, 'r') as file:
            config = yaml.safe_load(file)
        
        # Extract parameters for the specific node
        if node_name in config:
            node_params = config[node_name]['ros__parameters']
            for param_name, param_value in node_params.items():
                self.set_parameter(node_name, param_name, param_value)
    
    def set_parameter(self, node_name, param_name, param_value):
        """Set a parameter on a remote node"""
        # Implementation for setting parameter on remote node
        pass
    
    def sync_parameters(self):
        """Periodically synchronize parameters"""
        # Implementation for parameter synchronization
        pass
```

## Advanced Parameter Patterns

### Parameter Namespaces

Organizing parameters hierarchically:

```cpp
// C++ - Namespace-based parameter organization
class HierarchicalParameterNode : public rclcpp::Node
{
public:
  HierarchicalParameterNode() : Node("hierarchical_param_node")
  {
    // Declare parameters with hierarchical names
    this->declare_parameter("sensors.lidar.range_min", 0.1);
    this->declare_parameter("sensors.lidar.range_max", 30.0);
    this->declare_parameter("sensors.camera.width", 640);
    this->declare_parameter("sensors.camera.height", 480);
    
    this->declare_parameter("control.linear.kp", 1.0);
    this->declare_parameter("control.linear.ki", 0.1);
    this->declare_parameter("control.angular.kp", 2.0);
    this->declare_parameter("control.angular.ki", 0.2);
    
    // Access parameters with namespace
    auto lidar_range_min = this->get_parameter("sensors.lidar.range_min").as_double();
    auto linear_kp = this->get_parameter("control.linear.kp").as_double();
  }
};
```

```python
# Python - Hierarchical parameter access
class HierarchicalParameterNode(Node):
    def __init__(self):
        super().__init__('hierarchical_param_node')
        
        # Declare hierarchical parameters
        self.declare_parameter('sensors.lidar.range_min', 0.1)
        self.declare_parameter('sensors.lidar.range_max', 30.0)
        self.declare_parameter('sensors.camera.width', 640)
        self.declare_parameter('sensors.camera.height', 480)
        
        self.declare_parameter('control.linear.kp', 1.0)
        self.declare_parameter('control.linear.ki', 0.1)
        self.declare_parameter('control.angular.kp', 2.0)
        self.declare_parameter('control.angular.ki', 0.2)
        
        # Access parameters
        lidar_range_min = self.get_parameter('sensors.lidar.range_min').value
        linear_kp = self.get_parameter('control.linear.kp').value
```

### Dynamic Parameter Groups

Managing groups of related parameters:

```cpp
// C++ - Parameter group management
class ParameterGroupNode : public rclcpp::Node
{
public:
  ParameterGroupNode() : Node("parameter_group_node")
  {
    // Define parameter groups
    this->declare_parameter("navigation.planner.max_vel_x", 0.5);
    this->declare_parameter("navigation.planner.max_vel_theta", 1.0);
    this->declare_parameter("navigation.planner.min_vel_x", 0.1);
    this->declare_parameter("navigation.planner.min_vel_theta", 0.2);
    
    this->declare_parameter("navigation.controller.kp", 1.0);
    this->declare_parameter("navigation.controller.ki", 0.1);
    this->declare_parameter("navigation.controller.kd", 0.05);
    
    // Register callback for navigation group only
    this->set_on_parameters_set_callback(
      [this](const std::vector<rclcpp::Parameter> & parameters) 
      {
        bool nav_params_changed = false;
        
        for (const auto & param : parameters) {
          if (param.get_name().find("navigation.") == 0) {
            nav_params_changed = true;
            break;
          }
        }
        
        if (nav_params_changed) {
          this->reconfigure_navigation();
        }
        
        return rcl_interfaces::msg::SetParametersResult{true, ""};
      });
  }
  
private:
  void reconfigure_navigation() {
    // Reconfigure navigation system based on new parameters
    auto max_vel_x = this->get_parameter("navigation.planner.max_vel_x").as_double();
    auto kp = this->get_parameter("navigation.controller.kp").as_double();
    
    // Apply new configuration to navigation system
    RCLCPP_INFO(this->get_logger(), 
               "Reconfigured navigation: max_vel_x=%f, kp=%f", 
               max_vel_x, kp);
  }
};
```

### Parameter Persistence

Saving and restoring parameter states:

```python
# parameter_persistence.py
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType
import json
import os
from datetime import datetime

class PersistentParameterNode(Node):
    def __init__(self):
        super().__init__('persistent_parameter_node')
        
        # Declare persistent parameters
        self.declare_parameter('system.mode', 'idle')
        self.declare_parameter('control.gains.kp', 1.0)
        self.declare_parameter('control.gains.ki', 0.05)
        self.declare_parameter('control.gains.kd', 0.01)
        
        # Load saved parameters if they exist
        self.load_persistent_parameters()
        
        # Timer to periodically save parameters
        self.save_timer = self.create_timer(60.0, self.save_persistent_parameters)
    
    def save_persistent_parameters(self):
        """Save current parameters to persistent storage"""
        try:
            # Get all parameters
            all_params = self._parameters
            
            # Filter out non-persistent parameters
            persistent_params = {}
            for name, param in all_params.items():
                # Only save parameters that should persist
                if self.should_persist(name):
                    persistent_params[name] = self.param_to_dict(param)
            
            # Create timestamped backup
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            backup_file = f"params_backup_{timestamp}.json"
            
            # Save to primary file and backup
            with open('persistent_params.json', 'w') as f:
                json.dump(persistent_params, f, indent=2)
            
            with open(backup_file, 'w') as f:
                json.dump(persistent_params, f, indent=2)
                
            self.get_logger().info('Persistent parameters saved')
        except Exception as e:
            self.get_logger().error(f'Failed to save parameters: {e}')
    
    def load_persistent_parameters(self):
        """Load parameters from persistent storage"""
        try:
            if os.path.exists('persistent_params.json'):
                with open('persistent_params.json', 'r') as f:
                    persistent_params = json.load(f)
                
                # Apply loaded parameters
                for name, value_dict in persistent_params.items():
                    if self.has_parameter(name):
                        # Convert dict back to parameter
                        param_value = self.dict_to_param(value_dict)
                        self.set_parameter(Parameter(name, value=value_dict['value']))
                
                self.get_logger().info('Persistent parameters loaded')
        except Exception as e:
            self.get_logger().warn(f'Failed to load parameters: {e}')
    
    def should_persist(self, param_name):
        """Determine if a parameter should be persisted"""
        # Define which parameters to persist
        persistent_patterns = [
            'control.',
            'navigation.',
            'system.',
            'safety.'
        ]
        
        return any(pattern in param_name for pattern in persistent_patterns)
    
    def param_to_dict(self, param):
        """Convert parameter to dictionary representation"""
        return {
            'type': param.type_,
            'value': param.value
        }
    
    def dict_to_param(self, param_dict):
        """Convert dictionary to parameter representation"""
        return Parameter.Type(param_dict['type']), param_dict['value']
```

## Integration with Physical AI Systems

### Robot Configuration Management

Parameters for configuring robot-specific settings:

```python
# robot_config_manager.py
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from std_msgs.msg import String
import json

class RobotConfigManager(Node):
    def __init__(self):
        super().__init__('robot_config_manager')
        
        # Robot-specific parameters
        self.declare_parameter('robot.model', 'unitree_g1')
        self.declare_parameter('robot.serial_number', 'G1-001')
        self.declare_parameter('robot.max_payload_kg', 5.0)
        self.declare_parameter('robot.max_speed_mps', 2.0)
        
        # Hardware-specific parameters with constraints
        max_speed_descriptor = ParameterDescriptor()
        max_speed_descriptor.description = "Maximum robot speed in meters per second"
        max_speed_descriptor.floating_point_range = [
            FloatingPointRange(from_value=0.1, to_value=5.0, step=0.1)
        ]
        
        self.declare_parameter('robot.max_speed_mps', 2.0, max_speed_descriptor)
        
        # Configuration change publisher
        self.config_change_pub = self.create_publisher(String, 'config_changes', 10)
        
        # Register parameter callback for robot configuration
        self.add_on_set_parameters_callback(self.robot_config_callback)
    
    def robot_config_callback(self, parameters):
        """Handle robot configuration changes"""
        from rcl_interfaces.msg import SetParametersResult
        
        result = SetParametersResult()
        result.successful = True
        
        config_changes = []
        
        for param in parameters:
            if param.name.startswith('robot.'):
                config_changes.append({
                    'parameter': param.name,
                    'old_value': str(self.get_parameter(param.name).value),
                    'new_value': str(param.value)
                })
                
                # Validate specific robot parameters
                if param.name == 'robot.max_payload_kg':
                    if param.value <= 0:
                        result.successful = False
                        result.reason = "Max payload must be positive"
                        return result
                
                if param.name == 'robot.max_speed_mps':
                    # Validate against robot physical limits
                    if param.value > 5.0:  # Hard limit for safety
                        result.successful = False
                        result.reason = "Speed exceeds robot physical limits"
                        return result
        
        if config_changes:
            # Publish configuration changes
            msg = String()
            msg.data = json.dumps(config_changes)
            self.config_change_pub.publish(msg)
            
            self.get_logger().info(f'Robot configuration updated: {len(config_changes)} changes')
        
        return result
```

### Adaptive System Configuration

Parameters for adapting to different operational modes:

```python
# adaptive_config_node.py
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String
from enum import Enum

class RobotMode(Enum):
    NORMAL = "normal"
    SAFE = "safe"
    PERFORMANCE = "performance"
    DEBUG = "debug"

class AdaptiveConfigNode(Node):
    def __init__(self):
        super().__init__('adaptive_config_node')
        
        # Mode-specific parameters
        self.declare_parameter('system.mode', 'normal')
        self.declare_parameter('control.frequency.normal', 50.0)
        self.declare_parameter('control.frequency.safe', 10.0)
        self.declare_parameter('control.frequency.performance', 100.0)
        self.declare_parameter('control.frequency.debug', 20.0)
        
        # Safety parameters
        self.declare_parameter('safety.max_force_normal', 100.0)
        self.declare_parameter('safety.max_force_safe', 50.0)
        self.declare_parameter('safety.max_force_performance', 150.0)
        self.declare_parameter('safety.max_force_debug', 75.0)
        
        # Performance parameters
        self.declare_parameter('performance.max_cpu_usage', 80.0)
        self.declare_parameter('performance.max_memory_usage', 70.0)
        
        # Register parameter callback
        self.add_on_set_parameters_callback(self.adaptive_config_callback)
        
        # Apply initial configuration
        self.apply_current_mode()
    
    def adaptive_config_callback(self, parameters):
        """Handle adaptive configuration changes"""
        result = SetParametersResult()
        result.successful = True
        
        for param in parameters:
            if param.name == 'system.mode':
                if param.value not in [mode.value for mode in RobotMode]:
                    result.successful = False
                    result.reason = f"Invalid mode. Valid modes: {[mode.value for mode in RobotMode]}"
                    return result
        
        if result.successful:
            # Apply configuration changes
            for param in parameters:
                if param.name == 'system.mode':
                    self.apply_mode(RobotMode(param.value))
        
        return result
    
    def apply_current_mode(self):
        """Apply configuration for the current mode"""
        current_mode = RobotMode(self.get_parameter('system.mode').value)
        self.apply_mode(current_mode)
    
    def apply_mode(self, mode):
        """Apply configuration for a specific mode"""
        # Get mode-specific parameters
        control_freq = self.get_parameter(f'control.frequency.{mode.value}').value
        max_force = self.get_parameter(f'safety.max_force_{mode.value}').value
        
        # Apply configuration changes
        self.get_logger().info(f'Switched to {mode.value} mode:')
        self.get_logger().info(f'  Control frequency: {control_freq} Hz')
        self.get_logger().info(f'  Max force: {max_force} N')
        
        # Here you would apply the configuration to the actual control system
        # For example, change control loop frequency, adjust safety limits, etc.
        self.adjust_control_system(control_freq, max_force)
    
    def adjust_control_system(self, frequency, max_force):
        """Adjust the control system based on new parameters"""
        # Implementation would adjust control system parameters
        pass
```

## Performance Considerations

### Parameter Update Frequency

Managing parameter updates efficiently:

```cpp
// C++ - Efficient parameter handling
class EfficientParameterNode : public rclcpp::Node
{
public:
  EfficientParameterNode() : Node("efficient_param_node")
  {
    // Declare parameters
    this->declare_parameter("filter.alpha", 0.1);
    this->declare_parameter("filter.beta", 0.05);
    
    // Batch parameter updates to avoid excessive callbacks
    this->param_update_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { this->process_batched_updates(); }
    );
  }
  
  void process_batched_updates()
  {
    // Process any accumulated parameter changes
    if (filter_params_changed_) {
      update_filter_coefficients();
      filter_params_changed_ = false;
    }
  }
  
private:
  void update_filter_coefficients()
  {
    alpha_ = this->get_parameter("filter.alpha").as_double();
    beta_ = this->get_parameter("filter.beta").as_double();
    
    // Apply new coefficients to filter
    RCLCPP_DEBUG(this->get_logger(), 
                "Updated filter coefficients: alpha=%.3f, beta=%.3f", 
                alpha_, beta_);
  }
  
  rclcpp::TimerBase::SharedPtr param_update_timer_;
  bool filter_params_changed_ = false;
  double alpha_ = 0.1;
  double beta_ = 0.05;
  
  // Parameter callback that sets flags instead of immediate updates
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};
```

### Memory Management

Optimizing parameter storage and access:

```python
# memory_efficient_params.py
import rclpy
from rclpy.node import Node
from collections import OrderedDict
import weakref

class MemoryEfficientParameterNode(Node):
    def __init__(self):
        super().__init__('memory_efficient_param_node')
        
        # Use ordered dict to maintain parameter order
        self._param_cache = OrderedDict()
        
        # Declare parameters efficiently
        self.declare_parameter('sensor.range_min', 0.1)
        self.declare_parameter('sensor.range_max', 30.0)
        self.declare_parameter('control.kp', 1.0)
        self.declare_parameter('control.ki', 0.1)
        
        # Cache frequently accessed parameters
        self._cache_frequent_params()
    
    def _cache_frequent_params(self):
        """Cache frequently accessed parameters for faster access"""
        freq_params = ['control.kp', 'control.ki', 'sensor.range_min', 'sensor.range_max']
        
        for param_name in freq_params:
            if self.has_parameter(param_name):
                self._param_cache[param_name] = self.get_parameter(param_name).value
    
    def get_cached_parameter(self, name):
        """Get parameter from cache if available, otherwise from ROS parameter system"""
        if name in self._param_cache:
            return self._param_cache[name]
        else:
            if self.has_parameter(name):
                value = self.get_parameter(name).value
                # Add to cache for future access
                self._param_cache[name] = value
                return value
            else:
                raise KeyError(f"Parameter {name} not found")
    
    def update_cache(self):
        """Update parameter cache with latest values"""
        for param_name in self._param_cache.keys():
            if self.has_parameter(param_name):
                self._param_cache[param_name] = self.get_parameter(param_name).value
```

## Security Considerations

### Parameter Validation and Sanitization

Securing parameter inputs:

```python
# secure_parameter_node.py
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
import re

class SecureParameterNode(Node):
    def __init__(self):
        super().__init__('secure_parameter_node')
        
        # Declare parameters with validation
        self.declare_parameter('network.host', 'localhost')
        self.declare_parameter('network.port', 12345)
        self.declare_parameter('user_input.command', '')
        
        # Register secure parameter callback
        self.add_on_set_parameters_callback(self.secure_parameter_callback)
    
    def secure_parameter_callback(self, parameters):
        """Secure parameter validation callback"""
        result = SetParametersResult()
        result.successful = True
        
        for param in parameters:
            if param.name == 'network.host':
                if not self.is_valid_hostname(param.value):
                    result.successful = False
                    result.reason = f"Invalid hostname: {param.value}"
                    return result
            
            elif param.name == 'network.port':
                if not self.is_valid_port(param.value):
                    result.successful = False
                    result.reason = f"Invalid port: {param.value}"
                    return result
            
            elif param.name == 'user_input.command':
                if not self.is_safe_command(param.value):
                    result.successful = False
                    result.reason = f"Dangerous command detected: {param.value}"
                    return result
        
        return result
    
    def is_valid_hostname(self, hostname):
        """Validate hostname format"""
        if len(hostname) > 255:
            return False
        if hostname[-1] == ".":
            hostname = hostname[:-1]  # Strip the last dot if present
        allowed = re.compile("(?!-)[A-Z\\d-]{1,63}(?<!-)$", re.IGNORECASE)
        return all(allowed.match(x) for x in hostname.split("."))
    
    def is_valid_port(self, port):
        """Validate port number"""
        return isinstance(port, int) and 1 <= port <= 65535
    
    def is_safe_command(self, command):
        """Validate that command doesn't contain dangerous patterns"""
        dangerous_patterns = [
            r'\|\|',      # Command chaining
            r'&&',        # Command chaining
            r';',         # Command separator
            r'\$',        # Variable expansion
            r'\.\.',      # Directory traversal
            r'rm\s',      # File deletion
            r'chmod\s',   # Permission changes
            r'chown\s',   # Ownership changes
        ]
        
        command_lower = command.lower()
        for pattern in dangerous_patterns:
            if re.search(pattern, command_lower):
                return False
        return True
```

## Troubleshooting Common Issues

### Parameter Declaration Issues

```python
# parameter_troubleshooting.py
import rclpy
from rclpy.node import Node

class ParameterTroubleshootingNode(Node):
    def __init__(self):
        super().__init__('parameter_troubleshooting_node')
        
        # Common issues and solutions
        
        # Issue 1: Declaring parameter after trying to access it
        # Solution: Always declare before accessing
        self.declare_parameter('properly_declared_param', 42)
        value = self.get_parameter('properly_declared_param').value
        
        # Issue 2: Type mismatches
        # Solution: Use appropriate parameter types
        try:
            # This will work
            self.declare_parameter('typed_param', 123)  # integer
            int_val = self.get_parameter('typed_param').value
            
            # Verify type before using
            if isinstance(int_val, int):
                self.get_logger().info(f'Integer parameter: {int_val}')
            else:
                self.get_logger().error('Parameter is not of expected type')
        except Exception as e:
            self.get_logger().error(f'Parameter type error: {e}')
        
        # Issue 3: Parameter not found
        # Solution: Always check existence before getting
        param_name = 'maybe_exists'
        if self.has_parameter(param_name):
            value = self.get_parameter(param_name).value
        else:
            self.get_logger().warn(f'Parameter {param_name} does not exist')
            # Use default value instead
            value = self.get_parameter_or(param_name, 0)
```

### Debugging Parameter Flows

```bash
# Debug parameter issues
# 1. Check if node exists
ros2 node list | grep node_name

# 2. List available parameters
ros2 param list /node_name

# 3. Check parameter types
ros2 param types /node_name

# 4. Get specific parameter with details
ros2 param describe /node_name param_name

# 5. Monitor parameter changes
watch -n 1 'ros2 param get /node_name param_name'
```

## Best Practices

### Parameter Design Guidelines

1. **Use Descriptive Names**: Use clear, hierarchical names that indicate purpose
2. **Provide Meaningful Defaults**: Choose sensible default values for all parameters
3. **Validate Inputs**: Implement validation in parameter callbacks
4. **Document Parameters**: Use descriptors to document parameter purpose and constraints
5. **Group Related Parameters**: Use namespaces to organize related parameters
6. **Consider Performance**: Don't update parameters too frequently
7. **Plan for Security**: Validate all user-provided parameter values

### Configuration Management

1. **Version Control**: Keep parameter files in version control
2. **Environment-Specific Files**: Use different parameter files for different environments
3. **Backup and Recovery**: Implement parameter persistence for critical values
4. **Change Auditing**: Log parameter changes for debugging and security
5. **Testing**: Test parameter changes don't break system functionality

## Future Developments

### Emerging Patterns

#### Parameter Schemas
- Formal schema definitions for parameter structures
- Automatic validation based on schemas
- IDE integration for parameter editing

#### Parameter Synchronization
- Automatic parameter synchronization across nodes
- Distributed parameter management
- Parameter state consistency

#### AI-Integrated Parameters
- Machine learning for parameter optimization
- Adaptive parameter tuning
- Predictive parameter adjustment

## Conclusion

The ROS 2 parameter system provides a robust, flexible, and secure approach to runtime configuration management for robotic systems. Its distributed architecture eliminates single points of failure while providing the flexibility needed for complex robotic applications.

The system's strong typing, validation capabilities, and callback mechanisms enable the creation of adaptive, self-configuring robotic systems that can adjust to changing environments and requirements. The integration with launch files and command-line tools makes parameter management accessible to both developers and operators.

Understanding the parameter system is essential for creating maintainable and configurable robotic applications. The system's design encourages good software engineering practices like separation of configuration from code, validation of inputs, and clear documentation of configurable aspects.

As robotic systems become more complex and operate in more diverse environments, the parameter system will continue to be a critical component for enabling flexible, adaptable, and robust robotic applications.

## Exercises

1. Create a parameterized robot controller node that accepts gains, limits, and operational modes through parameters, with appropriate validation and callbacks.
2. Design a parameter management system that synchronizes parameters across multiple nodes and persists critical settings across restarts.
3. Implement an adaptive parameter system that adjusts control parameters based on performance metrics and environmental conditions.

## Further Reading

- ROS 2 Documentation: "Using Parameters in a Class"
- ROS 2 Documentation: "Parameters and Parameter Files"
- ROS 2 Documentation: "Parameters in Launch Files"
- Design article: "ROS 2 Parameter System Design"