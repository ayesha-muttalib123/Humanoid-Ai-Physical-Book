---
sidebar_label: ROS 2 Packages
title: ROS 2 Packages - Package Management and Workspace Organization
description: Understanding ROS 2 packages, package management, and workspace organization for robotics development
keywords: [ROS 2, packages, package management, workspace, build system, colcon, ament]
---

# 3.3 ROS 2 Packages

## Introduction

ROS 2 packages are the fundamental units of code organization in the ROS 2 ecosystem. A package contains nodes, libraries, datasets, configuration files, and other artifacts necessary for a specific functionality. Understanding package structure, management, and organization is essential for developing maintainable and reusable robotics software.

Packages in ROS 2 are more than just code containers; they define dependencies, interfaces, and build configurations that enable the creation of complex, modular robotic systems. The package system provides a standardized way to distribute, share, and reuse code across different projects and teams.

## Package Structure and Components

### Package Definition

A ROS 2 package is a directory that contains:

- **package.xml**: Manifest file describing the package
- **CMakeLists.txt**: Build configuration for C++ packages
- **setup.py**: Build configuration for Python packages
- **src/**: Source code files
- **include/**: Header files (for C++)
- **launch/**: Launch files for starting nodes
- **config/**: Configuration files
- **test/**: Test files
- **msg/**: Message definition files
- **srv/**: Service definition files
- **action/**: Action definition files
- **scripts/**: Standalone scripts
- **models/**: Robot model files (URDF, SDF, etc.)

### Package.xml Manifest

The `package.xml` file is the package manifest that describes the package to the build system and to other packages.

#### Basic Structure
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_package</name>
  <version>0.0.0</version>
  <description>Package description</description>
  <maintainer email="email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

#### Key Elements
- **name**: Unique identifier for the package
- **version**: Semantic versioning (MAJOR.MINOR.PATCH)
- **description**: Brief package description
- **maintainer**: Contact information for package maintainer
- **license**: Software license information
- **dependencies**: Other packages this package depends on
- **build_type**: Build system type (ament_cmake, ament_python, cmake)

### Build Configuration

#### CMakeLists.txt for C++ Packages
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Declare executable
add_executable(my_node src/my_node.cpp)
ament_target_dependencies(my_node rclcpp std_msgs)

# Install targets
install(TARGETS
  my_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

#### setup.py for Python Packages
```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='email@example.com',
    description='Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = my_package.my_node:main',
        ],
    },
)
```

## Package Types

### Executable Packages

Executable packages contain nodes that can be run directly:

#### Characteristics
- Contain executable files (nodes)
- Can be launched with `ros2 run`
- Usually have a main function
- May depend on other packages

#### Example Structure
```
my_robot_controller/
├── package.xml
├── CMakeLists.txt
├── src/
│   ├── main_controller.cpp
│   └── motion_planner.cpp
├── include/
│   └── my_robot_controller/
│       ├── controller.hpp
│       └── planner.hpp
├── launch/
│   └── controller.launch.py
└── config/
    └── controller_params.yaml
```

### Library Packages

Library packages provide reusable functionality without executable nodes:

#### Characteristics
- Provide libraries for other packages to use
- No executable nodes
- Focus on providing APIs
- Reusable across multiple projects

#### Example Structure
```
robot_math_utils/
├── package.xml
├── CMakeLists.txt
├── include/
│   └── robot_math_utils/
│       ├── transform.hpp
│       └── interpolation.hpp
├── src/
│   ├── transform.cpp
│   └── interpolation.cpp
└── test/
    └── test_transforms.cpp
```

### Message/Service/Action Packages

These packages define communication interfaces:

#### Characteristics
- Contain only message/service/action definitions
- No executable code
- Used by other packages for communication
- Language-neutral interface definitions

#### Example Structure
```
my_robot_msgs/
├── package.xml
├── CMakeLists.txt
├── msg/
│   ├── JointState.msg
│   └── RobotMode.msg
├── srv/
│   └── SetMode.srv
└── action/
    └── MoveToPose.action
```

## Build System: Colcon and Ament

### Colcon Build System

Colcon is the ROS 2 build system that replaced catkin_make and catkin_tools from ROS 1.

#### Key Features
- **Parallel Builds**: Builds multiple packages simultaneously
- **Flexible**: Supports different build systems
- **Extensible**: Plugin architecture for different languages
- **Workspace Management**: Manages multiple packages in a workspace

#### Common Commands
```bash
# Build workspace
colcon build

# Build specific package
colcon build --packages-select my_package

# Build with symlinks (for development)
colcon build --symlink-install

# Build with verbose output
colcon build --event-handlers console_direct+

# Run tests
colcon test
colcon test-result --all
```

### Ament Build System

Ament is the ROS 2 package build system framework:

#### Components
- **ament_cmake**: CMake-based build system
- **ament_python**: Python-based build system
- **ament_cargo**: Rust-based build system
- **cmake**: Direct CMake support

#### Ament vs. Catkin
- **Modularity**: More modular design
- **Language Support**: Better support for multiple languages
- **Dependency Management**: Improved dependency resolution
- **Testing**: Integrated testing framework

## Workspace Organization

### Workspace Structure

A typical ROS 2 workspace follows this structure:

```
ros2_workspace/
├── src/
│   ├── my_organization/
│   │   ├── robot_controller/
│   │   ├── sensor_drivers/
│   │   └── navigation_stack/
│   ├── other_org/
│   │   ├── slam_toolbox/
│   │   └── rviz/
│   └── vendor_packages/
│       ├── hardware_interfaces/
│       └── control_toolbox/
├── build/
├── install/
└── log/
```

### Source Directory Organization

#### By Function
```
src/
├── perception/
│   ├── lidar_processing/
│   ├── camera_processing/
│   └── sensor_fusion/
├── control/
│   ├── robot_controllers/
│   ├── trajectory_planners/
│   └── safety_monitors/
├── utils/
│   ├── math_utils/
│   ├── logging_tools/
│   └── diagnostic_tools/
└── applications/
    ├── autonomous_navigation/
    ├── manipulation_tasks/
    └── human_robot_interaction/
```

#### By Team/Department
```
src/
├── team_a/
│   ├── package_a/
│   └── package_b/
├── team_b/
│   ├── package_c/
│   └── package_d/
└── shared/
    ├── common_msgs/
    ├── hardware_interfaces/
    └── utilities/
```

## Package Management Best Practices

### Naming Conventions

#### Package Names
- Use lowercase with underscores
- Be descriptive but concise
- Avoid generic names like "utils" or "common"
- Prefix with organization name if needed

#### Examples
```
# Good
lidar_processing
robot_arm_controller
navigation_2d

# Avoid
utils
common
robot
```

### Versioning

#### Semantic Versioning
- **MAJOR**: Breaking changes
- **MINOR**: Backward-compatible features
- **PATCH**: Backward-compatible fixes

#### Release Process
- Tag releases in version control
- Update package.xml version
- Create release notes
- Publish to package repositories

### Dependency Management

#### Declaring Dependencies
```xml
<!-- Build tool dependencies -->
<buildtool_depend>ament_cmake</buildtool_depend>

<!-- Runtime dependencies -->
<exec_depend>rclcpp</exec_depend>
<exec_depend>std_msgs</exec_depend>

<!-- Build dependencies -->
<build_depend>libboost-dev</build_depend>

<!-- Test dependencies -->
<test_depend>ament_lint_auto</test_depend>
```

#### Dependency Categories
- **buildtool_depend**: Tools needed to build the package
- **build_depend**: Packages needed during compilation
- **exec_depend**: Packages needed to run the package
- **test_depend**: Packages needed for testing

## Package Development Workflow

### Creating New Packages

#### Using ros2 pkg command
```bash
# Create C++ package
ros2 pkg create --build-type ament_cmake my_new_package

# Create Python package
ros2 pkg create --build-type ament_python my_new_python_pkg

# Create with dependencies
ros2 pkg create --build-type ament_cmake --dependencies rclcpp std_msgs my_robot_msgs my_new_pkg
```

### Development Cycle

#### 1. Create Package Structure
```bash
ros2 pkg create --build-type ament_cmake my_robot_controller
cd my_robot_controller
```

#### 2. Modify package.xml
```xml
<description>Robot controller package</description>
<maintainer email="dev@example.com">Developer Name</maintainer>
<license>Apache License 2.0</license>

<depend>rclcpp</depend>
<depend>my_robot_msgs</depend>
```

#### 3. Implement Source Code
- Create source files in `src/`
- Create header files in `include/package_name/`
- Implement nodes and libraries

#### 4. Update CMakeLists.txt
```cmake
find_package(my_robot_msgs REQUIRED)

add_executable(robot_controller src/robot_controller.cpp)
ament_target_dependencies(robot_controller 
  rclcpp 
  my_robot_msgs)

install(TARGETS
  robot_controller
  DESTINATION lib/${PROJECT_NAME})
```

#### 5. Build and Test
```bash
cd ~/ros2_workspace
colcon build --packages-select my_robot_controller
source install/setup.bash
ros2 run my_robot_controller robot_controller
```

## Testing and Quality Assurance

### Unit Testing

#### C++ Testing with Google Test
```cpp
#include <gtest/gtest.h>
#include "my_robot_controller/controller.hpp"

TEST(ControllerTest, Initialization) {
  Controller ctrl;
  EXPECT_TRUE(ctrl.is_initialized());
}

TEST(ControllerTest, Movement) {
  Controller ctrl;
  auto result = ctrl.move_to_position(1.0, 2.0, 3.0);
  EXPECT_EQ(result, Controller::SUCCESS);
}
```

#### Python Testing with Pytest
```python
import unittest
from my_robot_controller.controller import Controller

class TestController(unittest.TestCase):
    def test_initialization(self):
        ctrl = Controller()
        self.assertTrue(ctrl.is_initialized())
    
    def test_movement(self):
        ctrl = Controller()
        result = ctrl.move_to_position(1.0, 2.0, 3.0)
        self.assertEqual(result, 'SUCCESS')
```

### Linting and Code Quality

#### Ament Lint Tools
```xml
<test_depend>ament_lint_auto</test_depend>
<test_depend>ament_lint_cmake</test_depend>
<test_depend>ament_copyright</test_depend>
<test_depend>ament_flake8</test_depend>
```

#### Running Linters
```bash
# Run all linters
colcon test --packages-select my_package --event-handlers console_direct+

# Run specific linter
ament_copyright --verify src/
ament_flake8 src/
```

## Package Distribution

### Creating Debian Packages

#### Binary Release
- Compile packages for target platforms
- Package with dependencies
- Distribute through package managers

#### Source Release
- Tag source code in repository
- Create release tarballs
- Publish to package repositories

### ROS Index Registration

#### rosdistro
- Register packages in rosdistro repository
- Enable building of Debian packages
- Make packages available through apt

#### bloom
- Tool for releasing packages
- Automates release process
- Creates release repositories

## Advanced Package Concepts

### Composition

#### Composable Nodes
```cpp
// In CMakeLists.txt
rclcpp_components_register_nodes(my_component "MyComponent")

// In source code
#include "rclcpp_components/register_node_macro.hpp"

class MyComponent : public rclcpp::Node
{
  // Implementation
};

RCLCPP_COMPONENTS_REGISTER_NODE(MyComponent)
```

#### Benefits
- Reduced resource usage
- Faster startup times
- Better integration
- Single process for multiple nodes

### Plugins

#### Plugin Architecture
- Extend functionality without modifying core code
- Dynamic loading of components
- Standardized interfaces

#### Example Plugin Structure
```
plugin_example/
├── package.xml
├── CMakeLists.txt
├── include/
│   └── plugin_example/
│       └── processor_interface.hpp
├── src/
│   ├── processor_impl1.cpp
│   └── processor_impl2.cpp
└── plugin_description.xml
```

### Cross-compilation

#### Embedded Targets
- Build packages for ARM processors
- Cross-compile for embedded systems
- Optimize for resource-constrained environments

#### Toolchain Configuration
- Set up cross-compilation toolchains
- Configure build flags
- Handle architecture-specific dependencies

## Security Considerations

### Package Verification

#### Code Signing
- Sign packages for authenticity
- Verify package integrity
- Establish trust chains

#### Dependency Checking
- Audit dependencies for vulnerabilities
- Monitor for security issues
- Update dependencies regularly

### Access Control

#### Permissions
- Set appropriate file permissions
- Control who can modify packages
- Secure sensitive configuration files

#### Sandboxing
- Run packages in isolated environments
- Limit system access
- Prevent privilege escalation

## Troubleshooting Common Issues

### Build Issues

#### Missing Dependencies
```bash
# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Check for missing dependencies
rosdep check --from-paths src --ignore-src
```

#### CMake Errors
- Check CMake version requirements
- Verify package.xml dependencies
- Look for missing include directories

### Runtime Issues

#### Library Loading
- Ensure libraries are properly installed
- Check LD_LIBRARY_PATH
- Verify shared library dependencies

#### Node Discovery
- Source the workspace setup file
- Check network configuration
- Verify ROS_DOMAIN_ID settings

## Integration with Physical AI Systems

### Hardware Abstraction

#### Hardware Interface Packages
- Abstract hardware-specific implementations
- Provide standardized interfaces
- Enable hardware-independence

#### Example Structure
```
hardware_interfaces/
├── package.xml
├── CMakeLists.txt
├── include/
│   └── hardware_interfaces/
│       ├── motor_driver.hpp
│       └── sensor_interface.hpp
└── src/
    ├── motor_driver.cpp
    └── sensor_interface.cpp
```

### Sensor Integration

#### Sensor Driver Packages
- Handle specific sensor types
- Convert raw data to ROS 2 messages
- Provide calibration and configuration

#### Standardized Interfaces
- Use common message types
- Follow ROS 2 conventions
- Enable sensor interchangeability

### Control System Packages

#### Controller Architecture
- Implement control algorithms
- Provide standardized interfaces
- Enable controller switching

#### Configuration Management
- Use ROS 2 parameters
- Support runtime reconfiguration
- Enable controller composition

## Performance Considerations

### Build Performance

#### Incremental Builds
- Only rebuild changed packages
- Use build caches
- Optimize build parallelism

#### Memory Usage
- Monitor build memory requirements
- Optimize compilation flags
- Consider resource constraints

### Runtime Performance

#### Package Loading
- Optimize startup times
- Use lazy loading where appropriate
- Minimize memory footprint

#### Resource Management
- Monitor CPU and memory usage
- Optimize for target hardware
- Consider real-time requirements

## Future Developments

### Emerging Trends

#### Micro-ROS Integration
- Packages for microcontroller environments
- Resource-optimized implementations
- Edge computing packages

#### Cloud Robotics
- Packages for cloud integration
- Distributed computing packages
- Remote operation packages

### Standardization Efforts

#### ROS Enhancement Proposals (REPs)
- Standardized package structures
- Consistent interfaces
- Best practice recommendations

## Conclusion

ROS 2 packages provide the fundamental building blocks for creating modular, maintainable, and reusable robotics software. Understanding package structure, dependencies, build systems, and best practices is essential for developing effective robotic systems.

The package system enables the creation of complex, distributed robotic applications while maintaining modularity and reusability. Proper package organization, dependency management, and quality assurance practices are crucial for long-term maintainability and scalability.

As robotics systems become more complex and diverse, the package system continues to evolve to meet new challenges while maintaining backward compatibility and ease of use.

## Exercises

1. Create a new ROS 2 package that implements a simple robot controller, including proper package.xml, CMakeLists.txt, source code, and launch files.
2. Design a package structure for a complete robotic system (navigation, perception, control) following best practices for organization and dependencies.
3. Implement a composable node and demonstrate how multiple nodes can run in a single process.

## Further Reading

- ROS 2 Documentation: "Creating a Package"
- ROS 2 Documentation: "Building a Package"
- ROS 2 Documentation: "Package Manifest Format"
- Colcon Documentation: "Colcon Build System"
- Ament Documentation: "Ament Build System Framework"