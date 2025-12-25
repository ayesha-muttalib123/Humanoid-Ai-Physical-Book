---
sidebar_label: ROS 2 Architecture
title: ROS 2 Architecture - Core Architecture and Concepts
description: Understanding the architecture and core concepts of ROS 2 for robotics development
keywords: [ROS 2, architecture, DDS, robotics, middleware, nodes, communication]
---

# 3.1 ROS 2 Architecture

## Introduction

ROS 2 (Robot Operating System 2) represents a significant evolution from ROS 1, addressing many of the limitations of the previous system while maintaining the core philosophy of code reuse and rapid development in robotics. The architecture of ROS 2 is built around a distributed system model that enables robust, scalable, and production-ready robotic applications.

Unlike ROS 1, which relied on a single master node for coordination, ROS 2 uses a peer-to-peer discovery model based on the Data Distribution Service (DDS) standard. This architectural change enables ROS 2 to support distributed systems, real-time performance, and improved security, making it suitable for production environments.

## Core Architecture Components

### DDS (Data Distribution Service)

The foundation of ROS 2's architecture is the Data Distribution Service (DDS) standard, which provides:

#### Key Features
- **Decentralized Discovery**: Nodes can discover each other without a central coordinator
- **Quality of Service (QoS)**: Configurable policies for reliability, durability, and performance
- **Real-time Support**: Deterministic behavior for time-critical applications
- **Security**: Built-in security mechanisms for authentication and encryption

#### Implementation Variants
ROS 2 supports multiple DDS implementations:
- **Fast DDS** (formerly Fast RTPS): Default implementation in ROS 2
- **Cyclone DDS**: Lightweight, open-source implementation
- **RTI Connext DDS**: Commercial implementation with enterprise features
- **OpenSplice DDS**: Open-source implementation

### Nodes

Nodes are the fundamental computational units in ROS 2, equivalent to processes in traditional operating systems.

#### Characteristics
- **Process-based**: Each node typically runs in its own process
- **Language Agnostic**: Can be written in any supported language (C++, Python, etc.)
- **Self-Describing**: Nodes advertise their interfaces through introspection
- **Distributed**: Can run on different machines in a network

#### Node Structure
- **Executor**: Manages callbacks and spins the node
- **Parameters**: Configurable values that can be set at runtime
- **Interfaces**: Publishers, subscribers, services, and actions
- **Timers**: Periodic callback scheduling

### Communication Primitives

ROS 2 provides several communication mechanisms:

#### Publishers and Subscribers (Topics)
- **Publish/Subscribe Model**: One-to-many communication
- **Asynchronous**: Publishers don't wait for subscribers
- **Typed Messages**: Strongly typed message definitions
- **Topic Names**: String identifiers for message channels

#### Services
- **Request/Response Model**: Synchronous, one-to-one communication
- **Blocking Calls**: Client waits for server response
- **Typed Requests/Responses**: Strongly typed message pairs
- **Service Names**: String identifiers for services

#### Actions
- **Goal/Result/Feedback**: Asynchronous, long-running operations
- **Cancel Capability**: Ability to interrupt ongoing operations
- **State Machine**: Goal, executing, succeeded, aborted, canceled
- **Action Names**: String identifiers for actions

## Quality of Service (QoS) Policies

QoS policies allow fine-tuning of communication behavior:

### Reliability Policy
- **RELIABLE**: All messages guaranteed to be delivered
- **BEST_EFFORT**: Messages may be lost but faster delivery

### Durability Policy
- **TRANSIENT_LOCAL**: Late-joining subscribers receive historical data
- **VOLATILE**: Only receive new messages after joining

### History Policy
- **KEEP_ALL**: Store all messages (limited by resource limits)
- **KEEP_LAST**: Store only the most recent messages

### Lifespan Policy
- Controls how long messages are kept in the system

### Deadline Policy
- Specifies the maximum interval between consecutive messages

## Execution Model

### Executors
Executors manage the execution of callbacks in ROS 2:

#### Single-threaded Executor
- Executes all callbacks in a single thread
- Simple to reason about but may block
- Default executor in ROS 2

#### Multi-threaded Executor
- Distributes callbacks across multiple threads
- Better performance but requires thread safety
- Suitable for CPU-intensive operations

#### Static Single-threaded Executor
- Pre-allocated memory for efficient operation
- Suitable for embedded systems with memory constraints

### Callback Groups
- Organize callbacks for execution control
- **Mutually Exclusive**: Only one callback executes at a time
- **Reentrant**: Multiple callbacks can execute simultaneously

## Parameter System

ROS 2 provides a unified parameter system:

### Parameter Types
- **Integer, Double, String, Bool, Lists**: Basic types
- **Parameter Descriptors**: Metadata for parameters
- **Dynamic Reconfiguration**: Change parameters at runtime

### Parameter Namespaces
- Hierarchical organization of parameters
- Allows for component-based parameter management
- Supports inheritance and composition

## Launch System

The launch system manages the startup and coordination of multiple nodes:

### Launch Files
- **XML/YAML/Python**: Multiple syntax options
- **Composable Nodes**: Run multiple nodes in the same process
- **Conditional Launch**: Launch nodes based on conditions
- **Remapping**: Change node names, topics, and parameters

### Process Management
- **Lifecycle Management**: Start, stop, and monitor processes
- **Automatic Restart**: Restart failed nodes
- **Resource Allocation**: Manage system resources

## Client Library Architecture

ROS 2 supports multiple programming languages:

### rcl (ROS Client Library)
- **Common Interface**: Shared C library for all clients
- **Language Bindings**: Wrappers for different languages
- **Memory Management**: Safe memory allocation and deallocation

### Supported Languages
- **rclcpp**: C++ client library
- **rclpy**: Python client library
- **rclrs**: Rust client library (experimental)
- **rclc**: C client library for embedded systems

## Security Architecture

ROS 2 includes security features:

### Authentication
- **Identity Verification**: Verify node identity
- **Certificate-based**: PKI infrastructure for node certificates
- **Access Control**: Limit which nodes can participate

### Encryption
- **Transport Encryption**: Encrypt data in transit
- **Message Protection**: Protect message integrity
- **Secure Discovery**: Secure node discovery process

### Authorization
- **Permissions**: Define what nodes can do
- **Access Control Lists**: Fine-grained access control
- **Policy Management**: Centralized security policy management

## Comparison with ROS 1

### Key Differences
- **Architecture**: Master-slave vs. peer-to-peer
- **Middleware**: Custom transport vs. DDS-based
- **Real-time Support**: Limited vs. built-in support
- **Security**: No security vs. built-in security
- **Cross-platform**: Linux-focused vs. truly cross-platform

### Migration Considerations
- **API Changes**: Significant changes in client libraries
- **Communication Patterns**: New QoS concepts
- **Build System**: catkin vs. colcon
- **Launch System**: roslaunch vs. launch

## Deployment Architectures

### Single Machine
- All nodes run on one computer
- Ideal for development and testing
- Simplified networking requirements

### Distributed Systems
- Nodes spread across multiple computers
- Required for large, complex robots
- Network topology considerations

### Edge-Cloud Integration
- Edge devices for real-time processing
- Cloud for heavy computation and storage
- Communication between edge and cloud

## Tools and Utilities

### Core Tools
- **ros2 run**: Execute nodes
- **ros2 topic**: Inspect topics
- **ros2 service**: Inspect services
- **ros2 param**: Manage parameters
- **ros2 action**: Inspect actions

### Visualization Tools
- **rviz2**: 3D visualization environment
- **rqt**: GUI plugin framework
- **rosbag2**: Data recording and playback

### Development Tools
- **colcon**: Build system
- **ament**: Package management
- **rosdep**: Dependency management

## Best Practices

### Node Design
- **Single Responsibility**: Each node should have one clear purpose
- **Minimal Coupling**: Reduce dependencies between nodes
- **Robust Error Handling**: Handle errors gracefully
- **Resource Management**: Properly clean up resources

### Communication Design
- **Appropriate QoS**: Choose QoS policies for your use case
- **Message Design**: Design efficient, well-structured messages
- **Topic Naming**: Use consistent, descriptive names
- **Rate Limiting**: Avoid overwhelming the system

### Security Considerations
- **Minimal Permissions**: Grant only necessary permissions
- **Regular Updates**: Keep ROS 2 and dependencies updated
- **Network Segmentation**: Isolate sensitive systems
- **Monitoring**: Monitor for security events

## Performance Considerations

### Network Performance
- **Bandwidth**: Consider network capacity for message traffic
- **Latency**: Understand timing requirements
- **Reliability**: Choose appropriate QoS policies

### Resource Management
- **CPU Usage**: Monitor and optimize CPU consumption
- **Memory Usage**: Efficient memory management
- **Disk Space**: Log file and bag file management

### Real-time Requirements
- **Timing Constraints**: Understand real-time performance needs
- **Thread Priority**: Configure appropriate thread priorities
- **Interrupt Handling**: Consider interrupt latency requirements

## Future Developments

### Emerging Trends
- **Micro-ROS**: ROS 2 for microcontrollers
- **ROS 2 for Embedded Systems**: Resource-constrained environments
- **Integration with Cloud Platforms**: Cloud robotics integration
- **AI/ML Integration**: Better integration with AI frameworks

### Roadmap Items
- **Improved Real-time Support**: Enhanced deterministic behavior
- **Enhanced Security**: Additional security features
- **Better Tooling**: Improved development tools
- **Expanded Language Support**: Additional programming languages

## Conclusion

The architecture of ROS 2 represents a significant advancement over ROS 1, providing the foundation for robust, scalable, and production-ready robotic applications. The use of DDS as the underlying communication middleware enables distributed systems, real-time performance, and built-in security features.

Understanding the core architecture components, including nodes, communication primitives, QoS policies, and the execution model, is essential for developing effective robotic systems with ROS 2. The system's flexibility allows for deployment in various configurations, from single-machine development environments to distributed production systems.

As robotics continues to evolve toward more complex and safety-critical applications, ROS 2's architecture provides the necessary foundation for building reliable and maintainable robotic systems.

## Exercises

1. Compare the architecture of ROS 1 and ROS 2, analyzing the advantages and disadvantages of each approach for different robotics applications.
2. Design a distributed robotics system architecture using ROS 2, considering network topology, QoS policies, and security requirements.
3. Research and analyze the performance characteristics of different DDS implementations in ROS 2 for a specific robotics application.

## Further Reading

- Faconti, P., et al. (2018). "ROS 2 Design Overview."
- DDS Foundation. (2015). "DDS (Data Distribution Service) for Real-Time Systems."
- MXR Development Team. (2020). "ROS 2 Migration Guide."