---
sidebar_label: Nodes, Topics, Services, Actions
title: Nodes, Topics, Services, Actions - Communication Patterns in ROS 2
description: Understanding the communication patterns in ROS 2 including nodes, topics, services, and actions
keywords: [ROS 2, nodes, topics, services, actions, communication, robotics, messaging]
---

# 3.2 Nodes, Topics, Services, Actions

## Introduction

Communication in ROS 2 is based on a set of well-defined patterns that enable nodes to exchange information and coordinate activities. Understanding these communication patterns is essential for designing effective robotic systems. The four primary communication patterns in ROS 2 are nodes, topics, services, and actions, each serving specific purposes in the robot's architecture.

These patterns provide different trade-offs between simplicity, reliability, and expressiveness. Topics are ideal for continuous data streams, services are perfect for request-response interactions, and actions are designed for long-running, cancellable operations with feedback. Understanding when to use each pattern is crucial for effective system design.

## Nodes

### Definition and Purpose

A node is a fundamental computational unit in ROS 2. It represents a single process that performs computation. Nodes are the basic building blocks of ROS 2 programs and are used to separate different parts of the computation.

### Node Characteristics

#### Lifecycle
- **Initialization**: Node creation and resource allocation
- **Execution**: Running the node's main loop
- **Shutdown**: Cleanup and resource deallocation

#### Node Interfaces
- **Publishers**: Send messages on topics
- **Subscribers**: Receive messages from topics
- **Services**: Provide service requests/responses
- **Clients**: Make service requests
- **Actions**: Provide action goals/results/feedback
- **Action Clients**: Send action goals and receive results/feedback
- **Parameters**: Configurable values accessible at runtime

### Creating Nodes

#### C++ Implementation
```cpp
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node_name")
  {
    // Initialize publishers, subscribers, services, etc.
  }

private:
  // Member variables for interfaces
};
```

#### Python Implementation
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        # Initialize publishers, subscribers, services, etc.
```

### Node Execution

#### Executors
- **SingleThreadedExecutor**: All callbacks run in a single thread
- **MultiThreadedExecutor**: Callbacks run in multiple threads
- **StaticSingleThreadedExecutor**: Pre-allocated memory for efficiency

#### Spinning
- **spin()**: Continuously execute callbacks
- **spin_once()**: Execute callbacks once
- **spin_until_future_complete()**: Execute until a specific future is complete

### Node Best Practices

#### Design Principles
- **Single Responsibility**: Each node should have one clear purpose
- **Minimal Coupling**: Reduce dependencies between nodes
- **Clear Interfaces**: Well-defined inputs and outputs
- **Error Handling**: Robust error handling and recovery

#### Resource Management
- **Cleanup**: Properly clean up resources on shutdown
- **Memory Management**: Efficient memory usage
- **Thread Safety**: Consider thread safety when using multi-threaded executors

## Topics (Publishers and Subscribers)

### Publish/Subscribe Pattern

The publish/subscribe pattern enables one-to-many communication where publishers send messages to a topic and multiple subscribers can receive those messages. This pattern is ideal for continuous data streams like sensor data or robot state information.

### Topic Characteristics

#### Asynchronous Communication
- Publishers do not wait for subscribers to receive messages
- Subscribers receive messages as they arrive
- No direct connection between publisher and subscriber

#### Message Types
- All messages must be strongly typed
- Message definitions stored in `.msg` files
- Messages are serialized for transmission

#### Topic Names
- Hierarchical namespace using `/` separators
- Similar to file system paths
- Example: `/robot/sensors/laser_scan`

### Publishers

#### Publisher Creation
```cpp
publisher_ = this->create_publisher<MessageType>("topic_name", qos_profile);
```

```python
publisher = self.create_publisher(MessageType, 'topic_name', qos_profile)
```

#### Publishing Messages
```cpp
auto msg = MessageType();
// Populate message fields
publisher_->publish(msg);
```

```python
msg = MessageType()
# Populate message fields
publisher.publish(msg)
```

#### QoS Configuration
- **Reliability**: RELIABLE or BEST_EFFORT
- **Durability**: TRANSIENT_LOCAL or VOLATILE
- **History**: KEEP_ALL or KEEP_LAST
- **Depth**: Number of messages to buffer

### Subscribers

#### Subscriber Creation
```cpp
subscription_ = this->create_subscription<MessageType>(
  "topic_name", 
  qos_profile,
  std::bind(&MyNode::callback, this, std::placeholders::_1));
```

```python
subscription = self.create_subscription(
  MessageType,
  'topic_name',
  callback,
  qos_profile)
```

#### Callback Function
```cpp
void callback(const MessageType::SharedPtr msg)
{
  // Process received message
}
```

```python
def callback(self, msg):
    # Process received message
    pass
```

### Topic Use Cases

#### Sensor Data Broadcasting
- Laser scanner readings
- Camera images
- IMU data
- Odometry information

#### State Broadcasting
- Robot joint positions
- Battery levels
- System status
- TF transforms

#### Event Notifications
- Button presses
- Door openings
- System alerts
- Error conditions

### Topic Best Practices

#### Design Guidelines
- **Consistent Naming**: Use consistent topic naming conventions
- **Message Efficiency**: Design efficient message structures
- **Frequency Considerations**: Consider network and processing bandwidth
- **QoS Selection**: Choose appropriate QoS policies for your use case

#### Performance Considerations
- **Message Size**: Keep messages as small as possible
- **Publishing Rate**: Don't publish faster than subscribers can process
- **Buffer Management**: Configure appropriate buffer sizes

## Services

### Request/Response Pattern

Services provide synchronous request/response communication between nodes. A client sends a request to a service server, and the server returns a response. This pattern is ideal for operations that have a clear beginning and end, such as configuration changes or simple computations.

### Service Characteristics

#### Synchronous Communication
- Client waits for server response
- Blocking call until response received
- One-to-one communication pattern

#### Service Types
- Messages must have both request and response parts
- Defined in `.srv` files
- Strongly typed request/response pairs

#### Service Names
- Hierarchical namespace using `/` separators
- Example: `/robot/move_to_position`

### Service Servers

#### Service Server Creation
```cpp
service_ = this->create_service<ServiceType>(
  "service_name",
  std::bind(&MyNode::handle_request, this, 
            std::placeholders::_1, 
            std::placeholders::_2));
```

```python
service = self.create_service(
  ServiceType,
  'service_name',
  handle_request)
```

#### Request Handler
```cpp
void handle_request(
  const std::shared_ptr<Request> request,
  std::shared_ptr<Response> response)
{
  // Process request and populate response
}
```

```python
def handle_request(self, request, response):
    # Process request and populate response
    return response
```

### Service Clients

#### Client Creation
```cpp
client_ = this->create_client<ServiceType>("service_name");
```

```python
client = self.create_client(ServiceType, 'service_name')
```

#### Making Requests
```cpp
auto request = std::make_shared<ServiceType::Request>();
// Populate request fields

while (!client_->wait_for_service(std::chrono::seconds(1))) {
  if (!rclcpp::ok()) {
    RCLCPP_ERROR(rclcpp::get_logger("client"), "Interrupted while waiting for service");
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("client"), "Service not available, waiting again...");
}

auto future = client_->async_send_request(request);
```

```python
request = ServiceType.Request()
# Populate request fields

while not client.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('Service not available, waiting again...')

future = client.call_async(request)
```

### Service Use Cases

#### Configuration Changes
- Setting robot parameters
- Changing operational modes
- Updating calibration values
- Toggling system features

#### Simple Computations
- Coordinate transformations
- Path planning requests
- Object recognition queries
- State queries

#### Synchronous Operations
- Saving current state
- Taking snapshots
- Executing short actions
- Validation checks

### Service Best Practices

#### Design Guidelines
- **Short Duration**: Services should respond quickly
- **Stateless**: Avoid maintaining state between requests
- **Error Handling**: Proper error responses
- **Timeout Handling**: Implement appropriate timeouts

#### Performance Considerations
- **Response Time**: Ensure services respond within reasonable time
- **Concurrency**: Consider handling multiple requests simultaneously
- **Resource Usage**: Don't consume excessive resources

## Actions

### Goal/Result/Feedback Pattern

Actions are designed for long-running operations that may take seconds, minutes, or even longer to complete. They provide a way to send a goal to a server, receive feedback during execution, and get a final result. Actions also support cancellation of ongoing operations.

### Action Characteristics

#### Asynchronous with Feedback
- Client sends goal to action server
- Server provides continuous feedback during execution
- Server returns final result when complete
- Client can cancel the goal during execution

#### Action Types
- Defined in `.action` files
- Three message types: Goal, Result, Feedback
- Strongly typed action interfaces

#### Action Names
- Hierarchical namespace using `/` separators
- Example: `/robot/navigate_to_pose`

### Action Servers

#### Action Server Creation
```cpp
action_server_ = rclcpp_action::create_server<ActionType>(
  this->get_node_base_interface(),
  this->get_node_clock_interface(),
  this->get_node_logging_interface(),
  this->get_node_waitables_interface(),
  "action_name",
  std::bind(&MyNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
  std::bind(&MyNode::handle_cancel, this, std::placeholders::_1),
  std::bind(&MyNode::handle_accepted, this, std::placeholders::_1));
```

#### Action Callbacks
```cpp
rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const ActionType::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle)
{
  // Start executing the action in a separate thread
  using namespace std::placeholders;
  std::thread{std::bind(&MyNode::execute, this, _1), goal_handle}.detach();
}
```

#### Action Execution
```cpp
void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionType>> goal_handle)
{
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<ActionType::Feedback>();
  auto result = std::make_shared<ActionType::Result>();

  for (int i = 0; i < 10; ++i) {
    // Check if there was a cancel request
    if (goal_handle->is_canceling()) {
      result->sequence.clear();
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    // Update feedback
    feedback->sequence.push_back(i);
    goal_handle->publish_feedback(feedback);

    RCLCPP_INFO(this->get_logger(), "Publishing feedback: '%d'", i);

    loop_rate.sleep();
  }

  // Check if goal was cancelled to terminate early
  if (rclcpp::ok()) {
    result->sequence.insert(result->sequence.end(), feedback->sequence.begin(), feedback->sequence.end());
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}
```

### Action Clients

#### Client Creation
```cpp
action_client_ = rclcpp_action::create_client<ActionType>(this, "action_name");
```

```python
action_client = ActionClient(self, ActionType, 'action_name')
```

#### Sending Goals
```cpp
auto goal_msg = ActionType::Goal();
// Populate goal fields

auto send_goal_options = rclcpp_action::Client<ActionType>::SendGoalOptions();
send_goal_options.goal_response_callback = std::bind(&MyNode::goal_response_callback, this, std::placeholders::_1);
send_goal_options.feedback_callback = std::bind(&MyNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
send_goal_options.result_callback = std::bind(&MyNode::result_callback, this, std::placeholders::_1);

auto goal_handle_future = action_client_->async_send_goal(goal_msg, send_goal_options);
```

#### Callbacks
```cpp
void goal_response_callback(std::shared_future<GoalHandleType::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void feedback_callback(
  GoalHandleType::SharedPtr,
  const std::shared_ptr<const ActionType::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Got feedback: %d", feedback->sequence.back());
}

void result_callback(const GoalHandleType::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }

  RCLCPP_INFO(this->get_logger(), "Result received: %d", result.result->sequence.back());
}
```

### Action Use Cases

#### Long-Running Tasks
- Navigation to distant locations
- Complex manipulation sequences
- Image processing pipelines
- Calibration routines

#### Tasks with Feedback
- Progress reporting
- Intermediate results
- Adaptive behavior
- Monitoring execution

#### Cancellable Operations
- Navigation with obstacle avoidance
- Manipulation with force control
- Learning algorithms
- Search operations

### Action Best Practices

#### Design Guidelines
- **Clear Goals**: Define clear, achievable goals
- **Meaningful Feedback**: Provide useful feedback during execution
- **Graceful Cancellation**: Handle cancellation requests properly
- **Error Handling**: Report appropriate error states

#### Performance Considerations
- **Execution Time**: Consider the expected duration of actions
- **Resource Usage**: Monitor resource consumption during execution
- **Concurrent Goals**: Handle multiple simultaneous goals if needed

## Communication Pattern Comparison

### When to Use Each Pattern

#### Topics
- **Continuous Data**: Sensor readings, robot state
- **Broadcasting**: Multiple nodes need the same information
- **Real-time**: Low-latency, high-frequency updates
- **No Acknowledgment**: Don't need to know if messages are received

#### Services
- **Request/Response**: Need immediate response
- **Short Duration**: Operation completes quickly
- **Synchronous**: Calling node waits for result
- **One-off Operations**: Single request, single response

#### Actions
- **Long-Running**: Operation takes significant time
- **With Feedback**: Need to report progress
- **Cancellable**: Operation should be stoppable
- **Complex State**: Multiple possible outcomes

### Performance Characteristics

| Pattern | Latency | Throughput | Reliability | Complexity |
|---------|---------|------------|-------------|------------|
| Topics | Low | High | Configurable | Low |
| Services | Medium | Medium | High | Medium |
| Actions | High | Low | High | High |

### Resource Requirements

#### Topics
- **Memory**: Buffer for message queuing
- **CPU**: Serialization/deserialization
- **Network**: Bandwidth for message transmission

#### Services
- **Memory**: Request/response message storage
- **CPU**: Processing time for service execution
- **Network**: Bidirectional communication

#### Actions
- **Memory**: Goal, feedback, and result message storage
- **CPU**: Long-running execution management
- **Network**: Continuous bidirectional communication

## Advanced Communication Concepts

### Quality of Service (QoS) Policies

#### Reliability Policy
- **RELIABLE**: All messages are guaranteed to be delivered
- **BEST_EFFORT**: Messages may be dropped for performance

#### Durability Policy
- **TRANSIENT_LOCAL**: Late-joining subscribers get historical data
- **VOLATILE**: Only new messages are available

#### History Policy
- **KEEP_ALL**: Store all messages (resource-intensive)
- **KEEP_LAST**: Store only recent messages

### Message Definitions

#### Creating Messages
- **.msg files**: Define message structures
- **Fields**: Specify data types and names
- **Arrays**: Support for variable-length arrays
- **Nested Messages**: Messages can contain other message types

#### Creating Services
- **.srv files**: Define service request/response pairs
- **Request**: Input parameters
- **Response**: Output parameters

#### Creating Actions
- **.action files**: Define goal/result/feedback structures
- **Goal**: Input parameters for the action
- **Result**: Output parameters when action completes
- **Feedback**: Intermediate status updates

### Namespacing and Remapping

#### Node Namespaces
- **Hierarchical**: Organize nodes in logical groups
- **Composability**: Combine nodes into larger systems
- **Configuration**: Apply parameters to groups of nodes

#### Topic Remapping
- **Runtime Flexibility**: Change topic connections at runtime
- **Testing**: Redirect topics for simulation
- **Configuration**: Adapt to different deployment scenarios

## Integration with Physical AI Systems

### Sensor Integration
- **Topics**: Stream sensor data continuously
- **Services**: Configure sensor parameters
- **Actions**: Initiate complex sensor operations

### Actuator Control
- **Topics**: Send continuous control commands
- **Services**: Execute discrete actuator operations
- **Actions**: Perform complex actuator sequences

### Perception Systems
- **Topics**: Broadcast perception results
- **Services**: Request specific perception tasks
- **Actions**: Execute complex perception pipelines

### Planning and Control
- **Topics**: Share planning state and robot state
- **Services**: Request path planning or trajectory generation
- **Actions**: Execute complex navigation or manipulation tasks

## Troubleshooting Communication Issues

### Common Problems
- **Topic Connection Issues**: Publishers/subscribers not connecting
- **Message Type Mismatch**: Incompatible message definitions
- **QoS Incompatibility**: Mismatched QoS policies
- **Node Discovery**: Nodes not finding each other

### Debugging Tools
- **ros2 topic**: Inspect topic information
- **ros2 service**: Inspect service information
- **ros2 action**: Inspect action information
- **rqt_graph**: Visualize node connections
- **ros2 bag**: Record and replay communication

## Security Considerations

### Communication Security
- **Authentication**: Verify node identities
- **Encryption**: Protect sensitive data
- **Authorization**: Control access to topics/services/actions
- **Monitoring**: Track communication patterns

### Best Practices
- **Least Privilege**: Grant minimal required permissions
- **Secure Defaults**: Configure secure communication by default
- **Regular Updates**: Keep ROS 2 and dependencies updated
- **Network Segmentation**: Isolate sensitive communications

## Conclusion

The communication patterns in ROS 2 provide a rich set of tools for designing effective robotic systems. Understanding when and how to use nodes, topics, services, and actions is crucial for creating well-architected systems that are maintainable, efficient, and reliable.

Topics excel at broadcasting continuous data streams, services are ideal for request-response interactions, and actions provide the capability for long-running, cancellable operations with feedback. The flexibility of QoS policies allows tailoring communication behavior to specific application requirements.

As robotics systems become more complex and safety-critical, the proper use of these communication patterns becomes increasingly important for ensuring reliable operation and maintainable code.

## Exercises

1. Design a robotic system using appropriate communication patterns for different components (sensors, actuators, perception, planning, control).
2. Implement a simple robot control system using topics for continuous control, services for configuration, and actions for complex tasks.
3. Analyze the performance characteristics of different communication patterns for a specific robotics application and recommend the most appropriate approaches.

## Further Reading

- ROS 2 Documentation: "Understanding ROS 2 Nodes"
- ROS 2 Documentation: "Topics vs Services vs Actions"
- Collyer, B. (2018). "ROS 2 Design: Communication Architecture."
- DDS Foundation. (2015). "The DDS Specification: A Tutorial."