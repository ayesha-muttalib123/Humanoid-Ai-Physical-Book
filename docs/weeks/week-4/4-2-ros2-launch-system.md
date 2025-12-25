---
sidebar_label: ROS 2 Launch System
title: ROS 2 Launch System - Managing Complex Robot Deployments
description: Understanding the ROS 2 launch system for managing complex robot deployments with launch files, arguments, and composable nodes
keywords: [ROS 2, launch, launch files, arguments, composable nodes, robotics, deployment, orchestration]
---

# 4.2 ROS 2 Launch System

## Introduction

The ROS 2 launch system provides a powerful framework for managing the startup, configuration, and coordination of multiple nodes in complex robotic systems. Unlike ROS 1's roslaunch, the ROS 2 launch system offers enhanced flexibility, better integration with modern development practices, and improved capabilities for managing distributed systems. Understanding the launch system is essential for deploying real-world robotic applications that involve multiple nodes, parameters, and coordination mechanisms.

The launch system addresses the complexity of robotic applications by providing a declarative way to specify which nodes to run, how they should be configured, and how they should interact. This is particularly important in real-world deployments where multiple nodes must be orchestrated to achieve complex behaviors.

## Launch System Architecture

### Core Components

#### LaunchDescription
The LaunchDescription is the fundamental unit of a launch file, containing all the entities that will be launched:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch actions go here
    ])
```

#### Launch Actions
Launch actions define operations to perform during the launch process:

- **Node**: Start a ROS 2 node
- **DeclareLaunchArgument**: Define configurable arguments
- **LogInfo**: Output information during launch
- **RegisterEventHandler**: Handle events during execution
- **TimerAction**: Delayed execution of other actions
- **OpaqueFunction**: Execute custom Python functions during launch

### Launch File Structure

#### Python Launch Files (Recommended)
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    # Define nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'robot_description': open('/path/to/robot.urdf').read()}
        ],
        remappings=[
            ('/joint_states', 'filtered_joint_states')
        ]
    )
    
    # Return launch description
    return LaunchDescription([
        use_sim_time,
        robot_state_publisher,
    ])
```

#### YAML Launch Files (Alternative)
```yaml
launch:
  - declare_launch_argument:
      name: "use_sim_time"
      default_value: "false"
      description: "Use simulation clock if true"
  
  - node:
      pkg: "robot_state_publisher"
      exec: "robot_state_publisher"
      name: "robot_state_publisher"
      parameters:
        - {use_sim_time: "$(var use_sim_time)"}
        - {robot_description: "$(file /path/to/robot.urdf)"}
      remappings:
        - {from: "/joint_states", to: "filtered_joint_states"}
```

## Launch Arguments and Configuration

### Declaring Arguments

Launch arguments allow runtime configuration of launch files:

```python
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments with defaults
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='',
        description='Robot namespace for multi-robot systems'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='/path/to/default/config.yaml',
        description='Path to configuration file'
    )
    
    # Use launch configurations in node definitions
    nav2_bringup_node = Node(
        package='nav2_bringup',
        executable='nav2_bringup',
        name='nav2_bringup',
        namespace=LaunchConfiguration('robot_namespace'),
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        robot_namespace_arg,
        config_file_arg,
        nav2_bringup_node,
    ])
```

### Argument Validation and Conditions

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    # Conditional arguments
    debug_mode = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode'
    )
    
    # Conditional node launching
    debug_node = Node(
        package='my_package',
        executable='debug_node',
        name='debug_node',
        condition=IfCondition(LaunchConfiguration('debug'))
    )
    
    # Conditional parameter setting
    conditional_param = DeclareLaunchArgument(
        'use_advanced_features',
        default_value='false',
        description='Enable advanced features'
    )
    
    # Using Python expressions for complex conditions
    advanced_config = LogInfo(
        msg="Using advanced configuration",
        condition=IfCondition(
            PythonExpression([
                '"true" == "', LaunchConfiguration('use_advanced_features'), '"'
            ])
        )
    )
    
    return LaunchDescription([
        debug_mode,
        conditional_param,
        debug_node,
        advanced_config,
    ])
```

## Node Launching and Configuration

### Basic Node Launching

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Basic node with minimal configuration
    minimal_node = Node(
        package='demo_nodes_cpp',
        executable='talker',
        name='minimal_talker',
        namespace='robot1'
    )
    
    # Node with parameters
    parameterized_node = Node(
        package='turtlebot3_node',
        executable='turtlebot3_ros',
        name='turtlebot3_ros',
        parameters=[
            {'use_sim_time': True},
            {'publish_frequency': 50.0},
            {'cmd_vel_topic_name': '/cmd_vel'},
            {'odom_topic_name': '/odom'}
        ]
    )
    
    # Node with remappings
    remapped_node = Node(
        package='image_proc',
        executable='rectify',
        name='rectify_left',
        remappings=[
            ('image', '/camera/left/image_raw'),
            ('camera_info', '/camera/left/camera_info'),
            ('image_rect', '/camera/left/image_rect_color')
        ]
    )
    
    return LaunchDescription([
        minimal_node,
        parameterized_node,
        remapped_node,
    ])
```

### Advanced Node Configuration

```python
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Node with environment variables
    env_node = Node(
        package='my_package',
        executable='my_node',
        name='environment_node',
        # Set environment variables for this node
        additional_env={'CUSTOM_VAR': 'custom_value'},
        # Set working directory
        cwd=os.path.expanduser('~'),
        # Set output format
        output='screen',
        # Set respawn behavior
        respawn=True,
        respawn_delay=2.0,
        # Set launch prefix (for debugging)
        prefix='xterm -e gdb -ex run --args',
        # Set arguments to pass to the node
        arguments=['--log-level', 'DEBUG']
    )
    
    # Node with specific resource limits
    resource_constrained_node = Node(
        package='heavy_computation',
        executable='compute_intensive_node',
        name='heavy_node',
        # Memory and CPU limits would be set via system mechanisms
        # but node can be configured with resource-aware parameters
        parameters=[
            {'max_threads': 2},
            {'memory_limit_mb': 512},
            {'batch_size': 100}
        ]
    )
    
    return LaunchDescription([
        env_node,
        resource_constrained_node,
    ])
```

## Composable Nodes

### Composable Node Containers

Composable nodes allow multiple nodes to run in the same process, reducing communication overhead and improving performance:

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Create a container for composable nodes
    container = ComposableNodeContainer(
        name='image_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # First composable node
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_left',
                namespace='camera',
                parameters=[
                    {'use_sim_time': True}
                ],
                remappings=[
                    ('image', 'left/image_raw'),
                    ('camera_info', 'left/camera_info'),
                    ('image_rect', 'left/image_rect')
                ]
            ),
            # Second composable node
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_right',
                namespace='camera',
                parameters=[
                    {'use_sim_time': True}
                ],
                remappings=[
                    ('image', 'right/image_raw'),
                    ('camera_info', 'right/camera_info'),
                    ('image_rect', 'right/image_rect')
                ]
            ),
            # Third composable node - disparity computation
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::DisparityNode',
                name='disparity',
                namespace='camera',
                parameters=[
                    {'use_sim_time': True},
                    {'stereo_algorithm': 0}  # 0 for block matching
                ],
                remappings=[
                    ('left/image_rect', 'left/image_rect'),
                    ('left/camera_info', 'left/camera_info'),
                    ('right/image_rect', 'right/image_rect'),
                    ('right/camera_info', 'right/camera_info'),
                    ('disparity', 'disparity')
                ]
            )
        ],
        output='screen'
    )
    
    return LaunchDescription([container])
```

### Conditional Composable Nodes

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Declare arguments for conditional loading
    use_stereo = DeclareLaunchArgument(
        'use_stereo',
        default_value='false',
        description='Enable stereo processing'
    )
    
    # Container with conditional nodes
    stereo_container = ComposableNodeContainer(
        name='stereo_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Left rectify node (always loaded)
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_left',
                namespace='camera',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('image', 'left/image_raw'),
                    ('camera_info', 'left/camera_info'),
                    ('image_rect', 'left/image_rect')
                ]
            ),
            # Right rectify node (always loaded)
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify_right',
                namespace='camera',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('image', 'right/image_raw'),
                    ('camera_info', 'right/camera_info'),
                    ('image_rect', 'right/image_rect')
                ]
            ),
            # Disparity node (conditionally loaded)
            ComposableNode(
                package='stereo_image_proc',
                plugin='stereo_image_proc::DisparityNode',
                name='disparity',
                namespace='camera',
                parameters=[{'use_sim_time': True}],
                remappings=[
                    ('left/image_rect', 'left/image_rect'),
                    ('right/image_rect', 'right/image_rect'),
                    ('disparity', 'disparity')
                ],
                # Only load if stereo is enabled
                condition=IfCondition(LaunchConfiguration('use_stereo'))
            )
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_stereo,
        stereo_container
    ])
```

## Launch File Organization

### Modular Launch Files

Breaking complex launch files into modular components:

```python
# robot_description.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    robot_model = DeclareLaunchArgument(
        'robot_model',
        default_value='my_robot',
        description='Robot model to load'
    )
    
    # Include other launch files
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'),
                'launch',
                'robot_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': LaunchConfiguration('robot_model'),
            'use_sim_time': 'false'
        }.items()
    )
    
    # Additional nodes
    diagnostics_node = Node(
        package='diagnostics_aggregator',
        executable='aggregator_node',
        name='diagnostics_aggregator',
        parameters=[
            PathJoinSubstitution([
                FindPackageShare('my_robot_description'),
                'config',
                'diagnostics.yaml'
            ])
        ]
    )
    
    return LaunchDescription([
        robot_model,
        bringup_launch,
        diagnostics_node
    ])
```

### Multi-Robot Launch Files

Managing multiple robots in the same system:

```python
# multi_robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Declare arguments for robot configuration
    num_robots = DeclareLaunchArgument(
        'num_robots',
        default_value='2',
        description='Number of robots to launch'
    )
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    # Create launch description
    ld = LaunchDescription([
        num_robots,
        use_sim_time
    ])
    
    # Add robots based on count
    for i in range(int(LaunchConfiguration('num_robots').perform(None))):
        robot_name = f'robot_{i}'
        
        # Group actions for each robot with namespace
        robot_group = GroupAction(
            actions=[
                # Push namespace for this robot
                PushRosNamespace(robot_name),
                
                # Robot state publisher for this robot
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    parameters=[
                        {'use_sim_time': LaunchConfiguration('use_sim_time')},
                        {'robot_description': 
                            TextSubstitution(text=f'/path/to/{robot_name}.urdf')}
                    ]
                ),
                
                # Navigation stack for this robot
                Node(
                    package='nav2_bringup',
                    executable='nav2_bringup',
                    name='navigator',
                    parameters=[
                        PathJoinSubstitution([
                            FindPackageShare('multi_robot_config'),
                            'params',
                            f'{robot_name}_nav2_params.yaml'
                        ]),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}
                    ]
                )
            ]
        )
        
        ld.add_action(robot_group)
    
    return ld
```

## Advanced Launch Features

### Event Handling

Handling events during launch and execution:

```python
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, LogInfo, EmitEvent
from launch.event_handlers import OnProcessStart, OnProcessExit, OnProcessIO
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    # Create a node that might fail
    critical_node = Node(
        package='critical_package',
        executable='critical_node',
        name='critical_node'
    )
    
    # Event handler for when the node starts
    on_start_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=critical_node,
            on_start=[
                LogInfo(msg="Critical node started successfully"),
                LogInfo(msg=["Started node: ", critical_node.name])
            ]
        )
    )
    
    # Event handler for when the node exits
    on_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=critical_node,
            on_exit=[
                LogInfo(msg="Critical node exited"),
                # Shutdown the entire launch if critical node fails
                EmitEvent(event=Shutdown(reason="Critical node failure"))
            ]
        )
    )
    
    # Event handler for node output
    on_io_handler = RegisterEventHandler(
        OnProcessIO(
            target_action=critical_node,
            on_stdout=lambda event: print(f"Node output: {event.text.decode()}"),
            on_stderr=lambda event: print(f"Node error: {event.text.decode()}")
        )
    )
    
    return LaunchDescription([
        critical_node,
        on_start_handler,
        on_exit_handler,
        on_io_handler
    ])
```

### Timed Launch Actions

Executing actions with delays or periodically:

```python
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo, OpaqueFunction
from launch_ros.actions import Node
import time

def delayed_function(context):
    """Custom function to execute after delay"""
    print("Delayed function executed!")
    return []

def generate_launch_description():
    # Node that should start immediately
    primary_node = Node(
        package='primary_package',
        executable='primary_node',
        name='primary_node'
    )
    
    # Action that executes after 5 seconds
    delayed_action = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="5 seconds have passed"),
            Node(
                package='secondary_package',
                executable='secondary_node',
                name='secondary_node'
            )
        ]
    )
    
    # Another delayed action with custom function
    delayed_function_action = TimerAction(
        period=10.0,
        actions=[
            OpaqueFunction(function=delayed_function)
        ]
    )
    
    # Periodic action (executes repeatedly)
    periodic_action = TimerAction(
        period=2.0,  # Every 2 seconds
        actions=[
            LogInfo(msg=["Periodic check at: ", str(time.time())])
        ]
    )
    
    return LaunchDescription([
        primary_node,
        delayed_action,
        delayed_function_action,
        # Note: Be careful with periodic actions as they run indefinitely
    ])
```

### Conditional Launch with Complex Logic

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IfCondition
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    # Multiple configuration options
    launch_simulation = DeclareLaunchArgument(
        'launch_simulation',
        default_value='false',
        description='Launch simulation if true'
    )
    
    enable_vision = DeclareLaunchArgument(
        'enable_vision',
        default_value='true',
        description='Enable vision processing if true'
    )
    
    robot_count = DeclareLaunchArgument(
        'robot_count',
        default_value='1',
        description='Number of robots to launch'
    )
    
    # Simulation-specific nodes
    gzserver = Node(
        package='gazebo_ros',
        executable='gzserver',
        name='gzserver',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_simulation'))
    )
    
    gzclient = Node(
        package='gazebo_ros',
        executable='gzclient',
        name='gzclient',
        output='screen',
        condition=IfCondition(
            PythonExpression([
                '"true" == "', LaunchConfiguration('launch_simulation'), '"',
                ' and ',
                '"true" == "', LaunchConfiguration('enable_vision'), '"'
            ])
        )
    )
    
    # Vision processing nodes (conditional)
    vision_nodes = []
    if int(LaunchConfiguration('robot_count').perform(None)) > 1:
        # Multi-robot vision processing
        vision_nodes.append(Node(
            package='multi_robot_vision',
            executable='multi_vision_processor',
            name='multi_vision_processor',
            condition=IfCondition(LaunchConfiguration('enable_vision'))
        ))
    else:
        # Single robot vision processing
        vision_nodes.append(Node(
            package='single_robot_vision',
            executable='single_vision_processor',
            name='single_vision_processor',
            condition=IfCondition(LaunchConfiguration('enable_vision'))
        ))
    
    return LaunchDescription([
        launch_simulation,
        enable_vision,
        robot_count,
        gzserver,
        gzclient,
    ] + vision_nodes)
```

## Launch File Best Practices

### Configuration Management

```python
# config_manager.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetParameter
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Centralized parameter declaration
    config_path = DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_config'),
            'config'
        ]),
        description='Path to configuration files'
    )
    
    # Set global parameters
    global_params = [
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        SetParameter(name='robot_namespace', value=LaunchConfiguration('robot_namespace'))
    ]
    
    # Nodes that use the configurations
    sensor_processor = Node(
        package='sensor_processing',
        executable='sensor_processor',
        name='sensor_processor',
        parameters=[
            PathJoinSubstitution([
                LaunchConfiguration('config_path'),
                'sensor_config.yaml'
            ]),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    return LaunchDescription([
        config_path,
    ] + global_params + [
        sensor_processor
    ])
```

### Error Handling and Diagnostics

```python
from launch import LaunchDescription, LaunchContext
from launch.actions import RegisterEventHandler, LogInfo, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def check_launch_success(context: LaunchContext):
    """Check if all critical components launched successfully"""
    # Custom logic to verify launch success
    success = True  # Implement actual check
    if not success:
        context.emit_event(Shutdown(reason="Critical components failed to launch"))
    return []

def generate_launch_description():
    # Critical nodes
    navigation_node = Node(
        package='nav2_bringup',
        executable='nav2_bringup',
        name='navigation_system',
        respawn=True,
        respawn_delay=5.0
    )
    
    perception_node = Node(
        package='perception_pipeline',
        executable='perception_node',
        name='perception_system',
        respawn=True,
        respawn_delay=5.0
    )
    
    # Event handlers for monitoring
    nav_monitor = RegisterEventHandler(
        OnProcessExit(
            target_action=navigation_node,
            on_exit=[
                LogInfo(msg="Navigation system exited unexpectedly"),
                # Don't shutdown for restartable nodes, just log
            ]
        )
    )
    
    # Startup confirmation
    startup_confirmation = RegisterEventHandler(
        OnProcessStart(
            target_action=navigation_node,
            on_start=[
                LogInfo(msg="Navigation system started successfully")
            ]
        )
    )
    
    # Post-launch verification
    verification_step = OpaqueFunction(function=check_launch_success)
    
    return LaunchDescription([
        navigation_node,
        perception_node,
        nav_monitor,
        startup_confirmation,
        verification_step
    ])
```

## Integration with Physical AI Systems

### Robot Bringup Sequences

Launch files for coordinated robot startup:

```python
# robot_bringup.launch.py
from launch import LaunchDescription
from launch.actions import TimerAction, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    # Hardware interface node (starts first)
    hardware_interface = Node(
        package='my_robot_hardware',
        executable='hardware_interface',
        name='hardware_interface',
        parameters=[
            {'use_sim_time': False},
            {'control_frequency': 100.0}
        ]
    )
    
    # Delayed startup of dependent nodes
    controller_manager = TimerAction(
        period=2.0,  # Wait for hardware interface to initialize
        actions=[
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                name='controller_manager',
                parameters=[
                    {'use_sim_time': False}
                ]
            )
        ]
    )
    
    # Robot state publisher (after controllers are loaded)
    robot_state_publisher = TimerAction(
        period=4.0,  # Wait for controller manager
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[
                    {'use_sim_time': False}
                ]
            )
        ]
    )
    
    # Navigation stack (after robot state is published)
    navigation_stack = TimerAction(
        period=6.0,  # Wait for robot state publisher
        actions=[
            Node(
                package='nav2_bringup',
                executable='nav2_bringup',
                name='navigation_system'
            )
        ]
    )
    
    return LaunchDescription([
        hardware_interface,
        controller_manager,
        robot_state_publisher,
        navigation_stack
    ])
```

### Simulation vs. Real Robot Launch

Different launch configurations for simulation and real hardware:

```python
# deployment.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Deployment mode selection
    sim_mode = DeclareLaunchArgument(
        'simulation',
        default_value='false',
        description='Run in simulation mode'
    )
    
    # Hardware-specific launch
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_hardware'),
                'launch',
                'hardware.launch.py'
            ])
        ]),
        condition=UnlessCondition(LaunchConfiguration('simulation'))
    )
    
    # Simulation-specific launch
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_gazebo'),
                'launch',
                'simulation.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('simulation'))
    )
    
    # Common nodes (run in both modes)
    common_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('my_robot_bringup'),
                'launch',
                'common.launch.py'
            ])
        ])
    )
    
    return LaunchDescription([
        sim_mode,
        hardware_launch,
        simulation_launch,
        common_nodes
    ])
```

## Performance Considerations

### Resource Management

Optimizing launch files for resource-constrained environments:

```python
# optimized_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Resource constraints
    low_power_mode = DeclareLaunchArgument(
        'low_power_mode',
        default_value='false',
        description='Optimize for low-power devices'
    )
    
    # Resource-optimized nodes
    if IfCondition(LaunchConfiguration('low_power_mode')).evaluate({}):
        # Low-resource configuration
        perception_node = Node(
            package='lightweight_perception',
            executable='light_perception_node',
            name='perception_node',
            parameters=[
                {'use_sim_time': False},
                {'processing_rate': 5.0},  # Lower rate for low-power
                {'max_objects': 10},       # Fewer objects to track
                {'image_resize_factor': 0.5}  # Smaller images
            ]
        )
        
        navigation_node = Node(
            package='lightweight_navigation',
            executable='light_nav_node',
            name='navigation_node',
            parameters=[
                {'use_sim_time': False},
                {'planning_frequency': 1.0},  # Lower planning frequency
                {'control_frequency': 10.0}    # Lower control frequency
            ]
        )
    else:
        # Full-performance configuration
        perception_node = Node(
            package='full_perception',
            executable='full_perception_node',
            name='perception_node',
            parameters=[
                {'use_sim_time': False},
                {'processing_rate': 30.0},
                {'max_objects': 100},
                {'image_resize_factor': 1.0}
            ]
        )
        
        navigation_node = Node(
            package='full_navigation',
            executable='full_nav_node',
            name='navigation_node',
            parameters=[
                {'use_sim_time': False},
                {'planning_frequency': 5.0},
                {'control_frequency': 50.0}
            ]
        )
    
    return LaunchDescription([
        low_power_mode,
        perception_node,
        navigation_node
    ])
```

### Memory and Process Management

Managing memory and process resources effectively:

```python
# resource_managed_launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Memory constraints
    memory_limit = DeclareLaunchArgument(
        'memory_limit_mb',
        default_value='1024',
        description='Memory limit in MB for the system'
    )
    
    # For memory-constrained systems, use composable nodes
    if int(LaunchConfiguration('memory_limit_mb').perform({})) < 1024:
        # Use composable nodes to reduce process overhead
        processing_container = ComposableNodeContainer(
            name='processing_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            parameters=[{'memory_limit_mb': LaunchConfiguration('memory_limit_mb')}],
            composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_node',
                    parameters=[{'use_sim_time': False}]
                ),
                ComposableNode(
                    package='cv_bridge',
                    plugin='CvBridgeNode',
                    name='cv_bridge_node',
                    parameters=[{'use_sim_time': False}]
                )
            ]
        )
        
        # Minimal nodes only
        essential_nodes = [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'use_sim_time': False}]
            )
        ]
        
        return LaunchDescription([
            memory_limit,
            processing_container
        ] + essential_nodes)
    else:
        # For systems with more memory, use separate processes for better isolation
        return LaunchDescription([
            memory_limit,
            Node(
                package='image_proc',
                executable='rectify',
                name='rectify_node',
                parameters=[{'use_sim_time': False}]
            ),
            Node(
                package='cv_bridge',
                executable='cv_bridge',
                name='cv_bridge_node',
                parameters=[{'use_sim_time': False}]
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'use_sim_time': False}]
            )
        ])
```

## Troubleshooting Common Issues

### Launch File Debugging
```bash
# Enable debug output
ros2 launch my_package my_launch_file.py --log-level debug

# Check launch file syntax
python3 -m py_compile my_launch_file.py

# Verify arguments
ros2 launch my_package my_launch_file.py -s  # Show substitutions
```

### Process Monitoring
- Check if nodes are actually running: `ps aux | grep node_name`
- Verify ROS graph: `ros2 node list`, `ros2 topic list`
- Check parameter values: `ros2 param list`, `ros2 param get`

### Diagnostic Launch Files

```python
# diagnostic_launch.py
from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # Diagnostic nodes
    system_monitor = Node(
        package='system_metrics_collector',
        executable='metrics_collector',
        name='system_monitor',
        parameters=[
            {'collection_frequency': 1.0}
        ]
    )
    
    # Startup sequence verification
    startup_sequence = [
        TimerAction(
            period=1.0,
            actions=[LogInfo(msg="1 second elapsed - checking hardware interface")]
        ),
        TimerAction(
            period=3.0,
            actions=[LogInfo(msg="3 seconds elapsed - checking controllers")]
        ),
        TimerAction(
            period=5.0,
            actions=[LogInfo(msg="5 seconds elapsed - checking navigation")]
        ),
        TimerAction(
            period=10.0,
            actions=[LogInfo(msg="System startup verification complete")]
        )
    ]
    
    return LaunchDescription([
        system_monitor,
    ] + startup_sequence)
```

## Security Considerations

### Secure Launch Practices

```python
# secure_launch.py
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Secure environment variables
    secure_env = SetEnvironmentVariable(
        name='ROS_SECURITY_ENABLE',
        value='true'
    )
    
    # Security keystore path
    security_keystore = DeclareLaunchArgument(
        'security_keystore_path',
        default_value='/etc/ros/security',
        description='Path to security keystore'
    )
    
    # Nodes with security enabled
    secured_node = Node(
        package='secure_node_package',
        executable='secure_node',
        name='secured_node',
        parameters=[
            {'security.keystore': LaunchConfiguration('security_keystore_path')},
            {'security.authentication': True},
            {'security.encryption': True}
        ],
        # Set security environment variables
        additional_env={
            'ROS_SECURITY_KEYSTORE': LaunchConfiguration('security_keystore_path'),
            'ROS_SECURITY_STRATEGY': 'Enforce'
        }
    )
    
    return LaunchDescription([
        secure_env,
        security_keystore,
        secured_node
    ])
```

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

The ROS 2 launch system provides a comprehensive framework for managing complex robotic deployments. Its flexibility, modularity, and rich feature set enable the orchestration of sophisticated robotic systems with proper configuration management, error handling, and resource optimization.

The system's distributed architecture eliminates single points of failure while providing the flexibility needed for complex robotic applications. The integration with launch files and command-line tools makes parameter management accessible to both developers and operators.

Understanding the launch system is essential for creating maintainable and configurable robotic applications. The system's design encourages good software engineering practices like separation of configuration from code, validation of inputs, and clear documentation of configurable aspects.

As robotic systems become more complex and operate in more diverse environments, the launch system will continue to be a critical component for enabling flexible, adaptable, and robust robotic applications.

## Exercises

1. Create a launch file for a mobile robot that conditionally launches different nodes based on whether it's running in simulation or on real hardware.
2. Design a launch file that manages multiple robots in a coordinated system, with proper namespacing and configuration.
3. Implement an adaptive launch system that adjusts node configurations based on performance metrics and environmental conditions.

## Further Reading

- ROS 2 Documentation: "Using Parameters in a Class"
- ROS 2 Documentation: "Parameters and Parameter Files"
- ROS 2 Documentation: "Parameters in Launch Files"
- Design article: "ROS 2 Parameter System Design"