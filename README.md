# ROS Fundamentals

## Overview
This guide covers the fundamental concepts of ROS (Robot Operating System), including core commands and concepts that form the foundation of any ROS application.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Core Concepts](#core-concepts)
- [Essential Commands](#essential-commands)
- [Practical Examples](#practical-examples)
- [Troubleshooting](#troubleshooting)
- [Next Steps](#next-steps)

## Prerequisites
- Ubuntu 18.04/20.04 LTS
- ROS Melodic/Noetic installed
- Basic Linux command line knowledge

## Core Concepts

### ROS Master (`roscore`)
The ROS Master is the central coordination service that:
- Manages communication between nodes
- Maintains a registry of active nodes, topics, and services
- Enables nodes to find and connect to each other

**Key Features:**
- Must be running before any ROS nodes can communicate
- Acts as a name service and registration service
- Runs on port 11311 by default

### ROS Nodes (`rosnode`)
Nodes are individual processes that perform specific computations:
- Each node is a separate executable
- Nodes communicate via topics, services, and parameters
- Can be written in Python, C++, or other supported languages

### ROS Topics (`rostopic`)
Topics provide a publish-subscribe mechanism for data flow:
- Asynchronous, many-to-many communication
- Nodes publish messages to topics
- Other nodes subscribe to receive those messages
- Message types define the data structure

### ROS Services (`rosservice`)
Services provide synchronous, request-response communication:
- Client-server model
- Blocking calls that wait for responses
- Useful for commands that need immediate feedback

## Essential Commands

### Starting ROS
```bash
# Start the ROS master
roscore

# Alternative: Start with specific settings
roscore -p 11311  # Specify port
```

### Node Management
```bash
# List all running nodes
rosnode list

# Get information about a specific node
rosnode info /node_name

# Test if a node is running
rosnode ping /node_name

# Kill a specific node
rosnode kill /node_name

# Clean up dead node entries
rosnode cleanup
```

### Topic Operations
```bash
# List all active topics
rostopic list

# Show topic information
rostopic info /topic_name

# Display message type for a topic
rostopic type /topic_name

# Monitor messages on a topic
rostopic echo /topic_name

# Publish a message to a topic
rostopic pub /topic_name message_type "data"

# Check publishing frequency
rostopic hz /topic_name

# Get message definition
rostopic type /topic_name | rosmsg show
```

### Service Operations
```bash
# List all available services
rosservice list

# Get service type
rosservice type /service_name

# Call a service
rosservice call /service_name "arguments"

# Get service information
rosservice info /service_name

# Find services by type
rosservice find service_type
```

## Practical Examples

### Example 1: Basic Node Communication
```bash
# Terminal 1: Start ROS master
roscore

# Terminal 2: Start a talker node (publisher)
rosrun rospy_tutorials talker

# Terminal 3: Start a listener node (subscriber)
rosrun rospy_tutorials listener

# Terminal 4: Monitor the communication
rostopic echo /chatter
```

### Example 2: Service Interaction
```bash
# Start a service server
rosrun rospy_tutorials add_two_ints_server

# Call the service
rosservice call /add_two_ints "a: 5, b: 3"

# Check service type
rosservice type /add_two_ints
```

### Example 3: Topic Publishing
```bash
# Publish a string message
rostopic pub /hello std_msgs/String "data: 'Hello ROS!'"

# Publish continuously at 1 Hz
rostopic pub -r 1 /hello std_msgs/String "data: 'Continuous Hello!'"
```

## Understanding ROS Communication

### Publisher-Subscriber Pattern
```
[Publisher Node] ---> [Topic] ---> [Subscriber Node(s)]
```

### Service-Client Pattern
```
[Client Node] <---> [Service Node]
    request  --->
            <--- response
```

## Best Practices

1. **Always start `roscore` first**
   ```bash
   roscore &  # Run in background
   ```

2. **Use meaningful node and topic names**
   ```bash
   # Good naming convention
   /robot/sensors/camera
   /robot/control/velocity
   ```

3. **Monitor system health**
   ```bash
   # Check all nodes
   rosnode list
   
   # Check topic activity
   rostopic list -v
   ```

4. **Clean shutdown**
   ```bash
   # Kill specific nodes gracefully
   rosnode kill /node_name
   
   # Or use Ctrl+C in each terminal
   ```

## Common Message Types

| Message Type | Description | Usage |
|--------------|-------------|-------|
| `std_msgs/String` | Simple text message | Basic communication |
| `std_msgs/Int32` | Integer value | Counters, IDs |
| `std_msgs/Float64` | Floating point | Sensor readings |
| `geometry_msgs/Twist` | Velocity commands | Robot movement |
| `sensor_msgs/Image` | Image data | Camera feeds |

## Troubleshooting

### Common Issues

1. **"Unable to contact ROS master"**
   ```bash
   # Check if roscore is running
   ps aux | grep roscore
   
   # Restart roscore
   killall roscore
   roscore
   ```

2. **Nodes not communicating**
   ```bash
   # Check ROS_MASTER_URI
   echo $ROS_MASTER_URI
   
   # Should be: http://localhost:11311
   export ROS_MASTER_URI=http://localhost:11311
   ```

3. **Topic not found**
   ```bash
   # List all topics
   rostopic list
   
   # Check if publisher/subscriber are running
   rosnode list
   ```

### Debugging Commands
```bash
# View ROS computation graph
rosrun rqt_graph rqt_graph

# Monitor system resources
rosrun rqt_top rqt_top

# Check ROS environment
printenv | grep ROS
```

## Environment Setup

### Essential Environment Variables
```bash
# Add to ~/.bashrc
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

### Verification
```bash
# Check ROS installation
rosversion -d

# Verify environment
echo $ROS_DISTRO
echo $ROS_PACKAGE_PATH
```

## Next Steps
- Learn about [ROS Package Structure](../02-ros-package-structure/README.md)
- Understand workspace creation and management
- Practice with simple publisher/subscriber examples

## Resources
- [ROS Wiki](http://wiki.ros.org/)
- [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [ROS Command Line Tools](http://wiki.ros.org/ROS/CommandLineTools)

---
**Note:** This guide assumes ROS Noetic on Ubuntu 20.04. Commands may vary slightly for other ROS distributions.
