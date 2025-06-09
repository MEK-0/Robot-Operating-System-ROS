# ROS Parameters

## Overview
ROS parameter server provides runtime configuration management for nodes

## Basic Parameter Commands
```bash
# Set parameter
rosparam set /robot/max_speed 2.0

# Get parameter
rosparam get /robot/max_speed

# List all parameters
rosparam list

# Delete parameter
rosparam delete /robot/max_speed

# Load from YAML file
rosparam load config.yaml

# Save to YAML file
rosparam dump output.yaml
```

## YAML Configuration File
Create `config/robot_config.yaml`:
```yaml
robot:
  max_speed: 2.0
  min_speed: 0.1
  wheel_radius: 0.05
  base_width: 0.3
  
sensors:
  camera:
    width: 640
    height: 480
    fps: 30
  lidar:
    range_max: 10.0
    angle_min: -3.14159
    angle_max: 3.14159
```

## Python Parameter Usage
```python
#!/usr/bin/env python3
import rospy

class RobotNode:
    def __init__(self):
        rospy.init_node('robot_node')
        
        # Get parameters with defaults
        self.max_speed = rospy.get_param('~max_speed', 1.0)
        self.robot_name = rospy.get_param('~robot_name', 'robot1')
        
        # Get nested parameters
        self.camera_width = rospy.get_param('/sensors/camera/width', 640)
        
        # Check if parameter exists
        if rospy.has_param('~debug'):
            self.debug = rospy.get_param('~debug')
        else:
            self.debug = False
            rospy.set_param('~debug', False)
        
        rospy.loginfo(f"Robot {self.robot_name} initialized")
        rospy.loginfo(f"Max speed: {self.max_speed}")
        
    def update_parameters(self):
        """Update parameters during runtime"""
        new_speed = rospy.get_param('~max_speed', self.max_speed)
        if new_speed != self.max_speed:
            self.max_speed = new_speed
            rospy.loginfo(f"Speed updated to: {self.max_speed}")

if __name__ == '__main__':
    robot = RobotNode()
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        robot.update_parameters()
        rate.sleep()
```

## C++ Parameter Usage
```cpp
#include <ros/ros.h>

class RobotNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    double max_speed_;
    std::string robot_name_;
    
public:
    RobotNode() : private_nh_("~") {
        // Get parameters with defaults
        private_nh_.param("max_speed", max_speed_, 1.0);
        private_nh_.param<std::string>("robot_name", robot_name_, "robot1");
        
        // Get global parameter
        double camera_width;
        nh_.param("/sensors/camera/width", camera_width, 640.0);
        
        ROS_INFO("Robot %s initialized", robot_name_.c_str());
        ROS_INFO("Max speed: %.2f", max_speed_);
    }
    
    void updateParameters() {
        double new_speed;
        if (private_nh_.getParam("max_speed", new_speed)) {
            if (new_speed != max_speed_) {
                max_speed_ = new_speed;
                ROS_INFO("Speed updated to: %.2f", max_speed_);
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_node");
    RobotNode robot;
    
    ros::Rate rate(1);
    while (ros::ok()) {
        robot.updateParameters();
        rate.sleep();
    }
    return 0;
}
```

## Launch File with Parameters
```xml
<launch>
    <!-- Load parameter file -->
    <rosparam file="$(find my_package)/config/robot_config.yaml" command="load"/>
    
    <!-- Set individual parameters -->
    <param name="robot_name" value="robot1"/>
    <param name="debug" value="true"/>
    
    <!-- Node with private parameters -->
    <node name="robot_node" pkg="my_package" type="robot_node.py">
        <param name="max_speed" value="2.5"/>
        <param name="robot_name" value="speedy"/>
        <rosparam file="$(find my_package)/config/private_config.yaml" command="load"/>
    </node>
    
    <!-- Group parameters -->
    <group ns="sensors">
        <param name="camera_enabled" value="true"/>
        <param name="lidar_enabled" value="false"/>
    </group>
</launch>
```

## Parameter Namespaces
```python
# Global parameters
rospy.get_param('/global_param')

# Node private parameters
rospy.get_param('~private_param')

# Relative parameters
rospy.get_param('relative_param')

# Namespace parameters
rospy.get_param('/namespace/param')
```

## Dynamic Parameter Updates
```python
#!/usr/bin/env python3
import rospy

class DynamicConfigNode:
    def __init__(self):
        rospy.init_node('dynamic_config')
        
        # Set initial parameters
        rospy.set_param('~gains/kp', 1.0)
        rospy.set_param('~gains/ki', 0.1)
        rospy.set_param('~gains/kd', 0.01)
        
        # Store current values
        self.kp = rospy.get_param('~gains/kp')
        self.ki = rospy.get_param('~gains/ki')
        self.kd = rospy.get_param('~gains/kd')
        
    def check_parameter_updates(self):
        """Check for parameter changes"""
        new_kp = rospy.get_param('~gains/kp', self.kp)
        new_ki = rospy.get_param('~gains/ki', self.ki)
        new_kd = rospy.get_param('~gains/kd', self.kd)
        
        if new_kp != self.kp or new_ki != self.ki or new_kd != self.kd:
            self.kp, self.ki, self.kd = new_kp, new_ki, new_kd
            rospy.loginfo(f"PID gains updated: kp={self.kp}, ki={self.ki}, kd={self.kd}")

if __name__ == '__main__':
    node = DynamicConfigNode()
    
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        node.check_parameter_updates()
        rate.sleep()
```

## Parameter Server GUI Tools
```bash
# Launch parameter reconfigure GUI
rosrun rqt_reconfigure rqt_reconfigure

# Parameter visualization
rosrun rqt_param rqt_param
```

## Common Parameter Patterns
```yaml
# Robot configuration
robot:
  name: "mobile_robot"
  type: "differential_drive"
  max_linear_velocity: 1.0
  max_angular_velocity: 2.0

# Sensor configuration  
sensors:
  camera:
    device_id: 0
    resolution: [640, 480]
    fps: 30
  laser:
    port: "/dev/ttyUSB0"
    baud_rate: 115200

# Algorithm parameters
navigation:
  goal_tolerance: 0.1
  path_resolution: 0.05
  max_planning_time: 5.0
```

## Best Practices
- Use meaningful parameter names with namespaces
- Provide default values for all parameters
- Group related parameters in namespaces
- Use YAML files for complex configurations
- Update parameters during runtime when needed
- Document parameter purposes and ranges

## Testing Parameters
```bash
# Set test parameters
rosparam set /test/param 42

# Check parameter
rosparam get /test/param

# Load configuration
rosparam load test_config.yaml /test

# Monitor parameter changes
rostopic echo /rosout | grep -i param
```

## Next Steps
Learn [Sensor Data Reading](../06-sensor-data/README.md)
