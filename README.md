# Service & Service Client

## Overview
ROS service communication provides synchronous request-response interaction between nodes

## Concept
```
[Client] <---> [Service Server]
  request --->
         <--- response
```

## Service Definition
Create `srv/AddTwoInts.srv`:
```
int32 a
int32 b
---
int32 sum
```

## Python Service Server
```python
#!/usr/bin/env python3
import rospy
from my_package.srv import AddTwoInts, AddTwoIntsResponse

def handle_add_two_ints(req):
    result = req.a + req.b
    rospy.loginfo(f"Adding {req.a} + {req.b} = {result}")
    return AddTwoIntsResponse(result)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    service = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    rospy.loginfo("Ready to add two ints")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
```

## Python Service Client
```python
#!/usr/bin/env python3
import rospy
from my_package.srv import AddTwoInts
import sys

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        response = add_two_ints(x, y)
        return response.sum
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        x, y = 1, 2
    
    rospy.init_node('add_two_ints_client')
    result = add_two_ints_client(x, y)
    rospy.loginfo(f"Result: {x} + {y} = {result}")
```

## C++ Service Server
```cpp
#include <ros/ros.h>
#include "my_package/AddTwoInts.h"

bool add(my_package::AddTwoInts::Request &req,
         my_package::AddTwoInts::Response &res) {
    res.sum = req.a + req.b;
    ROS_INFO("Request: x=%d, y=%d", req.a, req.b);
    ROS_INFO("Response: sum=%d", res.sum);
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle nh;
    
    ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
    ROS_INFO("Ready to add two ints");
    ros::spin();
    
    return 0;
}
```

## C++ Service Client
```cpp
#include <ros/ros.h>
#include "my_package/AddTwoInts.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "add_two_ints_client");
    
    if (argc != 3) {
        ROS_INFO("Usage: client X Y");
        return 1;
    }
    
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<my_package::AddTwoInts>("add_two_ints");
    
    my_package::AddTwoInts srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    
    if (client.call(srv)) {
        ROS_INFO("Sum: %d", srv.response.sum);
    } else {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    
    return 0;
}
```

## CMakeLists.txt Updates
```cmake
# Add service files
add_service_files(
  FILES
  AddTwoInts.srv
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Add executable dependencies
add_dependencies(server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
```

## package.xml Updates
```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

## Launch File
```xml
<launch>
    <node name="server" pkg="my_package" type="server.py" output="screen"/>
    <node name="client" pkg="my_package" type="client.py" args="5 3" output="screen"/>
</launch>
```

## Testing Commands
```bash
# List services
rosservice list

# Get service type
rosservice type /add_two_ints

# Call service manually
rosservice call /add_two_ints "a: 5, b: 3"

# Get service info
rosservice info /add_two_ints
```

## Running Example
```bash
# Terminal 1
roscore

# Terminal 2
rosrun my_package server.py

# Terminal 3
rosrun my_package client.py 5 3

# Or call directly
rosservice call /add_two_ints "a: 5, b: 3"
```

## Advanced Service Example
```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.move_service = rospy.Service('/start_moving', SetBool, self.handle_movement)
        self.is_moving = False
        
    def handle_movement(self, req):
        if req.data:
            self.start_moving()
            return SetBoolResponse(True, "Started moving")
        else:
            self.stop_moving()
            return SetBoolResponse(True, "Stopped moving")
    
    def start_moving(self):
        self.is_moving = True
        rate = rospy.Rate(10)
        
        while self.is_moving and not rospy.is_shutdown():
            twist = Twist()
            twist.linear.x = 0.5
            self.cmd_pub.publish(twist)
            rate.sleep()
    
    def stop_moving(self):
        self.is_moving = False
        twist = Twist()  # Zero velocity
        self.cmd_pub.publish(twist)

if __name__ == '__main__':
    controller = RobotController()
    rospy.spin()
```

## Best Practices
- Use descriptive service names
- Handle service exceptions
- Wait for service availability with `rospy.wait_for_service()`
- Keep service calls short and simple
- Use standard service types when possible (`std_srvs`)

## Common Service Types
```bash
std_srvs/SetBool      # Enable/disable functionality
std_srvs/Empty        # Trigger without parameters
std_srvs/Trigger      # Trigger with success/message response
```

## Next Steps
Learn [ROS Parameters](../05-ros-parameters/README.md)
