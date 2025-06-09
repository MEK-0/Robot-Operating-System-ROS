# Publisher & Subscriber

## Overview
ROS publish-subscribe communication pattern with `std_msgs/String` examples

## Concept
```
[Publisher] ---> [Topic] ---> [Subscriber(s)]
```

## Python Publisher
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def publisher():
    rospy.init_node('publisher')
    pub = rospy.Publisher('hello_topic', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz
    
    count = 0
    while not rospy.is_shutdown():
        msg = String()
        msg.data = f"Hello World {count}"
        pub.publish(msg)
        rospy.loginfo(msg.data)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    publisher()
```

## Python Subscriber
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo(f"Received: {msg.data}")

def subscriber():
    rospy.init_node('subscriber')
    rospy.Subscriber('hello_topic', String, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
```

## C++ Publisher
```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "publisher");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<std_msgs::String>("hello_topic", 10);
    ros::Rate rate(1);
    
    int count = 0;
    while (ros::ok()) {
        std_msgs::String msg;
        msg.data = "Hello World " + std::to_string(count++);
        pub.publish(msg);
        ROS_INFO("%s", msg.data.c_str());
        rate.sleep();
    }
    return 0;
}
```

## C++ Subscriber
```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

void callback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Received: %s", msg->data.c_str());
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("hello_topic", 10, callback);
    ros::spin();
    return 0;
}
```

## Launch File
```xml
<launch>
    <node name="publisher" pkg="my_package" type="publisher.py" output="screen"/>
    <node name="subscriber" pkg="my_package" type="subscriber.py" output="screen"/>
</launch>
```

## Testing Commands
```bash
# Monitor topic
rostopic echo /hello_topic

# Check topic info
rostopic info /hello_topic

# Manual publish
rostopic pub /hello_topic std_msgs/String "data: 'Test Message'"

# Check message rate
rostopic hz /hello_topic
```

## Common Message Types
```bash
std_msgs/String     # Text data
std_msgs/Int32      # Integer
std_msgs/Float64    # Floating point
std_msgs/Bool       # Boolean
geometry_msgs/Twist # Velocity commands
```

## Running Example
```bash
# Terminal 1
roscore

# Terminal 2  
rosrun my_package publisher.py

# Terminal 3
rosrun my_package subscriber.py

# Terminal 4 (monitoring)
rostopic echo /hello_topic
```

## Best Practices
- Use appropriate queue sizes (10 is typical)
- Handle rospy.ROSInterruptException
- Use meaningful topic names
- Log important events
- Set appropriate publishing rates

## Next Steps
Learn [Services & Service Client](../04-service-client/README.md)
