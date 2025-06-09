# Sensor Data Reading

## Overview
Reading camera and laser sensor data using `sensor_msgs` in ROS

## Camera Data with sensor_msgs

### Basic Camera Subscriber (Python)
```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraReader:
    def __init__(self):
        rospy.init_node('camera_reader')
        self.bridge = CvBridge()
        
        # Subscribe to camera topic
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        rospy.loginfo("Camera reader started")
    
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process image
            self.process_image(cv_image)
            
        except Exception as e:
            rospy.logerr(f"Image processing error: {e}")
    
    def process_image(self, image):
        # Example processing
        height, width, channels = image.shape
        rospy.loginfo(f"Image: {width}x{height}, {channels} channels")
        
        # Display image (optional)
        cv2.imshow("Camera Feed", image)
        cv2.waitKey(1)

if __name__ == '__main__':
    reader = CameraReader()
    rospy.spin()
    cv2.destroyAllWindows()
```

### Camera Publisher (Python)
```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher:
    def __init__(self):
        rospy.init_node('camera_publisher')
        self.bridge = CvBridge()
        
        # Create publisher
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        
        # Open camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_HEIGHT, 480)
        
        rospy.loginfo("Camera publisher started")
    
    def publish_images(self):
        rate = rospy.Rate(30)  # 30 FPS
        
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                try:
                    # Convert OpenCV image to ROS message
                    image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    image_msg.header.stamp = rospy.Time.now()
                    image_msg.header.frame_id = "camera_frame"
                    
                    self.image_pub.publish(image_msg)
                    
                except Exception as e:
                    rospy.logerr(f"Publishing error: {e}")
            
            rate.sleep()
        
        self.cap.release()

if __name__ == '__main__':
    publisher = CameraPublisher()
    publisher.publish_images()
```

## Laser Data with sensor_msgs

### LaserScan Subscriber (Python)
```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class LaserReader:
    def __init__(self):
        rospy.init_node('laser_reader')
        
        # Subscribe to laser topic
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        rospy.loginfo("Laser reader started")
    
    def laser_callback(self, msg):
        # Extract laser data
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter invalid readings
        valid_indices = np.isfinite(ranges)
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        # Process laser data
        self.process_laser_data(valid_ranges, valid_angles, msg)
    
    def process_laser_data(self, ranges, angles, scan_msg):
        # Find closest obstacle
        min_distance = np.min(ranges)
        min_index = np.argmin(ranges)
        min_angle = angles[min_index]
        
        rospy.loginfo(f"Closest obstacle: {min_distance:.2f}m at {np.degrees(min_angle):.1f}°")
        
        # Detect obstacles in front (±30 degrees)
        front_indices = np.abs(angles) < np.radians(30)
        front_ranges = ranges[front_indices]
        
        if len(front_ranges) > 0:
            min_front = np.min(front_ranges)
            if min_front < 1.0:  # Less than 1 meter
                rospy.logwarn(f"Obstacle ahead: {min_front:.2f}m")

if __name__ == '__main__':
    reader = LaserReader()
    rospy.spin()
```

### Laser Data Visualization
```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

class LaserVisualizer:
    def __init__(self):
        rospy.init_node('laser_visualizer')
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        # Setup plot
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(projection='polar'))
        self.ax.set_ylim(0, 10)  # 10 meter range
        
    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        # Filter infinite values
        ranges[ranges == float('inf')] = msg.range_max
        ranges[ranges == float('-inf')] = 0
        
        # Clear and plot
        self.ax.clear()
        self.ax.plot(angles, ranges, 'b.', markersize=1)
        self.ax.set_ylim(0, msg.range_max)
        self.ax.set_title('Laser Scan Data')
        plt.pause(0.01)

if __name__ == '__main__':
    visualizer = LaserVisualizer()
    rospy.spin()
```

## C++ Camera Reader
```cpp
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class CameraReader {
private:
    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    cv_bridge::CvImagePtr cv_ptr_;

public:
    CameraReader() {
        image_sub_ = nh_.subscribe("/camera/image_raw", 1, &CameraReader::imageCallback, this);
        ROS_INFO("Camera reader started");
    }
    
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
        try {
            cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            processImage(cv_ptr_->image);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
    
    void processImage(const cv::Mat& image) {
        ROS_INFO("Image: %dx%d", image.cols, image.rows);
        cv::imshow("Camera Feed", image);
        cv::waitKey(1);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_reader");
    CameraReader reader;
    ros::spin();
    cv::destroyAllWindows();
    return 0;
}
```

## C++ Laser Reader
```cpp
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>

class LaserReader {
private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;

public:
    LaserReader() {
        laser_sub_ = nh_.subscribe("/scan", 1, &LaserReader::laserCallback, this);
        ROS_INFO("Laser reader started");
    }
    
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        // Find minimum distance
        auto min_it = std::min_element(msg->ranges.begin(), msg->ranges.end());
        
        if (min_it != msg->ranges.end() && std::isfinite(*min_it)) {
            double min_distance = *min_it;
            int min_index = std::distance(msg->ranges.begin(), min_it);
            double min_angle = msg->angle_min + min_index * msg->angle_increment;
            
            ROS_INFO("Closest: %.2fm at %.1f°", min_distance, min_angle * 180.0 / M_PI);
            
            if (min_distance < 1.0) {
                ROS_WARN("Obstacle detected!");
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "laser_reader");
    LaserReader reader;
    ros::spin();
    return 0;
}
```

## Sensor Message Types

### Image Message Structure
```python
sensor_msgs/Image:
  std_msgs/Header header
  uint32 height          # image height
  uint32 width           # image width  
  string encoding        # pixel format
  uint8 is_bigendian     # endianness
  uint32 step           # bytes per row
  uint8[] data          # image data
```

### LaserScan Message Structure
```python
sensor_msgs/LaserScan:
  std_msgs/Header header
  float32 angle_min      # start angle [rad]
  float32 angle_max      # end angle [rad]
  float32 angle_increment # angular distance between measurements [rad]
  float32 time_increment # time between measurements [s]
  float32 scan_time     # time between scans [s]
  float32 range_min     # minimum range value [m]
  float32 range_max     # maximum range value [m]
  float32[] ranges      # range data [m]
  float32[] intensities # intensity data
```

## Launch File for Sensors
```xml
<launch>
    <!-- Camera node -->
    <node name="usb_cam" pkg="usb_cam" type="usb_cam_node">
        <param name="video_device" value="/dev/video0"/>
        <param name="image_width" value="640"/>
        <param name="image_height" value="480"/>
        <param name="pixel_format" value="yuyv"/>
        <param name="camera_frame_id" value="camera_frame"/>
        <param name="io_method" value="mmap"/>
    </node>
    
    <!-- Laser scanner node -->
    <node name="rplidar_node" pkg="rplidar_ros" type="rplidarNode">
        <param name="serial_port" type="string" value="/dev/ttyUSB0"/>
        <param name="serial_baudrate" type="int" value="115200"/>
        <param name="frame_id" type="string" value="laser_frame"/>
    </node>
    
    <!-- Sensor processing nodes -->
    <node name="camera_reader" pkg="my_package" type="camera_reader.py"/>
    <node name="laser_reader" pkg="my_package" type="laser_reader.py"/>
</launch>
```

## Common Sensor Topics
```bash
/camera/image_raw          # Raw camera images
/camera/image_compressed   # Compressed images  
/scan                      # 2D laser scan
/velodyne_points          # 3D point cloud
/imu                      # IMU data
/gps/fix                  # GPS coordinates
```

## Testing Sensor Data
```bash
# View image
rosrun image_view image_view image:=/camera/image_raw

# Monitor laser scan
rostopic echo /scan

# Check sensor topics
rostopic list | grep -E "(image|scan|imu)"

# Measure data rates
rostopic hz /camera/image_raw
rostopic hz /scan
```

## Best Practices
- Use cv_bridge for image conversion
- Handle sensor data exceptions
- Filter invalid laser readings (inf, nan)
- Set appropriate queue sizes for high-frequency data
- Use proper frame_ids for coordinate transforms

## Next Steps
Learn [Robot Simulation with Gazebo](../07-gazebo-simulation/README.md)
