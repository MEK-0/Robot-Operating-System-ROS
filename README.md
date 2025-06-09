# ROS Package Structure & Configuration

## Overview
ROS package organization, `package.xml`, `CMakeLists.txt`, and dependency management

## Package Structure
```
my_package/
├── CMakeLists.txt      # Build instructions
├── package.xml         # Package manifest
├── src/               # Source code
├── scripts/           # Python executables
├── launch/            # Launch files
├── config/            # Configuration files
├── msg/               # Custom messages
└── srv/               # Service definitions
```

## package.xml Template
```xml
<?xml version="1.0"?>
<package format="2">
  <name>my_package</name>
  <version>1.0.0</version>
  <description>Package description</description>
  <maintainer email="dev@example.com">Developer</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>catkin</buildtool_depend>
  
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  
  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
</package>
```

## CMakeLists.txt Template
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

# C++ executable
add_executable(my_node src/my_node.cpp)
target_link_libraries(my_node ${catkin_LIBRARIES})

# Python scripts
catkin_install_python(PROGRAMS
  scripts/my_script.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

## Creating New Package
```bash
cd ~/catkin_ws/src
catkin_create_pkg my_package roscpp rospy std_msgs
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Common Dependencies
```xml
<!-- Core -->
<depend>roscpp</depend>         <!-- C++ -->
<depend>rospy</depend>          <!-- Python -->
<depend>std_msgs</depend>       <!-- Standard messages -->
<depend>geometry_msgs</depend>  <!-- Geometry messages -->
<depend>sensor_msgs</depend>    <!-- Sensor messages -->

<!-- Vision -->
<depend>cv_bridge</depend>      <!-- OpenCV bridge -->
<depend>image_transport</depend>

<!-- Navigation -->
<depend>move_base</depend>
<depend>tf2</depend>
```

## Building Commands
```bash
# Build all packages
catkin_make

# Build specific package
catkin_make --only-pkg-with-deps my_package

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y
```

## Best Practices
- Use descriptive package names
- Minimal necessary dependencies
- Consistent file organization
- Document in README.md

## Next Steps
Learn [Publisher & Subscriber](../03-publisher-subscriber/README.md)
