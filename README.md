# Robot Simulation: Gazebo

## Overview
Setting up and running robot simulations in Gazebo with ROS integration

## Basic Robot URDF
Create `urdf/simple_robot.urdf`:
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
    <!-- Base link -->
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.6 0.4 0.2"/>
            </geometry>
            <material name="blue">
                <color rgba="0 0 1 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.6 0.4 0.2"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="10"/>
            <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
        </inertial>
    </link>
    
    <!-- Left wheel -->
    <link name="left_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <material name="black">
                <color rgba="0 0 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
    </link>
    
    <!-- Right wheel -->
    <link name="right_wheel">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <material name="black">
                <color rgba="0 0 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="1"/>
            <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
        </inertial>
    </link>
    
    <!-- Joints -->
    <joint name="left_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="left_wheel"/>
        <origin xyz="0 0.2 -0.1" rpy="1.5708 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>
    
    <joint name="right_wheel_joint" type="continuous">
        <parent link="base_link"/>
        <child link="right_wheel"/>
        <origin xyz="0 -0.2 -0.1" rpy="1.5708 0 0"/>
        <axis xyz="0 0 1"/>
    </joint>
</robot>
```

## Gazebo Launch File
Create `launch/gazebo_simulation.launch`:
```xml
<launch>
    <!-- Gazebo world -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find my_package)/worlds/simple_world.world"/>
        <arg name="paused" value="false"/>
        <arg name="use_sim_time" value="true"/>
        <arg name="gui" value="true"/>
        <arg name="headless" value="false"/>
        <arg name="debug" value="false"/>
    </include>
    
    <!-- Robot description -->
    <param name="robot_description" command="$(find xacro)/xacro $(find my_package)/urdf/simple_robot.urdf"/>
    
    <!-- Spawn robot in Gazebo -->
    <node name="spawn_robot" pkg="gazebo_ros" type="spawn_model"
          args="-urdf -model simple_robot -param robot_description -x 0 -y 0 -z 0.1"/>
    
    <!-- Robot state publisher -->
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
    
    <!-- Joint state publisher -->
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher"/>
</launch>
```

## Robot with Sensors URDF
Create `urdf/robot_with_sensors.urdf`:
```xml
<?xml version="1.0"?>
<robot name="robot_with_sensors">
    <!-- Include basic robot -->
    <!-- ... (previous robot links) ... -->
    
    <!-- Camera link -->
    <link name="camera_link">
        <visual>
            <geometry>
                <box size="0.05 0.05 0.05"/>
            </geometry>
            <material name="red">
                <color rgba="1 0 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <box size="0.05 0.05 0.05"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.1"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>
    
    <!-- Camera joint -->
    <joint name="camera_joint" type="fixed">
        <parent link="base_link"/>
        <child link="camera_link"/>
        <origin xyz="0.3 0 0.1" rpy="0 0 0"/>
    </joint>
    
    <!-- Laser link -->
    <link name="laser_link">
        <visual>
            <geometry>
                <cylinder radius="0.05" length="0.03"/>
            </geometry>
            <material name="green">
                <color rgba="0 1 0 1"/>
            </material>
        </visual>
        <collision>
            <geometry>
                <cylinder radius="0.05" length="0.03"/>
            </geometry>
        </collision>
        <inertial>
            <mass value="0.2"/>
            <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
        </inertial>
    </link>
    
    <!-- Laser joint -->
    <joint name="laser_joint" type="fixed">
        <parent link="base_link"/>
        <child link="laser_link"/>
        <origin xyz="0 0 0.15" rpy="0 0 0"/>
    </joint>
    
    <!-- Gazebo plugins -->
    <gazebo reference="camera_link">
        <sensor type="camera" name="camera1">
            <update_rate>30.0</update_rate>
            <camera name="head">
                <horizontal_fov>1.3962634</horizontal_fov>
                <image>
                    <width>800</width>
                    <height>600</height>
                    <format>R8G8B8</format>
                </image>
                <clip>
                    <near>0.02</near>
                    <far>300</far>
                </clip>
            </camera>
            <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
                <alwaysOn>true</alwaysOn>
                <updateRate>0.0</updateRate>
                <cameraName>robot/camera1</cameraName>
                <imageTopicName>image_raw</imageTopicName>
                <cameraInfoTopicName>camera_info</cameraInfoTopicName>
                <frameName>camera_link</frameName>
            </plugin>
        </sensor>
    </gazebo>
    
    <gazebo reference="laser_link">
        <sensor type="gpu_ray" name="laser">
            <pose>0 0 0 0 0 0</pose>
            <visualize>false</visualize>
            <update_rate>40</update_rate>
            <ray>
                <scan>
                    <horizontal>
                        <samples>720</samples>
                        <resolution>1</resolution>
                        <min_angle>-1.570796</min_angle>
                        <max_angle>1.570796</max_angle>
                    </horizontal>
                </scan>
                <range>
                    <min>0.10</min>
                    <max>30.0</max>
                    <resolution>0.01</resolution>
                </range>
            </ray>
            <plugin name="gazebo_ros_laser" filename="libgazebo_ros_gpu_laser.so">
                <topicName>/scan</topicName>
                <frameName>laser_link</frameName>
            </plugin>
        </sensor>
    </gazebo>
</robot>
```

## Robot Control Node
Create `scripts/robot_controller.py`:
```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class Robot
