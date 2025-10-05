````markdown
# Autocar ROS Package

Author: Mahboob Alam  
Date: 2025-10-04  
License: MIT  

## Overview

This ROS package (`my_package`) provides a **fully custom 4-wheel robot setup** with:

- Custom **wheel control** using `ros_control` transmissions (all 4 wheels individually powered)
- URDF/Xacro model of the robot (`urdf/car.xacro`)
- Visualization in RViz (`rviz/full_stack.rviz`)
- 2D map support for localization (`mapping/map/map01.pgm` & `.yaml`)
- No Gazebo simulation required  

The robot uses **individual wheel transmissions**, giving full control over all wheels via custom Python controllers.  

---

## Features

- **Custom Wheel Controller**:  
  The `controller.py` node allows individual wheel velocity control through ROS topics.  
- **URDF/Xacro Model**:  
  Fully defined robot with chassis, 4 wheels, LiDAR, IMU, GPS, and camera.  
- **RViz Visualization**:  
  Launch RViz to view the robot, sensors, and map.  
- **Map Support**:  
  Includes a sample 2D map (`map01.pgm` + `.yaml`) for localization and navigation.  
- **ROS Control**:  
  Wheels use `transmission_interface/SimpleTransmission` for precise velocity control.  

---

## Dependencies

- ROS Noetic
- `robot_state_publisher`
- `joint_state_publisher`
- `rviz`
- `map_server`
- `tf2_ros`

---

## Launch Instructions

1. Build your workspace:
```bash
cd ~/autocar_ws
catkin_make
source devel/setup.bash
````

2. Launch the full stack:

```bash
roslaunch my_package display.launch
```

### What this launch file does:

1. Loads the **robot URDF** via Xacro.
2. Starts the **Robot State Publisher** to broadcast TF frames.
3. Starts **Joint State Publisher**.
4. Launches **RViz** for visualization.
5. Runs your **custom wheel controller** (`controller.py`).
6. Starts the **Map Server** with your 2D map for localization.
7. Publishes a **static transform** from `map` to `odom` frame.

---

## Topics

* `/front_left_wheel_cmd` : `std_msgs/Float64` — velocity command for front-left wheel
* `/front_right_wheel_cmd` : `std_msgs/Float64` — velocity command for front-right wheel
* `/rear_left_wheel_cmd` : `std_msgs/Float64` — velocity command for rear-left wheel
* `/rear_right_wheel_cmd` : `std_msgs/Float64` — velocity command for rear-right wheel
* `/joint_states` : `sensor_msgs/JointState` — robot joint states for visualization
* `/lidar_points` : `sensor_msgs/PointCloud2` — simulated 3D LiDAR points

---

## Notes

* This package **does not use Gazebo**. All control is handled via Python nodes and `ros_control` transmissions.
* The robot can be used for **custom simulations**, localization, and sensor visualization.
* Modify `controller.py` to integrate with your path planning or autonomous algorithms.

---

## License

MIT License © 2025 Mahboob Alam

```