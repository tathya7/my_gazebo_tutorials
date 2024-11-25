# ROS2 - Gazebo Tutorials

This repository contains a ROS2 package of a simple Roomba Robot with alternating rotation in clockwise and anti-clockwise whenever an obstacle is encountered. Made by **Tathya Bhatt** for the course *ENPM700-Software Development for Robotics*

## Author
- **Name** : Tathya Bhatt
- **UID** : 120340246

## State Machine Design 

As robots require robust state transitions encountering complex behaviors, state machine design pattern is used for the project. This design pattern incorporates transition to Forward and Rotating State while detecting objects using LiDAR Sensor with Turtlebot3 Burger.

## Dependencies

This project uses the ROS2 Humble distribution. Turtlebot3 Package is assumed to be installed as the project requires to use one of the three turtlebot3 models.

## Roomba in Action!

Watch the robot in action in a complex map with obstacles: 

[![Watch the Video](https://i9.ytimg.com/vi/IVSIc9BGPhI/mq2.jpg?sqp=CIiKj7oG-oaymwEmCMACELQB8quKqQMa8AEB-AHUBoAC4AOKAgwIABABGGUgZShlMA8=&rs=AOn4CLDla52OFE5MEgdvO2k5N2OZ7cap-Q)](https://youtu.be/IVSIc9BGPhI)

## Building & Running Steps

- ### Cloning the package into a workspace folder:
```
# Make a workspace directory
mkdir -p ~/ros2_ws/src

# Go to the directory and clone the repository
git clone 

#  Install resdep dependencies before building the package
rosdep install -i --from-path src --rosdistro humble -y
```
- ### Build the package and source:
```
# Go back to the workspace folder and build the package
colcon build --packages-select walker --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1


# After successful build, source the package
source install/setup.bash

```

- ### Export Turtlebot3
```export TURTLEBOT3_MODEL=burger```

- ### Launching the Gazebo Environment:
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

```

- ### Running the Roomba Node:
```
# With rosbag recording enabled
ros2 launch walker walker.launch.py record_bag:=true

# With rosbag recording disabled
ros2 launch walker walker.launch.py record_bag:=false

```
- ### Inspecting rosbag
```
cd ros2_ws/src/walker/results/
ros2 bag info my_bag.db3

```

- ### Replaying rosbag
```
cd ros2_ws/src/walker/results/
ros2 bag play my_bag.db3
```

## Checking Google C++ Style Guide and static code analysis

```
# Go to the package directory and src folder
cd ~/ros_ws/src/walker/src

# Format and modify the code 
clang-format -style=Google -i walker.cpp

# Go back to workspace directory and perform clang-tidy checks
cd ..
cd ..
cd ..
clang-tidy -p build/ src/walker/src/*.cpp

# Static code analysis, go to the package directory 
cd src/walker
cpplint --filter=-build/c++11,+build/c++17,-build/namespaces,-build/include_order  src/*.cpp > cpplint.txt

```
