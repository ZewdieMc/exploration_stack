# PhD assignment - autonomous exploration 

This ROS package is designed for robot planning tasks, including state machine execution, frontier-based exploration, global path planning, and local path following using DWA algorithm.

## How to Run

```sh
# Install dependencies
sudo apt update

# Gmapping SLAM
sudo apt install ros-noetic-slam-gmapping
```

## clone the repository and build the workspace
```sh
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone git@github.com:ZewdieMc/exploration_stack.git
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

1. **Launch for Task 1**
    ```sh
      roslaunch phd_luea task1.launch

      # in new terminal (separate tab) or circle follower
      rosrun phd_luea circle_follower.py

      # or pose-to-pose navigation
      rosrun phd_luea pose_goal_publisher.py
    ```

2. **Launch for Task 2 & 3**
    ```sh
      roslaunch phd_luea custom.launch
      roslaunch phd_luea planning.launch
      rosservice call /start_exploration "data: true"
    ```

3. **Visualize the State Machine**
    Use `smach_viewer` to visualize the state machine:
    ```sh
      sudo apt-get install ros-<your-ros-distro>-smach-viewer
      rosrun smach_viewer smach_viewer.py
    ```

## To test explore_lite
```sh

# Frontier Exploration
sudo apt install ros-noetic-explore-lite

# Navigation stack & dependencies
sudo apt install ros-noetic-navigation ros-noetic-map-server

roslaunch phd_luea task2.launch # then open rviz config (src/phd_luea/rviz/task2.rviz)
```

## Demo
[Watch this video on YouTube](https://www.youtube.com/watch?v=BC9ialCNbX4)
