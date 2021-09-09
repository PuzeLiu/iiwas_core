## Prerequisite 
Make sure you have installed ROS and create a new catkin workspace. 
Additionally, to the full desktop version of ROS, the following ROS packages are needed:
`joint-trajectory-controller`, `pinocchio`
Install prerequisite packages as
```
sudo apt-get install ros-[DISTRO]-joint-trajectory-controller ros-[DISTRO]-rqt-joint-trajectory-controller ros-[DISTRO]-pinocchio
```

## Installation 
Clone the repository into the src folder of your catkin workspace. 
Build the package with Release mode
```asm
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Usage

Starting up the gazebo simulation

```
roslaunch iiwas_gazebo iiwas_gazebo.launch use_front_iiwa:=[true|false] front_controllers:=[CONTROLLER_TYPE] use_back_iiwa:=[true|false] back_controllers:=[CONTROLLER_TYPE]
```

Available controllers can be found here in the package **[iiwas_control]**

