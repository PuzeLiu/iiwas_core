## Installation 
Make sure you have installed ROS and create a new catkin workspace. 
Additionally, to the full desktop version of ROS, the following ROS packages are needed:
1. joint-trajectory-controller
2. pinocchio

and these third-party dependencies:

1. tinyspline  https://github.com/msteinbeck/tinyspline/releases

Clone the repository into the src folder of your catkin workspace. Do not forget to also clone the [IIWAS FRI Client](https://git.ias.informatik.tu-darmstadt.de/ros/iiwas/iiwa_fri_client) into the catkin workspace. 
After that you can build the project with **catkin_make**. (Note that the first build will probably fail because of some complicated dependencies between the core and the FRI package. However simply running **catkin_make** once again will work.)
Before continuing add your workspace to the ROS_PACKAGE_PATH by sourcing new devel/setup.*sh file (in the bashrc).

### pybullet_ros
iiwas_pybullet package is used to use **pybullet** together with **ros_control** and **hardware_interface** as Gazebo simulator has some weired issues on the torque controller. If you don't want to use this package, you could put a **CATKIN_IGNORE** in the **iiwas_pybullet** package using following:

```console
touch iiwas_pybullet/CATKIN_IGNORE
```

If you want to use the pybullet as simulator, you could clone [pybullet_ros](https://github.com/PuzeLiu/pybullet_ros) into your catkin_workspace.

You should do above procedure on both fermat and lovelace, since lovelace is the real_time computer on which the actual control will later run. On the fermat computer, you simply need the packages to have access to the launch files.

## Usage

Starting up the control and visualization package can easily be done via the following launch command

```
roslaunch iiwas_bringup iiwas_remote.launch use_front_iiwa:=[TRUE|FALSE] front_controllers:=[CONTROLLER_TYPE] use_back_iiwa:=[TRUE|FALSE] back_controllers:=[CONTROLLER_TYPE] user_name:=[USER] env_load_dir:=[WS_PATH]/src/iiwas_core/iiwas_bringup/env.sh
```

In above line, the important arguments for that launch file are highlighted. The use_*_iiwa arguments control for which arms the controllers are running. The *_controllers argument decide upon how the active arms are controlled. There are three options to choose from:

* joint_torque_trajectory_controller: PID torque controller -> Uses torque overlay commands to control the robot. It's the most precise controller, but the actuator signal can be too noisy.
* joint_position_trajectory_controller: PD-Controller -> Uses Kuka's position control. Achieves good tracking and steady-state precision but is not compliant.
* joint_position_impedance_trajectory_controller: Impedance Controller -> Uses Kuka's joint impedance control. It's compliant but less precise than position control.

The default argument is to use impedance control, as it is the safest operation mode. Please only deviate from this default when you are familiar with the robot and need higher preicision for your task.

The two final arguments are more interesting. The **user_name** argument needs to be set to the name of your account on the **lovelace** computer. The [WS_PATH] placeholder in **env_load_dir** needs to be replaced by the path to the catkin workspace on the **lovelace** computer.

One more step that needs to be done before above launch script will work is to do some changes in the **[WS_PATH]/src/iiwas_core/iiwas_bringup/env.sh** file on **lovelace**. That is, you need to replace

```
source ~/catkin_ws/devel/setup.bash  #TODO change use your own workspace
```

in **env.sh** by the path to your workspace setup.bash. After that everything should work smoothly. In the future, we will create a default user on the **lovelace** computer such that starting does not require above changes except from the arms you want to control.

