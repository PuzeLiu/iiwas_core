## Installation 

Clone the repository into your catkin workspace. Do not forget to also clone the [IIWAS FRI Client](https://git.ias.informatik.tu-darmstadt.de/ros/iiwas/iiwa_fri_client) into the catkin workspace. After that you can build the project. Note that the first build will probably fail because of some complicated dependencies between the core and the FRI package. However simply running **catkin_make** once again will work.

You should do above procedure on both fermat and lovelace, since lovelace is the real_time computer on which the actual control will later run. On the fermat computer, you simply need the packages to have access to the launch files.

## Usage

Starting up the control and visualization package can easily be done via the following launch command

```
roslaunch iiwas_bringup iiwas_remote.launch use_front_iiwa:=[TRUE|FALSE] front_controllers:=[CONTROLLER_TYPE] use_back_iiwa:=[TRUE|FALSE] back_controllers:=[CONTROLLER_TYPE] user_name:=[USER] env_load_dir:=[WS_PATH]/src/iiwas_core/iiwas_bringup/env.sh
```

In above line, the important arguments for that launch file are highlighted. The use_*_iiwa arguments control for which arms the controllers are running. The *_controllers argument decide upon how the active arms are controlled. There are three options to choose from:

* joint_torque_trajectory_controller: TODO
* joint_position_trajectory_controller: PD-Controller -> achieves higher precision but is less compliant
* joint_position_impedance_trajectory_controller: Impedance Controller -> compliance but less precise

The default argument is to use impedance control, as it is the safest operation mode. Please only deviate from this default when you are familiar with the robot and need higher preicision for your task.

The two final arguments are more interesting. The **user_name** argument needs to be set to the name of your account on the **lovelace** computer. The [WS_PATH] placeholder in **env_load_dir** needs to be replaced by the path to the catkin workspace on the **lovelace** computer.

One more step that needs to be done before above launch script will work is to do some changes in the **[WS_PATH]/src/iiwas_core/iiwas_bringup/env.sh** file on **lovelace**. That is, you need to replace

```
source ~/catkin_ws/devel/setup.bash  #TODO change use your own workspace
```

in **env.sh** by the path to your workspace setup.bash. After that everything should work smoothly. In the future, we will create a default user on the **lovelace** computer such that starting does not require above changes except from the arms you want to control.

