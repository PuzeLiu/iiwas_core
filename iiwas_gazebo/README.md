# iiwas_gazebo Package

This package starts the simulation in gazebo. To simulate the real world's KUKA controller, we add a gravity compensation plugin in the simulation. 

To start the simulation:
```console
roslaunch iiwas_gazebo iiwas_gazebo.launch
```

Controller type can be change via changing:
```xml
    <arg name="hardware_interface" default="hardware_interface/PositionJointInterface"/>
    <arg name="controller_type" default="joint_position_trajectory_controller"/>
```
Here are possible controller options (change the controller with the hardware_interface simultaneously):
* `joint_position_trajectory_controller` - (hardware_interface/PositionJointInterface)
* `joint_torque_trajectory_controller` - (hardware_interface/EffortJointInterface)
* `joint_feedforward_trajectory_controller` - (hardware_interface/EffortJointInterface)