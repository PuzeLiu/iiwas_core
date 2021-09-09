# iiwas_control 

This package is used for the controller specification. Available controller:

* [name_space]/*joint_state_controller*
* [name_space]/*joint_position_trajectory_controller*
* [name_space]/*joint_torque_trajectory_controller*
* [name_space]/*joint_feedforward_trajectory_controller*

An addtional *feedforward_trajectory_controller* is implemented to compensate the inertia term. The robot dynamics is

$$M(q)\ddot{q} + c(q, \dot{q}) + g(q) = \tau$$

$g(q)$ is compensated in iiwas_gazebo plugin, the feedforward term tries to compensate $M(q)\ddot{q}$.