#!/usr/bin/env python

import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rosbag

if __name__ == '__main__':
    # type_name = 'position'
    type_name = 'torque'
    use_front = False

    topic_name = '/iiwa_front/' if use_front else '/iiwa_back/'
    joint_prefix = 'F' if use_front else 'B'

    rospy.init_node("sine_command", anonymous=True)
    cmdPub = rospy.Publisher(topic_name + "joint_" + type_name + "_trajectory_controller/command", JointTrajectory, queue_size=1)
    rospy.sleep(2.0)
    q_init = np.array([0., 0., 0., 0., 0., 0., 0.])

    t_final = 0.5
    goal = 0.5
    period = 8

    joint_id = 2

    traj = JointTrajectory()

    traj.joint_names.append(joint_prefix + "_joint_1")
    traj.joint_names.append(joint_prefix + "_joint_2")
    traj.joint_names.append(joint_prefix + "_joint_3")
    traj.joint_names.append(joint_prefix + "_joint_4")
    traj.joint_names.append(joint_prefix + "_joint_5")
    traj.joint_names.append(joint_prefix + "_joint_6")
    traj.joint_names.append(joint_prefix + "_joint_7")

    init_position = [0., np.pi/4, 0., -np.pi/2, 0., 0, 0.]
    traj_point_goal = JointTrajectoryPoint()
    traj_point_goal.positions = init_position
    traj_point_goal.velocities = [0., 0., 0., 0., 0., 0., 0.]
    traj_point_goal.time_from_start = rospy.Time(3.0)
    traj.points.append(traj_point_goal)
    traj.header.stamp = rospy.Time.now()
    cmdPub.publish(traj)
    rospy.sleep(3.0)

    traj.points.clear()
    for i in np.linspace(0, period * t_final, int(period * t_final * 100)):
        traj_point = JointTrajectoryPoint()

        traj_point.positions = init_position.copy()
        traj_point.positions[joint_id] = -np.cos(np.pi / 2 / t_final * i) * goal + goal
        traj_point.velocities = [0., 0., 0., 0., 0., 0., 0.]
        traj_point.velocities[joint_id] = np.pi / 2 / t_final * np.sin(np.pi / 2 / t_final * i) * goal
        traj_point.time_from_start = rospy.Time(i)

        traj.points.append(traj_point)

    while True:
        traj.header.stamp = rospy.Time.now()
        cmdPub.publish(traj)
        rospy.sleep(period * t_final)
