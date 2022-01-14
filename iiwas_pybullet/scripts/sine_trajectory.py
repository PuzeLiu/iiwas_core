#!/usr/bin/env python

import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rosbag

if __name__ == '__main__':
    type_name = 'joint_feedforward'
    use_front = True

    topic_name = '/iiwa_front/' if use_front else '/iiwa_back/'
    joint_prefix = 'F' if use_front else 'B'

    rospy.init_node("sine_command", anonymous=True)
    cmdPub = rospy.Publisher(topic_name + type_name + "_trajectory_controller/command", JointTrajectory, queue_size=1)
    rospy.sleep(2.0)

    t_final = 1
    init_position = np.array([0., 3., 0., 0., -0., 0., 0.]) * np.pi / 6.
    goal_position = np.array([2., 2, -2., -2., 2., 2., 2.]) * np.pi / 6.
    period = 2

    joint_id = [0]

    traj = JointTrajectory()

    traj.joint_names.append(joint_prefix + "_joint_1")
    traj.joint_names.append(joint_prefix + "_joint_2")
    traj.joint_names.append(joint_prefix + "_joint_3")
    traj.joint_names.append(joint_prefix + "_joint_4")
    traj.joint_names.append(joint_prefix + "_joint_5")
    traj.joint_names.append(joint_prefix + "_joint_6")
    traj.joint_names.append(joint_prefix + "_joint_7")


    traj_point_goal = JointTrajectoryPoint()
    traj_point_goal.positions = init_position
    traj_point_goal.velocities = [0., 0., 0., 0., 0., 0., 0.]
    traj_point_goal.time_from_start = rospy.Time(3.0)
    traj.points.append(traj_point_goal)
    traj.header.stamp = rospy.Time.now()
    cmdPub.publish(traj)
    rospy.sleep(3.0)

    traj.points.clear()
    for i in np.linspace(0, 4 * period * t_final, int(4 * period * t_final * 100) + 1):
        traj_point = JointTrajectoryPoint()

        traj_point.positions = init_position.copy()
        traj_point.velocities = np.zeros(7)
        # traj_point.accelerations = np.zeros(7)

        for joint_id_i in joint_id:
            q_0_i = init_position[joint_id_i]
            q_f_i = goal_position[joint_id_i]
            omega = np.pi / 2 / t_final
            traj_point.positions[joint_id_i] = -np.cos(omega * i) * (q_f_i - q_0_i) / 2 + (q_f_i + q_0_i) / 2
            traj_point.velocities[joint_id_i] = omega * np.sin(omega * i) * (q_f_i - q_0_i) / 2
            # traj_point.accelerations[joint_id_i] = (omega) ** 2 * np.cos(omega * i) * (q_f_i - q_0_i) / 2

        traj_point.time_from_start = rospy.Time(i)

        traj.points.append(traj_point)

    del traj.points[0]

    while not rospy.is_shutdown():
        traj.header.stamp = rospy.Time.now() + rospy.Duration(0.1)
        cmdPub.publish(traj)
        rospy.sleep(4 * period * t_final)
        # break

    # exit(0)
