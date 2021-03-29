import os

import matplotlib.pyplot as plt
import numpy as np
import rosbag


def read_bag(bag, joint_id):
    position = []
    velocity = []
    acceleration = []
    effort = []

    start = False
    count = 0
    for topic, msg, t in bag.read_messages():
        if topic in ["/iiwa_front/joint_position_trajectory_controller/state",
                     "/iiwa_front/joint_feedforward_trajectory_controller/state"]:
            if not start:
                if abs(msg.desired.positions[joint_id]) > 1e-2:
                    start = True
                else:
                    continue

            if count < 8000:
                count += 1
                position.append([msg.desired.positions[joint_id], msg.actual.positions[joint_id], msg.error.positions[joint_id]])
                velocity.append([msg.desired.velocities[joint_id], msg.actual.velocities[joint_id], msg.error.velocities[joint_id]])
                acceleration.append([msg.desired.accelerations[joint_id]])
                if len(msg.desired.effort) > 0:
                    effort.append([msg.desired.effort[joint_id]])
            else:
                break
        elif topic == ["iiwa_front/joint_states"]:
            if not start:
                continue
            elif count < 8000:
                effort.append([msg.effort[joint_id]])
    return np.array(position), np.array(velocity), np.array(acceleration), np.array(effort)


if __name__ == '__main__':
    bag_1 = rosbag.Bag(os.path.join("/home/puze/air_hockey_record/test_inertia", "inertia_1" + ".bag"))
    # bag_grav = rosbag.Bag(os.path.join("/home/puze/air_hockey_record/gravComp_Vs_FF/Joint_4", "position_pid_gravity" + ".bag"))
    bag_10 = rosbag.Bag(os.path.join("/home/puze/air_hockey_record/test_inertia", "inertia_10" + ".bag"))
    bag_100 = rosbag.Bag(os.path.join("/home/puze/air_hockey_record/test_inertia", "inertia_100" + ".bag"))

    joint_id = 3

    position_1, velocity_1, acceleration_1, effort_1 = read_bag(bag_1, joint_id)
    position_10, velocity_10, acceleration_10, effort_10 = read_bag(bag_10, joint_id)
    position_100, velocity_100, acceleration_100, effort_100 = read_bag(bag_100, joint_id)

    ax = plt.subplot()
    ax.plot(position_1[:, 2], label='1')
    ax.plot(position_10[:, 2], label='10')
    ax.plot(position_100[:, 2], label='100')
    ax.legend()
    ax.title.set_text('error')

    fig, ax = plt.subplots(1)
    ax.plot(position_1[:, 1], label='1')
    ax.plot(position_10[:, 1], label='10')
    ax.plot(position_100[:, 1], label='10')
    ax.plot(position_10[:, 0], label='desired')
    ax.legend()
    ax.title.set_text('position')

    fig, ax = plt.subplots(1)
    ax.plot(effort_1, label='1')
    ax.plot(effort_10, label='10')
    ax.plot(effort_100, label='100')
    ax.legend()
    ax.title.set_text('torque')
    plt.show()
