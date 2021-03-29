import os

import matplotlib.pyplot as plt
import numpy as np
import rosbag


def read_bag(bag, joint_id, duration=8):
    position = []
    velocity = []
    acceleration = []
    effort_desired = []
    effort_actual = []

    steps = duration * 1000
    start = False
    count_1 = 0
    count_2 = 0
    for topic, msg, t in bag.read_messages():
        if topic in ["/iiwa_front/joint_position_trajectory_controller/state",
                     "/iiwa_front/joint_feedforward_trajectory_controller/state"]:
            if not start:
                if abs(msg.desired.positions[joint_id]) > 1e-2:
                    start = True
                else:
                    continue

            if count_1 < steps:
                count_1 += 1
                position.append([msg.desired.positions[joint_id], msg.actual.positions[joint_id], msg.error.positions[joint_id]])
                velocity.append([msg.desired.velocities[joint_id], msg.actual.velocities[joint_id], msg.error.velocities[joint_id]])
                acceleration.append(msg.desired.accelerations[joint_id])
                if len(msg.desired.effort) > 0:
                    effort_desired.append(-msg.desired.effort[joint_id])
        elif topic == "/iiwa_front/joint_states":
            if not start:
                continue
            elif count_2 < steps:
                count_2 += 1
                effort_actual.append(msg.effort[joint_id])

        if count_1 >= steps and count_2 >= steps:
            break
    return np.array(position), np.array(velocity), np.array(acceleration), np.stack([effort_desired, effort_actual], axis=1)


if __name__ == '__main__':
    bag_1 = rosbag.Bag(os.path.join("/home/puze/Desktop/real_trajectory/check_idm", "inertia_1_1" + ".bag"))
    bag_10 = rosbag.Bag(os.path.join("/home/puze/Desktop/real_trajectory/check_idm", "inertia_10_1" + ".bag"))
    bag_100 = rosbag.Bag(os.path.join("/home/puze/Desktop/real_trajectory/check_idm", "inertia_100_1" + ".bag"))

    joint_id = 3

    position_1, velocity_1, acceleration_1, effort_1 = read_bag(bag_1, joint_id, 12)
    position_10, velocity_10, acceleration_10, effort_10 = read_bag(bag_10, joint_id, 12)
    position_100, velocity_100, acceleration_100, effort_100 = read_bag(bag_100, joint_id, 12)

    fig, axes = plt.subplots(4, sharex=True)
    axes[0].plot(position_1[:, 1], label='1')
    axes[0].plot(position_10[:, 1], label='10')
    axes[0].plot(position_100[:, 1], label='100')
    axes[0].plot(position_100[:, 0], label='desired')
    axes[0].legend()
    axes[0].title.set_text('position')

    axes[1].plot(position_1[:, 2], label='1')
    axes[1].plot(position_10[:, 2], label='10')
    axes[1].plot(position_100[:, 2], label='100')
    axes[1].legend()
    axes[1].title.set_text('error')

    axes[2].plot(velocity_1[:, 1], label='1')
    axes[2].plot(velocity_10[:, 1], label='10')
    axes[2].plot(velocity_100[:, 1], label='100')
    axes[2].plot(velocity_100[:, 0], label='desired')
    axes[2].legend()
    axes[2].title.set_text('velocity')

    axes[3].plot(effort_1[:, 0], label='1')
    axes[3].plot(effort_10[:, 0], label='10')
    axes[3].plot(effort_100[:, 0], label='100')
    axes[3].plot(effort_1[:, 1], label='measured')
    axes[3].legend()
    axes[3].title.set_text('torque')

    plt.show()
