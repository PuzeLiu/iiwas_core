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
                if abs(msg.desired.positions[joint_id]) > 1e-3:
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
    bag_pid = rosbag.Bag(os.path.join("/home/puze/Desktop/real_trajectory/feedforward/Joint_1", "pid_0" + ".bag"))
    # bag_grav = rosbag.Bag(os.path.join("/home/puze/air_hockey_record/gravComp_Vs_FF/Joint_4", "position_pid_gravity" + ".bag"))
    bag_ff = rosbag.Bag(os.path.join("/home/puze/Desktop/real_trajectory/feedforward/Joint_1", "pid_ff_0" + ".bag"))

    joint_id = 0

    position_pid, velocity_pid, acceleration_pid, effort_pid = read_bag(bag_pid, joint_id)
    # position_grav, velocity_grav, effort_grav = read_bag(bag_grav)
    position_ff, velocity_ff, acceleration_ff, effort_ff = read_bag(bag_ff, joint_id)

    ax = plt.subplot()
    ax.plot(position_pid[:, 2], label='pid')
    # ax.plot(np.abs(position_grav[:, 2]), label='pid + gravity')
    ax.plot(position_ff[:, 2], label='pid + ff')
    ax.legend()
    ax.title.set_text('error')

    fig, ax = plt.subplots(1)
    ax.plot(position_pid[:, 1], label='pid')
    # ax.plot(np.abs(position_grav[:, 2]), label='pid + gravity')
    ax.plot(position_ff[:, 1], label='pid + ff')
    ax.plot(position_ff[:, 0], label='desired')
    ax.legend()
    ax.title.set_text('actual')

    fig, ax = plt.subplots(1)
    ax.plot(effort_pid, label='pid')
    # axes[0].plot(effort_grav, label='pid + gravity')
    ax.plot(effort_ff, label='pid + ff')
    ax.legend()
    ax.title.set_text('torque')
    plt.show()
