import os

import matplotlib.pyplot as plt
import numpy as np
import rosbag


def read_bag(bag):
    position = []
    velocity = []
    effort = []

    start = False
    count = 0
    for topic, msg, t in bag.read_messages():
        if topic in ["/iiwa_front/joint_position_trajectory_controller/state",
                     "/iiwa_front/joint_feedforward_trajectory_controller/state"]:
            if not start:
                if abs(msg.desired.positions[3]) > 1e-3:
                    start = True
                else:
                    continue

            if count < 8000:
                count += 1
                position.append([msg.desired.positions[3], msg.actual.positions[3], msg.error.positions[3]])
                velocity.append([msg.desired.velocities[3], msg.actual.velocities[3], msg.error.velocities[3]])
                # if len(msg.desired.effort) > 0:
                #     effort.append([msg.desired.effort[3]])
            else:
                break
        elif topic == ["iiwa_front/joint_states"]:
            if not start:
                continue
            elif count < 8000:
                effort.append(msg.effort[3])
    return np.array(position), np.array(velocity), np.array(effort)


if __name__ == '__main__':
    bag_pid = rosbag.Bag(os.path.join("/home/puze/air_hockey_record/gravComp_Vs_FF/Joint_4", "position_pid" + ".bag"))
    bag_grav = rosbag.Bag(os.path.join("/home/puze/air_hockey_record/gravComp_Vs_FF/Joint_4", "position_pid_gravity" + ".bag"))
    bag_ff = rosbag.Bag(os.path.join("/home/puze/air_hockey_record/gravComp_Vs_FF/Joint_4", "position_pid_gravity_ff" + ".bag"))

    position_pid, velocity_pid, effort_pid = read_bag(bag_pid)
    position_grav, velocity_grav, effort_grav = read_bag(bag_grav)
    position_ff, velocity_ff, effort_ff = read_bag(bag_ff)

    ax = plt.subplot()
    # ax.plot(np.abs(position_pid[:, 2]), label='pid')
    ax.plot(np.abs(position_grav[:, 2]), label='pid + gravity')
    ax.plot(np.abs(position_ff[:, 2]), label='pid + gravity + ff')
    ax.legend()
    ax.title.set_text('error')

    axes = plt.subplots(11)
    axes[0].plot(effort_pid, label='pid')
    axes[0].plot(effort_grav, label='pid + gravity')
    axes[0].plot(effort_ff, label='pid + gravity + ff')

    plt.show()
