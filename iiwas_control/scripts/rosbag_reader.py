import os
import numpy as np
import matplotlib.pyplot as plt
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
    file = "2021-03-23-18-32-14"
    bag = rosbag.Bag(os.path.join("/home/puze/Desktop/real_trajectory/feedforward", file + ".bag"))

    pos, vel, acc, torque = read_bag(bag, joint_id=5)

    fig, axes = plt.subplots(3)
    t = np.linspace(0, 8, 8000)
    axes[0].plot(t, pos[:, 0], label="desired_pos")
    axes[0].plot(t, pos[:, 1], label="actual_pos")
    axes[1].plot(t, vel[:, 0], label="desired_vel")
    axes[1].plot(t, vel[:, 1], label="actual_vel")
    axes[2].scatter(t, acc[:, 0], label="desired_acc", s=1)
    # axes[2].plot(acc[:, 1], label="actual_acc")
    axes[0].legend()
    axes[1].legend()
    axes[2].legend()
    plt.show()