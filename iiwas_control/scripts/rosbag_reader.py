import os
import numpy as np
import matplotlib.pyplot as plt
import rosbag

if __name__ == '__main__':
    torque_file = "torque_2"
    position_file = "torque_3"
    bag_torque = rosbag.Bag(os.path.join("/home/puze/Desktop", torque_file + ".bag"))
    bag_position = rosbag.Bag(os.path.join("/home/puze/Desktop", position_file + ".bag"))

    des_positions_torque = []
    actual_positions_torque = []
    error_positions_torque = []

    des_velocities_torque = []
    actual_velocities_torque = []
    error_velocities_torque = []

    des_positions_position = []
    actual_positions_position = []
    error_positions_position = []

    des_velocities_position = []
    actual_velocities_position = []
    error_velocities_position = []

    start = False
    for topic, msg, t in bag_torque.read_messages():
        if abs(msg.desired.positions[3]) > 1e-3:
            start = True
        if start:
            des_positions_torque.append(msg.desired.positions[3])
            actual_positions_torque.append(msg.actual.positions[3])
            error_positions_torque.append(np.abs(msg.error.positions[3]))

            des_velocities_torque.append(msg.desired.velocities[3])
            actual_velocities_torque.append(msg.actual.velocities[3])
            error_velocities_torque.append(msg.error.velocities[3])

    start = False
    for topic, msg, t in bag_position.read_messages():
        if abs(msg.desired.positions[3]) > 1e-3:
            start = True
        if start:
            des_positions_position.append(msg.desired.positions[3])
            actual_positions_position.append(msg.actual.positions[3])
            error_positions_position.append(np.abs(msg.error.positions[3]))

            des_velocities_position.append(msg.desired.velocities[3])
            actual_velocities_position.append(msg.actual.velocities[3])
            error_velocities_position.append(msg.error.velocities[3])

    fig, axes = plt.subplots(3)
    axes[0].plot(des_positions_torque, label="torque")
    axes[0].plot(des_positions_position, label="position")
    axes[1].plot(actual_positions_torque, label="torque")
    axes[1].plot(actual_positions_position, label="position")
    axes[2].plot(error_positions_torque, label="torque")
    axes[2].plot(error_positions_position, label="position")
    plt.legend()

    fig, axes = plt.subplots(3)
    axes[0].plot(des_velocities_torque, label="torque")
    axes[0].plot(des_velocities_position, label="position")
    axes[1].plot(actual_velocities_torque, label="torque")
    axes[1].plot(actual_velocities_position, label="position")
    axes[2].plot(error_velocities_torque, label="torque")
    axes[2].plot(error_velocities_position, label="position")
    plt.legend()
    plt.show()