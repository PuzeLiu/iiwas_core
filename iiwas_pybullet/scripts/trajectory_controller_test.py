import time
import copy
import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

if __name__ == '__main__':
    rospy.init_node("feed_forward_controller_test", anonymous=True)
    pubilsher = rospy.Publisher("iiwa_front/feedforward_trajectory_controller/command", JointTrajectory, queue_size=1)
    time.sleep(1.0)
    msg = JointTrajectory()
    msg.header.stamp = rospy.Time.now()
    msg.joint_names = ['F_joint_1', 'F_joint_2', 'F_joint_3', 'F_joint_4', 'F_joint_5', 'F_joint_6', 'F_joint_7']

    point = JointTrajectoryPoint()
    point.positions = np.zeros(7)
    point.positions[5] = np.pi/2
    point.velocities = np.zeros(7)
    point.accelerations = np.zeros(7)
    time_from_start = 0
    for i in range(10):
        time_from_start += 3
        point.positions[6] = 0
        point.time_from_start = rospy.Duration(time_from_start)
        msg.points.append(copy.deepcopy(point))
        time_from_start += 3
        point.positions[6] = np.pi / 2
        point.time_from_start = rospy.Duration(time_from_start)
        msg.points.append(copy.deepcopy(point))
    pubilsher.publish(msg)