#!/usr/bin/env python3

import copy
import numpy as np
import pinocchio as pino
import rospy
from control_msgs.msg import JointTrajectoryControllerState
from iiwas_pybullet.srv import SetGains, SetGainsResponse
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class FeedForwardTrajectoryController:
    def __init__(self, pybullet, namespace, model_spec, **kwargs):
        # get "import pybullet as pb" and store in self.pb
        self.pb = pybullet
        self.namespace = namespace
        self.model_id = model_spec['model_id']
        self.period = rospy.Duration(1 / kwargs['publish_rate'])
        # subscribe controller command
        rospy.Subscriber(namespace + '/joint_feedforward_trajectory_controller/command',
                         JointTrajectory, self.ff_controller_cb, queue_size=1)

        # publish controller message
        self.state_publisher = rospy.Publisher(namespace + "/joint_feedforward_trajectory_controller/state",
                                               JointTrajectoryControllerState, queue_size=5)
        self.state_msg = JointTrajectoryControllerState()

        # Service to set gains
        rospy.Service(namespace + '/joint_feedforward_trajectory_controller/set_gains', SetGains, self.set_gains_cb)

        self.command_buffer = list()

        # Load urdf and pinocchio model
        self.pino_model = pino.buildModelFromUrdf(model_spec['urdf_file'])
        self.pino_data = self.pino_model.createData()
        self.pino_positions = np.zeros(self.pino_model.nq)  # Value used to calculate gravity
        self.pino_indices = list()

        self.n_joints = len(kwargs['gains'])
        self.joint_indices = np.zeros(self.n_joints)
        self.position_gains = np.zeros(self.n_joints)
        self.velocity_gains = np.zeros(self.n_joints)
        self.current_positions = np.zeros(self.n_joints)
        self.current_velocities = np.zeros(self.n_joints)
        self.current_effort = np.zeros(self.n_joints)
        self.desired_positions = np.zeros(self.n_joints)
        self.desired_velocities = np.zeros(self.n_joints)
        self.desired_accelerations = np.zeros(self.n_joints)
        self.error_positions = np.zeros(self.n_joints)
        self.error_velocities = np.zeros(self.n_joints)
        self.feedforward_command = np.zeros(self.n_joints)
        self.closeloop_command = np.zeros(self.n_joints)
        self.command = np.zeros(self.n_joints)
        for i, joint_name in enumerate(kwargs['gains']):
            self.state_msg.joint_names.append(joint_name)
            self.joint_indices[i] = model_spec['joint_map'][joint_name][1]
            self.position_gains[i] = kwargs['gains'][joint_name]['p']
            self.velocity_gains[i] = kwargs['gains'][joint_name]['d']
            self.pb.setJointMotorControl2(*model_spec['joint_map'][joint_name],
                                          controlMode=self.pb.VELOCITY_CONTROL, force=0.)
            self.pino_indices.append(self.get_pino_index(joint_name))

        self.update_current_state()
        self.sim_time = rospy.Time.from_sec(0.)

        self.segment_start_point = JointTrajectoryPoint()
        self.segment_start_point.positions = self.current_positions
        self.segment_start_point.velocities = np.zeros(self.n_joints)
        self.segment_start_point.accelerations = np.zeros(self.n_joints)
        self.segment_start_point.time_from_start = self.sim_time
        self.segment_hold_point = copy.deepcopy(self.segment_start_point)

        self.segment_coefficient = np.zeros((self.n_joints, 6))
        self.segment_duration = 0.
        self.update_segment_coefficient(self.segment_start_point, self.segment_hold_point)
        self.next_update_time = self.sim_time + self.period
        self.next_publish_time = self.sim_time + self.period

    def execute(self, sim_time):
        self.sim_time = sim_time
        self.update_current_state()
        self.update_desired_state()
        self.apply_command()
        self.publish_state()
        self.next_update_time = self.sim_time + self.period

    def update_current_state(self):
        iiwas_states = np.array(self.pb.getJointStates(self.model_id, self.joint_indices), dtype=object)
        self.current_positions = iiwas_states[:, 0].astype(float)
        self.current_velocities = iiwas_states[:, 1].astype(float)
        self.current_effort = iiwas_states[:, 3].astype(float)
        self.pino_positions[self.pino_indices] = self.current_positions

    def update_desired_state(self):
        if len(self.command_buffer) < 1:
            self.sample(self.sim_time)
        else:
            self.update_segment(self.sim_time)
            self.sample(self.sim_time)

    def apply_command(self):
        self.error_positions = self.desired_positions - self.current_positions
        self.error_velocities = self.desired_velocities - self.current_velocities
        self.feedforward_command = pino.computeGeneralizedGravity(self.pino_model, self.pino_data,
                                                                  self.pino_positions)[self.pino_indices]

        self.closeloop_command = self.position_gains * self.error_positions + \
                                 self.velocity_gains * self.error_velocities
        self.command = np.clip(self.closeloop_command + self.feedforward_command,
                               -self.pino_model.effortLimit[self.pino_indices],
                               self.pino_model.effortLimit[self.pino_indices])
        self.pb.setJointMotorControlArray(self.model_id, self.joint_indices, controlMode=self.pb.TORQUE_CONTROL,
                                          forces=self.command)

    def publish_state(self):
        if (self.sim_time - self.next_publish_time).to_sec() >= -1e-8:
            self.state_msg.header.stamp = copy.deepcopy(self.sim_time)
            self.state_msg.desired.positions = self.desired_positions.tolist()
            self.state_msg.desired.velocities = self.desired_velocities.tolist()
            self.state_msg.desired.accelerations = self.desired_accelerations.tolist()
            self.state_msg.desired.effort = self.feedforward_command.tolist()
            self.state_msg.actual.positions = self.current_positions.tolist()
            self.state_msg.actual.velocities = self.current_velocities.tolist()
            self.state_msg.actual.effort = self.current_effort.tolist()
            self.state_msg.error.positions = self.error_positions.tolist()
            self.state_msg.error.velocities = self.error_velocities.tolist()
            self.state_msg.error.effort = self.closeloop_command.tolist()
            self.state_publisher.publish(self.state_msg)
            self.next_publish_time = self.sim_time + self.period

    def sample(self, time):
        t = (time - self.segment_start_point.time_from_start).to_sec()
        t = np.clip(t, 0., self.segment_duration)
        t_power = np.array([t ** i for i in range(6)])
        self.desired_positions = self.segment_coefficient @ t_power
        self.desired_velocities = self.segment_coefficient[:, 1:] @ np.diag([1., 2., 3., 4., 5.]) @ t_power[:-1]
        self.desired_accelerations = self.segment_coefficient[:, 2:] @ np.diag([2., 6., 12., 20.]) @ t_power[:-2]

    def update_segment(self, time):
        if time >= self.command_buffer[0].time_from_start:
            self.segment_start_point = self.command_buffer.pop(0)
            # check if there remaining trajectory point
            if len(self.command_buffer) < 1:
                # Set same point will hold there
                self.update_segment_coefficient(self.segment_start_point, self.segment_start_point)
            else:
                self.update_segment_coefficient(self.segment_start_point, self.command_buffer[0])
                self.sample(time)

    def update_segment_coefficient(self, start_point, stop_point):
        self.segment_duration = (stop_point.time_from_start - start_point.time_from_start).to_sec()
        self.segment_coefficient[:] = 0

        has_velocity = (len(start_point.velocities) > 0) and \
                       (len(stop_point.velocities) > 0)
        has_acceleration = (len(start_point.accelerations) > 0) and \
                           (len(stop_point.accelerations) > 0)

        # Linear Interpolation
        if not has_velocity:
            self.segment_coefficient[:, 0] = start_point.positions
            # duration != 0
            if self.segment_duration > 0.:
                self.segment_coefficient[:, 1] = (stop_point.positions - start_point.positions) / self.segment_duration
        # Cubic Interpolation
        elif not has_acceleration:
            self.segment_coefficient[:, 0] = start_point.positions
            self.segment_coefficient[:, 1] = start_point.velocities
            # duration != 0
            if self.segment_duration > 0.:
                self.segment_coefficient[:, 2] = (-3.0 * (start_point.positions - stop_point.positions) -
                                                  (2.0 * start_point.velocities +
                                                   stop_point.velocities) * self.segment_duration) / \
                                                 (self.segment_duration ** 2)
                self.segment_coefficient[:, 3] = (2.0 * (start_point.positions - stop_point.positions) +
                                                  (start_point.velocities + stop_point.velocities) *
                                                  self.segment_duration) / (self.segment_duration ** 3)
        # Quintic Interpolation
        else:
            self.segment_coefficient[:, 0] = start_point.positions
            self.segment_coefficient[:, 1] = start_point.velocities
            self.segment_coefficient[:, 2] = 0.5 * start_point.accelerations
            if self.segment_duration > 0.:
                self.segment_coefficient[:, 3] = (-20.0 * (start_point.positions - stop_point.positions) -
                                                  (3.0 * start_point.accelerations -
                                                   stop_point.accelerations) * (self.segment_duration ** 2) -
                                                  (12.0 * start_point.velocities +
                                                   8.0 * stop_point.velocities) * self.segment_duration) / \
                                                 (2 * (self.segment_duration ** 3))
                self.segment_coefficient[:, 4] = (30.0 * (start_point.positions - stop_point.positions) +
                                                  (3.0 * start_point.accelerations -
                                                   2.0 * stop_point.accelerations) * (self.segment_duration ** 2) +
                                                  (16.0 * start_point.velocities +
                                                   14.0 * stop_point.velocities) * self.segment_duration) / \
                                                 (2 * self.segment_duration ** 4)
                self.segment_coefficient[:, 5] = (-12.0 * (start_point.positions - stop_point.positions) -
                                                  (start_point.accelerations -
                                                   stop_point.accelerations) * (self.segment_duration ** 2) -
                                                  6.0 * (start_point.velocities +
                                                         stop_point.velocities) * self.segment_duration) / \
                                                 (2 * self.segment_duration ** 5)

    def ff_controller_cb(self, msg):
        if not self.convert_trajectory_msg(msg):
            self.sample(self.next_update_time)
            self.command_buffer.clear()
            point_tmp = copy.deepcopy(self.segment_start_point)
            point_tmp.positions = self.desired_positions
            point_tmp.velocities = self.desired_velocities
            point_tmp.accelerations = self.desired_accelerations
            self.update_segment_coefficient(point_tmp, point_tmp)

    def set_gains_cb(self, req):
        if len(self.position_gains) == len(req.p_gain.data) and len(self.velocity_gains) == len(req.d_gain.data):
            self.position_gains = np.array(req.p_gain.data)
            self.velocity_gains = np.array(req.d_gain.data)
            return SetGainsResponse(success=True)
        else:
            rospy.logwarn("Service SetGains is not success as the dimension doesn't match")
            return SetGainsResponse(success=False)

    def convert_trajectory_msg(self, msg):
        self.command_buffer.clear()

        if len(msg.points) == 0:
            rospy.logdebug("Trajectory message contains empty trajectory. Nothing to convert.")
            return False

        if not self.is_time_strictly_increasing(msg):
            rospy.logerr("Trajectory message contains waypoints that are not strictly increasing in time.")
            return False

        if msg.joint_names != self.state_msg.joint_names:
            rospy.logerr("Cannot create trajectory from message. Received message names does not match sending names.")
            return False

        msg_start_time = self.next_update_time if msg.header.stamp.is_zero() else msg.header.stamp

        first_segment_id = 0
        # check if the next update time lies between trajectory point
        if self.next_update_time > msg_start_time + msg.points[-1].time_from_start:
            rospy.logwarn("Drop all {} trajectory points, as they occur before the current time. "
                          "Last point is {}s in the past"
                          .format(len(msg.points), (self.next_update_time -
                                                    (msg_start_time + msg.points[-1].time_from_start)).to_sec()))
            return False
        elif self.next_update_time > msg_start_time + msg.points[0].time_from_start:
            for first_segment_id, point in enumerate(msg.points):
                if self.next_update_time < msg_start_time + point.time_from_start:
                    break
                else:
                    msg.points.pop(0)
            rospy.logwarn("Drop first {} trajectory point(s), as they occur before the current time. "
                          "First valid point will be reached in {}s"
                          .format(first_segment_id, ((msg_start_time + msg.points[first_segment_id].time_from_start) -
                                                     self.next_update_time).to_sec()))
        else:
            # the next update point is before the trajectory start
            pass

        self.sample(self.next_update_time)
        point_tmp = copy.deepcopy(msg.points[0])
        point_tmp.positions = self.desired_positions.copy()
        point_tmp.velocities = self.desired_velocities.copy()
        point_tmp.accelerations = self.desired_accelerations.copy()
        self.command_buffer.append(self.wrap_point_and_time(point_tmp, self.next_update_time))
        for point in msg.points:
            self.command_buffer.append(self.wrap_point_and_time(point, msg_start_time + point.time_from_start))
        return True

    def wrap_point_and_time(self, point_tmp, msg_start_time):
        point_tmp.positions = np.array(point_tmp.positions)
        point_tmp.velocities = np.array(point_tmp.velocities)
        point_tmp.accelerations = np.array(point_tmp.accelerations)
        point_tmp.time_from_start = msg_start_time
        return point_tmp

    def is_time_strictly_increasing(self, msg):
        if len(msg.points) < 2:
            return True
        for i, point in enumerate(msg.points[:-1]):
            if point.time_from_start > msg.points[i + 1].time_from_start:
                return False
        return True

    def get_pino_index(self, joint_name):
        for i, pino_name in enumerate(self.pino_model.names):
            if pino_name == joint_name:
                return self.pino_model.joints[i].idx_q
        raise ValueError("Did not find {} in pinocchio model".format(joint_name))
