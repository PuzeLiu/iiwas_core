/*
 * MIT License
 * Copyright (c) 2020 Puze Liu, Davide Tateo
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "../include/iiwa_gazebo/gravity_compensation_hw_sim.h"

namespace {
	double clamp(const double val, const double min_val, const double max_val) {
		return std::min(std::max(val, min_val), max_val);
	}
} // namespace

namespace iiwas_gazebo {
	bool GravityCompensationHWSim::initSim(const std::string &robot_namespace, ros::NodeHandle model_nh,
	                                       gazebo::physics::ModelPtr parent_model, const urdf::Model *const urdf_model,
	                                       std::vector<transmission_interface::TransmissionInfo> transmissions) {
		if (!DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions))
			return false;
		urdf::Model iiwa_urdf_model;

		std::string urdf_iiwa;
		if(model_nh.getParam("iiwa_only_description", urdf_iiwa)){
			iiwa_urdf_model.initParamWithNodeHandle("iiwa_only_description", model_nh);
		} else {
			iiwa_urdf_model = *urdf_model;
		}
		auto model_ptr = boost::make_shared<urdf::ModelInterface>(iiwa_urdf_model);
		pinocchio::urdf::buildModel(model_ptr, pinoModel, false);
		pinoData = pinocchio::Data(pinoModel);
		pinoJointPosition.resize(pinoModel.nq);
		pinoJointVelocity.resize(pinoModel.nq);
		pinoJointEffort.resize(pinoModel.nq);
		pinoJointPosition.setZero();
		pinoJointVelocity.setZero();
		pinoJointEffort.setZero();

		return true;
	}

	void GravityCompensationHWSim::readSim(ros::Time time, ros::Duration period) {
		for (unsigned int j = 0; j < n_dof_; j++) {

#if GAZEBO_MAJOR_VERSION >= 8
			double position = sim_joints_[j]->Position(0);
#else
			double position = sim_joints_[j]->GetAngle(0).Radian();
#endif

			// Gazebo has an interesting API...
			if (joint_types_[j] == urdf::Joint::PRISMATIC) {
				joint_position_[j] = position;
			} else {
				joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
				                                                        position);
			}
			joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
			joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int) (0));

			double alpha = 1;
			pinoJointPosition[j] = joint_position_[j];
			pinoJointVelocity[j] = joint_velocity_[j] * alpha + pinoJointVelocity[j] * (1- alpha);
			pinoJointEffort[j] = joint_effort_[j];
		}
	}

	void GravityCompensationHWSim::writeSim(ros::Time time, ros::Duration period) {
		pinocchio::nonLinearEffects(pinoModel, pinoData, pinoJointPosition, pinoJointVelocity);
		Eigen::VectorXd compensationTerm = pinoData.nle;

		// If the E-stop is active, joints controlled by position commands will maintain their positions.
		if (e_stop_active_) {
			if (!last_e_stop_active_) {
				last_joint_position_command_ = joint_position_;
				last_e_stop_active_ = true;
			}
			joint_position_command_ = last_joint_position_command_;
		} else {
			last_e_stop_active_ = false;
		}

		ej_sat_interface_.enforceLimits(period);
		ej_limits_interface_.enforceLimits(period);
		pj_sat_interface_.enforceLimits(period);
		pj_limits_interface_.enforceLimits(period);
		vj_sat_interface_.enforceLimits(period);
		vj_limits_interface_.enforceLimits(period);

		for (unsigned int j = 0; j < n_dof_; j++) {
			switch (joint_control_methods_[j]) {
				case EFFORT: {
					const double effort_limit = joint_effort_limits_[j];
					const double effort = e_stop_active_ ? 0 : clamp(joint_effort_command_[j] + compensationTerm[j],
					                                                 -effort_limit,
					                                                 effort_limit);
					sim_joints_[j]->SetForce(0, effort);
				}
					break;

				case POSITION:
#if GAZEBO_MAJOR_VERSION >= 9
					sim_joints_[j]->SetPosition(0, joint_position_command_[j], true);
#else
					ROS_WARN_ONCE("The default_robot_hw_sim plugin is using the Joint::SetPosition method without preserving the link velocity.");
					ROS_WARN_ONCE("As a result, gravity will not be simulated correctly for your model.");
					ROS_WARN_ONCE("Please set gazebo_pid parameters, switch to the VelocityJointInterface or EffortJointInterface, or upgrade to Gazebo 9.");
					ROS_WARN_ONCE("For details, see https://github.com/ros-simulation/gazebo_ros_pkgs/issues/612");
					sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
#endif
					break;

				case POSITION_PID: {
					double error;
					switch (joint_types_[j]) {
						case urdf::Joint::REVOLUTE:
							angles::shortest_angular_distance_with_limits(joint_position_[j],
							                                              joint_position_command_[j],
							                                              joint_lower_limits_[j],
							                                              joint_upper_limits_[j],
							                                              error);
							break;
						case urdf::Joint::CONTINUOUS:
							error = angles::shortest_angular_distance(joint_position_[j],
							                                          joint_position_command_[j]);
							break;
						default:
							error = joint_position_command_[j] - joint_position_[j];
					}

					const double effort_limit = joint_effort_limits_[j];
					const double effort = clamp(pid_controllers_[j].computeCommand(error, period) + compensationTerm[j],
					                            -effort_limit, effort_limit);
					sim_joints_[j]->SetForce(0, effort);
				}
					break;

				case VELOCITY:
					sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
					break;

				case VELOCITY_PID:
					double error;
					if (e_stop_active_)
						error = -joint_velocity_[j];
					else
						error = joint_velocity_command_[j] - joint_velocity_[j];
					const double effort_limit = joint_effort_limits_[j];
					const double effort = clamp(pid_controllers_[j].computeCommand(error, period) + compensationTerm[j],
					                            -effort_limit, effort_limit);
					sim_joints_[j]->SetForce(0, effort);
					break;
			}
		}
	}
}

PLUGINLIB_EXPORT_CLASS(iiwas_gazebo::GravityCompensationHWSim, gazebo_ros_control::RobotHWSim)