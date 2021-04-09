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


#ifndef SRC_FF_JOINT_TRAJECTORY_CONTROLLER_H
#define SRC_FF_JOINT_TRAJECTORY_CONTROLLER_H

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <spline/spline.h>

namespace feedforward_controllers {

	template<class SegmentImpl>
	class FeedForwardJointTrajectoryController :
			public joint_trajectory_controller::JointTrajectoryController<SegmentImpl, hardware_interface::EffortJointInterface> {
	protected:
		typedef hardware_interface::EffortJointInterface EffortJointInterface;
		typedef joint_trajectory_controller::JointTrajectoryController<SegmentImpl, EffortJointInterface>
				JointTrajectoryController;

		using typename JointTrajectoryController::Scalar;
		using typename JointTrajectoryController::Segment;
		using typename JointTrajectoryController::Trajectory;
		using typename JointTrajectoryController::TrajectoryPtr;
		using typename JointTrajectoryController::JointTrajectoryConstPtr;
		using typename JointTrajectoryController::TrajectoryPerJoint;
		using typename JointTrajectoryController::TimeData;
		using typename JointTrajectoryController::RealtimeGoalHandlePtr;


		using JointTrajectoryController::name_;
		using JointTrajectoryController::joint_names_;
		using JointTrajectoryController::verbose_;
		using JointTrajectoryController::rt_active_goal_;
		using JointTrajectoryController::default_tolerances_;
		using JointTrajectoryController::angle_wraparound_;
		using JointTrajectoryController::allow_partial_joints_goal_;

		using JointTrajectoryController::current_state_;         ///< Preallocated workspace variable.
		using JointTrajectoryController::desired_state_;         ///< Preallocated workspace variable.
		using JointTrajectoryController::old_desired_state_;     ///< Preallocated workspace variable.
		using JointTrajectoryController::state_error_;           ///< Preallocated workspace variable.
		using JointTrajectoryController::desired_joint_state_;   ///< Preallocated workspace variable.
		using JointTrajectoryController::state_joint_error_;     ///< Preallocated workspace variable.

		using JointTrajectoryController::state_publisher_period_;
		using JointTrajectoryController::last_state_publish_time_;
		using JointTrajectoryController::state_publisher_;
		using JointTrajectoryController::time_data_;
		using JointTrajectoryController::old_time_data_;
		using JointTrajectoryController::curr_trajectory_box_;
		using JointTrajectoryController::successful_joint_traj_;
		using JointTrajectoryController::hw_iface_adapter_;


	public:
		/** \brief Override the init function of the base class. */
		virtual bool init(EffortJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

		/** \brief Override the update function of the base class. */
		virtual void update(const ros::Time &time, const ros::Duration &period);

	protected:
		pinocchio::Model pinoModel;
		pinocchio::Data pinoData;
		std::vector<double> desired_torque_;
		std::vector<double> pid_torque_;
		std::vector<double> id_torque_;
		bool calculatedTorque;

		/**
		 * \brief Publish current controller state at a throttled frequency.
		 * \note This method is realtime-safe and is meant to be called from \ref update, as it shares data with it without
		 * any locking.
		 */
		void publishState(const ros::Time &time);

		void trajectoryCommandCB(const JointTrajectoryConstPtr &msg);

		trajectory_msgs::JointTrajectory::ConstPtr cubicSplineInterpolate(const JointTrajectoryConstPtr &msg);
	};

	template<class SegmentImpl>
	bool FeedForwardJointTrajectoryController<SegmentImpl>::init(
			FeedForwardJointTrajectoryController::EffortJointInterface *hw, ros::NodeHandle &root_nh,
			ros::NodeHandle &controller_nh) {
		if (!JointTrajectoryController::init(hw, root_nh, controller_nh))
			return false;

		std::string description_xml;
		if (!root_nh.getParam("robot_description", description_xml)) {
			ROS_ERROR_STREAM("Did not find the " << root_nh.getNamespace() << "robot_description");
			return false;
		}

		pinocchio::urdf::buildModelFromXML(description_xml, pinoModel);
		pinoData = pinocchio::Data(pinoModel);
		desired_torque_.resize(pinoModel.nq);
		pid_torque_.resize(pinoModel.nq);
		id_torque_.resize(pinoModel.nq);
		return true;
	}

	template<class SegmentImpl>
	void FeedForwardJointTrajectoryController<SegmentImpl>::update(const ros::Time &time, const ros::Duration &period) {
		calculatedTorque = false;
		JointTrajectoryController::update(time, period);

		/** Add Feedforward Term*/
		Eigen::VectorXd pinoJointPosition(pinoModel.nq);
		Eigen::VectorXd pinoJointVelocity(pinoModel.nq);
		Eigen::VectorXd pinoJointAcceleration(pinoModel.nq);
		for (int j = 0; j < JointTrajectoryController::desired_state_.position.size(); ++j) {
			pinoJointPosition[j] = JointTrajectoryController::current_state_.position[j];
			pinoJointVelocity[j] = JointTrajectoryController::current_state_.velocity[j];
			pinoJointAcceleration[j] = JointTrajectoryController::desired_state_.acceleration[j];
		}
		pinocchio::crba(pinoModel, pinoData, pinoJointPosition);
		pinoData.M.triangularView<Eigen::StrictlyLower>() = pinoData.M.transpose().triangularView<Eigen::StrictlyLower>();
		Eigen::VectorXd ffTerm = pinoData.M * pinoJointAcceleration;

		pinocchio::rnea(pinoModel, pinoData, pinoJointPosition, pinoJointVelocity, pinoJointAcceleration);
		Eigen::VectorXd idTorque = pinoData.tau + pinoModel.friction.cwiseProduct(pinoJointVelocity.cwiseSign())
		                           + pinoModel.damping.cwiseProduct(pinoJointVelocity);

		for (unsigned int i = 0; i < JointTrajectoryController::joint_names_.size(); ++i) {
			desired_torque_[i] = ffTerm[i];
			id_torque_[i] = idTorque[i];
			pid_torque_[i] = JointTrajectoryController::joints_[i].getCommand();
			JointTrajectoryController::joints_[i].setCommand(
					JointTrajectoryController::joints_[i].getCommand() + ffTerm[i]);
		}

		calculatedTorque = true;

		// Update time data
		publishState(old_time_data_.uptime + period);
	}

	template<class SegmentImpl>
	void FeedForwardJointTrajectoryController<SegmentImpl>::publishState(const ros::Time &time) {
		if (!state_publisher_period_.isZero() && last_state_publish_time_ + state_publisher_period_ < time
		    && calculatedTorque) {
			if (state_publisher_ && state_publisher_->trylock()) {
				last_state_publish_time_ += state_publisher_period_;

				state_publisher_->msg_.header.stamp = time;
				state_publisher_->msg_.desired.positions = desired_state_.position;
				state_publisher_->msg_.desired.velocities = desired_state_.velocity;
				state_publisher_->msg_.desired.accelerations = desired_state_.acceleration;
				state_publisher_->msg_.desired.effort = desired_torque_;
				state_publisher_->msg_.actual.positions = current_state_.position;
				state_publisher_->msg_.actual.velocities = current_state_.velocity;
				state_publisher_->msg_.actual.effort = id_torque_;
				state_publisher_->msg_.error.positions = state_error_.position;
				state_publisher_->msg_.error.velocities = state_error_.velocity;
				state_publisher_->msg_.error.effort = pid_torque_;

				state_publisher_->unlockAndPublish();
			}
		}
	}

	template<class SegmentImpl>
	void FeedForwardJointTrajectoryController<SegmentImpl>::trajectoryCommandCB(const JointTrajectoryConstPtr &msg) {
		trajectory_msgs::JointTrajectory::ConstPtr cubicSplineTrajectory = cubicSplineInterpolate(msg);
		const bool update_ok = JointTrajectoryController::updateTrajectoryCommand(cubicSplineTrajectory, RealtimeGoalHandlePtr());
		if (update_ok) { JointTrajectoryController::preemptActiveGoal(); }
	}

	template<class SegmentImpl>
	trajectory_msgs::JointTrajectory::ConstPtr
	FeedForwardJointTrajectoryController<SegmentImpl>::cubicSplineInterpolate(const JointTrajectoryConstPtr &msg) {
		if (msg->points.empty() || msg->points.size() < 2){
			return msg;
		}
		if (msg->points[0].velocities.empty() || !msg->points[0].accelerations.empty()){
			ROS_DEBUG_STREAM("Desired trajectory is linear or quintic, skip the spline interpolation");
			return msg;
		}

		trajectory_msgs::JointTrajectory cubicSpline = *msg;
		int n = msg->points.size();
		std::vector<Scalar> x(n);
		std::vector<Scalar> y(n);

		for (int i = 0; i < msg->points[0].positions.size(); ++i) {
			for (int j = 0; j < n; ++j) {
				x[j] = msg->points[j].time_from_start.toSec();
				y[j] = msg->points[j].positions[i];
			}

			tk::spline spline(x, y, tk::spline::cspline, false,
			                  tk::spline::first_deriv, 0.0,
			                  tk::spline::first_deriv, 0.0);

			for (int j = 0; j < n; ++j) {
				cubicSpline.points[j].velocities[i] = spline.deriv(1, x[j]);
			}
		}
		return boost::make_shared<trajectory_msgs::JointTrajectory const>(cubicSpline);
	}
}

#endif //SRC_FF_JOINT_TRAJECTORY_CONTROLLER_H
