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
#include <joint_trajectory_controller/joint_trajectory_controller.h>

namespace feedforward_controllers {

	template<class SegmentImpl>
	class FeedForwardJointTrajectoryController :
			public joint_trajectory_controller::JointTrajectoryController<SegmentImpl, hardware_interface::EffortJointInterface> {
	protected:
		typedef hardware_interface::EffortJointInterface EffortJointInterface;
		typedef joint_trajectory_controller::JointTrajectoryController<SegmentImpl, EffortJointInterface>
				JointTrajectoryController;

		using JointTrajectoryController::state_publisher_period_;
		using JointTrajectoryController::last_state_publish_time_;
		using JointTrajectoryController::state_publisher_;
		using JointTrajectoryController::time_data_;
		using JointTrajectoryController::old_time_data_;
		using JointTrajectoryController::desired_state_;
		using JointTrajectoryController::current_state_;
		using JointTrajectoryController::state_error_;


	public:
		/** \brief Override the init function of the base class. */
		virtual bool init(EffortJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

		/** \brief Override the update function of the base class. */
		virtual void update(const ros::Time &time, const ros::Duration &period);

	protected:
		pinocchio::Model pinoModel;
		pinocchio::Data pinoData;
		std::vector<double> desired_torque_;
		bool calculatedTorque;

		/**
		 * \brief Publish current controller state at a throttled frequency.
		 * \note This method is realtime-safe and is meant to be called from \ref update, as it shares data with it without
		 * any locking.
		 */
		void publishState(const ros::Time& time);
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
		state_publisher_->msg_.desired.effort.resize(pinoModel.nq);
		return true;
	}

	template<class SegmentImpl>
	void FeedForwardJointTrajectoryController<SegmentImpl>::update(const ros::Time &time, const ros::Duration &period) {
		calculatedTorque = false;
		JointTrajectoryController::update(time, period);

		/** Add Feedforward Term*/
		Eigen::VectorXd pinoJointPosition =
				Eigen::VectorXd::Map(JointTrajectoryController::desired_state_.position.data(),
				                     JointTrajectoryController::desired_state_.position.size());
		Eigen::VectorXd pinoJointAcceleration =
				Eigen::VectorXd::Map(JointTrajectoryController::desired_state_.acceleration.data(),
				                     JointTrajectoryController::desired_state_.acceleration.size());

		pinocchio::crba(pinoModel, pinoData, pinoJointPosition);
		pinoData.M.triangularView<Eigen::StrictlyLower>() = pinoData.M.transpose().triangularView<Eigen::StrictlyLower>();
		Eigen::VectorXd ffTerm = pinoData.M * pinoJointAcceleration;
		for (unsigned int i = 0; i < JointTrajectoryController::joint_names_.size(); ++i) {
			desired_torque_[i] = JointTrajectoryController::joints_[i].getCommand() + ffTerm[i];
			JointTrajectoryController::joints_[i].setCommand(desired_torque_[i]);
		}
		calculatedTorque = true;

		// Update time data
		old_time_data_ = *(time_data_.readFromRT());
		publishState(old_time_data_.uptime + period);
	}

	template<class SegmentImpl>
	void FeedForwardJointTrajectoryController<SegmentImpl>::publishState(const ros::Time &time) {
		if (!state_publisher_period_.isZero() && last_state_publish_time_ + state_publisher_period_ < time
		    && calculatedTorque)
		{
			if (state_publisher_ && state_publisher_->trylock())
			{
				last_state_publish_time_ += state_publisher_period_;

				state_publisher_->msg_.header.stamp          = time_data_.readFromRT()->time;
				state_publisher_->msg_.desired.positions     = desired_state_.position;
				state_publisher_->msg_.desired.velocities    = desired_state_.velocity;
				state_publisher_->msg_.desired.accelerations = desired_state_.acceleration;
				state_publisher_->msg_.desired.effort        = desired_torque_;
				state_publisher_->msg_.actual.positions      = current_state_.position;
				state_publisher_->msg_.actual.velocities     = current_state_.velocity;
				state_publisher_->msg_.error.positions       = state_error_.position;
				state_publisher_->msg_.error.velocities      = state_error_.velocity;

				state_publisher_->unlockAndPublish();
			}
		}
	}
}

#endif //SRC_FF_JOINT_TRAJECTORY_CONTROLLER_H
