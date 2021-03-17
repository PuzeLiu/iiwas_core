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

	public:
		/** \brief Override the init function of the base class. */
		virtual bool init(EffortJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

		/** \brief Override the update function of the base class. */
		virtual void update(const ros::Time &time, const ros::Duration &period);

	protected:
		pinocchio::Model pinoModel;
		pinocchio::Data pinoData;
	};

	template<class SegmentImpl>
	bool FeedForwardJointTrajectoryController<SegmentImpl>::init(
			FeedForwardJointTrajectoryController::EffortJointInterface *hw, ros::NodeHandle &root_nh,
			ros::NodeHandle &controller_nh) {
		if (!JointTrajectoryController::init(hw, root_nh, controller_nh))
			return false;

		ROS_INFO_STREAM("###########" << root_nh.getNamespace());
		ROS_INFO_STREAM("###########" << controller_nh.getNamespace());

		std::string description_xml;
		if (!root_nh.getParam("robot_description", description_xml)) {
			ROS_ERROR_STREAM("Did not find the " << root_nh.getNamespace() << "robot_description");
			return false;
		}

		pinocchio::urdf::buildModelFromXML(description_xml, pinoModel);
		pinoData = pinocchio::Data(pinoModel);
		return true;
	}

	template<class SegmentImpl>
	void FeedForwardJointTrajectoryController<SegmentImpl>::update(const ros::Time &time, const ros::Duration &period) {
		JointTrajectoryController::update(time, period);

		/** Add Feedforward Term*/
		double command_i = 0.;
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
			command_i = JointTrajectoryController::joints_[i].getCommand() + ffTerm[i];
			JointTrajectoryController::joints_[i].setCommand(command_i);
		}
	}
}

#endif //SRC_FF_JOINT_TRAJECTORY_CONTROLLER_H
