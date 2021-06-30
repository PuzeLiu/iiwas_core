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
#include <boost/algorithm/clamp.hpp>
#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include "spline/spline.h"


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

		/**
		 * \brief Publish current controller state at a throttled frequency.
		 * \note This method is realtime-safe and is meant to be called from \ref update, as it shares data with it without
		 * any locking.
		 */
		virtual void publishState(const ros::Time &time);

		void setActionFeedback();

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
		if (!root_nh.getParam("iiwa_only_description", description_xml)) {
			if (!root_nh.getParam("robot_description", description_xml)) {
				ROS_ERROR_STREAM("Did not find the " << root_nh.getNamespace() << "robot_description");
				return false;
			}
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
		// Get currently followed trajectory
		TrajectoryPtr curr_traj_ptr;
		curr_trajectory_box_.get(curr_traj_ptr);
		Trajectory& curr_traj = *curr_traj_ptr;

		old_time_data_ = *(time_data_.readFromRT());

		// Update time data
		TimeData time_data;
		time_data.time   = time;                                     // Cache current time
		time_data.period = period;                                   // Cache current control period
		time_data.uptime = old_time_data_.uptime + period; // Update controller uptime
		time_data_.writeFromNonRT(time_data); // TODO: Grrr, we need a lock-free data structure here!

		// NOTE: It is very important to execute the two above code blocks in the specified sequence: first get current
		// trajectory, then update time data. Hopefully the following paragraph sheds a bit of light on the rationale.
		// The non-rt thread responsible for processing new commands enqueues trajectories that can start at the _next_
		// control cycle (eg. zero start time) or later (eg. when we explicitly request a start time in the future).
		// If we reverse the order of the two blocks above, and update the time data first; it's possible that by the time we
		// fetch the currently followed trajectory, it has been updated by the non-rt thread with something that starts in the
		// next control cycle, leaving the current cycle without a valid trajectory.

		JointTrajectoryController::updateStates(time_data.uptime, curr_traj_ptr.get());

		// Update current state and state error
		for (unsigned int i = 0; i < JointTrajectoryController::getNumberOfJoints(); ++i)
		{
			typename TrajectoryPerJoint::const_iterator segment_it = sample(curr_traj[i], time_data.uptime.toSec(), desired_joint_state_);
			if (curr_traj[i].end() == segment_it)
			{
				// Non-realtime safe, but should never happen under normal operation
				ROS_ERROR_NAMED(name_,
				                "Unexpected error: No trajectory defined at current time. Please contact the package maintainer.");
				return;
			}

			// Get state error for current joint
			state_joint_error_.position[0] = state_error_.position[i];
			state_joint_error_.velocity[0] = state_error_.velocity[i];
			state_joint_error_.acceleration[0] = state_error_.acceleration[i];

			//Check tolerances
			const RealtimeGoalHandlePtr rt_segment_goal = segment_it->getGoalHandle();
			if (rt_segment_goal && rt_segment_goal == rt_active_goal_)
			{
				// Check tolerances
				if (time_data.uptime.toSec() < segment_it->endTime())
				{
					// Currently executing a segment: check path tolerances
					const joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar>& joint_tolerances = segment_it->getTolerances();
					if (!checkStateTolerancePerJoint(state_joint_error_, joint_tolerances.state_tolerance))
					{
						if (verbose_)
						{
							ROS_ERROR_STREAM_NAMED(name_,"Path tolerances failed for joint: " << joint_names_[i]);
							checkStateTolerancePerJoint(state_joint_error_, joint_tolerances.state_tolerance, true);
						}
						rt_segment_goal->preallocated_result_->error_code =
								control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
						rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
						rt_active_goal_.reset();
						successful_joint_traj_.reset();
					}
				}
				else if (segment_it == --curr_traj[i].end())
				{
					if (verbose_)
						ROS_DEBUG_STREAM_THROTTLE_NAMED(1,name_,"Finished executing last segment, checking goal tolerances");

					// Controller uptime
					const ros::Time uptime = time_data_.readFromRT()->uptime;

					// Checks that we have ended inside the goal tolerances
					const joint_trajectory_controller::SegmentTolerancesPerJoint<Scalar>& tolerances = segment_it->getTolerances();
					const bool inside_goal_tolerances = checkStateTolerancePerJoint(state_joint_error_, tolerances.goal_state_tolerance);

					if (inside_goal_tolerances)
					{
						successful_joint_traj_[i] = 1;
					}
					else if (uptime.toSec() < segment_it->endTime() + tolerances.goal_time_tolerance)
					{
						// Still have some time left to meet the goal state tolerances
					}
					else
					{
						if (verbose_)
						{
							ROS_ERROR_STREAM_NAMED(name_,"Goal tolerances failed for joint: "<< joint_names_[i]);
							// Check the tolerances one more time to output the errors that occurs
							checkStateTolerancePerJoint(state_joint_error_, tolerances.goal_state_tolerance, true);
						}

						rt_segment_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED;
						rt_segment_goal->setAborted(rt_segment_goal->preallocated_result_);
						rt_active_goal_.reset();
						successful_joint_traj_.reset();
					}
				}
			}
		}

		//If there is an active goal and all segments finished successfully then set goal as succeeded
		RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
		if (current_active_goal && successful_joint_traj_.count() == JointTrajectoryController::getNumberOfJoints())
		{
			current_active_goal->preallocated_result_->error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
			current_active_goal->setSucceeded(current_active_goal->preallocated_result_);
			current_active_goal.reset(); // do not publish feedback
			rt_active_goal_.reset();
			successful_joint_traj_.reset();
		}

		// Hardware interface adapter: Generate and send commands
		hw_iface_adapter_.updateCommand(time_data.uptime, time_data.period,
		                                desired_state_, state_error_);

		setActionFeedback();

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

		double command_i = 0;
		for (unsigned int i = 0; i < JointTrajectoryController::joint_names_.size(); ++i) {
			desired_torque_[i] = ffTerm[i];
			id_torque_[i] = idTorque[i];
			pid_torque_[i] = JointTrajectoryController::joints_[i].getCommand();
			command_i = pid_torque_[i] + ffTerm[i];
			JointTrajectoryController::joints_[i].setCommand(command_i);
		}

		// Update time data
		publishState(time_data.uptime);
	}

	template<class SegmentImpl>
	void FeedForwardJointTrajectoryController<SegmentImpl>::publishState(const ros::Time &time) {
		if (!state_publisher_period_.isZero() && last_state_publish_time_ + state_publisher_period_ < time) {
			if (state_publisher_ && state_publisher_->trylock()) {
				last_state_publish_time_ += state_publisher_period_;

				state_publisher_->msg_.header.stamp = time_data_.readFromRT()->time;
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

	template <class SegmentImpl>
	void FeedForwardJointTrajectoryController<SegmentImpl>::setActionFeedback()
	{
		RealtimeGoalHandlePtr current_active_goal(rt_active_goal_);
		if (!current_active_goal)
		{
			return;
		}

		current_active_goal->preallocated_feedback_->header.stamp          = time_data_.readFromRT()->time;
		current_active_goal->preallocated_feedback_->desired.positions     = desired_state_.position;
		current_active_goal->preallocated_feedback_->desired.velocities    = desired_state_.velocity;
		current_active_goal->preallocated_feedback_->desired.accelerations = desired_state_.acceleration;
		current_active_goal->preallocated_feedback_->actual.positions      = current_state_.position;
		current_active_goal->preallocated_feedback_->actual.velocities     = current_state_.velocity;
		current_active_goal->preallocated_feedback_->error.positions       = state_error_.position;
		current_active_goal->preallocated_feedback_->error.velocities      = state_error_.velocity;
		current_active_goal->setFeedback( current_active_goal->preallocated_feedback_ );

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
