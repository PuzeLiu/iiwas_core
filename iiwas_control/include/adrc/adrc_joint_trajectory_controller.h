/*
 * MIT License
 * Copyright (c) 2021 Puze Liu, Davide Tateo
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


#ifndef SRC_ADRC_JOINT_TRAJECTORY_CONTROLLER_H
#define SRC_ADRC_JOINT_TRAJECTORY_CONTROLLER_H

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <boost/algorithm/clamp.hpp>

#include <joint_trajectory_controller/joint_trajectory_controller.h>
#include <joint_trajectory_controller/hold_trajectory_builder.h>
#include <joint_trajectory_controller/stop_trajectory_builder.h>


#include "spline/spline.h"
#include "adrc_single.h"


namespace adrc_controllers {

	template<class SegmentImpl>
	class ADRCJointTrajectoryController : public joint_trajectory_controller::
			JointTrajectoryController<SegmentImpl, hardware_interface::EffortJointInterface> {
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
		using typename JointTrajectoryController::StatePublisher;
		using typename JointTrajectoryController::ActionServer;


		using JointTrajectoryController::name_;
		using JointTrajectoryController::joints_;
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
		using JointTrajectoryController::action_monitor_period_;
		using JointTrajectoryController::stop_trajectory_duration_;
		using JointTrajectoryController::last_state_publish_time_;
		using JointTrajectoryController::state_publisher_;
		using JointTrajectoryController::time_data_;
		using JointTrajectoryController::old_time_data_;
		using JointTrajectoryController::curr_trajectory_box_;
		using JointTrajectoryController::hold_traj_builder_;
		using JointTrajectoryController::successful_joint_traj_;

		using JointTrajectoryController::controller_nh_;
		using JointTrajectoryController::trajectory_command_sub_;
		using JointTrajectoryController::action_server_;
		using JointTrajectoryController::hold_trajectory_ptr_;
		using JointTrajectoryController::query_state_service_;

		using JointTrajectoryController::trajectoryCommandCB;
//		using JointTrajectoryController::queryStateService;


	public:
		/** \brief Override the init function of the base class. */
		virtual bool init(EffortJointInterface *hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh);

		/** \brief Override the starting function of the base class. */
		virtual void starting(const ros::Time& time);

		/** \brief Override the update function of the base class. */
		virtual void update(const ros::Time &time, const ros::Duration &period);

	protected:
		void setCommand();

		void applyInertiaFeedForward();
		void applyFrictionCompensation();

		void applySingleJointCmd(int joint_index);

	protected:
		pinocchio::Model pinoModel;
		pinocchio::Data pinoData;
		typedef std::shared_ptr<ADRCJoint> ADRCJointPtr;
		std::vector<ADRCJointPtr> adrcs_;
		std::vector<double> u_ff_, u_adrc_, u_, u_max_, u_stiction_, vs_;
		std::vector<double> estimation_error_, disturbance_, velocity_error_;

		std::vector<double> Kp_safe, Kd_safe, qStop;
		Eigen::VectorXd diagOffset_;
		bool isSafe;

		/**
		 * \brief Publish current controller state at a throttled frequency.
		 * \note This method is realtime-safe and is meant to be called from \ref update, as it shares data with it without
		 * any locking.
		 */
		virtual void publishState(const ros::Time &time);

		void setActionFeedback();

		void trajectoryCommandCB(const JointTrajectoryConstPtr &msg);

		virtual bool queryStateService(control_msgs::QueryTrajectoryState::Request&  req,
		                               control_msgs::QueryTrajectoryState::Response& resp);

		trajectory_msgs::JointTrajectory::ConstPtr cubicSplineInterpolate(const JointTrajectoryConstPtr &msg);

	};

	template<class SegmentImpl>
	bool ADRCJointTrajectoryController<SegmentImpl>::init(
			ADRCJointTrajectoryController::EffortJointInterface *hw, ros::NodeHandle &root_nh,
			ros::NodeHandle &controller_nh) {

		using namespace joint_trajectory_controller::internal;

		// Cache controller node handle
		controller_nh_ = controller_nh;

		// Controller name
		name_ = getLeafNamespace(controller_nh_);

		// State publish rate
		double state_publish_rate = 50.0;
		controller_nh_.getParam("state_publish_rate", state_publish_rate);
		ROS_DEBUG_STREAM_NAMED(name_, "Controller state will be published at " << state_publish_rate << "Hz.");
		state_publisher_period_ = ros::Duration(1.0 / state_publish_rate);

		// Action status checking update rate
		double action_monitor_rate = 20.0;
		controller_nh_.getParam("action_monitor_rate", action_monitor_rate);
		action_monitor_period_ = ros::Duration(1.0 / action_monitor_rate);
		ROS_DEBUG_STREAM_NAMED(name_, "Action status changes will be monitored at " << action_monitor_rate << "Hz.");

		// Stop trajectory duration
		stop_trajectory_duration_ = 0.0;
		controller_nh_.getParam("stop_trajectory_duration", stop_trajectory_duration_);
		ROS_DEBUG_STREAM_NAMED(name_, "Stop trajectory has a duration of " << stop_trajectory_duration_ << "s.");

		// Checking if partial trajectories are allowed
		controller_nh_.template param<bool>("allow_partial_joints_goal", allow_partial_joints_goal_, false);
		if (allow_partial_joints_goal_)
		{
			ROS_DEBUG_NAMED(name_, "Goals with partial set of joints are allowed");
		}

		// List of controlled joints
		joint_names_ = getStrings(controller_nh_, "joints");
		if (joint_names_.empty()) {return false;}
		const unsigned int n_joints = joint_names_.size();

		// URDF joints
		urdf::ModelSharedPtr urdf = getUrdf(root_nh, "robot_description");
		if (!urdf) {return false;}

		std::vector<urdf::JointConstSharedPtr> urdf_joints = getUrdfJoints(*urdf, joint_names_);
		if (urdf_joints.empty()) {return false;}
		assert(n_joints == urdf_joints.size());

		// Initialize members
		joints_.resize(n_joints);
		angle_wraparound_.resize(n_joints);
		for (unsigned int i = 0; i < n_joints; ++i)
		{
			// Joint handle
			try {joints_[i] = hw->getHandle(joint_names_[i]);}
			catch (...)
			{
				ROS_ERROR_STREAM_NAMED(name_, "Could not find joint '" << joint_names_[i] << "' in '" <<
				                                                       this->getHardwareInterfaceType() << "'.");
				return false;
			}

			// Whether a joint is continuous (ie. has angle wraparound)
			angle_wraparound_[i] = urdf_joints[i]->type == urdf::Joint::CONTINUOUS;
			const std::string not_if = angle_wraparound_[i] ? "" : "non-";

			ROS_DEBUG_STREAM_NAMED(name_, "Found " << not_if << "continuous joint '" << joint_names_[i] << "' in '" <<
			                                       this->getHardwareInterfaceType() << "'.");
		}

		assert(joints_.size() == angle_wraparound_.size());
		ROS_DEBUG_STREAM_NAMED(name_, "Initialized controller '" << name_ << "' with:" <<
		                                                         "\n- Number of joints: " << JointTrajectoryController::getNumberOfJoints() <<
		                                                         "\n- Hardware interface type: '" << this->getHardwareInterfaceType() << "'" <<
		                                                         "\n- Trajectory segment type: '" << hardware_interface::internal::demangledTypeName<SegmentImpl>() << "'");

		// Default tolerances
		ros::NodeHandle tol_nh(controller_nh_, "constraints");
		default_tolerances_ = joint_trajectory_controller::getSegmentTolerances<Scalar>(tol_nh, joint_names_);


		// ROS API: Subscribed topics
		trajectory_command_sub_ = controller_nh_.subscribe("command", 1,
														   &ADRCJointTrajectoryController::trajectoryCommandCB,
														   this);

		// ROS API: Published topics
		state_publisher_.reset(new StatePublisher(controller_nh_, "state", 1));

		// ROS API: Action interface
		action_server_.reset(new ActionServer(controller_nh_, "follow_joint_trajectory",
		                                      boost::bind(&ADRCJointTrajectoryController::goalCB,   this, _1),
		                                      boost::bind(&ADRCJointTrajectoryController::cancelCB, this, _1),
		                                      false));
		action_server_->start();

		// ROS API: Provided services
		query_state_service_ = controller_nh_.advertiseService("query_state",
		                                                       &ADRCJointTrajectoryController::queryStateService,
		                                                       this);

		// Preeallocate resources
		current_state_       = typename Segment::State(n_joints);
		old_desired_state_   = typename Segment::State(n_joints);
		desired_state_       = typename Segment::State(n_joints);
		state_error_         = typename Segment::State(n_joints);
		desired_joint_state_ = typename Segment::State(1);
		state_joint_error_   = typename Segment::State(1);

		successful_joint_traj_ = boost::dynamic_bitset<>(JointTrajectoryController::getNumberOfJoints());

		hold_trajectory_ptr_ = JointTrajectoryController::createHoldTrajectory(n_joints);
		assert(joint_names_.size() == hold_trajectory_ptr_->size());

		if (stop_trajectory_duration_ == 0.0)
		{
			hold_traj_builder_ = std::unique_ptr<joint_trajectory_controller::TrajectoryBuilder<SegmentImpl> >
			        (new joint_trajectory_controller::HoldTrajectoryBuilder<SegmentImpl, EffortJointInterface>(joints_));
		}
		else
		{
			hold_traj_builder_ = std::unique_ptr<joint_trajectory_controller::TrajectoryBuilder<SegmentImpl> >
			        (new joint_trajectory_controller::StopTrajectoryBuilder<SegmentImpl>(stop_trajectory_duration_, desired_state_));
		}

		{
			state_publisher_->lock();
			state_publisher_->msg_.joint_names = joint_names_;
			state_publisher_->msg_.desired.positions.resize(n_joints);
			state_publisher_->msg_.desired.velocities.resize(n_joints);
			state_publisher_->msg_.desired.accelerations.resize(n_joints);
			state_publisher_->msg_.actual.positions.resize(n_joints);
			state_publisher_->msg_.actual.velocities.resize(n_joints);
			state_publisher_->msg_.error.positions.resize(n_joints);
			state_publisher_->msg_.error.velocities.resize(n_joints);
			state_publisher_->unlock();
		}

		// Load the pinocchio dynamics model for Feedforward Inertia
		std::string description_xml;
		if (!root_nh.getParam("iiwa_only_description", description_xml)) {
			if (!root_nh.getParam("robot_description", description_xml)) {
				ROS_ERROR_STREAM("Did not find the " << root_nh.getNamespace() << "robot_description");
				return false;
			}
		}
		pinocchio::urdf::buildModelFromXML(description_xml, pinoModel);
		pinoData = pinocchio::Data(pinoModel);

		// Load ADRC controller
		// The controlling frequency is fixed to 1000Hz(Fixed in the KUKA FRI).
		double h = 1 / 1000.;
		adrcs_.resize(JointTrajectoryController::getNumberOfJoints());
		u_max_.resize(JointTrajectoryController::getNumberOfJoints());
		u_stiction_.resize(JointTrajectoryController::getNumberOfJoints());
		vs_.resize(JointTrajectoryController::getNumberOfJoints());
		Kp_safe.resize(JointTrajectoryController::getNumberOfJoints());
		Kd_safe.resize(JointTrajectoryController::getNumberOfJoints());
		diagOffset_.resize(JointTrajectoryController::getNumberOfJoints());
		for (unsigned int j = 0; j < joint_names_.size(); ++j) {
			// Node handle to PID gains
			ros::NodeHandle joint_nh(controller_nh, std::string("parameters/") + joints_[j].getName());
			adrcs_[j].reset(new ADRCJoint());
			if (!adrcs_[j]->init(joint_nh, h,
								 pinoModel.lowerPositionLimit[j],
								 pinoModel.upperPositionLimit[j],
			                     pinoModel.velocityLimit[j])) {
				return false;
			}
			if (!joint_nh.getParam("u_max", u_max_[j])){
				ROS_ERROR("No u_max specified for ADRC.  Namespace: %s", joint_nh.getNamespace().c_str());
				return false;
			}
			if (!joint_nh.getParam("u_stiction", u_stiction_[j])){
				ROS_ERROR("No u_stiction specified for ADRC.  Namespace: %s", joint_nh.getNamespace().c_str());
				return false;
			}
			if (!joint_nh.getParam("vs", vs_[j])){
				ROS_ERROR("No vs specified for ADRC.  Namespace: %s", joint_nh.getNamespace().c_str());
				return false;
			}
			if (!joint_nh.getParam("diag_offset", diagOffset_[j])){
				ROS_ERROR("No diag_offset specified for ADRC.  Namespace: %s", joint_nh.getNamespace().c_str());
				return false;
			}

			ros::NodeHandle safe_controller_nh(root_nh, "joint_torque_trajectory_controller/gains/" + joints_[j].getName());
			if (!safe_controller_nh.getParam("p", Kp_safe[j])){
				ROS_ERROR("No p gain for safe controller.  Namespace: %s", safe_controller_nh.getNamespace().c_str());
				return false;
			}
			if (!safe_controller_nh.getParam("d", Kd_safe[j])){
				ROS_ERROR("No d gain for safe controller.  Namespace: %s", safe_controller_nh.getNamespace().c_str());
				return false;
			}

			Kp_safe[j] = Kp_safe[j];
			Kd_safe[j] = Kd_safe[j];
		}


		isSafe = true;
		qStop.resize(JointTrajectoryController::getNumberOfJoints());

		u_.resize(JointTrajectoryController::getNumberOfJoints());
		u_ff_.resize(JointTrajectoryController::getNumberOfJoints());
		u_adrc_.resize(JointTrajectoryController::getNumberOfJoints());
		estimation_error_.resize(JointTrajectoryController::getNumberOfJoints());
		disturbance_.resize(JointTrajectoryController::getNumberOfJoints());
		velocity_error_.resize(JointTrajectoryController::getNumberOfJoints());
		return true;
	}

	template<class SegmentImpl>
	void ADRCJointTrajectoryController<SegmentImpl>::starting(const ros::Time &time) {
		// Update time data
		TimeData time_data;
		time_data.time   = time;
		time_data.uptime = ros::Time(0.0);
		time_data_.initRT(time_data);

		// Initialize the desired_state with the current state on startup
		for (unsigned int i = 0; i < joints_.size(); ++i)
		{
			desired_state_.position[i] = joints_[i].getPosition();
			desired_state_.velocity[i] = joints_[i].getVelocity();

			// Start the ADRC initialization
			adrcs_[i]->starting(desired_state_.position[i]);
			joints_[i].setCommand(0.0);
		}

		// Hold current position
		JointTrajectoryController::setHoldPosition(time_data.uptime);

		// Initialize last state update time
		last_state_publish_time_ = time_data.uptime;
	}

	template<class SegmentImpl>
	void ADRCJointTrajectoryController<SegmentImpl>::update(const ros::Time &time, const ros::Duration &period) {
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

//		applyInertiaFeedForward();
//		applyFrictionCompensation();

		setCommand();
//		applySingleJointCmd(1); // Function to test adrd on real robot. 

		setActionFeedback();

		// Update time data
		publishState(time_data.time);
	}

	template<class SegmentImpl>
	void ADRCJointTrajectoryController<SegmentImpl>::publishState(const ros::Time &time) {
		if (!state_publisher_period_.isZero() && last_state_publish_time_ + state_publisher_period_ < time) {
			if (state_publisher_ && state_publisher_->trylock()) {
				last_state_publish_time_ += state_publisher_period_;

				state_publisher_->msg_.header.stamp = time;
				state_publisher_->msg_.desired.positions = desired_state_.position;
				state_publisher_->msg_.desired.velocities = desired_state_.velocity;
				state_publisher_->msg_.desired.accelerations = desired_state_.acceleration;
				state_publisher_->msg_.desired.effort = u_ff_;
				state_publisher_->msg_.actual.positions = current_state_.position;
				state_publisher_->msg_.actual.velocities = current_state_.velocity;
				state_publisher_->msg_.actual.accelerations = disturbance_;
				state_publisher_->msg_.actual.effort = u_;
				state_publisher_->msg_.error.positions = state_error_.position;
				state_publisher_->msg_.error.velocities = state_error_.velocity;
				state_publisher_->msg_.error.accelerations = estimation_error_;
				state_publisher_->msg_.error.effort = u_adrc_;

				state_publisher_->unlockAndPublish();
			}
		}
	}

	template<class SegmentImpl>
	void ADRCJointTrajectoryController<SegmentImpl>::trajectoryCommandCB(const JointTrajectoryConstPtr &msg) {
		trajectory_msgs::JointTrajectory::ConstPtr cubicSplineTrajectory = cubicSplineInterpolate(msg);
		const bool update_ok = JointTrajectoryController::updateTrajectoryCommand(cubicSplineTrajectory, RealtimeGoalHandlePtr());
		if (update_ok) { JointTrajectoryController::preemptActiveGoal(); }
	}

	template <class SegmentImpl>
	void ADRCJointTrajectoryController<SegmentImpl>::setActionFeedback()
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
	ADRCJointTrajectoryController<SegmentImpl>::cubicSplineInterpolate(const JointTrajectoryConstPtr &msg) {
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

	template <class SegmentImpl>
	bool ADRCJointTrajectoryController<SegmentImpl>::queryStateService(control_msgs::QueryTrajectoryState::Request&  req,
	                  control_msgs::QueryTrajectoryState::Response& resp)
	{
		// Preconditions
		if (!this->isRunning())
		{
			ROS_ERROR_NAMED(name_, "Can't sample trajectory. Controller is not running.");
			return false;
		}

		// Convert request time to internal monotonic representation
		TimeData* time_data = time_data_.readFromRT();
		const ros::Duration time_offset = req.time - time_data->time;
		const ros::Time sample_time = time_data->uptime + time_offset;

		// Sample trajectory at requested time
		TrajectoryPtr curr_traj_ptr;
		curr_trajectory_box_.get(curr_traj_ptr);
		Trajectory& curr_traj = *curr_traj_ptr;

		typename Segment::State response_point = typename Segment::State(joint_names_.size());

		for (unsigned int i = 0; i < JointTrajectoryController::getNumberOfJoints(); ++i)
		{
			typename Segment::State state;
			typename TrajectoryPerJoint::const_iterator segment_it = sample(curr_traj[i], sample_time.toSec(), state);
			if (curr_traj[i].end() == segment_it)
			{
				ROS_ERROR_STREAM_NAMED(name_, "Requested sample time precedes trajectory start time.");
				return false;
			}

			response_point.position[i]     = state.position[0];
			response_point.velocity[i]     = state.velocity[0];
			response_point.acceleration[i] = state.acceleration[0];
		}

		// Populate response
		resp.name         = joint_names_;
		resp.position     = response_point.position;
		resp.velocity     = response_point.velocity;
		resp.acceleration = response_point.acceleration;

		return true;
	}

	template<class SegmentImpl>
	void ADRCJointTrajectoryController<SegmentImpl>::applyInertiaFeedForward() {
		// Feedforward Term
		// Follow the structure of Figure 4. in "Active Disturbance Rejection Control of Multi-Joint Industrial Robots
		// Based on Dynamic Feedforward"
		Eigen::VectorXd pinoJointPosition(pinoModel.nq);
		Eigen::VectorXd pinoJointVelocity(pinoModel.nq);
		Eigen::VectorXd pinoJointAcceleration(pinoModel.nq);
		Eigen::VectorXd u_a(pinoModel.nq);
		for (int j = 0; j < JointTrajectoryController::desired_state_.position.size(); ++j) {
			pinoJointPosition[j] = JointTrajectoryController::current_state_.position[j];
			pinoJointVelocity[j] = JointTrajectoryController::current_state_.velocity[j];
			pinoJointAcceleration[j] = JointTrajectoryController::desired_state_.acceleration[j];
		}
		pinocchio::crba(pinoModel, pinoData, pinoJointPosition);
		pinoData.M.triangularView<Eigen::StrictlyLower>() = pinoData.M.transpose().triangularView<Eigen::StrictlyLower>();
		Eigen::VectorXd u_ff = pinoData.M * pinoJointAcceleration;

		for (unsigned int i = 0; i < JointTrajectoryController::getNumberOfJoints(); ++i) {
			u_ff_[i] = u_ff[i];
		}
	}

	template<class SegmentImpl>
	void ADRCJointTrajectoryController<SegmentImpl>::applyFrictionCompensation() {
		for (int j = 0; j < JointTrajectoryController::getNumberOfJoints(); ++j)
		{
			double v_sign = copysign(1.0, desired_state_.velocity[j]);
			if (std::abs(desired_state_.velocity[j]) < 1e-5) v_sign = 0.;

			u_ff_[j] = u_stiction_[j] * exp(-pow(desired_state_.velocity[j] / vs_[j], 2)) * v_sign;
		}
	}

	template<class SegmentImpl>
	void ADRCJointTrajectoryController<SegmentImpl>::setCommand()
	{
		Eigen::VectorXd u, u_joint;
		u.resize(JointTrajectoryController::getNumberOfJoints());
		u_joint.resize(JointTrajectoryController::getNumberOfJoints());

		// Inerita Compatible ADRC
		Eigen::VectorXd pinoJointPosition(pinoModel.nq);

		int test_id = 0;
		for (unsigned int i = test_id; i < JointTrajectoryController::getNumberOfJoints(); ++i) {
			u_joint[i] = adrcs_[i]->update(current_state_.position[i], desired_state_.position[i],
									 desired_state_.velocity[i]);

			u_adrc_[i] = u_joint[i];
			estimation_error_[i] = adrcs_[i]->z1 - current_state_.position[i];
			velocity_error_[i] = adrcs_[i]->z2 - desired_state_.velocity[i];
			disturbance_[i] = adrcs_[i]->z3;

			// Inertia Compatible ADRC
			pinoJointPosition[i] = JointTrajectoryController::current_state_.position[i];
		}

		// Check if the error is too big that potentially cause instability
		if (isSafe) {
			for (unsigned int i = 0; i < JointTrajectoryController::getNumberOfJoints(); ++i) {
				if (state_error_.position[i] > 0.05 || state_error_.position[i] < -0.05) {
					ROS_ERROR_STREAM(
						name_ << " Joint " << i + 1 << " has detected tracking errors: " << state_error_.position[i]
							  << " bigger than 0.05, start the safe mode");
					isSafe = false;
					for (int j = 0; j < JointTrajectoryController::getNumberOfJoints(); ++j) {
						qStop[j] = current_state_.position[j];
					}
					break;
				}
			}
		}

		if (isSafe) {
			pinocchio::crba(pinoModel, pinoData, pinoJointPosition);
			pinoData.M.triangularView<Eigen::StrictlyLower>() =
				pinoData.M.transpose().triangularView<Eigen::StrictlyLower>();

			Eigen::MatrixXd M = pinoData.M;
//			diagOffset << 0.2, 0.0, 0.08, 0.0, 0.03, 0.05, 0.005;
			u = M.diagonal().cwiseMax(diagOffset_).template cwiseProduct(u_joint);

			for (int i = test_id; i < JointTrajectoryController::getNumberOfJoints(); ++i) {
				u[i] = boost::algorithm::clamp(u[i], -pinoModel.effortLimit[i], pinoModel.effortLimit[i]);
				joints_[i].setCommand(u[i] + u_ff_[i]);
				u_[i] = u[i] + u_ff_[i];
			}

			// Debug for single joint
			for (unsigned int i = 0; i < test_id; ++i) {
				u[i] = Kp_safe[i] * (desired_state_.position[i] - current_state_.position[i]) +
					Kd_safe[i] * (desired_state_.velocity[i] - current_state_.velocity[i]);
				u[i] = boost::algorithm::clamp(u[i], -pinoModel.effortLimit[i], pinoModel.effortLimit[i]);
				joints_[i].setCommand(u[i]);
				u_[i] = u[i] + u_ff_[i];
			}
		} else {
			for (unsigned int i = 0; i < JointTrajectoryController::getNumberOfJoints(); ++i) {
				u[i] = Kp_safe[i] * (qStop[i] - current_state_.position[i]) +
					Kd_safe[i] * (- current_state_.velocity[i]);
				u[i] = boost::algorithm::clamp(u[i], -pinoModel.effortLimit[i], pinoModel.effortLimit[i]);
				joints_[i].setCommand(u[i]);
				u_[i] = u[i];
			}
		}
	}

	template<class SegmentImpl>
	void ADRCJointTrajectoryController<SegmentImpl>::applySingleJointCmd(int joint_idx) {

		Eigen::VectorXd u;
		u.resize(JointTrajectoryController::getNumberOfJoints());
		for (unsigned int i = 0; i < JointTrajectoryController::getNumberOfJoints(); ++i) {
			u[i] = Kp_safe[i] * (desired_state_.position[i] - current_state_.position[i]) +
				Kd_safe[i] * (desired_state_.velocity[i] - current_state_.velocity[i]);
		}

		u[joint_idx] = adrcs_[joint_idx]->update(current_state_.position[joint_idx],
												 desired_state_.position[joint_idx],
												 desired_state_.velocity[joint_idx]);

		for (unsigned int i = 0; i < JointTrajectoryController::getNumberOfJoints(); ++i) {
			u[i] = boost::algorithm::clamp(u[i], -u_max_[i], u_max_[i]);
			joints_[i].setCommand(u[i] + u_ff_[i]);
			u_[i] = u[i] + u_ff_[i];
		}
	}
}

#endif //SRC_ADRC_JOINT_TRAJECTORY_CONTROLLER_H
