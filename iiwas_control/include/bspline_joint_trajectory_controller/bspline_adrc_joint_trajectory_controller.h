///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2022, Piotr Kicki
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Piotr Kicki

#pragma once


// C++ standard
#include <cassert>
#include <stdexcept>
#include <string>
#include <memory>

// pinocchio
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea.hpp>

// Boost
#include <boost/algorithm/clamp.hpp>

// ROS
#include <ros/node_handle.h>

// URDF
#include <urdf/model.h>

// ros_controls
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include "bspline_joint_trajectory_controller.h"
#include "adrc/adrc_single.h"

namespace adrc_controllers
{

template<class SegmentImpl>
class BsplineJointTrajectoryController :
  public joint_trajectory_controller::BsplineJointTrajectoryController
            <SegmentImpl, hardware_interface::EffortJointInterface> {

    bool customInit(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override {
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
        pinoJointPosition.resize(pinoModel.nq);
        pinoJointPosition.setZero();

        // Map controller joint with pinocchio joint index;
        pinoJointIdxMap.resize(this->getNumberOfJoints());
        for (int j = 0; j < this->getNumberOfJoints(); ++j)
        {
            pinoJointIdxMap[j] = pinoModel.getJointId(this->joint_names_[j]) - 1;
        }

        uADRC.resize(this->getNumberOfJoints());
        uCmd.resize(this->getNumberOfJoints());
        uTmp.resize(this->getNumberOfJoints());
        q_ddot.resize(this->getNumberOfJoints());
        q_ddot.setZero();
        uMax.resize(this->getNumberOfJoints());
        uMax.setZero();
        inertiaADRC.resize(this->getNumberOfJoints(), this->getNumberOfJoints());

        // Load ADRC controller
        // The controlling frequency is fixed to 1000Hz(Fixed in the KUKA FRI).
        double h = 1 / 1000.;
        adrcs_.resize(this->getNumberOfJoints());
        u_stiction_.resize(this->getNumberOfJoints());
        vs_.resize(this->getNumberOfJoints());
        Kp_safe.resize(this->getNumberOfJoints());
        Kd_safe.resize(this->getNumberOfJoints());
        diagOffset_.resize(this->getNumberOfJoints());

        controller_nh.param("centralize", isCentralized, false);
        for (unsigned int j = 0; j < this->joint_names_.size(); ++j) {
            // Node handle to PID gains
            ros::NodeHandle joint_nh(controller_nh, std::string("parameters/") + this->joints_[j].getName());
            adrcs_[j].reset(new ADRCJoint());
            if (!adrcs_[j]->init(joint_nh, h,
                                 pinoModel.lowerPositionLimit[j],
                                 pinoModel.upperPositionLimit[j],
                                 pinoModel.velocityLimit[j])) {
                return false;
            }
            if (!joint_nh.getParam("u_max", uMax[j])){
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

            ros::NodeHandle safe_controller_nh(root_nh, "joint_torque_trajectory_controller/gains/" + this->joints_[j].getName());
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
        qStop.resize(this->getNumberOfJoints());

        u_.resize(this->getNumberOfJoints());
        u_ff_.resize(this->getNumberOfJoints());
        u_adrc_.resize(this->getNumberOfJoints());
        estimation_error_.resize(this->getNumberOfJoints());
        disturbance_.resize(this->getNumberOfJoints());
        velocity_error_.resize(this->getNumberOfJoints());
        return true;
    }

    void customController() override {
        for (unsigned int i = 0; i < this->getNumberOfJoints(); ++i){
            pinoJointPosition[pinoJointIdxMap[i]] = this->current_state_.position[i];
        }
        // Compute inertia matrix
        pinocchio::crba(pinoModel, pinoData, pinoJointPosition);
        pinoData.M.triangularView<Eigen::StrictlyLower>() = pinoData.M.transpose().triangularView<Eigen::StrictlyLower>();

        // Map pinocchio inertia to adrc
        for (unsigned int j = 0; j < this->getNumberOfJoints(); ++j)
        {
            for (int k = 0; k < this->getNumberOfJoints(); ++k)
            {
                inertiaADRC(j, k) = pinoData.M(pinoJointIdxMap[j], pinoJointIdxMap[k]);
            }
        }

        int test_id = 0;
        for (unsigned int i = test_id; i < this->getNumberOfJoints(); ++i) {
            uADRC[i] = adrcs_[i]->update(this->current_state_.position[i], this->desired_state_.position[i],
                                         this->desired_state_.velocity[i], uADRC[i]);
            u_adrc_[i] = uADRC[i];
            estimation_error_[i] = adrcs_[i]->z1 - this->current_state_.position[i];
            velocity_error_[i] = adrcs_[i]->z2 - this->desired_state_.velocity[i];
            disturbance_[i] = adrcs_[i]->z3;
	    uADRC[i] += this->desired_state_.acceleration[i];
        }


        if (isCentralized) {
            uCmd = inertiaADRC * uADRC;
            uCmd = uCmd.cwiseMax(-uMax).cwiseMin(uMax);
            uADRC = inertiaADRC.inverse() * uCmd;
        }
        else{
            uCmd = inertiaADRC.diagonal().cwiseMax(diagOffset_).template cwiseProduct(uADRC);
            uCmd = uCmd.cwiseMax(-uMax).cwiseMin(uMax);
            uADRC = inertiaADRC.diagonal().cwiseMax(diagOffset_).cwiseInverse().template cwiseProduct(uCmd);
        }

        //applyFF();
        //applyFrictionCompensation();

        for (int i = test_id; i < this->getNumberOfJoints(); ++i) {
            u_[i] = uCmd[i] + u_ff_[i];
            this->joints_[i].setCommand(u_[i]);
        }
    }

    void customStarting() override {
        // Start the ADRC initialization
        for (unsigned int i = 0; i < this->getNumberOfJoints(); ++i)
        {
            adrcs_[i]->starting(this->desired_state_.position[i]);
            this->joints_[i].setCommand(0.0);
        }
        uADRC.setZero();
        uCmd.setZero();
    }

    void applyFrictionCompensation() {
        for (int j = 0; j < this->getNumberOfJoints(); ++j)
        {
            double v_sign = copysign(1.0, this->desired_state_.velocity[j]);
            if (std::abs(this->desired_state_.velocity[j]) < 1e-5) v_sign = 0.;

            u_ff_[j] = u_stiction_[j] * exp(-pow(this->desired_state_.velocity[j] / vs_[j], 2)) * v_sign;
        }
    }

    void applyFF() {
        for (int j = 0; j < this->getNumberOfJoints(); ++j) {
            q_ddot[j] = this->desired_state_.acceleration[j];
        }

        uTmp = inertiaADRC * q_ddot;
        for (int j = 0; j < this->getNumberOfJoints(); ++j)
        {
            u_ff_[j] = uTmp[j];
        }
    }

    void publishState(const ros::Time &time) override {
        if (!this->state_publisher_period_.isZero() && this->last_state_publish_time_ + this->state_publisher_period_ < time) {
            if (this->state_publisher_ && this->state_publisher_->trylock()) {
                this->last_state_publish_time_ += this->state_publisher_period_;

                this->state_publisher_->msg_.header.stamp = time;
                this->state_publisher_->msg_.desired.positions = this->desired_state_.position;
                this->state_publisher_->msg_.desired.velocities = this->desired_state_.velocity;
                this->state_publisher_->msg_.desired.accelerations = this->desired_state_.acceleration;
                this->state_publisher_->msg_.desired.effort = u_ff_;
                this->state_publisher_->msg_.actual.positions = this->current_state_.position;
                this->state_publisher_->msg_.actual.velocities = this->current_state_.velocity;
                this->state_publisher_->msg_.actual.accelerations = disturbance_;
                this->state_publisher_->msg_.actual.effort = u_;
                this->state_publisher_->msg_.error.positions = this->state_error_.position;
                this->state_publisher_->msg_.error.velocities = this->state_error_.velocity;
                this->state_publisher_->msg_.error.accelerations = estimation_error_;
                this->state_publisher_->msg_.error.effort = u_adrc_;

                this->state_publisher_->unlockAndPublish();
            }
        }
    }

protected:
    pinocchio::Model pinoModel;
    pinocchio::Data pinoData;
    Eigen::VectorXd pinoJointPosition;
    Eigen::VectorXd uCmd, uADRC, q_ddot, uMax, uTmp;
    Eigen::MatrixXd inertiaADRC;
    Eigen::ArrayXi pinoJointIdxMap;

    typedef std::shared_ptr<ADRCJoint> ADRCJointPtr;
    std::vector<ADRCJointPtr> adrcs_;
    std::vector<double> u_ff_, u_adrc_, u_, u_stiction_, vs_;
    std::vector<double> estimation_error_, disturbance_, velocity_error_;

    std::vector<double> Kp_safe, Kd_safe, qStop;
    Eigen::VectorXd diagOffset_;
    bool isSafe, isCentralized;

};

} // namespace
