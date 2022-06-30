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

namespace feedforward_controllers
{

template<class SegmentImpl>
class BsplineJointTrajectoryController :
  public joint_trajectory_controller::BsplineJointTrajectoryController
            <SegmentImpl, hardware_interface::EffortJointInterface> {

    bool customInit(hardware_interface::EffortJointInterface* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override {
        // Hardware interface adapter
        this->hw_iface_adapter_.init(this->joints_, this->controller_nh_);
        std::string description_xml;
        if (!root_nh.getParam("iiwa_only_description", description_xml)) {
            if (!root_nh.getParam("robot_description", description_xml)) {
                ROS_ERROR_STREAM("Did not find the " << root_nh.getNamespace() << "robot_description");
                return false;
            }
        }

        pinocchio::urdf::buildModelFromXML(description_xml, pinoModel);
        pinoData = pinocchio::Data(pinoModel);
        ff_torque_.resize(pinoModel.nq);
        pid_torque_.resize(pinoModel.nq);
        actual_torque_.resize(pinoModel.nq);
        return true;
    }

    void customController() override {
        this->hw_iface_adapter_.updateCommand(ros::Time::now(), ros::Duration(0.1),
                                              this->desired_state_, this->state_error_);
        /** Add Feedforward Term*/
        Eigen::VectorXd pinoJointPosition(pinoModel.nq);
        Eigen::VectorXd pinoJointVelocity(pinoModel.nq);
        Eigen::VectorXd pinoJointAcceleration(pinoModel.nq);
        for (int j = 0; j < this->desired_state_.position.size(); ++j) {
            pinoJointPosition[j] = this->current_state_.position[j];
            pinoJointVelocity[j] = this->current_state_.velocity[j];
            pinoJointAcceleration[j] = this->desired_state_.acceleration[j];
        }
        pinocchio::crba(pinoModel, pinoData, pinoJointPosition);
        pinoData.M.triangularView<Eigen::StrictlyLower>() =
                pinoData.M.transpose().triangularView<Eigen::StrictlyLower>();
        Eigen::VectorXd ffTerm = pinoData.M * pinoJointAcceleration;

        pinocchio::rnea(pinoModel, pinoData, pinoJointPosition, pinoJointVelocity, pinoJointAcceleration);
        Eigen::VectorXd idTorque = pinoData.tau + pinoModel.friction.cwiseProduct(pinoJointVelocity.cwiseSign())
                + pinoModel.damping.cwiseProduct(pinoJointVelocity);

        for (unsigned int i = 0; i < this->joint_names_.size(); ++i) {
            ff_torque_[i] = idTorque[i] + pid_torque_[i] + ffTerm[i];
            pid_torque_[i] = this->joints_[i].getCommand();
            actual_torque_[i] = boost::algorithm::clamp(pid_torque_[i] + ffTerm[i], -pinoModel.effortLimit[i], pinoModel.effortLimit[i]);
            this->joints_[i].setCommand(actual_torque_[i]);
        }
    }
    void customStarting() override {
        // hardware interface adapter
        this->hw_iface_adapter_.starting(ros::Time(0.0));
    };

    pinocchio::Model pinoModel;
    pinocchio::Data pinoData;
    std::vector<double> ff_torque_;
    std::vector<double> pid_torque_;
    std::vector<double> actual_torque_;


};

} // namespace