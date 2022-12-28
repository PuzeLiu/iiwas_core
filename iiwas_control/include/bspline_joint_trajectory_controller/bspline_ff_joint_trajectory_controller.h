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
#include <pinocchio/algorithm/frames.hpp>

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
        Eigen::VectorXd pinoDesiredJointPosition(pinoModel.nq);
        Eigen::VectorXd pinoJointPosition(pinoModel.nq);
        Eigen::VectorXd pinoJointVelocity(pinoModel.nq);
        Eigen::VectorXd pinoJointAcceleration(pinoModel.nq);
        for (int j = 0; j < this->desired_state_.position.size(); ++j) {
            pinoJointPosition[j] = this->current_state_.position[j];
            pinoJointVelocity[j] = this->current_state_.velocity[j];
            pinoJointAcceleration[j] = this->desired_state_.acceleration[j];
            pinoDesiredJointPosition[j] = this->desired_state_.position[j];
        }

        auto q = pinoJointPosition;
        //std::cout << "PINO DESIRED: " << this->desired_state_.position[6] << std::endl;
        q[6] = 0.;

        pinocchio::forwardKinematics(pinoModel, pinoData, q);
        pinocchio::updateFramePlacements(pinoModel, pinoData);
        auto pinoFrameId = pinoModel.getFrameId("F_striker_tip");
        Eigen::Matrix3d mat = pinoData.oMf[pinoFrameId].rotation();
        //Eigen::Vector3d zAxis(0., 0., -1.);
        Eigen::Vector3d zAxis(0., 0., 1.);
        auto yDes = zAxis.cross(mat.col(2)).normalized();
        double target = acos(boost::algorithm::clamp(mat.col(1).dot(yDes), -1., 1.));
        Eigen::Vector3d axis = mat.col(1).cross(yDes).normalized();
        target = target * axis.dot(mat.col(2));


        //std::cout << "TARGET: " <<  target << std::endl;
        //std::cout << "POS: " <<  pinoJointPosition[6] << std::endl;

        //if (target - pinoJointPosition[6] > M_PI_2) {
        //    //std::cout << "TOO BIG: " <<  target - pinoJointPosition[6] << std::endl;
        //    target -= M_PI;
        //} else if (target - pinoJointPosition[6] < -M_PI_2) {
        //    //std::cout << "TOO SMALL: " <<  target - pinoJointPosition[6] << std::endl;
        //    target += M_PI;
        //}

        auto rate = 1 / 100.;
        //auto dq6 = boost::algorithm::clamp((target - pinoJointPosition[6]) * rate,
        //                                -pinoModel.velocityLimit[6],
        //                                pinoModel.velocityLimit[6]);
        //auto q6 = boost::algorithm::clamp(pinoJointPosition[6] + dq6 / rate,
        //                               pinoModel.lowerPositionLimit[6],
        //                               pinoModel.upperPositionLimit[6]);
        auto dq6 = (target - pinoJointPosition[6]) * rate;
        auto q6 = target;
        this->desired_state_.position[6] = q6;
        this->desired_state_.velocity[6] = dq6;
        this->state_error_.position[6] = angles::shortest_angular_distance(this->current_state_.position[6], q6);
        //std::cout << "Q6: " << q6 << std::endl;
        //std::cout << "DQ6: " << dq6 << std::endl;
        q[6] = q6;



        this->hw_iface_adapter_.updateCommand(ros::Time::now(), ros::Duration(0.1),
                                              this->desired_state_, this->state_error_);
        /** Add Feedforward Term*/
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