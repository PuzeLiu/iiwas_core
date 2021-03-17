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


#ifndef SRC_GRAVITY_COMPENSATION_HW_SIM_H
#define SRC_GRAVITY_COMPENSATION_HW_SIM_H

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <gazebo_ros_control/default_robot_hw_sim.h>


namespace iiwas_gazebo {
	class GravityCompensationHWSim : public gazebo_ros_control::DefaultRobotHWSim {
	public:
		virtual bool initSim(const std::string &robot_namespace,
		                     ros::NodeHandle model_nh,
		                     gazebo::physics::ModelPtr parent_model,
		                     const urdf::Model *const urdf_model,
		                     std::vector<transmission_interface::TransmissionInfo> transmissions) override;

		virtual void readSim(ros::Time time, ros::Duration period) override;

		virtual void writeSim(ros::Time time, ros::Duration period) override;

	protected:
		pinocchio::Model pinoModel;
		pinocchio::Data pinoData;
		Eigen::VectorXd pinoJointPosition;
		Eigen::VectorXd pinoJointVelocity;
		Eigen::VectorXd pinoJointEffort;
	};
}


#endif //SRC_GRAVITY_COMPENSATION_HW_SIM_H
