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

#ifndef _IIWAS_HW_INTERFACE_H
#define _IIWAS_HW_INTERFACE_H

#include <string>

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

#include <urdf/model.h>

#include "iiwas_driver/iiwa_ros.h"
#include "iiwas_driver/configuration_client.h"


const int DEFAULT_CONTROL_FREQUENCY = 1000;  // Hz

int NUMBER_OF_JOINTS = 7;

namespace iiwa_hw {
    class HardwareInterface : public hardware_interface::RobotHW {
    public:
        HardwareInterface();

        ~HardwareInterface();

        bool init();

        /**
            * \brief Reads the current robot state via the interfae provided by iiwa_ros and sends the values to the Device struct.
        */
        void read(const ros::Time& time, const ros::Duration& period) override;

        /**
            * \brief Sends the command joint position to the robot via the interface provided by iiwa_ros.
        */
        void write(const ros::Time& time, const ros::Duration& period) override;

        inline const bool getIsAppServerStarted(){ return isAppServerStarted;};

        inline const bool getFRICommandingWait(){ return false;};

        void stop(){
            stopFRI();
            ros::shutdown();
        };

        const std::vector<JState> getJointState(){
        	return jointState;
        }

        const std::vector<DJState> getJointCommand(){
        	return jointCommand;
        }

    protected:

        void loadParam();

        bool initHardwareInterface();

        bool initFRI();

        void stopFRI();

        int loadURDF(std::string param_name);

        virtual void registerJointLimits_(const hardware_interface::JointHandle &jointHandlePosition,
                                         const hardware_interface::JointHandle &jointHandleVelocity,
                                         const hardware_interface::JointHandle &jointHandleEffort,
                                         std::size_t joint_id);

    private:
        /** Robot Parameter */
        std::vector<JState> jointState, jointStateLast;
        std::vector<DJState> jointCommand;

        bool useURDFJointLimits;
        bool useSoftLimits;
        bool hasSoftLimits;
        bool hasJointLimits;

        std::vector<std::string> jointNames;

        std::vector<double> jointPositionLowerLimits;
        std::vector<double> jointPositionUpperLimits;
        std::vector<double> jointVelocityLimits;
        std::vector<double> jointEffortLimits;

        /** ROS Parameter */
        ros::NodeHandle nh;

        urdf::Model *urdfModel;

        std::string iiwaDescription;

        /** Hardware Interface*/
        hardware_interface::JointStateInterface jointStateInterface;   /**< Interface for joint state */
        hardware_interface::PositionJointInterface jointPositionInterface; /**< Interface for joint position control */
        hardware_interface::VelocityJointInterface jointVelocityInterface; /**< Interface for joint position control */
        hardware_interface::EffortJointInterface jointEffortInterface;     /**< Interface for joint impedance control */

        /** Interfaces for limits */
        joint_limits_interface::PositionJointSaturationInterface positionJointSatLimitsInterface;
        joint_limits_interface::PositionJointSoftLimitsInterface positionJointSoftLimitsInterface;
        joint_limits_interface::EffortJointSaturationInterface effortJointSatLimitsInterface;
        joint_limits_interface::EffortJointSoftLimitsInterface effortJointSoftLimitsInterface;
        joint_limits_interface::VelocityJointSaturationInterface velocityJointSatLimitsInterface;
        joint_limits_interface::VelocityJointSoftLimitsInterface velocityJointSoftLimitsInterface;

        /** FRI Connection Parameter for FRI Application server */
        std::string friServerIP;
        int friServerPort;

        bool isAppServerStarted;

    };
}

#endif //_IIWAS_HW_INTERFACE_H
