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

#include <iiwa_hw_interface.h>

using namespace KUKA::FRI;

namespace iiwa_hw{
    HardwareInterface::HardwareInterface() : nh(){
        jointState.resize(LBRState::NUMBER_OF_JOINTS);
        jointStateLast.resize(LBRState::NUMBER_OF_JOINTS);
        jointCommand.resize(LBRState::NUMBER_OF_JOINTS);

        jointPositionLowerLimits.resize(LBRState::NUMBER_OF_JOINTS);
        jointPositionUpperLimits.resize(LBRState::NUMBER_OF_JOINTS);
        jointVelocityLimits.resize(LBRState::NUMBER_OF_JOINTS);
        jointEffortLimits.resize(LBRState::NUMBER_OF_JOINTS);

        loadParam();

        loadURDF(iiwaDescription);

        isAppServerStarted = false;
    }

    HardwareInterface::~HardwareInterface(){
        stop();
    };

    bool HardwareInterface::init() {

        if(!initHardwareInterface())
            return false;

        if(!initFRI())
            return false;

        return true;
    }

    void HardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
        if (friClient->isDataAvailable()) {
            jointStateLast = jointState;
            for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++) {
                auto velocityMeasure = (friClient->latest_measured_joint_pos[i] - jointStateLast[i].th) / period.toSec();
                jointState[i].th = friClient->latest_measured_joint_pos[i];
                jointState[i].thd = (velocityMeasure + jointStateLast[i].thd)/2;
                jointState[i].load = friClient->latest_measured_joint_torque[i];
            }
        }
    }

    void HardwareInterface::write(const ros::Time &time, const ros::Duration &period) {
        if (friClient->isCommandingActive()) {
				if (hasSoftLimits) {
					positionJointSoftLimitsInterface.enforceLimits(period);
					effortJointSoftLimitsInterface.enforceLimits(period);
					velocityJointSoftLimitsInterface.enforceLimits(period);
				} else{
					positionJointSatLimitsInterface.enforceLimits(period);
					effortJointSatLimitsInterface.enforceLimits(period);
					velocityJointSatLimitsInterface.enforceLimits(period);
				}
				
                if (friClient->robotState().getClientCommandMode() == KUKA::FRI::TORQUE){
                    for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++) {
                        friClient->joint_pos_des[i] = friClient->latest_measured_joint_pos[i];
                        friClient->joint_torques_des[i] = jointCommand[i].uff;
                    }
//                    friClient->joint_pos_des[0] = 0.;
//                    friClient->joint_pos_des[1] = 0.;
                } else{
                    for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++) {
                       friClient->joint_pos_des[i] = jointCommand[i].th;
                    }
                }
                
        	}
        else if (friClient->isCommandingWait()){
			if (hasSoftLimits) {
				positionJointSoftLimitsInterface.reset();
			} else{
				positionJointSatLimitsInterface.reset();
			}
        	for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++) {
        						friClient->joint_pos_des[i] = jointCommand[i].th;
        						friClient->joint_torques_des[i] = jointCommand[i].uff;
        	}
        }

        friApp->step();
    }

    void HardwareInterface::loadParam(){
        nh.param("use_urdf_joint_limits", useURDFJointLimits, true);
        nh.param("use_soft_limits", useSoftLimits, false);

		std::vector<double> init_pos(LBRState::NUMBER_OF_JOINTS);
		nh.param("init_position", init_pos);

		for(int i=0; i < LBRState::NUMBER_OF_JOINTS; i++)
			jointCommand[i].th = init_pos[i];

        if (!nh.getParam("fri_ip", friServerIP))
            ROS_WARN_STREAM_ONCE(nh.getNamespace() + " Unable to load application server ip from Parameter Server, use Default: "
                                         << friServerIP);
        if (!nh.getParam("fri_port", friServerPort))
            ROS_WARN_STREAM_ONCE(nh.getNamespace() + "Unable to load application server port from Parameter Server, use Default: "
                                         << friServerPort);
        nh.param<std::string>("urdf_param", iiwaDescription, "/robot_description");

    }

    bool HardwareInterface::initHardwareInterface(){
        if (!nh.getParam("joints", jointNames)) {
            ROS_ERROR_STREAM_ONCE(nh.getNamespace() + " Unable to load joint names from Parameter Server");
            return false;
        }

        for (std::size_t joint_id = 0; joint_id < LBRState::NUMBER_OF_JOINTS; ++joint_id) {

            hardware_interface::JointStateHandle jointHandleState = hardware_interface::JointStateHandle(
                    jointNames[joint_id], &jointState[joint_id].th, &jointState[joint_id].thd,
                    &jointState[joint_id].load);
            jointStateInterface.registerHandle(jointHandleState);

            hardware_interface::JointHandle jointHandlePosition = hardware_interface::JointHandle(
                    jointHandleState, &jointCommand[joint_id].th);
            jointPositionInterface.registerHandle(jointHandlePosition);

            hardware_interface::JointHandle jointHandleVelocity = hardware_interface::JointHandle(
                    jointHandleState, &jointCommand[joint_id].thd);
            jointVelocityInterface.registerHandle(jointHandleVelocity);

            hardware_interface::JointHandle jointHandleEffort = hardware_interface::JointHandle(
                    jointHandleState, &jointCommand[joint_id].uff);
            jointEffortInterface.registerHandle(jointHandleEffort);

            registerJointLimits_(jointHandlePosition, jointHandleVelocity, jointHandleEffort, joint_id);
            }

        registerInterface(&jointStateInterface);
        registerInterface(&jointPositionInterface);
        registerInterface(&jointVelocityInterface);
        registerInterface(&jointEffortInterface);

        ROS_INFO_STREAM(nh.getNamespace() + " ROS Hardware Interface initialized");
        return true;
    }

    bool HardwareInterface::initFRI() {
        friClient = new FRIClient();
        friConnection = new KUKA::FRI::UdpConnection();
        friApp = new KUKA::FRI::ClientApplication(*friConnection, *friClient);

        ROS_INFO_STREAM(nh.getNamespace() + " Connecting to FRI Application Server");
        if(!friApp->connect(friServerPort, friServerIP.c_str()))
        {
            isAppServerStarted = false;
            ROS_ERROR_STREAM(nh.getNamespace() + " Failed to connect FRI Application Server");
            return false;
        } else{
            isAppServerStarted = true;
            ROS_INFO_STREAM(nh.getNamespace() + " Connection to FRI Application Server Succeed");
        }
        return true;
    }

    void HardwareInterface::stopFRI() {
        if(isAppServerStarted){
        	isAppServerStarted = false;
            friApp->disconnect();
        }
    }

    int HardwareInterface::loadURDF(std::string param_name) {
        std::string urdf_string;
        urdfModel = new urdf::Model();

        if (!nh.getParam(param_name, urdf_string)) {
        	ROS_ERROR_STREAM("Could not find URDF on the ROS parameters server at location:"<< param_name);
        	return -1;
        }

        if (!urdfModel->initString(urdf_string)) {
            ROS_ERROR_STREAM("Unable to load URDF model");
            return -1;
        }
        else {
            ROS_INFO_STREAM("Received URDF from parameters server");
            return 0;
        }

    }

    void HardwareInterface::registerJointLimits_(const hardware_interface::JointHandle &jointHandlePosition,
                                                 const hardware_interface::JointHandle &jointHandleVelocity,
                                                 const hardware_interface::JointHandle &jointHandleEffort,
                                                 std::size_t joint_id) {
        jointPositionLowerLimits[joint_id] = -std::numeric_limits<double>::max();
        jointPositionUpperLimits[joint_id] = std::numeric_limits<double>::max();
        jointVelocityLimits[joint_id] = std::numeric_limits<double>::max();
        jointEffortLimits[joint_id] = std::numeric_limits<double>::max();

        // Limits data structures
        joint_limits_interface::JointLimits jointLimits;     // Position
        joint_limits_interface::SoftJointLimits softLimits;  // Soft Position
        hasJointLimits = false;
        hasSoftLimits = false;

        // Get limits from URDF
        if (urdfModel == nullptr)
        {
            ROS_WARN_STREAM("No URDF model loaded, unable to get joint limits");
            return;
        }

        urdf::JointConstSharedPtr urdf_joint = urdfModel->getJoint(jointNames[joint_id]);

        // Get main joint limits
        if (urdf_joint == nullptr)
        {
            ROS_ERROR_STREAM(nh.getNamespace() + " URDF joint not found " << jointNames[joint_id]);
            return;
        }

        // Get limits from URDF
        if (joint_limits_interface::getJointLimits(urdf_joint, jointLimits))
        {
            hasJointLimits = true;
            ROS_DEBUG_STREAM(nh.getNamespace() + " Joint " << jointNames[joint_id] << " has URDF position limits ["
                                                   << jointLimits.min_position << ", "
                                                   << jointLimits.max_position << "]");
            if (jointLimits.has_velocity_limits)
                ROS_DEBUG_STREAM(nh.getNamespace() + " Joint " << jointNames[joint_id] << " has URDF velocity limit ["
                                                       << jointLimits.max_velocity << "]");
        }
        else
        {
            if (urdf_joint->type != urdf::Joint::CONTINUOUS)
                ROS_WARN_STREAM(nh.getNamespace() + " Joint " << jointNames[joint_id] << " does not have a URDF "
                        "position limit");
        }

        // Get limits from ROS param
        if (!useURDFJointLimits)
        {
            if (joint_limits_interface::getJointLimits(jointNames[joint_id], nh, jointLimits))
            {
                hasJointLimits = true;
                ROS_DEBUG_STREAM(nh.getNamespace() + " Joint " << jointNames[joint_id] << " has rosparam position limits ["
                                                << jointLimits.min_position << ", " << jointLimits.max_position << "]");
                if (jointLimits.has_velocity_limits)
                    ROS_DEBUG_STREAM(nh.getNamespace() + " Joint " << jointNames[joint_id]
                                                           << " has rosparam velocity limit ["
                                                           << jointLimits.max_velocity << "]");
            }  // the else debug message provided internally by joint_limits_interface
        }

        // Get soft limits from URDF
        if (useSoftLimits)
        {
            if (joint_limits_interface::getSoftJointLimits(urdf_joint, softLimits))
            {
                hasSoftLimits = true;
                ROS_DEBUG_STREAM(nh.getNamespace() + " Joint " << jointNames[joint_id] << " has soft joint limits.");
            }
            else
            {
                ROS_DEBUG_STREAM(nh.getNamespace() + " Joint " << jointNames[joint_id] << " does not have soft joint "
                        "limits");
            }
        }

        // Quit we we haven't found any limits in URDF or rosparam server
        if (!hasJointLimits)
        {
            ROS_WARN_STREAM(nh.getNamespace() + " Could not find any limist on joint " << jointNames[joint_id]);
            return;
        }

        // Copy position limits if available
        if (jointLimits.has_position_limits)
        {
            // Slightly reduce the joint limits to prevent floating point errors
            jointLimits.min_position += std::numeric_limits<double>::epsilon();
            jointLimits.max_position -= std::numeric_limits<double>::epsilon();

            jointPositionLowerLimits[joint_id] = jointLimits.min_position;
            jointPositionUpperLimits[joint_id] = jointLimits.max_position;
        }

        // Copy velocity limits if available
        if (jointLimits.has_velocity_limits)
        {

            jointVelocityLimits[joint_id] = jointLimits.max_velocity;
        }

        // Copy effort limits if available
        if (jointLimits.has_effort_limits)
        {

            jointEffortLimits[joint_id] = jointLimits.max_effort;
        }

        if (hasSoftLimits)  // Use soft limits
        {
            ROS_INFO_STREAM_ONCE(nh.getNamespace() + " Using soft saturation limits on Joints");
            const joint_limits_interface::PositionJointSoftLimitsHandle softHandlePosition(jointHandlePosition,
                                                                                             jointLimits, softLimits);
            positionJointSoftLimitsInterface.registerHandle(softHandlePosition);

            const joint_limits_interface::EffortJointSoftLimitsHandle softHandleEffort(jointHandleEffort,
                                                                                       jointLimits, softLimits);
            effortJointSoftLimitsInterface.registerHandle(softHandleEffort);

            const joint_limits_interface::VelocityJointSoftLimitsHandle softHandleVelocity(jointHandleVelocity,
                                                                                           jointLimits, softLimits);
            velocityJointSoftLimitsInterface.registerHandle(softHandleVelocity);
        }
        else  // Use saturation limits
        {
            ROS_INFO_STREAM_ONCE(nh.getNamespace() +  " Using saturation limits (not soft limits) on Joint");

            const joint_limits_interface::PositionJointSaturationHandle satHandlePosition(jointHandlePosition, jointLimits);
            positionJointSatLimitsInterface.registerHandle(satHandlePosition);

            const joint_limits_interface::EffortJointSaturationHandle satHandleEffort(jointHandleEffort, jointLimits);
            effortJointSatLimitsInterface.registerHandle(satHandleEffort);

            const joint_limits_interface::VelocityJointSaturationHandle satHandleVelocity(jointHandleVelocity, jointLimits);
            velocityJointSatLimitsInterface.registerHandle(satHandleVelocity);
        }

    }

}
