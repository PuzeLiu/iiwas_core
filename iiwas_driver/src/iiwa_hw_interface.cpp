/*=============================================================================
 ==============================================================================

 \file    iiwas_hw_interface.cpp

 \author  Puze Liu
 \date    29.07.2020

 ==============================================================================
 \remarks

 Runs a interface for ROS through friClient

 ============================================================================*/

#include <iiwa_hw_interface.h>

using namespace KUKA::FRI;

namespace iiwa_hw{
    HardwareInterface::HardwareInterface(ros::NodeHandle& nh, std::string ns) : nh(nh), ns("/" + ns) {
        jointState.resize(LBRState::NUMBER_OF_JOINTS);
        jointStateLast.resize(LBRState::NUMBER_OF_JOINTS);
        jointCommand.resize(LBRState::NUMBER_OF_JOINTS);

        jointPositionLowerLimits.resize(LBRState::NUMBER_OF_JOINTS);
        jointPositionUpperLimits.resize(LBRState::NUMBER_OF_JOINTS);
        jointVelocityLimits.resize(LBRState::NUMBER_OF_JOINTS);
        jointEffortLimits.resize(LBRState::NUMBER_OF_JOINTS);

        loadParam();

        loadURDF(nh, iiwaDescription);

        iiwaReady = false;
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

        iiwaReady = true;

        return true;
    }

    void HardwareInterface::read(const ros::Time &time, const ros::Duration &period) {
        if (friClient->isInitialized()) {
            jointStateLast = jointState;
            for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++) {
                jointState[i].th = friClient->latest_measured_joint_pos[i];
                jointState[i].thd = (friClient->latest_measured_joint_pos[i] - jointStateLast[i].th) / period.toSec();
                jointState[i].load = friClient->latest_measured_joint_torque[i];
            }
        }
    }

    void HardwareInterface::write(const ros::Time &time, const ros::Duration &period) {
        if (friClient->isInitialized()) {
            positionJointSoftLimitsInterface.enforceLimits(period);
            positionJointSatLimitsInterface.enforceLimits(period);
            effortJointSoftLimitsInterface.enforceLimits(period);
            effortJointSatLimitsInterface.enforceLimits(period);
            velocityJointSoftLimitsInterface.enforceLimits(period);
            velocityJointSatLimitsInterface.enforceLimits(period);

            for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++) {
                friClient->joint_pos_des[i] = jointCommand[i].th;
                friClient->joint_torques_des[i] = jointCommand[i].uff;

            }
        }

        iiwaReady = !friClient->isClosing() && friApp->step();
    }

    void HardwareInterface::loadParam(){
        ros::NodeHandle n_p("~");

        n_p.param(ns + "/use_ROS_Param_Joint_Limits_", useROSParamJointLimits ,false);
        n_p.param(ns + "/use_ROS_Param_Soft_Limits_If_Available", useSoftLimitsIfAvailable ,true);


        if (!n_p.param<std::string>(ns + "/fri_ip", friServerIP, "192.170.10.2"))
            ROS_WARN_STREAM_ONCE(ns + ": Unable to load application server ip from Parameter Server, use Default: "
                                         << friServerIP);
        if (!n_p.param(ns + "/fri_port", friServerPort, 30202))
            ROS_WARN_STREAM_ONCE(ns + ": Unbale to load application server port from Parameter Server, use Default: "
                                         << friServerPort);

        if (!n_p.param<std::string>(ns + "/robot_description", iiwaDescription, "/robot_description"));

    }

    bool HardwareInterface::initHardwareInterface() {
        ros::NodeHandle n_p("~");

        if (!n_p.getParam(ns + "/joints", jointNames)) {
            ROS_ERROR_STREAM_ONCE(n_p.getNamespace() + ": Unbale to load joint names from Parameter Server");
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

        ROS_INFO_STREAM(ns + ": ROS Hardware Interface initialized");
        return true;
    }

    bool HardwareInterface::initFRI() {
        friClient = new FRIClient();
        friConnection = new KUKA::FRI::UdpConnection();
        friApp = new KUKA::FRI::ClientApplication(*friConnection, *friClient);

        ROS_INFO_STREAM(ns + ": Connecting to FRI Application Server");
        if(!friApp->connect(friServerPort, friServerIP.c_str()))
        {
            isAppServerStarted = false;
            ROS_ERROR_STREAM(ns + ": Failed to connect FRI Application Server");
            return false;
        } else{
            isAppServerStarted = true;
            ROS_INFO_STREAM(ns + ": Connection to FRI Application Server Succeed");
        }

        return true;
    }

    void HardwareInterface::stopFRI() {
        if(isAppServerStarted)
            friApp->disconnect();

        ROS_INFO_STREAM(ns + ": Stop FRI connection");
    }

    int HardwareInterface::loadURDF(ros::NodeHandle &nh, std::string param_name) {
        std::string urdf_string;
        urdfModel = new urdf::Model();

        std::string search_param_name;
        if (nh.searchParam(param_name, search_param_name)) {
            if (!nh.getParam(search_param_name, urdf_string))
                ROS_ERROR_STREAM(ns + ": Counld not find URDF on the ROS param server at location:"<< search_param_name);
        }
        else{
            if (!nh.getParam(param_name, urdf_string)){
                ROS_ERROR_STREAM(ns + ": Counld not find URDF on the ROS param server at location:"<< param_name);
            }
        }

        if (!urdfModel->initString(urdf_string)) {
            ROS_ERROR_STREAM(ns + ": Unable to load URDF model");
            return -1;
        }
        else {
            ROS_INFO_STREAM(ns + ": Received URDF from param server");
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

        // Limits datastructures
        joint_limits_interface::JointLimits jointLimits;     // Position
        joint_limits_interface::SoftJointLimits softLimits;  // Soft Position
        bool hasJointLimits = false;
        bool hasSoftLimits = false;

        // Get limits from URDF
        if (urdfModel == nullptr)
        {
            ROS_WARN_STREAM(ns + "No URDF model loaded, unable to get joint limits");
            return;
        }

        urdf::JointConstSharedPtr urdf_joint = urdfModel->getJoint(jointNames[joint_id]);
        

        // Get main joint limits
        if (urdf_joint == nullptr)
        {
            ROS_ERROR_STREAM(ns + " URDF joint not found " << jointNames[joint_id]);
            return;
        }


        // Get limits from URDF
        if (joint_limits_interface::getJointLimits(urdf_joint, jointLimits))
        {
            hasJointLimits = true;
            ROS_DEBUG_STREAM(ns + "Joint " << jointNames[joint_id] << " has URDF position limits ["
                                                   << jointLimits.min_position << ", "
                                                   << jointLimits.max_position << "]");
            if (jointLimits.has_velocity_limits)
                ROS_DEBUG_STREAM(ns + "Joint " << jointNames[joint_id] << " has URDF velocity limit ["
                                                       << jointLimits.max_velocity << "]");
        }
        else
        {
            if (urdf_joint->type != urdf::Joint::CONTINUOUS)
                ROS_WARN_STREAM(ns + "Joint " << jointNames[joint_id] << " does not have a URDF "
                        "position limit");
        }

        // Get limits from ROS param
        if (useROSParamJointLimits)
        {
            if (joint_limits_interface::getJointLimits(jointNames[joint_id], nh, jointLimits))
            {
                hasJointLimits = true;
                ROS_DEBUG_STREAM(ns + " Joint " << jointNames[joint_id] << " has rosparam position limits ["
                                                << jointLimits.min_position << ", " << jointLimits.max_position << "]");
                if (jointLimits.has_velocity_limits)
                    ROS_DEBUG_STREAM("Namespace: " + ns +  " Joint " << jointNames[joint_id]
                                                           << " has rosparam velocity limit ["
                                                           << jointLimits.max_velocity << "]");
            }  // the else debug message provided internally by joint_limits_interface
        }

        // Get soft limits from URDF
        if (useSoftLimitsIfAvailable)
        {
            if (joint_limits_interface::getSoftJointLimits(urdf_joint, softLimits))
            {
                hasSoftLimits = true;
                ROS_DEBUG_STREAM(ns + ": Joint " << jointNames[joint_id] << " has soft joint limits.");
            }
            else
            {
                ROS_DEBUG_STREAM(ns + ": Joint " << jointNames[joint_id] << " does not have soft joint "
                        "limits");
            }
        }

        // Quit we we haven't found any limits in URDF or rosparam server
        if (!hasJointLimits)
        {
            ROS_WARN_STREAM(ns + ": Could not find any limist on joint " << jointNames[joint_id]);
            return;
        }

        // Copy position limits if available
        if (jointLimits.has_position_limits)
        {
            // Slighly reduce the joint limits to prevent floating point errors
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
            ROS_INFO_STREAM_ONCE(ns +  ": Using soft saturation limits on Joints");
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
            ROS_INFO_STREAM_ONCE(ns + ": Using saturation limits (not soft limits) on Joint");

            const joint_limits_interface::PositionJointSaturationHandle satHandlePosition(jointHandlePosition, jointLimits);
            positionJointSatLimitsInterface.registerHandle(satHandlePosition);

            const joint_limits_interface::EffortJointSaturationHandle satHandleEffort(jointHandleEffort, jointLimits);
            effortJointSatLimitsInterface.registerHandle(satHandleEffort);

            const joint_limits_interface::VelocityJointSaturationHandle satHandleVelocity(jointHandleVelocity, jointLimits);
            velocityJointSatLimitsInterface.registerHandle(satHandleVelocity);
        }

    }

}