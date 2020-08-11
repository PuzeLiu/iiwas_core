//
// Created by puze on 29.07.20.
//

#ifndef PROJECT_IIWAS_HW_INTERFACE_H
#define PROJECT_IIWAS_HW_INTERFACE_H

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

#include <iiwa_ros.h>
#include <urdf/model.h>

#include <fri_client.hpp>
#include <configuration_client.h>
#include <FRI/friUdpConnection.h>
#include <FRI/friClientApplication.h>


const int DEFAULT_CONTROL_FREQUENCY = 1000;  // Hz


namespace iiwa_hw {
    class HardwareInterface : public hardware_interface::RobotHW {
    public:
        HardwareInterface(ros::NodeHandle& nh, std::string ns);

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

        std::string getNamespace(){return ns; };

        bool isIiwaReady(){ return iiwaReady;};

        void stop(){
            stopFRI();
            ros::shutdown();
        };

    protected:

        void loadParam();

        bool initHardwareInterface();

        bool initFRI();

        void stopFRI();

        int loadURDF(ros::NodeHandle &nh, std::string param_name);

        virtual void registerJointLimits_(const hardware_interface::JointHandle &jointHandlePosition,
                                         const hardware_interface::JointHandle &jointHandleVelocity,
                                         const hardware_interface::JointHandle &jointHandleEffort,
                                         std::size_t joint_id);

    private:
        std::string ns;

        /** Robot Parameter */
        bool iiwaReady;

        std::vector<JState> jointState, jointStateLast;
        std::vector<DJState> jointCommand;

        bool useROSParamJointLimits;
        bool useSoftLimitsIfAvailable;

        std::vector<std::string> jointNames;

        std::vector<double> jointPositionLowerLimits;
        std::vector<double> jointPositionUpperLimits;
        std::vector<double> jointVelocityLimits;
        std::vector<double> jointEffortLimits;

        /** ROS Parameter */
        ros::NodeHandle nh;

        urdf::Model *urdfModel;

        std::string iiwaDescription;

        /** Hardward Interface*/
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

        FRIClient* friClient;
        KUKA::FRI::UdpConnection* friConnection;
        KUKA::FRI::ClientApplication* friApp;

        bool isAppServerStarted;

    };
}

#endif //PROJECT_IIWAS_HW_INTERFACE_H
