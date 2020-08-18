//
// Created by puze on 10.08.20.
//

#ifndef PROJECT_CONFIGURATION_MANAGER_H
#define PROJECT_CONFIGURATION_MANAGER_H

#include <ros/ros.h>
#include <string>

#include "configuration_client.h"

#include "iiwas_srv/CancelMotion.h"
#include "iiwas_srv/StartHandguiding.h"
#include "iiwas_srv/StartPositionControl.h"
#include "iiwas_srv/PTP.h"
#include "iiwas_srv/SetBlueLight.h"
#include "iiwas_srv/SetESMState.h"

class ConfigurationManager{
public:
    ConfigurationManager(bool useFrontIiwa, bool useBackIiwa);

    ~ConfigurationManager();

    bool init();

    bool startPositionControl();
    bool startFrontPositionControl();
    bool startBackPositionControl();

    bool cancelMotion(iiwas_srv::CancelMotion::Request &req, iiwas_srv::CancelMotion::Response &res);
    bool startHandguiding(iiwas_srv::StartHandguiding::Request &req, iiwas_srv::StartHandguiding::Response &res);
    bool startPositionCtrl(iiwas_srv::StartPositionControl::Request &req,
                              iiwas_srv::StartPositionControl::Response &res);
    bool ptp(iiwas_srv::PTP::Request &req, iiwas_srv::PTP::Response &res);
    bool setBlueLight(iiwas_srv::SetBlueLight::Request &req, iiwas_srv::SetBlueLight::Response &res);
    bool setESMState(iiwas_srv::SetESMState::Request &req, iiwas_srv::SetESMState::Response &res);

private:

    void initCommand();

    void jointStateCallback();

    struct ConfigurationData{
        ConfigurationData(std::string ns);
        std::string ns;
        std::vector<double> jointStiffness;
        std::vector<double> jointDamping;
    };

    ConfigurationClient* constructConfClient(std::string ns);

    bool init(ConfigurationClient* confClient, ConfigurationData* confData);

    ros::NodeHandle nh;

    ConfigurationClient* frontClient;
    ConfigurationClient* backClient;

    ConfigurationData* frontData;
    ConfigurationData* backData;

    ros::ServiceServer cancelMotionSrv;
    ros::ServiceServer startHandguidingSrv;
    ros::ServiceServer startPositionControlSrv;
    ros::ServiceServer setBlueLightSrv;
    ros::ServiceServer ptpSrv;
    ros::ServiceServer setESMStateSrv;

};

#endif //PROJECT_CONFIGURATION_MANAGER_H
