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

class ConfigurationManager{
public:
    ConfigurationManager(bool useFrontIiwa, bool useBackIiwa);

    ~ConfigurationManager();

    bool init();

    bool startPositionControl();
    bool startFrontPositionControl();
    bool startBackPositionControl();

    bool cancelMotionSrv(iiwas_srv::CancelMotion::Request &req, iiwas_srv::CancelMotion::Response &res);
    bool startHandguidingSrv(iiwas_srv::StartHandguiding::Request &req, iiwas_srv::StartHandguiding::Response &res);
    bool startPositionControlSrv(iiwas_srv::StartPositionControl::Request &req, iiwas_srv::StartPositionControl::Response &res);
    bool ptpSrv(iiwas_srv::PTP::Request &req, iiwas_srv::PTP::Response &res);
    bool setBlueLightSrv(iiwas_srv::SetBlueLight::Request &req, iiwas_srv::SetBlueLight::Response &res);

private:

    struct ConfigurationData{
        ConfigurationData(std::string ns);
        std::string ns;
        std::vector<double> jointStiffness;
        std::vector<double> jointDamping;
    };

    ConfigurationClient* constructConfClient(std::string ns);

    bool init(ConfigurationClient* confClient, ConfigurationData* confData);

    ros::NodeHandle n_p;

    ConfigurationClient* frontClient;
    ConfigurationClient* backClient;

    ConfigurationData* frontData;
    ConfigurationData* backData;


};

#endif //PROJECT_CONFIGURATION_MANAGER_H
