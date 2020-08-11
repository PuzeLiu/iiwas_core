//
// Created by puze on 10.08.20.
//

#ifndef PROJECT_CONFIGURATION_MANAGER_H
#define PROJECT_CONFIGURATION_MANAGER_H

#include <ros/ros.h>
#include "configuration_client.h"

class ConfigurationManager{
public:
    ConfigurationManager(bool useFrontIiwa, bool useBackIiwa);

    ~ConfigurationManager();

    bool init();

    bool startPositionControl();
    bool startFrontPositionControl();
    bool startBackPositionControl();

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
