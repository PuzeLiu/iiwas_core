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

#ifndef _CONFIGURATION_MANAGER_H
#define _CONFIGURATION_MANAGER_H

#include <ros/ros.h>
#include <string>

#include "iiwas_srv/CancelMotion.h"
#include "iiwas_srv/StartHandguiding.h"
#include "iiwas_srv/StartPositionControl.h"
#include "iiwas_srv/PTP.h"
#include "iiwas_srv/SetBlueLight.h"
#include "iiwas_srv/SetESMState.h"
#include "iiwas_srv/SetImpedanceParam.h"

#include "configuration_client.h"
#include "iiwa_ctrl_loop.h"
#include "iiwa_fri_client/friClientIf.h"

class ConfigurationManager{
public:
    ConfigurationManager(iiwa_hw::ControlLoop* frontLoop, iiwa_hw::ControlLoop* backLoop);

    ~ConfigurationManager();

    bool init();

    bool startPositionControl();
    void stopMotion();

    bool cancelMotion(iiwas_srv::CancelMotion::Request &req, iiwas_srv::CancelMotion::Response &res);
    bool startHandguiding(iiwas_srv::StartHandguiding::Request &req, iiwas_srv::StartHandguiding::Response &res);
    bool startPositionCtrl(iiwas_srv::StartPositionControl::Request &req,
                              iiwas_srv::StartPositionControl::Response &res);
    bool ptp(iiwas_srv::PTP::Request &req, iiwas_srv::PTP::Response &res);
    bool setBlueLight(iiwas_srv::SetBlueLight::Request &req, iiwas_srv::SetBlueLight::Response &res);
    bool setESMState(iiwas_srv::SetESMState::Request &req, iiwas_srv::SetESMState::Response &res);
    bool setImpedanceParam(iiwas_srv::SetImpedanceParam::Request &req, iiwas_srv::SetImpedanceParam::Response &res);

private:

    struct ConfigurationData{
        ConfigurationData(std::string ns);
        std::string ns;
        int controlMode;
        std::vector<double> jointStiffness;
        std::vector<double> jointDamping;
    };

    ConfigurationClient* constructConfClient(std::string ns);

    bool init(ConfigurationClient* confClient, ConfigurationData* confData);

    ros::NodeHandle nh;

    int controlMode;

    ConfigurationClient* frontClient;
    ConfigurationClient* backClient;

    iiwa_hw::ControlLoop* frontLoop;
    iiwa_hw::ControlLoop* backLoop;

    ConfigurationData* frontData;
    ConfigurationData* backData;

    ros::ServiceServer cancelMotionSrv;
    ros::ServiceServer startHandguidingSrv;
    ros::ServiceServer startPositionControlSrv;
    ros::ServiceServer setBlueLightSrv;
    ros::ServiceServer ptpSrv;
    ros::ServiceServer setESMStateSrv;
    ros::ServiceServer setImpedanceParamSrv;

};

#endif //_CONFIGURATION_MANAGER_H
