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

#include "configuration_manager.h"

#include <iiwa_fri_client/friLBRState.h>

using namespace KUKA::FRI;

ConfigurationManager::ConfigurationManager(iiwa_hw::ControlLoop* frontLoop, iiwa_hw::ControlLoop* backLoop) :
        frontLoop(frontLoop), backLoop(backLoop){
    frontClient = nullptr;
    backClient = nullptr;

    if(frontLoop) {
        frontData = new ConfigurationData("iiwa_front");
        frontClient = constructConfClient(frontData->ns);
    }

    if(backLoop){
        backData = new ConfigurationData("iiwa_back");
        backClient = constructConfClient(backData->ns);
    }

    cancelMotionSrv = nh.advertiseService("cancel_motion", &ConfigurationManager::cancelMotion,
                                          this);
    startHandguidingSrv = nh.advertiseService("start_handguiding", &ConfigurationManager::startHandguiding,
                                              this);
    startPositionControlSrv = nh.advertiseService("start_position_control", &ConfigurationManager::startPositionCtrl,
                                                 this);
    setBlueLightSrv = nh.advertiseService("set_blue_light", &ConfigurationManager::setBlueLight,
                                          this);
    ptpSrv = nh.advertiseService("ptp", &ConfigurationManager::ptp, this);
    setESMStateSrv = nh.advertiseService("set_esm_state", &ConfigurationManager::setESMState, this);
}

ConfigurationManager::~ConfigurationManager() {
    if (frontClient) {
        if (frontClient->isMotionActive())
            frontClient->cancelMotion();

        if (frontClient->isConnected())
            frontClient->closeConnection();

        delete frontData;
        delete frontClient;
    }

    if (backClient) {
        if (backClient->isMotionActive())
            backClient->cancelMotion();

        if (backClient->isConnected())
            backClient->closeConnection();

        delete backData;
        delete backClient;
    }
}

ConfigurationManager::ConfigurationData::ConfigurationData(std::string ns) : ns("/" + ns) {
    jointStiffness.resize(LBRState::NUMBER_OF_JOINTS);
    jointDamping.resize(LBRState::NUMBER_OF_JOINTS);
}

bool ConfigurationManager::startPositionControl() {
    bool status = true;

    if (frontClient)
        status = frontClient->startPositionControl();

    if (backClient)
        status = status && backClient->startPositionControl();

    return status;
}

void ConfigurationManager::stopMotion(){
    if (frontClient)
        frontClient->cancelMotion();

    if (backClient)
        backClient->cancelMotion();
}

ConfigurationClient* ConfigurationManager::constructConfClient(std::string ns){
    std::string confServerIP;
    int confServerPort;

    if (!nh.param<std::string>(ns + "/conf_ip", confServerIP, "172.31.1.148"))
        ROS_WARN_STREAM_ONCE(ns + ": Unable to load configuration server ip from Parameter Server, use Default: "
                                     << confServerIP);
    if (!nh.param(ns + "/conf_port", confServerPort, 30402))
        ROS_WARN_STREAM_ONCE(ns + ": Unbale to load configuration server port from Parameter Server, use Default: "
                                     << confServerPort);
    return new ConfigurationClient(confServerIP, confServerPort);
}

bool ConfigurationManager::init(){
    bool success = true;
    if (frontClient)
        success = init(frontClient, frontData);

    if (backClient)
        success = success && init(backClient, backData);

    return success;

}

bool ConfigurationManager::init(ConfigurationClient* confClient, ConfigurationData* confData) {
    ros::NodeHandle n_p("~");

    std::vector<double> init_pos;
    init_pos.resize(LBRState::NUMBER_OF_JOINTS, 0.0);
    if (!n_p.getParam(confData->ns + "/init_position", init_pos)){
        ROS_WARN_STREAM(confData->ns + ": Unable to load joint initial position from Parameter Server, Use Default: \n"
                                << init_pos[0] << ", "
                                << init_pos[1] << ", "
                                << init_pos[2] << ", "
                                << init_pos[3] << ", "
                                << init_pos[4] << ", "
                                << init_pos[5] << ", "
                                << init_pos[6]);
    }


    if(!n_p.getParam(confData->ns + "/stiffness", confData->jointStiffness)){
        ROS_ERROR_STREAM_ONCE(confData->ns + ": Fail to load the stiffness of the arm");
        return false;
    }
    if(!n_p.getParam(confData->ns + "/damping", confData->jointDamping)){
        ROS_ERROR_STREAM_ONCE(confData->ns + ": Fail to load the damping of the arm");
        return false;
    }

    if (!confClient->connectToServer()){
        ROS_ERROR_STREAM(confData->ns + ": Failed to connect Configuration Server");
        return false;
    }

    /** Check the iiwa referencing status */
    int referenced = confClient->checkReferencing();
    switch (referenced) {
        case 0:
            ROS_INFO_STREAM(confData->ns + ": Start Position and GMS referencing");
            if (!confClient->performSensorReferencing())
                return false;
            break;

        case 1:
            ROS_INFO_STREAM(confData->ns + ": Robot is already position and GMS referenced");
            break;
        case -1:
            ROS_INFO_STREAM(confData->ns + ": Failure in check referencing");
            return false;

        default:
            ROS_INFO_STREAM(confData->ns + ": Unexpected return code in check referencing");
            return false;
    }


    ROS_INFO_STREAM(confData->ns << ": Go to default Position: "
                       << init_pos[0] << ", "
                       << init_pos[1] << ", "
                       << init_pos[2] << ", "
                       << init_pos[3] << ", "
                       << init_pos[4] << ", "
                       << init_pos[5] << ", "
                       << init_pos[6]);

    if (!confClient->ptp(init_pos))
        return false;

    if (!confClient->waitMotionEnd())
        return false;

    ROS_INFO_STREAM(confData->ns + ": Start FRI");
    if (!confClient->startFRI())
        return false;

    ros::Duration(2).sleep();

    if (!confClient->checkConnectionQuality()) {
        ROS_ERROR_STREAM(confData->ns + ": Checking connection quality failed");
        return false;
    }


//    if (!confClient->startJointImpedanceCtrlMode()) {
//        ROS_ERROR_STREAM(confData->ns + ": Starting impedance control mode failed");
//        return false;
//    }
//
//    if (!confClient->setStiffness(&confData->jointStiffness[0])) {
//        ROS_ERROR_STREAM(confData->ns + ": Setting stiffness failed");
//        return false;
//    }
//
//    if (!confClient->setDamping(&confData->jointDamping[0])) {
//        ROS_ERROR_STREAM(confData->ns + ": Setting damping failed");
//        return false;
//    }

    if (!confClient->startJointPositionCtrlMode()) {
        ROS_ERROR_STREAM(confData->ns + ": Starting position control mode failed");
        return false;
    }

    return true;
}

bool ConfigurationManager::cancelMotion(iiwas_srv::CancelMotion::Request &req,
                                        iiwas_srv::CancelMotion::Response &res) {
    res.success = false;

    std::stringstream ss;
    ss << "Iiwa: " << (req.which_iiwa == 1 ? "Front" : "Back") << " | ";

    if (req.which_iiwa==1 && frontClient){
        res.success = frontClient->cancelMotion();
        ss << frontClient->getLastResponse();
    } else if (req.which_iiwa==2 && backClient){
        res.success = backClient->cancelMotion();
        ss << backClient->getLastResponse();
    } else {
        return false;
    }

    res.msg = ss.str();
    return true;
}

bool ConfigurationManager::startHandguiding(iiwas_srv::StartHandguiding::Request &req,
                                            iiwas_srv::StartHandguiding::Response &res){
    res.success = false;

    std::stringstream ss;
    ss << "Iiwa: " << (req.which_iiwa == 1 ? "Front" : "Back") << " | ";

    if (req.which_iiwa==1 && frontClient){
        res.success = frontClient->startHandguiding();
        ss << frontClient->getLastResponse();
    } else if (req.which_iiwa==2 && backClient){
        res.success = backClient->startHandguiding();
        ss << backClient->getLastResponse();
    } else {
        return false;
    }

    res.msg = ss.str();
    return true;
}

bool ConfigurationManager::startPositionCtrl(iiwas_srv::StartPositionControl::Request &req,
                                                iiwas_srv::StartPositionControl::Response &res){
    res.success = false;

    std::stringstream ss;
    ss << "Iiwa: " << (req.which_iiwa == 1 ? "Front" : "Back") << " | ";

    if (req.which_iiwa==1 && frontClient){
        frontLoop->resetControllers();
        sleep(1.0);
        res.success = frontClient->startPositionControl();
        ss << frontClient->getLastResponse();
    } else if (req.which_iiwa==2 && backClient){
        backLoop->resetControllers();
        sleep(1.0);
        res.success = backClient->startPositionControl();
        ss << backClient->getLastResponse();
    } else {
        return false;
    }

    res.msg = ss.str();
    return true;
}

bool ConfigurationManager::ptp(iiwas_srv::PTP::Request &req, iiwas_srv::PTP::Response &res){
    res.success = false;

    std::stringstream ss;
    ss << "Iiwa: " << (req.which_iiwa == 1 ? "Front" : "Back") << " | ";

    std::vector<double> goalVec;
    if(!req.goal.size() == LBRState::NUMBER_OF_JOINTS){
        res.success = false;
        return res.success;
    } else{
        goalVec.resize(LBRState::NUMBER_OF_JOINTS);
        for (int i=0; i < LBRState::NUMBER_OF_JOINTS; i++)
            goalVec[i] = req.goal[i];
    }

    if (req.which_iiwa==1 && frontClient){
        res.success = frontClient->ptp(req.goal);
        ss << frontClient->getLastResponse();
    } else if (req.which_iiwa==2 && backClient){
        res.success = backClient->ptp(req.goal);
        ss << backClient->getLastResponse();
    } else {
        return false;
    }

    res.msg = ss.str();
    return true;
}

bool ConfigurationManager::setBlueLight(iiwas_srv::SetBlueLight::Request &req, iiwas_srv::SetBlueLight::Response &res){
    res.success = false;

    std::stringstream ss;
    ss << "Iiwa: " << (req.which_iiwa == 1 ? "Front" : "Back") << " | ";

    if (req.which_iiwa==1 && frontClient){
        res.success = frontClient->setBlueLight(req.on);
        ss << frontClient->getLastResponse();
    } else if (req.which_iiwa==2 && backClient){
        res.success = backClient->setBlueLight(req.on);
        ss << backClient->getLastResponse();
    } else {
        return false;
    }

    res.msg = ss.str();
    return true;
}

bool ConfigurationManager::setESMState(iiwas_srv::SetESMState::Request &req, iiwas_srv::SetESMState::Response &res){
    res.success = false;

    std::stringstream ss;
    ss << "Iiwa: " << (req.which_iiwa == 1 ? "Front" : "Back") << " | ";

    if (req.which_iiwa==1 && frontClient){
        res.success = frontClient->setESMState(req.state);
        ss << frontClient->getLastResponse();
    } else if (req.which_iiwa==2 && backClient){
        res.success = backClient->setESMState(req.state);
        ss << backClient->getLastResponse();
    } else {
        return false;
    }

    res.msg = ss.str();
    return true;
}
