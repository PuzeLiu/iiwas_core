//
// Created by puze on 10.08.20.
//
#include "configuration_manager.h"

#include <iiwa_fri_client/friLBRState.h>

using namespace KUKA::FRI;

ConfigurationManager::ConfigurationManager(bool useFrontIiwa, bool useBackIiwa){
    frontClient = nullptr;
    backClient = nullptr;

    if(useFrontIiwa) {
        frontData = new ConfigurationData("iiwa_front");
        frontClient = constructConfClient(frontData->ns);
    }

    if(useBackIiwa){
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

bool ConfigurationManager::startFrontPositionControl() {
    if (frontClient)
        return frontClient->startPositionControl();

    return false;
}

bool ConfigurationManager::startBackPositionControl(){
    if (backClient)
        return backClient->startPositionControl();

    return false;
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

    if (!confClient->ptp(init_pos)) return false;

    ROS_INFO_STREAM(confData->ns + ": Start FRI");
    if (!confClient->startFRI()) return false;

    ros::Duration(2).sleep();

    if (!confClient->checkConnectionQuality()) {
        ROS_ERROR_STREAM(confData->ns + ": Checking connection quality failed");
        return false;
    }


    if (!confClient->startJointImpedanceCtrlMode()) {
        ROS_ERROR_STREAM(confData->ns + ": Starting impedance control mode failed");
        return false;
    }

    if (!confClient->setStiffness(&confData->jointStiffness[0])) {
        ROS_ERROR_STREAM(confData->ns + ": Setting stiffness failed");
        return false;
    }

    if (!confClient->setDamping(&confData->jointDamping[0])) {
        ROS_ERROR_STREAM(confData->ns + ": Setting damping failed");
        return false;
    }

    return true;
}

bool ConfigurationManager::cancelMotion(iiwas_srv::CancelMotion::Request &req,
                                        iiwas_srv::CancelMotion::Response &res) {
    res.success = false;

    if (req.which_iiwa==1 && frontClient){
        res.success = frontClient->cancelMotion();
    }

    if (req.which_iiwa==2 && backClient){
        res.success = backClient->cancelMotion();
    }
    std::stringstream ss;
    ss << "Service: CancelMotion | Which iiwa: " << req.which_iiwa << " | Success: " << bool(res.success);
    res.msg = ss.str();
    return true;
}

bool ConfigurationManager::startHandguiding(iiwas_srv::StartHandguiding::Request &req,
                                            iiwas_srv::StartHandguiding::Response &res){
    res.success = false;

    if (req.which_iiwa==1 && frontClient){
        res.success = frontClient->startHandguiding();
    }

    if (req.which_iiwa==2 && backClient){
        res.success = backClient->startHandguiding();
    }

    std::stringstream ss;
    ss << "Service: StartHandguiding | Which iiwa: " << req.which_iiwa << " | Success: " << bool(res.success);
    res.msg = ss.str();
    return true;
}

bool ConfigurationManager::startPositionCtrl(iiwas_srv::StartPositionControl::Request &req,
                                                iiwas_srv::StartPositionControl::Response &res){
    res.success = false;

    if (req.which_iiwa==1 && frontClient){
        res.success = frontClient->startPositionControl();
    }

    if (req.which_iiwa==2 && backClient){
        res.success = backClient->startPositionControl();
    }

    std::stringstream ss;
    ss << "Service: StartPositionControl | Which iiwa: " << req.which_iiwa << " | Success: " << bool(res.success);
    res.msg = ss.str();
    return true;
}

bool ConfigurationManager::ptp(iiwas_srv::PTP::Request &req, iiwas_srv::PTP::Response &res){
    res.success = false;

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
    }

    if (req.which_iiwa==2 && backClient){
        res.success = backClient->ptp(req.goal);
    }

    std::stringstream ss;
    ss << "Service: PTP | Which iiwa: " << req.which_iiwa << " | Success: " << bool(res.success);
    res.msg = ss.str();
    return true;
}

bool ConfigurationManager::setBlueLight(iiwas_srv::SetBlueLight::Request &req, iiwas_srv::SetBlueLight::Response &res){
    res.success = false;

    if (req.which_iiwa==1 && frontClient){
        res.success = frontClient->setBlueLight(req.on);
    }

    if (req.which_iiwa==2 && backClient){
        res.success = backClient->setBlueLight(req.on);
    }

    std::stringstream ss;
    ss << "Service: SetBlueLight | Which iiwa: " << req.which_iiwa << " | Success: " << bool(res.success);
    res.msg = ss.str();
    return true;
}

bool ConfigurationManager::setESMState(iiwas_srv::SetESMState::Request &req, iiwas_srv::SetESMState::Response &res){
    res.success = false;

    if (req.which_iiwa==1 && frontClient){
        res.success = frontClient->setESMState(req.state);
    }

    if (req.which_iiwa==2 && backClient){
        res.success = backClient->setESMState(req.state);
    }

    std::stringstream ss;
    ss << "Service: SetESMState | Which iiwa: " << req.which_iiwa << " | Success: " << bool(res.success);
    res.msg = ss.str();
    return true;
}
