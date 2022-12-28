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

#include "iiwas_driver/configuration_manager.h"

using namespace KUKA::FRI;

ConfigurationManager::ConfigurationManager(iiwa_hw::ControlLoop &controlLoop_) :
		nh(), controlLoop(controlLoop_) {
	confClient = constructConfClient();

	cancelMotionSrv = nh.advertiseService("cancel_motion",
			&ConfigurationManager::cancelMotion, this);
	startHandguidingSrv = nh.advertiseService("start_handguiding",
			&ConfigurationManager::startHandguiding, this);
	startPositionControlSrv = nh.advertiseService("start_position_control",
			&ConfigurationManager::startPositionCtrl, this);
	setBlueLightSrv = nh.advertiseService("set_blue_light",
			&ConfigurationManager::setBlueLight, this);
	ptpSrv = nh.advertiseService("ptp", &ConfigurationManager::ptp, this);
	setESMStateSrv = nh.advertiseService("set_esm_state",
			&ConfigurationManager::setESMState, this);
	setImpedanceParamSrv = nh.advertiseService("set_impedance_parameter",
			&ConfigurationManager::setImpedanceParam, this);
}

ConfigurationManager::~ConfigurationManager() {
}

bool ConfigurationManager::startPositionControl() {
	bool status = true;

	if (confClient)
		status = confClient->startControl();

	return status;
}

void ConfigurationManager::stop() {
	if (confClient) {
		if (confClient->isMotionActive())
			confClient->cancelMotion();

		if (confClient->isConnected())
			confClient->closeConnection();

		delete confClient;
	}
}

bool ConfigurationManager::init() {
	bool success = true;
	if (confClient)
		success = init(confClient);
	return success;

}

ConfigurationClient* ConfigurationManager::constructConfClient() {
	std::string confServerIP;
	int confServerPort;

	if (!nh.getParam("conf_ip", confServerIP))
		ROS_WARN_STREAM_ONCE(
				nh.getNamespace() + " Unable to load configuration server ip from Parameter Server, use Default: " << confServerIP);
	if (!nh.getParam("conf_port", confServerPort))
		ROS_WARN_STREAM_ONCE(
				nh.getNamespace() + " Unable to load configuration server port from Parameter Server, use Default: " << confServerPort);
	return new ConfigurationClient(confServerIP, confServerPort);
}

bool ConfigurationManager::init(ConfigurationClient *confClient) {
	std::vector<double> init_pos;
	init_pos.resize(LBRState::NUMBER_OF_JOINTS, 0.0);
	if (!nh.getParam("init_position", init_pos)) {
		ROS_WARN_STREAM(
				nh.getNamespace() << " Unable to load joint initial position from Parameter Server, Use Default: " << std::endl << init_pos[0] << ", " << init_pos[1] << ", " << init_pos[2] << ", " << init_pos[3] << ", " << init_pos[4] << ", " << init_pos[5] << ", " << init_pos[6]);
	}

	if (!nh.getParam("stiffness", jointStiffness)) {
		ROS_ERROR_STREAM_ONCE(
				nh.getNamespace() + " Fail to load the stiffness of the arm");
		return false;
	}

	if (!nh.getParam("damping", jointDamping)) {
		ROS_ERROR_STREAM_ONCE(
				nh.getNamespace() + " Fail to load the damping of the arm");
		return false;
	}

	controlMode = ControlMode::IMPEDANCE_CONTROL;
	controlMode = nh.param("control_mode", controlMode);

	if (!confClient->connectToServer()) {
		ROS_ERROR_STREAM(
				nh.getNamespace() + " Failed to connect Configuration Server");
		return false;
	}

	/** Check the iiwa referencing status */
	int referenced = confClient->checkReferencing();
	switch (referenced) {
	case 0:
		ROS_INFO_STREAM(
				nh.getNamespace() + " Start Position and GMS referencing");
		if (!confClient->performSensorReferencing())
			return false;
		break;

	case 1:
		ROS_INFO_STREAM(
				nh.getNamespace()
						+ " Robot is already position and GMS referenced");
		break;
	case -1:
		ROS_INFO_STREAM(nh.getNamespace() + " Failure in check referencing");
		return false;

	default:
		ROS_INFO_STREAM(
				nh.getNamespace()
						+ " Unexpected return code in check referencing");
		return false;
	}

	ROS_INFO_STREAM(
			nh.getNamespace() << " Go to default Position: " << init_pos[0] << ", " << init_pos[1] << ", " << init_pos[2] << ", " << init_pos[3] << ", " << init_pos[4] << ", " << init_pos[5] << ", " << init_pos[6]);

	if (!confClient->ptp(init_pos))
		return false;

	if (!confClient->waitMotionEnd())
		return false;

	ROS_INFO_STREAM(nh.getNamespace() + " Start FRI");
	if (!confClient->startFRI())
		return false;

	ros::Duration(2).sleep();

	if (!confClient->checkConnectionQuality()) {
		ROS_ERROR_STREAM(
				nh.getNamespace() + " Checking connection quality failed");
		return false;
	}

	if (!setControlMode()){
        return false;
	}

	return true;
}

bool ConfigurationManager::setControlMode() {
	switch (controlMode) {
	case ControlMode::IMPEDANCE_CONTROL:
		if (!confClient->setJointImpedanceCtrlMode()) {
			ROS_ERROR_STREAM(
					nh.getNamespace() << " Setting impedance control mode failed");
			return false;
		}

		if (!confClient->setStiffness(&jointStiffness[0])) {
			ROS_ERROR_STREAM(nh.getNamespace() << ": Setting stiffness failed");
			return false;
		}

		if (!confClient->setDamping(&jointDamping[0])) {
			ROS_ERROR_STREAM(nh.getNamespace() << " Setting damping failed");
			return false;
		}
		break;

	case ControlMode::POSITION_CONTROL:
		if (!confClient->setJointPositionCtrlMode()) {
			ROS_ERROR_STREAM(
					nh.getNamespace() << " Setting position control mode failed");
			return false;
		}
		break;
	case ControlMode::TORQUE_CONTROL:
		if (!confClient->setJointTorqueCtrlMode()) {
			ROS_ERROR_STREAM(
					nh.getNamespace() << " Setting torque control mode failed");
			return false;
		}
		break;

	default:
		ROS_ERROR_STREAM(nh.getNamespace() << " Unknown control mode");
		return false;
	}

	return true;

}

bool ConfigurationManager::cancelMotion(iiwas_srv::CancelMotion::Request &req,
		iiwas_srv::CancelMotion::Response &res) {
	res.success = false;

	res.success = confClient->cancelMotion();
	res.msg = confClient->getLastResponse();
	return true;
}

bool ConfigurationManager::startHandguiding(
		iiwas_srv::StartHandguiding::Request &req,
		iiwas_srv::StartHandguiding::Response &res) {
	res.success = false;

	res.success = confClient->startHandguiding();
	res.msg = confClient->getLastResponse();
	return true;
}

bool ConfigurationManager::startPositionCtrl(
		iiwas_srv::StartPositionControl::Request &req,
		iiwas_srv::StartPositionControl::Response &res) {

	controlLoop.resetControllers();
	sleep(1.0);

	res.success = setControlMode() && confClient->startControl();
	res.msg = confClient->getLastResponse();

	return true;
}

bool ConfigurationManager::ptp(iiwas_srv::PTP::Request &req,
		iiwas_srv::PTP::Response &res) {
	res.success = false;

	std::vector<double> goalVec;
	if (!req.goal.size() == LBRState::NUMBER_OF_JOINTS) {
		res.success = false;
		return res.success;
	} else {
		goalVec.resize(LBRState::NUMBER_OF_JOINTS);
		for (int i = 0; i < LBRState::NUMBER_OF_JOINTS; i++)
			goalVec[i] = req.goal[i];
	}

	res.success = confClient->ptp(req.goal);
	res.msg = confClient->getLastResponse();
	return true;
}

bool ConfigurationManager::setBlueLight(iiwas_srv::SetBlueLight::Request &req,
		iiwas_srv::SetBlueLight::Response &res) {
	res.success = false;
	res.success = confClient->setBlueLight(req.on);
	res.msg = confClient->getLastResponse();
	return true;
}

bool ConfigurationManager::setESMState(iiwas_srv::SetESMState::Request &req,
		iiwas_srv::SetESMState::Response &res) {
	res.success = false;
	res.success = confClient->setESMState(req.state);
	res.msg = confClient->getLastResponse();
	return true;
}

bool ConfigurationManager::setImpedanceParam(
		iiwas_srv::SetImpedanceParam::Request &req,
		iiwas_srv::SetImpedanceParam::Response &res) {
	res.success = true;

	if (req.stiffness.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS
			|| req.stiffness.size() != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
		ROS_WARN_STREAM(
				KUKA::FRI::LBRState::NUMBER_OF_JOINTS << " joints of stiffness and damping is needed");
		return false;
	}
	jointStiffness = req.stiffness;
	jointDamping = req.damping;
	res.msg = "Joint impedance stored";

	return true;
}
