//
// Created by arenz on 29.05.17.
// Modified by puze on 31.07.20
//

#include "configuration_client.h"
#include <iostream>

using namespace boost::asio;
using ip::udp;

const int timeout = 200;

ConfigurationClient::ConfigurationClient(std::string ip, int port) :  socket(io_service){
    this->ip = ip;
    this->port = port;
    started = false;
}

ConfigurationClient::~ConfigurationClient() {
  socket.close();
}

bool ConfigurationClient::connectToServer() {
    auto address = boost::asio::ip::address::from_string(ip);
    auto endpoint = boost::asio::ip::tcp::endpoint(address, port);
    try{
        socket.connect(endpoint);

        ::setsockopt(socket.native_handle(), SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout, sizeof timeout);//SO_SNDTIMEO for send op
        started = true;
        return true;
    }
    catch (boost::system::system_error e){
        std::cerr << e.what() << std::endl;
        return false;
    }
}

bool ConfigurationClient::read(std::string& reply){
    boost::system::error_code error;
    boost::asio::streambuf receive_buffer;
    boost::asio::read_until(socket, receive_buffer, "\n", error);
    if( error && error != boost::asio::error::eof ) {
        std::cerr << "receive failed: " << error.message() << std::endl;
        return false;
    }

    boost::asio::streambuf::const_buffers_type bufs = receive_buffer.data();
    reply = std::string(boost::asio::buffers_begin(bufs),
                    boost::asio::buffers_begin(bufs) + receive_buffer.size());
    return true;
}

bool ConfigurationClient::write(std::string msg){
    boost::system::error_code error;
//    socket.send( boost::asio::buffer(msg + "\n"), endpoint, 0, error );
    boost::asio::write(socket, boost::asio::buffer(msg + "\n"), error);
    if(error) {
        std::cerr << "Sending " << msg << " failed. Error: " << error.message() << std::endl;
        return false;
    }
    return true;
}

bool ConfigurationClient::communicate(std::string cmd, std::string params, std::string* response){
    std::string reply;

    if(!write(cmd + params)) {
        return false;
    }

    if(!read(reply)) {
        return false;
    }

    if(response != nullptr){
        *response = reply;
    }

    if(reply.find("OK " + cmd) == 0) {
        return true;
    }else{
        std::cerr << cmd << " failed: " << reply << std::endl;
        return false;
    }
}

int ConfigurationClient::checkReferencing() {
    std::string cmd = "CHECK_REF";
    std::string response;

    if(communicate(cmd, "", &response)) {
        if (response.find("OK CHECK_REF 1") == 0) return 1;
        else if (response.find("OK CHECK_REF 0") == 0) return 0;
    }

    return -1;
}

bool ConfigurationClient::performSensorReferencing() {
    std::string cmd = "POS_AND_GMS_REF";
    return communicate(cmd);
}

bool ConfigurationClient::startFRI() {
    std::string cmd = "START_FRI";
    return communicate(cmd);
}

bool ConfigurationClient::cancelMotion() {
    std::string cmd = "CANCEL_MOTION";
    return communicate(cmd);
}

bool ConfigurationClient::attachSake() {
    std::string cmd = "ATTACH_SAKE";
    return communicate(cmd);
}

bool ConfigurationClient::checkConnectionQuality() {
    std::string cmd = "CHECK_CONNECTION_QUALITY";
    std::string response;

    if(communicate(cmd, "", &response))
    {
        if (response.find("EXCELLENT") != std::string::npos)
            return true;
    }

    return false;
}

bool ConfigurationClient::getStiffness(double* stiffness){
    std::string cmd = "GET_JOINT_STIFFNESS";
    std::string response;

    if(communicate(cmd, "", &response)){
        std::cout << response;
        return true;
    } else{
        return false;
    }
}

bool ConfigurationClient::getDamping(double* damping){
    std::string cmd = "GET_JOINT_DAMPING";
    std::string response;

    if(communicate(cmd, "", &response)){
        std::cout << response;
        return true;
    } else{
        return false;
    }
}

bool ConfigurationClient::ptp(std::vector<double> goalPos){
    std::string cmd = "PTP";
    std::stringstream params;
    params << " " << goalPos[0] * 180 / M_PI
           << " " << goalPos[1] * 180 / M_PI
           << " " << goalPos[2] * 180 / M_PI
           << " " << goalPos[3] * 180 / M_PI
           << " " << goalPos[4] * 180 / M_PI
           << " " << goalPos[5] * 180 / M_PI
           << " " << goalPos[6] * 180 / M_PI;

    return communicate(cmd, params.str());
}

bool ConfigurationClient::setStiffness(double* stiffness){
    std::string cmd = "SET_JOINT_STIFFNESS";
    std::stringstream param;
    param << " " << stiffness[0]
          << " " << stiffness[1]
          << " " << stiffness[2]
          << " " << stiffness[3]
          << " " << stiffness[4]
          << " " << stiffness[5]
          << " " << stiffness[6];

    return communicate(cmd, param.str());
}

bool ConfigurationClient::setDamping(double* damping){
    std::string cmd = "SET_JOINT_DAMPING";
    std::stringstream params;
    params << " " << damping[0]
           << " " << damping[1]
           << " " << damping[2]
           << " " << damping[3]
           << " " << damping[4]
           << " " << damping[5]
           << " " << damping[6];

    return communicate(cmd, params.str());
}

bool ConfigurationClient::startJointImpedanceCtrlMode(){
    std::string cmd = "JOINT_IMPEDANCE_MODE";
    return communicate(cmd);
}

bool ConfigurationClient::startJointPositionCtrlMode(){
    std::string cmd = "JOINT_POSITION_MODE";
    return communicate(cmd);
}

bool ConfigurationClient::setESMState(int state){
    std::string cmd = "SET_ESM";
    std::stringstream params;
    params << " " << state;

    return communicate(cmd, params.str());
}

bool ConfigurationClient::startHandguiding() {
    std::string cmd = "HANDGUIDING";
    return communicate(cmd);
}

bool ConfigurationClient::setBlueLight(bool enabled) {
    std::string cmd = "TURN_BLUE";
    std::string params = enabled ? " 1" : " 0";
    return communicate(cmd, params);
}

bool ConfigurationClient::startPositionControl(){
    // Never wait for response, the connection will die!!
    std::string cmd = "POSITION_CONTROL";
    return write(cmd);
}

bool ConfigurationClient::closeConnection(){
    std::string cmd = "CLOSE_CONNECTION";
    return write(cmd);
}
