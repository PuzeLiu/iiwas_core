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

#include "configuration_client.h"
#include <iostream>
#include <boost/timer.hpp>
#include <boost/bind.hpp>

using namespace boost::asio;
using ip::udp;

const int timeout = 200;

ConfigurationClient::ConfigurationClient(std::string ip, int port) :  socket(io_service){
    this->ip = ip;
    this->port = port;
    connected = false;
    motionActive = false;
}

ConfigurationClient::~ConfigurationClient() {
  socket.close();
}

bool ConfigurationClient::connectToServer() {
    auto address = boost::asio::ip::address::from_string(ip);
    auto endpoint = boost::asio::ip::tcp::endpoint(address, port);
    boost::asio::deadline_timer deadline(io_service);

    deadline.expires_from_now(boost::posix_time::seconds(5));
    socket.async_connect(endpoint,
                         boost::bind(&ConfigurationClient::connectHandle, this,
                                     boost::asio::placeholders::error));

    bool timeout = false;
    deadline.async_wait(boost::bind(&ConfigurationClient::timerHandle, this,
                                    boost::asio::placeholders::error, boost::ref(timeout)));

    do {
        io_service.run_one();
    } while (socket.is_open() && !timeout && !connected);

    return connected;
}

void ConfigurationClient::connectHandle(const boost::system::error_code &ec) {
    if(!ec)
        connected = true;
    return;
}

void ConfigurationClient::timerHandle(const boost::system::error_code &ec, bool &timeout) {
    if(!ec){
        timeout = true;
    }
    return;
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
    boost::asio::write(socket, boost::asio::buffer(msg + "\n"), error);
    if(error) {
        std::cerr << "Sending " << msg << " failed. Error: " << error.message() << std::endl;
        connected = false;
        return false;
    }
    return true;
}

bool ConfigurationClient::communicate(std::string cmd, std::string params){
    std::string reply;

    if(!write(cmd + params)) {
        return false;
    }

    if(!read(reply)) {
        return false;
    }

    reply.pop_back();
    reply.pop_back();
    last_response = reply;

    if(reply.find("OK " + cmd) == 0) {
        return true;
    }else{
        std::cout << cmd << " failed: " << reply;
        return false;
    }
}

int ConfigurationClient::checkReferencing() {
    std::string cmd = "CHECK_REF";

    if(communicate(cmd)) {
        if (last_response.find("OK CHECK_REF 1") == 0) return 1;
        else if (last_response.find("OK CHECK_REF 0") == 0) return 0;
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

    if (!communicate(cmd))
        return false;

    motionActive = false;
    return true;
}

bool ConfigurationClient::attachSake() {
    std::string cmd = "ATTACH_SAKE";
    return communicate(cmd);
}

bool ConfigurationClient::checkConnectionQuality() {
    std::string cmd = "CHECK_CONNECTION_QUALITY";

    if(communicate(cmd))
    {
        if (last_response.find("EXCELLENT") != std::string::npos)
            return true;
    }

    return false;
}

bool ConfigurationClient::getStiffness(double* stiffness){
    std::string cmd = "GET_JOINT_STIFFNESS";

    if(communicate(cmd)){
        std::cout << last_response;
        return true;
    } else{
        return false;
    }
}

bool ConfigurationClient::getDamping(double* damping){
    std::string cmd = "GET_JOINT_DAMPING";

    if(communicate(cmd)){
        std::cout << last_response;
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

    if( !communicate(cmd))
        return false;

    motionActive = true;
    return true;
}

bool ConfigurationClient::setBlueLight(bool enabled) {
    std::string cmd = "TURN_BLUE";
    std::string params = enabled ? " 1" : " 0";
    return communicate(cmd, params);
}

bool ConfigurationClient::startPositionControl(){
    std::string cmd = "POSITION_CONTROL";

    if(!communicate(cmd))
        return false;

    motionActive = true;
    return true;
}

bool ConfigurationClient::closeConnection(){
    std::string cmd = "CLOSE_CONNECTION";
    connected = false;
    return write(cmd);
}

bool ConfigurationClient::waitMotionEnd() {
    std::string cmd = "WAIT_MOTION";
    return communicate(cmd);
}
