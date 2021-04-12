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

#ifndef _CONFIGURATION_CLIENT_H
#define _CONFIGURATION_CLIENT_H

#include <string>
#include <boost/asio.hpp>


#include "iiwas_driver/iiwa_ros.h"


class ConfigurationClient {
  private:
    std::string ip;
    int port;
    boost::asio::io_service io_service;
    boost::asio::ip::tcp::socket socket;
    std::string last_response;


    bool read(std::string& reply);
    bool write(std::string msg);
    bool communicate(std::string cmd, std::string params="");
    void connectHandle(const boost::system::error_code& error);
    void timerHandle(const boost::system::error_code& error, bool &timeout);

    bool connected;
    bool motionActive;


  public:
    ConfigurationClient(std::string ip, int port);
    ~ConfigurationClient(void);

    bool connectToServer();
    bool closeConnection();
    int checkReferencing();
    bool performSensorReferencing();
    bool startFRI();
    bool setJointImpedanceCtrlMode();
    bool setJointPositionCtrlMode();
    bool setJointTorqueCtrlMode();
    bool startControl();
    bool cancelMotion();
    bool waitMotionEnd();
    bool getStiffness(double *stiffness);
    bool setStiffness(double *stiffness);
    bool getDamping(double *damping);
    bool setDamping(double *damping);
    bool ptp(std::vector<double> goalPos);
    bool checkConnectionQuality();
    bool attachSake();
    bool setESMState(int state);
    bool setBlueLight(bool enabled);
    bool startHandguiding();
    inline bool isConnected(){ return connected;}
    inline bool isMotionActive() { return motionActive;}
    inline std::string getLastResponse() { return last_response; }
};


#endif //_CONFIGURATION_CLIENT_H
