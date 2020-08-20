//
// Created by arenz on 29.05.17.
// Modified by puze on 29.07.20.
//

#ifndef ROBOLAB_FRICONFCONNECTION_H
#define ROBOLAB_FRICONFCONNECTION_H

#include <string>
#include <boost/asio.hpp>


#include "iiwa_ros.h"


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
    bool startJointImpedanceCtrlMode();
    bool startJointPositionCtrlMode();
    bool startPositionControl();
    bool cancelMotion();
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


#endif //ROBOLAB_FRICONFCONNECTION_H
