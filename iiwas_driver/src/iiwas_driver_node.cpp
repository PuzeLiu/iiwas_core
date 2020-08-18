//
// Created by puze on 30.07.20.
//
#include <iostream>

#include "iiwa_ctrl_loop.h"
#include "configuration_manager.h"


int main(int argc, char* argv[]){
    ros::init(argc, argv, "iiwas_drive", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    bool useFrontIiwa, useBackIiwa;

    if(argc != 3) {
        ROS_ERROR_STREAM("Missing command line parameters, " << argc - 1 << " provided, need 2");
        return -1;
    }

    useFrontIiwa = std::string("true") == argv[1];
    useBackIiwa = std::string("true") == argv[2];

    iiwa_hw::ControlLoop* frontControlLoop =  nullptr;
    iiwa_hw::ControlLoop* backControlLoop = nullptr;

    if (useFrontIiwa) {
        frontControlLoop = new iiwa_hw::ControlLoop("iiwa_front", 1);
        frontControlLoop->start();
    }

    if (useBackIiwa) {
        backControlLoop = new iiwa_hw::ControlLoop("iiwa_back", 2);
        backControlLoop->start();
    }

    ConfigurationManager configurationManager(useFrontIiwa, useBackIiwa);

    ROS_INFO_STREAM("Initializing Configuration Server");

    if (!configurationManager.init()) {
        ROS_ERROR_STREAM("Initialization failed");
        ros::shutdown();

        if (useFrontIiwa)
            frontControlLoop->stop();

        if (useBackIiwa)
            backControlLoop->stop();

        return -1;
    }

    if (!configurationManager.startPositionControl()) {
        ROS_ERROR_STREAM("Failed to start position control");
        ros::shutdown();

        if (useFrontIiwa)
            frontControlLoop->stop();

        if (useBackIiwa)
            backControlLoop->stop();

        return -1;
    }



    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    if (useFrontIiwa)
        frontControlLoop->stop();

    if (useBackIiwa)
        backControlLoop->stop();

    ROS_INFO_STREAM("Killing and Exit");
}
