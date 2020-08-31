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

    ConfigurationManager configurationManager(frontControlLoop, backControlLoop);

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

    configurationManager.stopMotion();

    if (useFrontIiwa)
        frontControlLoop->stop();

    if (useBackIiwa)
        backControlLoop->stop();

    ROS_INFO_STREAM("Killing and Exit");
}
