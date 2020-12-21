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
#include "iiwa_fri_client/friClientIf.h"

int main(int argc, char* argv[]){
    ros::init(argc, argv, "iiwas_drive", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    int coreID;
    if(!nh.getParam("core_id", coreID)){
        ROS_ERROR_STREAM("Fail to get the core ID for the driver");
        return -1;
    }

	iiwa_hw::ControlLoop controlLoop(coreID);
	controlLoop.start();

    ConfigurationManager configurationManager(controlLoop);

    ROS_INFO_STREAM(nh.getNamespace() + " Initializing Configuration Server");

    if (!configurationManager.init()) {
        ROS_ERROR_STREAM("Initialization failed");
        ros::shutdown();
        controlLoop.stop();
        return -1;
    }

    if (!configurationManager.startPositionControl()) {
        ROS_ERROR_STREAM("Failed to start position control");
        ros::shutdown();
        controlLoop.stop();
        return -1;
    }

    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    configurationManager.stop();

    controlLoop.stop();

    return -1;
}
