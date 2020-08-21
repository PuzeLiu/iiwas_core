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

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"


int main(int argc, char* argv[]){
    ros::init(argc, argv, "iiwas_mask", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<sensor_msgs::JointState>("joint_states",1);

    bool useFrontIiwa, useBackIiwa;

    if(argc != 3) {
        ROS_ERROR_STREAM("Missing command line parameters, " << argc - 1 << " provided, need 2");
        return -1;
    }

    useFrontIiwa = std::string("true") == argv[1];
    useBackIiwa = std::string("true") == argv[2];

    if (!useFrontIiwa && !useBackIiwa){
        ROS_ERROR_STREAM("At least one Real robot should be chosen. Otherwise please use other simulation!");
        return -1;
    }

    if (useFrontIiwa && useBackIiwa){
        return -1;
    }

    ros::Rate rate(500);

    sensor_msgs::JointState msg;

    if(!useFrontIiwa){
        for(int i=0; i<7; i++){
            std::stringstream ss;
            ss << "F_joint_" << i + 1;
            msg.name.push_back(ss.str());
            msg.position.push_back(0.0);
            msg.velocity.push_back(0.0);
            msg.effort.push_back(0.0);
        }
    }

    if(!useBackIiwa){
        for(int i=0; i<7; i++){
            std::stringstream ss;
            ss << "B_joint_" << i + 1;
            msg.name.push_back(ss.str());
            msg.position.push_back(0.0);
            msg.velocity.push_back(0.0);
            msg.effort.push_back(0.0);
        }
    }

    while (ros::ok()){
        msg.header.stamp = ros::Time::now();
        pub.publish(msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
