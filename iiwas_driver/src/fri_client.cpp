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

#include <cstring>
#include <cstdio>
#include <cmath>
#include <iostream>

#include "iiwas_driver/fri_client.hpp"

using namespace KUKA::FRI;


FRIClient::FRIClient()
{
    for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){
       joint_torques_des[i] = 0.0;
       joint_pos_des[i] = 0.0;
    }

    state = -1;

    latest_measured_joint_pos = nullptr;
    latest_measured_joint_torque = nullptr;
    latest_measured_external_torque = nullptr;

}


FRIClient::~FRIClient()
{
}
      

void FRIClient::onStateChange(ESessionState oldState, ESessionState newState)
{
	state = newState;
    std::string nameOld, nameNew;

    switch (oldState)
    {
        case IDLE:
            nameOld = "IDLE";
            break;
        case MONITORING_WAIT:
            nameOld = "MONITORING_WAIT";
            break;
        case MONITORING_READY:
            nameOld = "MONITORING_READY";
            break;
        case COMMANDING_WAIT:
            nameOld = "COMMANDING_WAIT";
            break;
        case COMMANDING_ACTIVE:
            nameOld = "COMMANDING_ACTIVE";
            break;
    }

    switch (newState)
    {
        case IDLE:
            nameNew = "IDLE";
            break;
        case MONITORING_WAIT:
            nameNew = "MONITORING_WAIT";
            break;
        case MONITORING_READY:
            for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){
                joint_torques_des[i] = 0.0;
            }
            nameNew = "MONITORING_READY";
            break;
        case COMMANDING_WAIT:
            nameNew = "COMMANDING_WAIT";
            break;
        case COMMANDING_ACTIVE:
            nameNew = "COMMANDING_ACTIVE";
            break;

      default:
         break;
   }

    std::cout << "state changed from " << nameOld << " to " << nameNew << std::endl;
}

void FRIClient::monitor() {
  latest_measured_joint_pos = robotState().getMeasuredJointPosition();
  latest_measured_joint_torque = robotState().getMeasuredTorque();
  latest_measured_external_torque = robotState().getExternalTorque();
}

void FRIClient::waitForCommand()
{
   latest_measured_joint_pos = robotState().getMeasuredJointPosition();

   latest_measured_joint_torque = robotState().getMeasuredTorque();
   latest_measured_external_torque = robotState().getExternalTorque();


   // In waitForCommand(), the joint values have to be mirrored. Which is done, by calling
   // the base method.
   LBRClient::waitForCommand();
   
   // If we want to command torques, we have to command them all the time; even in
   // waitForCommand(). This has to be done due to consistency checks. In this state it is 
   // only necessary, that some torque values are sent. The LBR does not take the 
   // specific value into account.
   if (robotState().getClientCommandMode() == TORQUE)
   {
      robotCommand().setTorque(joint_torques_des);
   }
}

void FRIClient::command()
{
    latest_measured_joint_pos = robotState().getMeasuredJointPosition();
    latest_measured_joint_torque = robotState().getMeasuredTorque();
    latest_measured_external_torque = robotState().getExternalTorque();

    // In command(), the joint values have to be sent. Which is done by calling
    // the base method.
    LBRClient::command();

    // Check for correct ClientCommandMode.
    switch (robotState().getClientCommandMode()) {
        case POSITION:
            robotCommand().setJointPosition(joint_pos_des);
            break;
        case TORQUE:
            robotCommand().setTorque(joint_torques_des);
            robotCommand().setJointPosition(joint_pos_des);
            break;
        default:
            break;
    }
}
