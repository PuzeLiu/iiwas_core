/**

DISCLAIMER OF WARRANTY

The Software is provided "AS IS" and "WITH ALL FAULTS," 
without warranty of any kind, including without limitation the warranties 
of merchantability, fitness for a particular purpose and non-infringement. 
KUKA makes no warranty that the Software is free of defects or is suitable 
for any particular purpose. In no event shall KUKA be responsible for loss 
or damages arising from the installation or use of the Software, 
including but not limited to any indirect, punitive, special, incidental 
or consequential damages of any character including, without limitation, 
damages for loss of goodwill, work stoppage, computer failure or malfunction, 
or any and all other commercial damages or losses. 
The entire risk to the quality and performance of the Software is not borne by KUKA. 
Should the Software prove defective, KUKA is not liable for the entire cost 
of any service and repair.


COPYRIGHT

All Rights Reserved
Copyright (C)  2014-2016 
KUKA Roboter GmbH
Augsburg, Germany

This material is the exclusive property of KUKA Roboter GmbH and must be returned 
to KUKA Roboter GmbH immediately upon request.  
This material and the information illustrated or contained herein may not be used, 
reproduced, stored in a retrieval system, or transmitted in whole 
or in part in any way - electronic, mechanical, photocopying, recording, 
or otherwise, without the prior written consent of KUKA Roboter GmbH.  






\file
\version {1.10}
*/
#include <cstring>
#include <cstdio>
// Visual studio needs extra define to use math constants
#define _USE_MATH_DEFINES
#include <cmath>
#include "fri_client.hpp"
#include <iostream>

using namespace KUKA::FRI;

//******************************************************************************
FRIClient::FRIClient()
{
    for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){
       joint_torques_des[i] = 0.0;
       joint_pos_des[i] = 0.0;
    }

    initialized = false;
    closing = false;
}

//******************************************************************************
FRIClient::~FRIClient()
{
}
      
//******************************************************************************
void FRIClient::onStateChange(ESessionState oldState, ESessionState newState)
{
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
            initialized = false;
            closing = true;
            nameNew = "IDLE";
            break;
        case MONITORING_WAIT:
            initialized = true;
            nameNew = "MONITORING_WAIT";
            break;
        case MONITORING_READY:
            initialized = true;
            for(int i = 0; i< LBRState::NUMBER_OF_JOINTS; i++){
                joint_torques_des[i] = 0.0;
            }

            nameNew = "MONITORING_READY";
            break;
        case COMMANDING_WAIT:
            initialized = true;
            nameNew = "COMMANDING_WAIT";
            break;
        case COMMANDING_ACTIVE:
            initialized = true;
            nameNew = "COMMANDING_ACTIVE";
            break;

      default:
         break;
   }

    std::cout << "state changed from " << nameOld << " to " << nameNew << std::endl;
}
//******************************************************************************
void FRIClient::monitor() {
  latest_measured_joint_pos = robotState().getMeasuredJointPosition();
  latest_measured_joint_torque = robotState().getMeasuredTorque();
  latest_measured_external_torque = robotState().getExternalTorque();
}
//******************************************************************************
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
//******************************************************************************
void FRIClient::command()
{
    latest_measured_joint_pos = robotState().getMeasuredJointPosition();
    latest_measured_joint_torque = robotState().getMeasuredTorque();
    latest_measured_external_torque = robotState().getExternalTorque();

    // In command(), the joint values have to be sent. Which is done by calling
    // the base method.
    LBRClient::command();

    // Check for correct ClientCommandMode.
    if (robotState().getClientCommandMode() == TORQUE)
    { 
       // Set superposed joint torques.
       robotCommand().setTorque(joint_torques_des);
       robotCommand().setJointPosition(joint_pos_des);
    }
}

// clean up additional defines
#ifdef _USE_MATH_DEFINES
#undef _USE_MATH_DEFINES
#endif
