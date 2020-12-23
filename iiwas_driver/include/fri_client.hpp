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

#ifndef _FRI_CLIENT_H
#define _FRI_CLIENT_H

#include <iiwa_fri_client/friLBRClient.h>
#include <iiwa_fri_client/friLBRState.h>

enum ControlMode{
    POSITION_CONTROL = 0,
    IMPEDANCE_CONTROL = 1,
    TORQUE_CONTROL = 2,
    NO_CONTROL = 3
};

/**
 * \brief Test client that superposes joint torques with sine waves.
 */
class FRIClient : public KUKA::FRI::LBRClient
{
private:
//    bool initialized;
//    bool closing;
    int state;
   
public:
    const double *latest_measured_joint_pos;
    const double *latest_measured_joint_torque;
    const double *latest_measured_external_torque;

    double joint_pos_des[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
    double joint_torques_des[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];

    /**
     * \brief Constructor.
     *
     * @param jointMask Bit mask that encodes the joint indices to be overlaid by sine waves
     * @param freqHz Sine frequency in Hertz
     * @param torqueAmplitude Sine amplitude in Nm
     */
   FRIClient();
   
   /** 
    * \brief Destructor.
    */
   ~FRIClient();
   
   /**
    * \brief Callback for FRI state changes.
    * 
    * @param oldState
    * @param newState
    */
   virtual void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState);
    /**
    * \brief Callback for the FRI state 'Monitoring Wait' and 'Monitoring Ready'.
    */
   virtual void monitor();

   /**
    * \brief Callback for the FRI session state 'Commanding Wait'.
    */
   virtual void waitForCommand();
   
   /**
    * \brief Callback for the FRI state 'Commanding Active'.
    */
   virtual void command();


    inline bool isIdle() {
    	return state == KUKA::FRI::ESessionState::IDLE;
    }

    inline bool isMonitoringWait(){
    	return state == KUKA::FRI::ESessionState::MONITORING_WAIT;
    }

    inline bool isMonitoringReady(){
    	return state = KUKA::FRI::ESessionState::MONITORING_READY;
    }

    inline bool isDataAvailable(){
    	return state > KUKA::FRI::ESessionState::MONITORING_WAIT;
    }

    inline bool isCommandingWait(){
    	return state == KUKA::FRI::ESessionState::COMMANDING_WAIT;
    }

    inline bool isCommandingActive(){
    	return state == KUKA::FRI::ESessionState::COMMANDING_ACTIVE;
    }

    const int getState(){
    	return state;
    }

};

#endif // _FRI_CLIENT_H
