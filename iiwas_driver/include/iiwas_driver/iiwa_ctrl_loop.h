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

#ifndef _IIWA_CTRL_LOOP_H
#define _IIWA_CTRL_LOOP_H


#include "iiwas_driver/iiwa_hw_interface.h"
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace iiwa_hw {

    class ControlLoop
    {
    public:
        ControlLoop(int coreId);

        void start();
        void stop();

        inline void resetControllers() { reset = true; /*hardwareInterface.resetCommand();*/ }


    protected:
        bool init();
        void run();
        void controlThread();


        int bindThreadToCore(int coreId);

        ros::NodeHandle nh;

        int coreId;

        boost::shared_ptr<controller_manager::ControllerManager> controllerManager;

        iiwa_hw::HardwareInterface hardwareInterface;

        //Timing
        ros::Duration elapsedTime;
        ros::Time oldTime;

        boost::thread* thread;
        bool running;
        bool reset;

    };

}

#endif //_IIWA_CTRL_LOOP_H
