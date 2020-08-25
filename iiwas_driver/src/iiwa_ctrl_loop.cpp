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

#include <iiwa_ctrl_loop.h>

namespace iiwa_hw{
    ControlLoop::ControlLoop(std::string ns, int coreId)
    : hardwareInterface(nh, ns), coreId(coreId){
        controllerManager.reset(new controller_manager::ControllerManager(&hardwareInterface, nh));
        oldTime = ros::Time::now();

        thread = nullptr;
        running = false;
        reset = false;
    }

    void ControlLoop::start() {
        if(init()) {
            running = true;
            thread = new boost::thread(boost::bind(&iiwa_hw::ControlLoop::controlThread, this), coreId);
        }
    }

    void ControlLoop::stop() {
        running = false;
        if(thread)
            thread->join();
        hardwareInterface.stop();
    }

    bool ControlLoop::init(){
        return hardwareInterface.init();
    }

    void ControlLoop::run(){
        while (running  && hardwareInterface.isIiwaReady()){
            auto new_t = ros::Time::now();
            elapsedTime = new_t - oldTime;
            oldTime = new_t;
            // Input
            hardwareInterface.read(oldTime, elapsedTime);

            // Control
            controllerManager->update(oldTime, elapsedTime, reset);
            reset = false;

            // Output
            hardwareInterface.write(oldTime, elapsedTime);
        }
    }

    void ControlLoop::controlThread(){
        /** Set realtime priority for this thread */
        struct sched_param param;
        param.sched_priority = sched_get_priority_max(SCHED_RR);
        int retvalue = sched_setscheduler(0, SCHED_RR, &param);
        if (retvalue != 0) {
            ROS_ERROR_STREAM("Thread priority NOT set to max " << retvalue);
            return;
        }

        if(bindThreadToCore(coreId) != 0){
            ROS_ERROR_STREAM("Could not bind "<< nh.getNamespace() << " thread to core: " << coreId);
            return;
        }

        run();
    };

    int ControlLoop::bindThreadToCore(int coreId) {
        int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
        if (coreId < 0 || coreId >= num_cores)
            return EINVAL;

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(coreId, &cpuset);

        pthread_t current_thread = pthread_self();
        return pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    }
}

