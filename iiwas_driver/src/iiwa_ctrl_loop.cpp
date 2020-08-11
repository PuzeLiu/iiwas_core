//
// Created by puze on 05.08.20.
//

#include <iiwa_ctrl_loop.h>

namespace iiwa_hw{
    ControlLoop::ControlLoop(std::string ns, int coreId)
    : hardwareInterface(nh, ns), coreId(coreId){
        controllerManager.reset(new controller_manager::ControllerManager(&hardwareInterface, nh));
        oldTime = ros::Time::now();

        thread = nullptr;
    }

    void ControlLoop::start() {
        if(init())
            thread = new boost::thread(boost::bind(&iiwa_hw::ControlLoop::controlThread, this), coreId);
    }

    void ControlLoop::stop() {
        if(thread)
            thread->join();
        hardwareInterface.stop();
    }

    bool ControlLoop::init(){
        return hardwareInterface.init();
    }

    void ControlLoop::run(){
        while (ros::ok() && hardwareInterface.isIiwaReady()){
            auto new_t = ros::Time::now();
            elapsedTime = new_t - oldTime;
            oldTime = new_t;
            // Input
            hardwareInterface.read(oldTime, elapsedTime);

            // Control
            controllerManager->update(oldTime, elapsedTime);

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

