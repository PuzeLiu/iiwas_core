//
// Created by puze on 05.08.20.
//

#ifndef PROJECT_IIWA_CTRL_LOOP_H
#define PROJECT_IIWA_CTRL_LOOP_H


#include <iiwa_hw_interface.h>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace iiwa_hw {
    static const double BILLION = 1000000000.0;

    class ControlLoop
    {
    public:
        ControlLoop(std::string ns, int coreId);

        void start();
        void stop();


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

    };

}

#endif //PROJECT_IIWA_CTRL_LOOP_H
