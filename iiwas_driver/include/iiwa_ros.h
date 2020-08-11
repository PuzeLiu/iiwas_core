//
// Created by puze on 31.07.20.
//

#ifndef PROJECT_IIWA_ROS_H
#define PROJECT_IIWA_ROS_H

struct JState{
    double   th;   /*!< theta */
    double   thd;  /*!< theta-dot */
    double   thdd; /*!< theta-dot-dot */
    double   ufb;  /*!< feedback portion of command */
    double   u;    /*!< torque command */
    double   load; /*!< sensed torque */
};

struct DJState{
    double   th;   /*!< theta */
    double   thd;  /*!< theta-dot */
    double   thdd; /*!< theta-dot-dot */
    double   uff;  /*!< feedforward torque command */
    double   uex;  /*!< externally imposed torque */
};

//int bind_thread_to_core(int core_id) {
//    int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
//    if (core_id < 0 || core_id >= num_cores)
//        return EINVAL;
//
//    cpu_set_t cpuset;
//    CPU_ZERO(&cpuset);
//    CPU_SET(core_id, &cpuset);
//
//    pthread_t current_thread = pthread_self();
//    return pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
//}

#endif //PROJECT_IIWA_ROS_H
