#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <chrono>
#include <ros/ros.h>
using namespace std;
using namespace pinocchio;

int main(int argc, char* argv[]){
    ros::init(argc, argv, "iiwas_dynamics_example", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    std::string robot_description;
    nh.getParam("iiwa_front/robot_description", robot_description);

    Model  model;
    pinocchio::urdf::buildModelFromXML(robot_description, model, false);
    Data data(model);

    Eigen::VectorXd q(7);

    /**
     * Example Inertia Matrix
     */
    cout << "#################################" << endl;
    cout << "#      Test Inertia Matrix      #" << endl;
    cout << "#################################" << endl;

    auto start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        q.setRandom();
        crba(model, data, q);
    }
    auto finish = chrono::high_resolution_clock::now();
    cout << "Inertia Matrix Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 10000. / 1.e6 << "ms\n";
    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
    cout << data.M << endl;


    /**
     * Example Forward Kinematics
     */
    cout << "#################################" << endl;
    cout << "#      Test Forward Kinematics      #" << endl;
    cout << "#################################" << endl;

    start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        q.setRandom();
        forwardKinematics(model, data, q);
    }
    finish = chrono::high_resolution_clock::now();
    cout << "Forward Kinematics Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 10000. / 1.e6 << "ms\n";
    cout << "Forward Kinematics: " << data.oMi[model.nq - 1] << endl;


    /**
     * Example Jacobian
     */
    cout << "#################################" << endl;
    cout << "#        Test Jacobian          #" << endl;
    cout << "#################################" << endl;
    start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        q.setRandom();
        computeJointJacobians(model, data, q);
    }
    finish = chrono::high_resolution_clock::now();
    cout << "Jacobian Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 10000. / 1.e6 << "ms\n";
    cout << "Jacobian: " << data.J << endl;

    return 0;
}