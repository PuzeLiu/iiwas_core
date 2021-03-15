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