//
// Created by puze on 02.12.20.
//

#include <iostream>
#include "iiwas_kinematics.h"
#include <chrono>

using namespace Eigen;
using namespace iiwas_kinematics;
using namespace std;

int main(int argc, char* argv[]){
    Kinematics kinematics = Kinematics();
    Kinematics::JointArrayType q;
    std::srand((unsigned int) time(0));

    Vector3d ee_pos;
    Quaterniond ee_quat;
    ee_pos.setRandom();
    ee_quat.setIdentity();

    auto start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        q.setRandom();
        kinematics.forward_kinematics(q, ee_pos, ee_quat);
    }
    auto finish = chrono::high_resolution_clock::now();
    cout << "Forward Kinematics Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 10000. / 1.e6 << "ms\n";

    Kinematics::JacobianPosType jacobian_lin;
    start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        q.setRandom();
        kinematics.jacobian_pos(q, jacobian_lin);
    }
    finish = chrono::high_resolution_clock::now();
    cout << "Linear Jacobian Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 10000. / 1.e6 << "ms\n";


    double jac_eps = 1e-6;
    Kinematics::JacobianPosType jacobian_numerical;
    Vector3d ee_pos_positive, ee_pos_negative;
    Kinematics::JointArrayType q_positive, q_negative;
    start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        q.setRandom();
        for (int j = 0; j < NUM_OF_JOINTS; ++j) {
            q_positive = q;
            q_positive(j) += jac_eps;
            kinematics.forward_kinematics(q_positive, ee_pos_positive, ee_quat);

            q_negative = q;
            q_negative(j) -= jac_eps;
            kinematics.forward_kinematics(q_negative, ee_pos_negative, ee_quat);

            jacobian_numerical.block<3, 1>(0, j) = (ee_pos_positive - ee_pos_negative) / 2 / (jac_eps);
        }
    }
    finish = chrono::high_resolution_clock::now();
    cout << "Linear Jacobian Numerical Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 10000. / 1.e6 << "ms\n";


    Kinematics::JacobianRotType jacobian_rot;
    start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        q.setRandom();
        kinematics.jacobian_rot(q, jacobian_rot);
    }
    finish = chrono::high_resolution_clock::now();
    cout << "Rotation Jacobian Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 10000. / 1.e6 << "ms\n";

    Kinematics::JacobianType jacobian;
    start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        q.setRandom();
        kinematics.jacobian(q, jacobian);
    }
    finish = chrono::high_resolution_clock::now();
    cout << "Total Jacobian Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 10000. / 1.e6 << "ms\n";


    return 0;
}