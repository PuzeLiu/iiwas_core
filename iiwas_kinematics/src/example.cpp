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

    /**
     * Example Forward Kinematics
     */
    cout << "#################################" << endl;
    cout << "#   Test Forward Kinematics     #" << endl;
    cout << "#################################" << endl;
    auto start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        q.setRandom();
        kinematics.forward_kinematics(q, ee_pos, ee_quat);
    }
    auto finish = chrono::high_resolution_clock::now();
    cout << "Forward Kinematics Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 10000. / 1.e6 << "ms\n";

    /**
     * Example Jacobian for Position
     */
    cout << "#################################" << endl;
    cout << "#    Test Jacobian Position     #" << endl;
    cout << "#################################" << endl;
    Kinematics::JacobianPosType jacobian_lin;
    start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        q.setRandom();
        kinematics.jacobian_pos(q, jacobian_lin);
    }
    finish = chrono::high_resolution_clock::now();
    cout << "Linear Jacobian Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 10000. / 1.e6 << "ms\n";


    /**
     * Example Jacobian for Rotation
     */
    cout << "#################################" << endl;
    cout << "#   Test Jacobian Rotation     #" << endl;
    cout << "#################################" << endl;
    Kinematics::JacobianRotType jacobian_rot;
    start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        q.setRandom();
        kinematics.jacobian_rot(q, jacobian_rot);
    }
    finish = chrono::high_resolution_clock::now();
    cout << "Rotation Jacobian Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 10000. / 1.e6 << "ms\n";

    /**
    * Example Jacobian from numerical
    */
    cout << "#################################" << endl;
    cout << "#     Test Total Jacobian       #" << endl;
    cout << "#################################" << endl;
    Kinematics::JacobianType jacobian;
    start = chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; ++i) {
        q.setRandom();
        kinematics.jacobian(q, jacobian);
    }
    finish = chrono::high_resolution_clock::now();
    cout << "Total Jacobian Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 10000. / 1.e6 << "ms\n";


    /**
     * Example Jacobian from numerical
     */
    cout << "#################################" << endl;
    cout << "#   Test Numerical Jacobian     #" << endl;
    cout << "#################################" << endl;
    double jac_eps = 1e-6;
    Kinematics::JacobianType jacobian_numerical, jacobian_test;
    Vector3d ee_pos_positive, ee_pos_negative;
    Quaterniond ee_rot_positive, ee_rot_negative;
    AngleAxisd rot_diff;
    Kinematics::JointArrayType q_positive, q_negative;
    double jac_diff;
    start = chrono::high_resolution_clock::now();

    for (int i = 0; i < 10000; ++i) {
        q = q.setRandom() * M_PI;
        kinematics.jacobian(q, jacobian_test);
        for (int j = 0; j < NUM_OF_JOINTS; ++j) {
            q_positive = q;
            q_positive(j) += jac_eps;
            kinematics.forward_kinematics(q_positive, ee_pos_positive, ee_rot_positive);

            q_negative = q;
            q_negative(j) -= jac_eps;
            kinematics.forward_kinematics(q_negative, ee_pos_negative, ee_rot_negative);

            jacobian_numerical.block<3, 1>(0, j) = (ee_pos_positive - ee_pos_negative) / 2 / (jac_eps);

            rot_diff = AngleAxis<double>(ee_rot_positive * ee_rot_negative.inverse());
            jacobian_numerical.block<3, 1>(3, j) =  rot_diff.axis() * rot_diff.angle() / 2 / (jac_eps);
        }
        jac_diff = (jacobian_test - jacobian_numerical).norm();

        if (jac_diff > 1e-6){
            cout << "Jacobian Error!" << endl;
            cout << " q: " << q << endl ;
            cout << "Jacobian Numerical: " << endl << jacobian_numerical << endl;
            cout << "Jacobian Analytical: " << endl << jacobian_test << endl;
        }
    }
    finish = chrono::high_resolution_clock::now();
    cout << "Jacobian Numerical Time: " << chrono::duration_cast<std::chrono::nanoseconds>(finish-start).count() / 10000. / 1.e6 << "ms\n";

    return 0;
}