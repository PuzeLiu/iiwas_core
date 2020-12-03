#include "iiwas_kinematics.h"
#include <iostream>

namespace iiwas_kinematics {


    Kinematics::Kinematics(Vector3d &tcp_position, Quaterniond &tcp_quat) : Kinematics() {
        T_ee.block<3, 1>(0, 3) = tcp_position;
        T_ee.block<3, 3>(0, 0) = tcp_quat.toRotationMatrix();
    }

    Kinematics::Kinematics() {
        double d_bs = 0.36;     // Distance from base to shoulder
        double d_se = 0.42;     // Distance from shoulder to elbow
        double d_ew = 0.4;      // Distance from elbow to wrist
        double d_wf = 0.151;    // Distance from elbow to finger(tip)

        dh_a << 0., 0., 0., 0., 0., 0., 0.;
        dh_alpha << -M_PI_2, M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, M_PI_2, 0.;
        dh_d << d_bs, 0., d_se, 0., d_ew, 0., d_wf;

        T_ee.setIdentity();
    }

    void Kinematics::transform_i(double q, int i, Kinematics::TransformMatrixType& T_i) {
        T_i << cos(q), -sin(q) * cos(dh_alpha[i]), sin(q) * sin(dh_alpha[i]), dh_a[i] * cos(q),
                sin(q), cos(q) * cos(dh_alpha[i]), -cos(q) * sin(dh_alpha[i]), dh_a[i] * sin(q),
                0., sin(dh_alpha[i]), cos(dh_alpha[i]), dh_d[i],
                0., 0., 0., 1.;
    }

    void Kinematics::forward_kinematics(const Kinematics::JointArrayType &q, Vector3d &out_ee_pos,
                                        Quaterniond &out_ee_quad) {
        TransformMatrixType T;
        T.setIdentity();
        TransformMatrixType T_tmp;
        for (int i = 0; i < NUM_OF_JOINTS; ++i) {
            transform_i(q[i], i, T_tmp);
            T = T * T_tmp;
        }
//        cout<< "Transformation Matrix: \n" << T << endl;
        out_ee_pos = T.block<3, 1>(0, 3);
        out_ee_quad = T.block<3, 3>(0, 0);
    }

}