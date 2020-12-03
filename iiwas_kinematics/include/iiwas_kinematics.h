//
// Created by puze on 02.12.20.
//

#ifndef SRC_IIWAS_KINEMATICS_H
#define SRC_IIWAS_KINEMATICS_H

#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

namespace iiwas_kinematics {
    const int NUM_OF_JOINTS = 7;

    class Kinematics {
    public:
//        typedef Transform<double, 3, Affine> TransformMatrixType;
        typedef Matrix4d TransformMatrixType;
        typedef Matrix<double, NUM_OF_JOINTS, 1> JointArrayType;

        Kinematics();

        Kinematics(Vector3d& tcp_position, Quaterniond& tcp_quaternion);

        void forward_kinematics(const JointArrayType& q, Vector3d& out_ee_pos, Quaterniond& out_ee_quad);

    private:
        TransformMatrixType transform_i(double q, int i);

    private:
        JointArrayType dh_a;
        JointArrayType dh_alpha;
        JointArrayType dh_d;

        TransformMatrixType T_ee;
    };
}
#endif //SRC_IIWAS_KINEMATICS_H
