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
        typedef Matrix<double, 6, NUM_OF_JOINTS> JacobianType;
        typedef Matrix<double, 3, NUM_OF_JOINTS> JacobianPosType;
        typedef Matrix<double, 3, NUM_OF_JOINTS> JacobianRotType;

        Kinematics();

        Kinematics(const Vector3d& tcp_position, const Quaterniond& tcp_quaternion);

        void forward_kinematics(const JointArrayType& q, Vector3d& out_ee_pos, Quaterniond& out_ee_quad);
        void jacobian(const JointArrayType& q, JacobianType& out_jacobian);
        void jacobian_pos(const JointArrayType& q, JacobianPosType& out_jacobian);
        void jacobian_rot(const JointArrayType& q, JacobianRotType& out_jacobian);

    private:
        void transform_i(double q, int i, TransformMatrixType& out_T);

    private:
        double d_bs;     // Distance from base to shoulder
        double d_se;     // Distance from shoulder to elbow
        double d_ew;      // Distance from elbow to wrist
        double d_wf;    // Distance from elbow to finger(tip)

        JointArrayType dh_a;
        JointArrayType dh_alpha;
        JointArrayType dh_d;

        Vector3d tcp_pos;
        Quaterniond tcp_quat;
        TransformMatrixType T_ee;
    };
}
#endif //SRC_IIWAS_KINEMATICS_H
