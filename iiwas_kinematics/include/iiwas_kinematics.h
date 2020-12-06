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

        void ForwardKinematics(const JointArrayType& q, Vector3d& out_ee_pos, Quaterniond& out_ee_quad);
        void Jacobian(const JointArrayType& q, JacobianType& out_jacobian);
        void JacobianPos(const JointArrayType& q, JacobianPosType& out_jacobian);
        void JacobianRot(const JointArrayType& q, JacobianRotType& out_jacobian);

    private:
        void transform_i(double q, int i, TransformMatrixType& out_T);

    private:
        double d_bs_;     // Distance from base to shoulder
        double d_se_;     // Distance from shoulder to elbow
        double d_ew_;     // Distance from elbow to wrist
        double d_wf_;     // Distance from elbow to finger(tip)

        JointArrayType dh_a_;       // DH-Parameter a
        JointArrayType dh_alpha_;   // DH-Parameter alpha
        JointArrayType dh_d_;       // DH-Parameter d

    public:
        JointArrayType pos_limits_upper;    // Joints position upper limits
        JointArrayType pos_limits_lower;    // Joints position lower limits
        JointArrayType vel_limits_upper;    // Joints velocity upper limits
        JointArrayType vel_limits_lower;    // Joints velocity lower limits

        Vector3d tcp_pos_;
        Quaterniond tcp_quat_;
        TransformMatrixType T, T_tmp, T_ee_;
    };
}
#endif //SRC_IIWAS_KINEMATICS_H
