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
        double dBs_;     // Distance from base to shoulder
        double dSe_;     // Distance from shoulder to elbow
        double dEw_;     // Distance from elbow to wrist
        double dWf_;     // Distance from elbow to finger(tip)

        JointArrayType dhA_;       // DH-Parameter a
        JointArrayType dhAlpha_;   // DH-Parameter alpha
        JointArrayType dhD_;       // DH-Parameter d

    public:
        JointArrayType posLimitsUpper_;    // Joints position upper limits
        JointArrayType posLimitsLower_;    // Joints position lower limits
        JointArrayType velLimitsUpper_;    // Joints velocity upper limits
        JointArrayType velLimitsLower_;    // Joints velocity lower limits

        Vector3d eePos_;
        Quaterniond eeQuat_;
        TransformMatrixType transform_, transformTmp_, transformEE_;
    };
}
#endif //SRC_IIWAS_KINEMATICS_H
