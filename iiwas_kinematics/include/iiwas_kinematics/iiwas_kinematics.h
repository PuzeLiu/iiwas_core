//
// Created by puze on 02.12.20.
//

#ifndef SRC_IIWAS_KINEMATICS_H
#define SRC_IIWAS_KINEMATICS_H

#include <eigen3/Eigen/Dense>

namespace iiwas_kinematics {
    const int NUM_OF_JOINTS = 7;

    class Kinematics {
    public:
//        typedef Transform<double, 3, Affine> TransformMatrixType;
        typedef Eigen::Matrix4d TransformMatrixType;
        typedef Eigen::Matrix<double, NUM_OF_JOINTS, 1> JointArrayType;
        typedef Eigen::Matrix<double, 6, NUM_OF_JOINTS> JacobianType;
        typedef Eigen::Matrix<double, 3, NUM_OF_JOINTS> JacobianPosType;
        typedef Eigen::Matrix<double, 3, NUM_OF_JOINTS> JacobianRotType;

        Kinematics();

        Kinematics(const Eigen::Vector3d &tcp_position,
                   const Eigen::Quaterniond &tcp_quaternion);

        void forwardKinematics(const JointArrayType &q, Eigen::Vector3d &out_ee_pos,
                               Eigen::Quaterniond &out_ee_quad);

        void forwardKinematics(const JointArrayType &q,
                               Eigen::Vector3d &out_ee_pos);

        void jacobian(const JointArrayType &q, JacobianType &out_jacobian);

        void jacobianPos(const JointArrayType &q, JacobianPosType &out_jacobian);

        void jacobianRot(const JointArrayType &q, JacobianRotType &out_jacobian);

        bool inverseKinematics(const Eigen::Vector3d &x,
                               const Eigen::Quaterniond &quat,
                               const Eigen::Vector3d &globalConfiguration, double &psi,
                               JointArrayType &out_q);

        bool numericalInverseKinematics(const Eigen::Vector3d &x,
                                        JointArrayType &qInOut,
                                        double tol = 1e-4,
                                        int max_iter = 100);

        void getRedundancy(const JointArrayType &q, Eigen::Vector3d &gc,
                           double &psi);

    private:
        void transform_i(double q, int i, TransformMatrixType &out_T);

        void tcp2EndEffector(const Eigen::Vector3d &x,
                             const Eigen::Quaterniond &quat, Eigen::Vector3d &xEE,
                             Eigen::Quaterniond &quatEE);

        bool getAuxiliaryParameter(const Eigen::Vector3d &xEE,
                                   const Eigen::Quaterniond &quatEE);

        bool checkFeasibility(const JointArrayType& q);

    private:
        double dBs_;     // Distance from base to shoulder
        double dSe_;     // Distance from shoulder to elbow
        double dEw_;     // Distance from elbow to wrist
        double dWf_;     // Distance from elbow to finger(tip)
        double q4_v_;    // Virtual Joint q_4

        JointArrayType dhA_;       // DH-Parameter a
        JointArrayType dhAlpha_;   // DH-Parameter alpha
        JointArrayType dhD_;       // DH-Parameter d

        Eigen::Matrix<double, 7, 2> auxA_, auxB_, auxC_; //auxiliary parameter for inverse kinematics
        JointArrayType globalConfiguration_;        //global configuration parameter

    public:
        JointArrayType posLimitsUpper_;    // Joints position upper limits
        JointArrayType posLimitsLower_;    // Joints position lower limits
        JointArrayType velLimitsUpper_;    // Joints velocity upper limits
        JointArrayType velLimitsLower_;    // Joints velocity lower limits

        Eigen::Vector3d eePos_;
        Eigen::Quaterniond eeQuat_;
        TransformMatrixType transform_, transformTmp_, transformEE_;
    };

}

#endif //SRC_IIWAS_KINEMATICS_H
