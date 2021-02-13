#include "iiwas_kinematics/iiwas_kinematics.h"
#include "math.h"
#include <iostream>

using namespace Eigen;
using namespace std;

namespace iiwas_kinematics {

    Kinematics::Kinematics(const Vector3d &tcp_position, const Quaterniond &tcp_quaternion) : Kinematics() {
        eePos_ = tcp_position;
        eeQuat_ = tcp_quaternion;
        transformEE_.block<3, 1>(0, 3) = eePos_;
        transformEE_.block<3, 3>(0, 0) = eeQuat_.toRotationMatrix();
    }

    Kinematics::Kinematics() {
        dBs_ = 0.36;     // Distance from base to shoulder
        dSe_ = 0.42;     // Distance from shoulder to elbow
        dEw_ = 0.4;      // Distance from elbow to wrist
        dWf_ = 0.151;    // Distance from elbow to finger(tip)

        dhA_ << 0., 0., 0., 0., 0., 0., 0.;
        dhAlpha_ << -M_PI_2, M_PI_2, M_PI_2, -M_PI_2, -M_PI_2, M_PI_2, 0.;
        dhD_ << dBs_, 0., dSe_, 0., dEw_, 0., dWf_;

        posLimitsLower_ << -170., -120., -170., -120., -170., -120., -175.;
        posLimitsUpper_ << 170., 120., 170., 120., 170., 120., 175.;
        velLimitsLower_ << -85., -85., -100., -75., -130., -135., -135.;
        velLimitsUpper_ << 85., 85., 100., 75., 130., 135., 135.;
        posLimitsLower_ = posLimitsLower_ / 180. * M_PI;
        posLimitsUpper_ = posLimitsUpper_ / 180. * M_PI;
        velLimitsLower_ = velLimitsLower_ / 180. * M_PI;
        velLimitsUpper_ = velLimitsUpper_ / 180. * M_PI;

        transformEE_.setIdentity();

        auxA_.setZero();
        auxB_.setZero();
        auxC_.setZero();
    }

    void Kinematics::transform_i(double q, int i, Kinematics::TransformMatrixType &T_i) {
        T_i << cos(q), -sin(q) * cos(dhAlpha_[i]), sin(q) * sin(dhAlpha_[i]), dhA_[i] * cos(q),
                sin(q), cos(q) * cos(dhAlpha_[i]), -cos(q) * sin(dhAlpha_[i]), dhA_[i] * sin(q),
                0., sin(dhAlpha_[i]), cos(dhAlpha_[i]), dhD_[i],
                0., 0., 0., 1.;
    }

    void Kinematics::forwardKinematics(const Kinematics::JointArrayType &q, Vector3d &out_ee_pos,
                                       Quaterniond &out_ee_quad) {
        transform_.setIdentity();
        for (int i = 0; i < NUM_OF_JOINTS; ++i) {
            transform_i(q[i], i, transformTmp_);
            transform_ = transform_ * transformTmp_;
        }
        transform_ = transform_ * transformEE_;
        out_ee_pos = transform_.block<3, 1>(0, 3);
        out_ee_quad = transform_.block<3, 3>(0, 0);
    }

    void Kinematics::forwardKinematics(const Kinematics::JointArrayType &q, Vector3d &out_ee_pos) {
        transform_.setIdentity();
        for (int i = 0; i < NUM_OF_JOINTS; ++i) {
            transform_i(q[i], i, transformTmp_);
            transform_ = transform_ * transformTmp_;
        }
        transform_ = transform_ * transformEE_;
        out_ee_pos = transform_.block<3, 1>(0, 3);
    }

    void Kinematics::jacobian(const JointArrayType &q, JacobianType &out_jacobian) {
        JacobianPosType jac_pos;
        JacobianRotType jac_rot;
        jacobianPos(q, jac_pos);
        jacobianRot(q, jac_rot);
        out_jacobian.block<3, NUM_OF_JOINTS>(0, 0) = jac_pos;
        out_jacobian.block<3, NUM_OF_JOINTS>(3, 0) = jac_rot;
    }

    void Kinematics::jacobianPos(const Kinematics::JointArrayType &q, Kinematics::JacobianPosType &out_jacobian) {
        double q_1 = q[0];
        double q_2 = q[1];
        double q_3 = q[2];
        double q_4 = q[3];
        double q_5 = q[4];
        double q_6 = q[5];
        double q_7 = q[6];
        
        double s1 = sin(q_1);
        double c1 = cos(q_1);
        double s2 = sin(q_2);
        double c2 = cos(q_2);
        double s3 = sin(q_3);
        double c3 = cos(q_3);
        double s4 = sin(q_4);
        double c4 = cos(q_4);
        double s5 = sin(q_5);
        double c5 = cos(q_5);
        double s6 = sin(q_6);
        double c6 = cos(q_6);
        double s7 = sin(q_7);
        double c7 = cos(q_7);

        out_jacobian(0, 0) = dEw_ * ((s1 * c2 * c3 + s3 * c1) * s4 - s1 * s2 * c4) - dSe_ * s1 * s2 + dWf_ * ((((-s1 * c2 * c3 - s3 * c1) * c4 - s1 * s2 * s4) * c5 + (s1 * s3 * c2 - c1 * c3) * s5) * s6 + (-(-s1 * c2 * c3 - s3 * c1) * s4 - s1 * s2 * c4) * c6) + eePos_.x() * (((((-s1 * c2 * c3 - s3 * c1) * c4 - s1 * s2 * s4) * c5 + (s1 * s3 * c2 - c1 * c3) * s5) * c6 + ((-s1 * c2 * c3 - s3 * c1) * s4 + s1 * s2 * c4) * s6) * c7 + ((-(-s1 * c2 * c3 - s3 * c1) * c4 + s1 * s2 * s4) * s5 + (s1 * s3 * c2 - c1 * c3) * c5) * s7) + eePos_.y() * ((-(((-s1 * c2 * c3 - s3 * c1) * c4 - s1 * s2 * s4) * c5 + (s1 * s3 * c2 - c1 * c3) * s5) * c6 - ((-s1 * c2 * c3 - s3 * c1) * s4 + s1 * s2 * c4) * s6) * s7 + ((-(-s1 * c2 * c3 - s3 * c1) * c4 + s1 * s2 * s4) * s5 + (s1 * s3 * c2 - c1 * c3) * c5) * c7) + eePos_.z() * ((((-s1 * c2 * c3 - s3 * c1) * c4 - s1 * s2 * s4) * c5 + (s1 * s3 * c2 - c1 * c3) * s5) * s6 + (-(-s1 * c2 * c3 - s3 * c1) * s4 - s1 * s2 * c4) * c6);
        out_jacobian(0, 1) = dEw_ * (s2 * s4 * c1 * c3 + c1 * c2 * c4) + dSe_ * c1 * c2 + dWf_ * (((-s2 * c1 * c3 * c4 + s4 * c1 * c2) * c5 + s2 * s3 * s5 * c1) * s6 + (s2 * s4 * c1 * c3 + c1 * c2 * c4) * c6) + eePos_.x() * ((((-s2 * c1 * c3 * c4 + s4 * c1 * c2) * c5 + s2 * s3 * s5 * c1) * c6 + (-s2 * s4 * c1 * c3 - c1 * c2 * c4) * s6) * c7 + ((s2 * c1 * c3 * c4 - s4 * c1 * c2) * s5 + s2 * s3 * c1 * c5) * s7) + eePos_.y() * ((-((-s2 * c1 * c3 * c4 + s4 * c1 * c2) * c5 + s2 * s3 * s5 * c1) * c6 - (-s2 * s4 * c1 * c3 - c1 * c2 * c4) * s6) * s7 + ((s2 * c1 * c3 * c4 - s4 * c1 * c2) * s5 + s2 * s3 * c1 * c5) * c7) + eePos_.z() * (((-s2 * c1 * c3 * c4 + s4 * c1 * c2) * c5 + s2 * s3 * s5 * c1) * s6 + (s2 * s4 * c1 * c3 + c1 * c2 * c4) * c6);
        out_jacobian(0, 2) = dEw_ * (s1 * c3 + s3 * c1 * c2) * s4 + dWf_ * (((s1 * s3 - c1 * c2 * c3) * s5 + (-s1 * c3 - s3 * c1 * c2) * c4 * c5) * s6 - (-s1 * c3 - s3 * c1 * c2) * s4 * c6) + eePos_.x() * ((((s1 * s3 - c1 * c2 * c3) * s5 + (-s1 * c3 - s3 * c1 * c2) * c4 * c5) * c6 + (-s1 * c3 - s3 * c1 * c2) * s4 * s6) * c7 + ((s1 * s3 - c1 * c2 * c3) * c5 - (-s1 * c3 - s3 * c1 * c2) * s5 * c4) * s7) + eePos_.y() * ((-((s1 * s3 - c1 * c2 * c3) * s5 + (-s1 * c3 - s3 * c1 * c2) * c4 * c5) * c6 - (-s1 * c3 - s3 * c1 * c2) * s4 * s6) * s7 + ((s1 * s3 - c1 * c2 * c3) * c5 - (-s1 * c3 - s3 * c1 * c2) * s5 * c4) * c7) + eePos_.z() * (((s1 * s3 - c1 * c2 * c3) * s5 + (-s1 * c3 - s3 * c1 * c2) * c4 * c5) * s6 - (-s1 * c3 - s3 * c1 * c2) * s4 * c6);
        out_jacobian(0, 3) = dEw_ * ((s1 * s3 - c1 * c2 * c3) * c4 - s2 * s4 * c1) + dWf_ * ((-(-s1 * s3 + c1 * c2 * c3) * s4 + s2 * c1 * c4) * s6 * c5 + (-(-s1 * s3 + c1 * c2 * c3) * c4 - s2 * s4 * c1) * c6) + eePos_.x() * (((-s1 * s3 + c1 * c2 * c3) * s4 - s2 * c1 * c4) * s5 * s7 + ((-(-s1 * s3 + c1 * c2 * c3) * s4 + s2 * c1 * c4) * c5 * c6 + ((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * s6) * c7) + eePos_.y() * (((-s1 * s3 + c1 * c2 * c3) * s4 - s2 * c1 * c4) * s5 * c7 + (-(-(-s1 * s3 + c1 * c2 * c3) * s4 + s2 * c1 * c4) * c5 * c6 - ((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * s6) * s7) + eePos_.z() * ((-(-s1 * s3 + c1 * c2 * c3) * s4 + s2 * c1 * c4) * s6 * c5 + (-(-s1 * s3 + c1 * c2 * c3) * c4 - s2 * s4 * c1) * c6);
        out_jacobian(0, 4) = dWf_ * (-((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * s5 + (-s1 * c3 - s3 * c1 * c2) * c5) * s6 + eePos_.x() * (((-(-s1 * s3 + c1 * c2 * c3) * c4 - s2 * s4 * c1) * c5 - (-s1 * c3 - s3 * c1 * c2) * s5) * s7 + (-((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * s5 + (-s1 * c3 - s3 * c1 * c2) * c5) * c6 * c7) + eePos_.y() * (((-(-s1 * s3 + c1 * c2 * c3) * c4 - s2 * s4 * c1) * c5 - (-s1 * c3 - s3 * c1 * c2) * s5) * c7 - (-((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * s5 + (-s1 * c3 - s3 * c1 * c2) * c5) * s7 * c6) + eePos_.z() * (-((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * s5 + (-s1 * c3 - s3 * c1 * c2) * c5) * s6;
        out_jacobian(0, 5) = dWf_ * ((((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * c5 + (-s1 * c3 - s3 * c1 * c2) * s5) * c6 - (-(-s1 * s3 + c1 * c2 * c3) * s4 + s2 * c1 * c4) * s6) + eePos_.x() * (-(((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * c5 + (-s1 * c3 - s3 * c1 * c2) * s5) * s6 + ((-s1 * s3 + c1 * c2 * c3) * s4 - s2 * c1 * c4) * c6) * c7 + eePos_.y() * ((((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * c5 + (-s1 * c3 - s3 * c1 * c2) * s5) * s6 - ((-s1 * s3 + c1 * c2 * c3) * s4 - s2 * c1 * c4) * c6) * s7 + eePos_.z() * ((((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * c5 + (-s1 * c3 - s3 * c1 * c2) * s5) * c6 - (-(-s1 * s3 + c1 * c2 * c3) * s4 + s2 * c1 * c4) * s6);
        out_jacobian(0, 6) = eePos_.x() * (-((((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * c5 + (-s1 * c3 - s3 * c1 * c2) * s5) * c6 + ((-s1 * s3 + c1 * c2 * c3) * s4 - s2 * c1 * c4) * s6) * s7 + (-((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * s5 + (-s1 * c3 - s3 * c1 * c2) * c5) * c7) + eePos_.y() * ((-(((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * c5 + (-s1 * c3 - s3 * c1 * c2) * s5) * c6 - ((-s1 * s3 + c1 * c2 * c3) * s4 - s2 * c1 * c4) * s6) * c7 - (-((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * s5 + (-s1 * c3 - s3 * c1 * c2) * c5) * s7);

        out_jacobian(1, 0) = dEw_ * ((s1 * s3 - c1 * c2 * c3) * s4 + s2 * c1 * c4) + dSe_ * s2 * c1 + dWf_ * ((((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * c5 + (-s1 * c3 - s3 * c1 * c2) * s5) * s6 + (-(-s1 * s3 + c1 * c2 * c3) * s4 + s2 * c1 * c4) * c6) + eePos_.x() * (((((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * c5 + (-s1 * c3 - s3 * c1 * c2) * s5) * c6 + ((-s1 * s3 + c1 * c2 * c3) * s4 - s2 * c1 * c4) * s6) * c7 + ((-(-s1 * s3 + c1 * c2 * c3) * c4 - s2 * s4 * c1) * s5 + (-s1 * c3 - s3 * c1 * c2) * c5) * s7) + eePos_.y() * ((-(((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * c5 + (-s1 * c3 - s3 * c1 * c2) * s5) * c6 - ((-s1 * s3 + c1 * c2 * c3) * s4 - s2 * c1 * c4) * s6) * s7 + ((-(-s1 * s3 + c1 * c2 * c3) * c4 - s2 * s4 * c1) * s5 + (-s1 * c3 - s3 * c1 * c2) * c5) * c7) + eePos_.z() * ((((-s1 * s3 + c1 * c2 * c3) * c4 + s2 * s4 * c1) * c5 + (-s1 * c3 - s3 * c1 * c2) * s5) * s6 + (-(-s1 * s3 + c1 * c2 * c3) * s4 + s2 * c1 * c4) * c6);
        out_jacobian(1, 1) = dEw_ * (s1 * s2 * s4 * c3 + s1 * c2 * c4) + dSe_ * s1 * c2 + dWf_ * (((-s1 * s2 * c3 * c4 + s1 * s4 * c2) * c5 + s1 * s2 * s3 * s5) * s6 + (s1 * s2 * s4 * c3 + s1 * c2 * c4) * c6) + eePos_.x() * ((((-s1 * s2 * c3 * c4 + s1 * s4 * c2) * c5 + s1 * s2 * s3 * s5) * c6 + (-s1 * s2 * s4 * c3 - s1 * c2 * c4) * s6) * c7 + ((s1 * s2 * c3 * c4 - s1 * s4 * c2) * s5 + s1 * s2 * s3 * c5) * s7) + eePos_.y() * ((-((-s1 * s2 * c3 * c4 + s1 * s4 * c2) * c5 + s1 * s2 * s3 * s5) * c6 - (-s1 * s2 * s4 * c3 - s1 * c2 * c4) * s6) * s7 + ((s1 * s2 * c3 * c4 - s1 * s4 * c2) * s5 + s1 * s2 * s3 * c5) * c7) + eePos_.z() * (((-s1 * s2 * c3 * c4 + s1 * s4 * c2) * c5 + s1 * s2 * s3 * s5) * s6 + (s1 * s2 * s4 * c3 + s1 * c2 * c4) * c6);
        out_jacobian(1, 2) = dEw_ * (s1 * s3 * c2 - c1 * c3) * s4 + dWf_ * (((-s1 * s3 * c2 + c1 * c3) * c4 * c5 + (-s1 * c2 * c3 - s3 * c1) * s5) * s6 - (-s1 * s3 * c2 + c1 * c3) * s4 * c6) + eePos_.x() * ((((-s1 * s3 * c2 + c1 * c3) * c4 * c5 + (-s1 * c2 * c3 - s3 * c1) * s5) * c6 + (-s1 * s3 * c2 + c1 * c3) * s4 * s6) * c7 + (-(-s1 * s3 * c2 + c1 * c3) * s5 * c4 + (-s1 * c2 * c3 - s3 * c1) * c5) * s7) + eePos_.y() * ((-((-s1 * s3 * c2 + c1 * c3) * c4 * c5 + (-s1 * c2 * c3 - s3 * c1) * s5) * c6 - (-s1 * s3 * c2 + c1 * c3) * s4 * s6) * s7 + (-(-s1 * s3 * c2 + c1 * c3) * s5 * c4 + (-s1 * c2 * c3 - s3 * c1) * c5) * c7) + eePos_.z() * (((-s1 * s3 * c2 + c1 * c3) * c4 * c5 + (-s1 * c2 * c3 - s3 * c1) * s5) * s6 - (-s1 * s3 * c2 + c1 * c3) * s4 * c6);
        out_jacobian(1, 3) = dEw_ * ((-s1 * c2 * c3 - s3 * c1) * c4 - s1 * s2 * s4) + dWf_ * ((-(s1 * c2 * c3 + s3 * c1) * s4 + s1 * s2 * c4) * s6 * c5 + (-(s1 * c2 * c3 + s3 * c1) * c4 - s1 * s2 * s4) * c6) + eePos_.x() * (((s1 * c2 * c3 + s3 * c1) * s4 - s1 * s2 * c4) * s5 * s7 + ((-(s1 * c2 * c3 + s3 * c1) * s4 + s1 * s2 * c4) * c5 * c6 + ((s1 * c2 * c3 + s3 * c1) * c4 + s1 * s2 * s4) * s6) * c7) + eePos_.y() * (((s1 * c2 * c3 + s3 * c1) * s4 - s1 * s2 * c4) * s5 * c7 + (-(-(s1 * c2 * c3 + s3 * c1) * s4 + s1 * s2 * c4) * c5 * c6 - ((s1 * c2 * c3 + s3 * c1) * c4 + s1 * s2 * s4) * s6) * s7) + eePos_.z() * ((-(s1 * c2 * c3 + s3 * c1) * s4 + s1 * s2 * c4) * s6 * c5 + (-(s1 * c2 * c3 + s3 * c1) * c4 - s1 * s2 * s4) * c6);
        out_jacobian(1, 4) = dWf_ * (-((s1 * c2 * c3 + s3 * c1) * c4 + s1 * s2 * s4) * s5 + (-s1 * s3 * c2 + c1 * c3) * c5) * s6 + eePos_.x() * (((-(s1 * c2 * c3 + s3 * c1) * c4 - s1 * s2 * s4) * c5 - (-s1 * s3 * c2 + c1 * c3) * s5) * s7 + (-((s1 * c2 * c3 + s3 * c1) * c4 + s1 * s2 * s4) * s5 + (-s1 * s3 * c2 + c1 * c3) * c5) * c6 * c7) + eePos_.y() * (((-(s1 * c2 * c3 + s3 * c1) * c4 - s1 * s2 * s4) * c5 - (-s1 * s3 * c2 + c1 * c3) * s5) * c7 - (-((s1 * c2 * c3 + s3 * c1) * c4 + s1 * s2 * s4) * s5 + (-s1 * s3 * c2 + c1 * c3) * c5) * s7 * c6) + eePos_.z() * (-((s1 * c2 * c3 + s3 * c1) * c4 + s1 * s2 * s4) * s5 + (-s1 * s3 * c2 + c1 * c3) * c5) * s6;
        out_jacobian(1, 5) = dWf_ * ((((s1 * c2 * c3 + s3 * c1) * c4 + s1 * s2 * s4) * c5 + (-s1 * s3 * c2 + c1 * c3) * s5) * c6 - (-(s1 * c2 * c3 + s3 * c1) * s4 + s1 * s2 * c4) * s6) + eePos_.x() * (-(((s1 * c2 * c3 + s3 * c1) * c4 + s1 * s2 * s4) * c5 + (-s1 * s3 * c2 + c1 * c3) * s5) * s6 + ((s1 * c2 * c3 + s3 * c1) * s4 - s1 * s2 * c4) * c6) * c7 + eePos_.y() * ((((s1 * c2 * c3 + s3 * c1) * c4 + s1 * s2 * s4) * c5 + (-s1 * s3 * c2 + c1 * c3) * s5) * s6 - ((s1 * c2 * c3 + s3 * c1) * s4 - s1 * s2 * c4) * c6) * s7 + eePos_.z() * ((((s1 * c2 * c3 + s3 * c1) * c4 + s1 * s2 * s4) * c5 + (-s1 * s3 * c2 + c1 * c3) * s5) * c6 - (-(s1 * c2 * c3 + s3 * c1) * s4 + s1 * s2 * c4) * s6);
        out_jacobian(1, 6) = eePos_.x() * (-((((s1 * c2 * c3 + s3 * c1) * c4 + s1 * s2 * s4) * c5 + (-s1 * s3 * c2 + c1 * c3) * s5) * c6 + ((s1 * c2 * c3 + s3 * c1) * s4 - s1 * s2 * c4) * s6) * s7 + (-((s1 * c2 * c3 + s3 * c1) * c4 + s1 * s2 * s4) * s5 + (-s1 * s3 * c2 + c1 * c3) * c5) * c7) + eePos_.y() * ((-(((s1 * c2 * c3 + s3 * c1) * c4 + s1 * s2 * s4) * c5 + (-s1 * s3 * c2 + c1 * c3) * s5) * c6 - ((s1 * c2 * c3 + s3 * c1) * s4 - s1 * s2 * c4) * s6) * c7 - (-((s1 * c2 * c3 + s3 * c1) * c4 + s1 * s2 * s4) * s5 + (-s1 * s3 * c2 + c1 * c3) * c5) * s7);

        out_jacobian(2, 0) = 0;
        out_jacobian(2, 1) = dEw_ * (-s2 * c4 + s4 * c2 * c3) - dSe_ * s2 + dWf_ * (((-s2 * s4 - c2 * c3 * c4) * c5 + s3 * s5 * c2) * s6 + (-s2 * c4 + s4 * c2 * c3) * c6) + eePos_.x() * ((((-s2 * s4 - c2 * c3 * c4) * c5 + s3 * s5 * c2) * c6 + (s2 * c4 - s4 * c2 * c3) * s6) * c7 + ((s2 * s4 + c2 * c3 * c4) * s5 + s3 * c2 * c5) * s7) + eePos_.y() * ((-((-s2 * s4 - c2 * c3 * c4) * c5 + s3 * s5 * c2) * c6 - (s2 * c4 - s4 * c2 * c3) * s6) * s7 + ((s2 * s4 + c2 * c3 * c4) * s5 + s3 * c2 * c5) * c7) + eePos_.z() * (((-s2 * s4 - c2 * c3 * c4) * c5 + s3 * s5 * c2) * s6 + (-s2 * c4 + s4 * c2 * c3) * c6);
        out_jacobian(2, 2) = -dEw_ * s2 * s3 * s4 + dWf_ * ((s2 * s3 * c4 * c5 + s2 * s5 * c3) * s6 - s2 * s3 * s4 * c6) + eePos_.x() * (((s2 * s3 * c4 * c5 + s2 * s5 * c3) * c6 + s2 * s3 * s4 * s6) * c7 + (-s2 * s3 * s5 * c4 + s2 * c3 * c5) * s7) + eePos_.y() * ((-(s2 * s3 * c4 * c5 + s2 * s5 * c3) * c6 - s2 * s3 * s4 * s6) * s7 + (-s2 * s3 * s5 * c4 + s2 * c3 * c5) * c7) + eePos_.z() * ((s2 * s3 * c4 * c5 + s2 * s5 * c3) * s6 - s2 * s3 * s4 * c6);
        out_jacobian(2, 3) = dEw_ * (s2 * c3 * c4 - s4 * c2) + dWf_ * ((s2 * s4 * c3 + c2 * c4) * s6 * c5 + (s2 * c3 * c4 - s4 * c2) * c6) + eePos_.x() * (((s2 * s4 * c3 + c2 * c4) * c5 * c6 + (-s2 * c3 * c4 + s4 * c2) * s6) * c7 + (-s2 * s4 * c3 - c2 * c4) * s5 * s7) + eePos_.y() * ((-(s2 * s4 * c3 + c2 * c4) * c5 * c6 - (-s2 * c3 * c4 + s4 * c2) * s6) * s7 + (-s2 * s4 * c3 - c2 * c4) * s5 * c7) + eePos_.z() * ((s2 * s4 * c3 + c2 * c4) * s6 * c5 + (s2 * c3 * c4 - s4 * c2) * c6);
        out_jacobian(2, 4) = dWf_ * (-(-s2 * c3 * c4 + s4 * c2) * s5 + s2 * s3 * c5) * s6 + eePos_.x() * ((-(-s2 * c3 * c4 + s4 * c2) * s5 + s2 * s3 * c5) * c6 * c7 + ((s2 * c3 * c4 - s4 * c2) * c5 - s2 * s3 * s5) * s7) + eePos_.y() * (-(-(-s2 * c3 * c4 + s4 * c2) * s5 + s2 * s3 * c5) * s7 * c6 + ((s2 * c3 * c4 - s4 * c2) * c5 - s2 * s3 * s5) * c7) + eePos_.z() * (-(-s2 * c3 * c4 + s4 * c2) * s5 + s2 * s3 * c5) * s6;
        out_jacobian(2, 5) = dWf_ * (((-s2 * c3 * c4 + s4 * c2) * c5 + s2 * s3 * s5) * c6 - (s2 * s4 * c3 + c2 * c4) * s6) + eePos_.x() * (-((-s2 * c3 * c4 + s4 * c2) * c5 + s2 * s3 * s5) * s6 + (-s2 * s4 * c3 - c2 * c4) * c6) * c7 + eePos_.y() * (((-s2 * c3 * c4 + s4 * c2) * c5 + s2 * s3 * s5) * s6 - (-s2 * s4 * c3 - c2 * c4) * c6) * s7 + eePos_.z() * (((-s2 * c3 * c4 + s4 * c2) * c5 + s2 * s3 * s5) * c6 - (s2 * s4 * c3 + c2 * c4) * s6);
        out_jacobian(2, 6) = eePos_.x() * (-(((-s2 * c3 * c4 + s4 * c2) * c5 + s2 * s3 * s5) * c6 + (-s2 * s4 * c3 - c2 * c4) * s6) * s7 + (-(-s2 * c3 * c4 + s4 * c2) * s5 + s2 * s3 * c5) * c7) + eePos_.y() * ((-((-s2 * c3 * c4 + s4 * c2) * c5 + s2 * s3 * s5) * c6 - (-s2 * s4 * c3 - c2 * c4) * s6) * c7 - (-(-s2 * c3 * c4 + s4 * c2) * s5 + s2 * s3 * c5) * s7);
    }

    void Kinematics::jacobianRot(const Kinematics::JointArrayType &q, Kinematics::JacobianRotType &out_jacobian) {
        transform_.setIdentity();
        for (int i = 0; i < NUM_OF_JOINTS; ++i) {
            out_jacobian.block<3, 1>(0, i) = transform_.block<3, 1>(0, 2);
            transform_i(q[i], i, transformTmp_);
            transform_ = transform_ * transformTmp_;
        }
    }

    bool Kinematics::inverseKinematics(const Vector3d &xTcp,
                                       const Quaterniond &quatTcp,
                                       const Vector3d &gc,
                                       double &psi,
                                       Kinematics::JointArrayType &out_q) {
        Vector3d xEE;
        Quaterniond quatEE;
        bool ret;
        // calculate transform from base to end effector
        tcp2EndEffector(xTcp, quatTcp, xEE, quatEE);
        // set global configuration for each joint
        globalConfiguration_ << gc(0), gc(0), gc(0), gc(1), gc(2), gc(2), gc(2);

        ret = getAuxiliaryParameter(xEE, quatEE);
        if (!ret) {
            return ret;
        }

        double cos_q2 = auxA_(1, 0) * sin(psi) + auxB_(1, 0) * cos(psi) + auxC_(1, 0);
        if (abs(cos_q2) > 1 + 1e-9){
            return false;
        }
        cos_q2 = std::max(-1., std::min(cos_q2, 1.));

        out_q(0) = atan2(globalConfiguration_(0) * (auxA_(0, 0) * sin(psi) + auxB_(0, 0) * cos(psi) + auxC_(0, 0)),
                           globalConfiguration_(0) * (auxA_(0, 1) * sin(psi) + auxB_(0, 1) * cos(psi) + auxC_(0, 1)));
        out_q(1) = globalConfiguration_(1) * acos(cos_q2);
        out_q(2) = atan2(globalConfiguration_(2) * (auxA_(2, 0) * sin(psi) + auxB_(2, 0) * cos(psi) + auxC_(2, 0)),
                           globalConfiguration_(2) * (auxA_(2, 1) * sin(psi) + auxB_(2, 1) * cos(psi) + auxC_(2, 1)));

        out_q(3) = q4_v_;

        double cos_q6 = auxA_(5, 0) * sin(psi) + auxB_(5, 0) * cos(psi) + auxC_(5, 0);
        if (abs(cos_q6) > 1 + 1e-9){
            return false;
        }
        cos_q6 = std::max(-1., std::min(cos_q6, 1.));

        out_q(4) = atan2(globalConfiguration_(4) * (auxA_(4, 0) * sin(psi) + auxB_(4, 0) * cos(psi) + auxC_(4, 0)),
                           globalConfiguration_(4) * (auxA_(4, 1) * sin(psi) + auxB_(4, 1) * cos(psi) + auxC_(4, 1)));
        out_q(5) = globalConfiguration_(5) * acos(cos_q6);
        out_q(6) = atan2(globalConfiguration_(6) * (auxA_(6, 0) * sin(psi) + auxB_(6, 0) * cos(psi) + auxC_(6, 0)),
                           globalConfiguration_(6) * (auxA_(6, 1) * sin(psi) + auxB_(6, 1) * cos(psi) + auxC_(6, 1)));

        return checkFeasibility(out_q);
    }

    void Kinematics::getRedundancy(const Kinematics::JointArrayType &q, Vector3d &gc, double &psi){
        gc(0) = copysign(1.0, q(1));
        gc(1) = copysign(1.0, q(3));
        gc(2) = copysign(1.0, q(5));

        Vector3d p_0_e, p_0_2, p_0_4, p_0_6, p_2_6, p_6_7, vec_sew;
        Vector3d r_0_1_z, p_0_2_v, p_0_4_v, vec_sew_v;
        Matrix3d R_0_E;
        double q_1_v, q_2_v, q_3_v, q_4_v, phi;
        Matrix4d T_Tmp, T_total;

        p_0_2 << 0., 0., dBs_;
        p_6_7 << 0., 0., dWf_;

        // get q4 and p2_6 Shoulder-Elbow-Wrist (SEW) Plane
        T_total.setIdentity();
        transform_i(q(0), 0, T_Tmp);
        T_total *= T_Tmp;
        transform_i(q(1), 1, T_Tmp);
        T_total *= T_Tmp;
        transform_i(q(2), 2, T_Tmp);
        T_total *= T_Tmp;
        transform_i(q(3), 3, T_Tmp);
        T_total *= T_Tmp;
        p_0_4 = T_total.block<3, 1>(0, 3);
        transform_i(q(4), 4, T_Tmp);
        T_total *= T_Tmp;
        transform_i(q(5), 5, T_Tmp);
        T_total *= T_Tmp;
        p_0_6 = T_total.block<3, 1>(0, 3);
        p_2_6 = p_0_6 - p_0_2;

        double cosq_4_v = (p_2_6.squaredNorm() - pow(dSe_, 2) - pow(dEw_, 2)) / (2 * dSe_ * dEw_);
        q_4_v = gc(1) * acos(cosq_4_v);

        r_0_1_z <<0., 0., 1.;
        if (p_2_6.cross(r_0_1_z).norm() < 1e-3){
            q_1_v = 0;
        } else{
            q_1_v = atan2(p_2_6[1], p_2_6[0]);
        }

        phi = acos((pow(dSe_, 2) + p_2_6.squaredNorm() - pow(dEw_, 2)) / (2 * dSe_ * p_2_6.norm()));
        q_2_v = atan2(p_2_6.block<2, 1>(0, 0).norm(), p_2_6[2]) + gc(1) * phi;
        q_3_v = 0.;

        T_total.setIdentity();
        transform_i(q_1_v, 0, T_Tmp);
        T_total *= T_Tmp;
        transform_i(q_2_v, 1, T_Tmp);
        T_total *= T_Tmp;
        p_0_2_v = T_total.block<3, 1>(0, 3);
        transform_i(q_3_v, 2, T_Tmp);
        T_total *= T_Tmp;
        transform_i(q_4_v, 3, T_Tmp);
        T_total *= T_Tmp;
        p_0_4_v = T_total.block<3, 1>(0, 3);
        vec_sew_v = (p_0_4_v - p_0_2_v).normalized().cross(p_2_6.normalized());
        vec_sew = (p_0_4 - p_0_2).normalized().cross((p_2_6).normalized());
        double sign_psi = copysign(1.0, vec_sew_v.cross(vec_sew).dot(p_2_6));
        psi = sign_psi * acos(vec_sew_v.dot(vec_sew) / (vec_sew.norm() * vec_sew_v.norm()));
    }

    void Kinematics::tcp2EndEffector(const Vector3d &xTcp,
                                     const Quaterniond &quatTcp,
                                     Vector3d &xEE,
                                     Quaterniond &quatEE) {
        // R_0^tcp = R_0^e @ R_e^tcp -> R_0_e = R_0_tcp @ R_e_tcp.T
        quatEE = quatTcp.toRotationMatrix() * transformEE_.block<3, 3>(0, 0).transpose();
        // p_0^tcp = p_0^e + R_0^e * p_e_tcp -> p_0^e = p_0^tcp - R_0^e ^ p_e_tcp
        xEE = xTcp - quatEE.toRotationMatrix() * transformEE_.block<3, 1>(0, 3);
    }

    bool Kinematics::getAuxiliaryParameter(const Vector3d &xEE, const Quaterniond &quatEE) {
        Vector3d p0_2, p2_6, p6_7;
        JointArrayType qVirtual;
        double cos_q4_v, phi, p2_6_norm;
        TransformMatrixType T_0_3_v, T_3_4, T_tmp;
        Matrix3d R_0_3_v, R_3_4, R_0_7;

        R_0_7 = quatEE.toRotationMatrix();
        T_0_3_v.setIdentity();

        p0_2 << 0., 0., dBs_;
        p6_7 << 0., 0., dWf_;

        // get q4 and p2_6 Shoulder-Elbow-Wrist (SEW) Plane
        p2_6 = xEE - p0_2 - quatEE.toRotationMatrix() * p6_7;
        p2_6_norm = p2_6.norm();
        cos_q4_v = (p2_6.squaredNorm() - pow(dSe_, 2) - pow(dEw_, 2)) / (2 * dSe_ * dEw_);
        if (abs(cos_q4_v) > 1 + 1e-6) {
            return false;
        } else {
            cos_q4_v = std::max(-1., std::min(cos_q4_v, 1.));
            qVirtual(3) = globalConfiguration_(3) * acos(cos_q4_v);
            q4_v_ = qVirtual(3);
        }
        qVirtual(0) = atan2(p2_6(1), p2_6(0));
        phi = acos((pow(dSe_, 2) + p2_6.squaredNorm() - pow(dEw_, 2)) / (2 * dSe_ * p2_6_norm));
        qVirtual(1) = atan2(p2_6.block<2, 1>(0, 0).norm(), p2_6(2)) + globalConfiguration_(3) * phi;
        qVirtual(2) = 0.;

        transform_i(qVirtual(0), 0, T_tmp);
        T_0_3_v *= T_tmp;
        transform_i(qVirtual(1), 1, T_tmp);
        T_0_3_v *= T_tmp;
        transform_i(qVirtual(2), 2,  T_tmp);
        T_0_3_v *= T_tmp;
        R_0_3_v = T_0_3_v.block<3, 3>(0, 0);

        Vector3d p2_6_hat = p2_6 / p2_6_norm;
        Matrix3d cpm, A_s, B_s, C_s, A_w, B_w, C_w;
        cpm << 0., -p2_6_hat(2), p2_6_hat(1),
                p2_6_hat(2), 0., -p2_6_hat(0),
                -p2_6_hat(1), p2_6_hat(0), 0.;

        A_s = cpm * R_0_3_v;
        B_s = - (cpm * cpm) * R_0_3_v;
        C_s = p2_6_hat * p2_6_hat.transpose() * R_0_3_v;

        transform_i(qVirtual(3), 3,  T_tmp);
        R_3_4 = T_tmp.block<3, 3>(0, 0);

        A_w = R_3_4.transpose() * A_s.transpose() * R_0_7;
        B_w = R_3_4.transpose() * B_s.transpose() * R_0_7;
        C_w = R_3_4.transpose() * C_s.transpose() * R_0_7;

        auxA_(0, 0) = A_s(1, 1);
        auxB_(0, 0) = B_s(1, 1);
        auxC_(0, 0) = C_s(1, 1);
        auxA_(0, 1) = A_s(0, 1);
        auxB_(0, 1) = B_s(0, 1);
        auxC_(0, 1) = C_s(0, 1);

        auxA_(1, 0) = A_s(2, 1);
        auxB_(1, 0) = B_s(2, 1);
        auxC_(1, 0) = C_s(2, 1);

        auxA_(2, 0) = -A_s(2, 2);
        auxB_(2, 0) = -B_s(2, 2);
        auxC_(2, 0) = -C_s(2, 2);
        auxA_(2, 1) = -A_s(2, 0);
        auxB_(2, 1) = -B_s(2, 0);
        auxC_(2, 1) = -C_s(2, 0);

        auxA_(4, 0) = A_w(1, 2);
        auxA_(4, 1) = A_w(0, 2);
        auxB_(4, 0) = B_w(1, 2);
        auxB_(4, 1) = B_w(0, 2);
        auxC_(4, 0) = C_w(1, 2);
        auxC_(4, 1) = C_w(0, 2);

        auxA_(5, 0) = A_w(2, 2);
        auxB_(5, 0) = B_w(2, 2);
        auxC_(5, 0) = C_w(2, 2);

        auxA_(6, 0) = A_w(2, 1);
        auxA_(6, 1) = -A_w(2, 0);
        auxB_(6, 0) = B_w(2, 1);
        auxB_(6, 1) = -B_w(2, 0);
        auxC_(6, 0) = C_w(2, 1);
        auxC_(6, 1) = -C_w(2, 0);

        return true;
    }

    bool Kinematics::numericalInverseKinematics(const Vector3d &x, JointArrayType &qInOut, double tol, int max_iter) {
        JacobianPosType jac;
        Vector3d xCur, err;
        Matrix3d damper;
        damper = damper.setIdentity() * 1e-6;

        Matrix<double, 7, 7> E;
        E.setIdentity();
        for (int i = 0; i < max_iter; ++i) {
            jacobianPos(qInOut, jac);
            forwardKinematics(qInOut, xCur);

            err = x - xCur;

            if (err.norm() < tol){
                return checkFeasibility(qInOut);
            }
            if (err.norm() > 0.1){
                err = err.normalized() * 0.1;
            }

            auto jacInv = jac.transpose() * (jac * jac.transpose() + damper).inverse();
            qInOut += jacInv * err - 0.01 * (E - jacInv * jac) * qInOut;

            for (int j = 0; j < NUM_OF_JOINTS; ++j) {
                if (qInOut[j] < posLimitsLower_[j]) qInOut[j] = posLimitsLower_[j] + 0.1;
                else if (qInOut[j] > posLimitsUpper_[j]) qInOut[j] = posLimitsUpper_[j] - 0.1;
            }
        }
        return false;
    }

    bool Kinematics::checkFeasibility(const Kinematics::JointArrayType &qResult) {
        for (int i = 0; i < NUM_OF_JOINTS; ++i) {
            if (qResult(i) < posLimitsLower_(i) || qResult(i) > posLimitsUpper_(i) ){
                return false;
            }
        }
        return true;
    }

}
