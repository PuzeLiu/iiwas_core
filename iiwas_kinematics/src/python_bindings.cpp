#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <iiwas_kinematics/iiwas_kinematics.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tuple>

namespace py = pybind11;


PYBIND11_MODULE(iiwas_kinematics, m) {
	py::class_<iiwas_kinematics::Kinematics>(m, "Kinematics")
		.def(py::init<>())
		.def(py::init([](Eigen::Vector3d vec, Eigen::Vector4d quat_raw) {
					Eigen::Quaterniond quat(quat_raw(0), quat_raw(1), quat_raw(2), quat_raw(3));
					return iiwas_kinematics::Kinematics(vec, quat);
		}))
		.def("forward_kinematics", [](iiwas_kinematics::Kinematics *self, iiwas_kinematics::Kinematics::JointArrayType q){
				Eigen::Vector3d vec;
				Eigen::Quaterniond quat;
				self->forwardKinematics(q, vec, quat);
				Eigen::Vector4d quat_raw(quat.w(), quat.x(), quat.y(), quat.z());
				return std::make_tuple(vec, quat_raw);
		})
		.def("get_redundancy", [](iiwas_kinematics::Kinematics *self, iiwas_kinematics::Kinematics::JointArrayType q){
				double psi;
				Eigen::Vector3d gc;
				self->getRedundancy(q, gc, psi);
				return std::make_tuple(gc, psi);

		})
		.def("inverse_kinematics", [](iiwas_kinematics::Kinematics *self, Eigen::Vector3d x, Eigen::Vector4d quat_raw, Eigen::Vector3d global_configuration, double psi){
				Eigen::Quaterniond quat(quat_raw(0), quat_raw(1), quat_raw(2), quat_raw(3));
				iiwas_kinematics::Kinematics::JointArrayType q;
				self->inverseKinematics(x, quat, global_configuration, psi, q);
				return q;
		})
		.def("numerical_inverse_kinematics", [](iiwas_kinematics::Kinematics *self, Eigen::Vector3d x, iiwas_kinematics::Kinematics::JointArrayType q, double tol, int max_iter){
				iiwas_kinematics::Kinematics::JointArrayType q_res;
				for(uint32_t i = 0; i < 7; i++){
					q_res(i) = q(i);
				}
				self->numericalInverseKinematics(x, q_res, tol, max_iter);
				return q_res;
		})
		.def("jacobian", [](iiwas_kinematics::Kinematics *self, iiwas_kinematics::Kinematics::JointArrayType q){
				iiwas_kinematics::Kinematics::JacobianType jac;
				self->jacobian(q, jac);
				return jac;
		})
		.def("jacobian_pos", [](iiwas_kinematics::Kinematics *self, iiwas_kinematics::Kinematics::JointArrayType q){
				iiwas_kinematics::Kinematics::JacobianPosType jac;
				self->jacobianPos(q, jac);
				return jac;
		})
		.def("jacobian_rot", [](iiwas_kinematics::Kinematics *self, iiwas_kinematics::Kinematics::JointArrayType q){
				iiwas_kinematics::Kinematics::JacobianRotType jac;
				self->jacobianRot(q, jac);
				return jac;
		});

}
