import torch
import numpy as np
import pytorch3d.transforms as torch_trans

# import matplotlib.pyplot as plt

IIWA_JOINT_MIN_LIMITS = torch.deg2rad(torch.tensor([-170., -120., -170., -120., -170., -120., -175.]))
IIWA_JOINT_MAX_LIMITS = torch.deg2rad(torch.tensor([170., 120., 170., 120., 170., 120., 175.]))


class KinematicsTorch:
    def __init__(self, tcp_pos: torch.Tensor = None, tcp_quat: torch.Tensor = None, dtype: torch.dtype = torch.float,
                 device: str = "cpu"):
        """
        Analytic Kinematics for the Kuka IIWA LWR. Computations are based on
        https://repositorium.sdum.uminho.pt/bitstream/1822/48313/1/MAMT_3048.pdf

        :param tcp_pos: TCP Offset in the endeffector frame as a tensor of shape (3,)
        :param tcp_quat: TCP Orientation in the endeffector frame as a quaternion in (w,x,y,z) convention
        :param dtype: Data type to be used for computations. It is assumed that all arguments handed to this class are
                      in the correct datatype
        :param device: The device on which to perform the computations. It is assumed that all arguments are on the
                       device
        """

        self.d_bs = torch.tensor(0.36, dtype=dtype, device=device)
        self.d_se = torch.tensor(0.42, dtype=dtype, device=device)
        self.d_ew = torch.tensor(0.4, dtype=dtype, device=device)
        self.d_wf = torch.tensor(0.151, dtype=dtype, device=device)

        # DH_paramters a_i, alpha_i, d_i
        self.dh_a = torch.tensor([0., 0., 0., 0., 0., 0., 0.], dtype=dtype, device=device)
        self.dh_alpha = torch.tensor([-np.pi / 2, np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0.],
                                     dtype=dtype, device=device)
        self.dh_d = torch.tensor([self.d_bs, 0., self.d_se, 0., self.d_ew, 0., self.d_wf], dtype=dtype, device=device)

        self.joint_limits = torch.stack((IIWA_JOINT_MIN_LIMITS, IIWA_JOINT_MAX_LIMITS), dim=0).T.type(dtype).to(device)
        self.singularity_eps = 0.1

        self.T_ee = torch.eye(4, dtype=dtype, device=device)
        if tcp_pos is not None:
            self.T_ee[:3, 3] = tcp_pos
        if tcp_quat is not None:
            # Quat is given in (w,x,y,z) format
            self.T_ee[:3, :3] = torch_trans.quaternion_to_matrix(tcp_quat)

        self.dtype = dtype
        self.device = device
        self._ats, self._atw = self._build_assignment_tensor()

    def _build_assignment_tensor(self):
        """
        Helper function to create the tensors that shuffle the computed matrices in _get_auxiliary_parameter into the
        desired shape

        :return: Two permutation tensors
        """
        ats = torch.zeros((7, 2, 3, 3), dtype=self.dtype, device=self.device)
        ats[0, 0, 1, 1] = 1
        ats[0, 1, 0, 1] = 1
        ats[1, 0, 2, 1] = 1
        ats[2, 0, 2, 2] = -1
        ats[2, 1, 2, 0] = -1

        atw = torch.zeros((7, 2, 3, 3), dtype=self.dtype, device=self.device)
        atw[4, 0, 1, 2] = 1
        atw[4, 1, 0, 2] = 1
        atw[5, 0, 2, 2] = 1
        atw[6, 0, 2, 1] = 1
        atw[6, 1, 2, 0] = -1
        return ats, atw

    def _transform_i(self, index: int, theta_i: torch.Tensor):
        """
        Computes the homogenous transform of joint 'index' for the given angles

        :param index: The index of the joint for which the transform is to be computed
        :param theta_i: Angles of the specified joint. A tensor of arbitrary shape as batching is supported
        :return: Transformation matrices (..., 4, 4) where (...) corresponds to the shape of (theta_i)
        """

        alpha_i = self.dh_alpha[index]
        a_i = self.dh_a[index]
        d_i = self.dh_d[index]
        T_i = torch.diag_embed(torch.ones(theta_i.shape + (4,), dtype=self.dtype, device=self.device))
        T_i[..., 0, 0] = torch.cos(theta_i)
        T_i[..., 0, 1] = -torch.sin(theta_i) * torch.cos(alpha_i)
        T_i[..., 0, 2] = torch.sin(theta_i) * torch.sin(alpha_i)
        T_i[..., 0, 3] = a_i * torch.cos(theta_i)
        T_i[..., 1, 0] = torch.sin(theta_i)
        T_i[..., 1, 1] = torch.cos(theta_i) * torch.cos(alpha_i)
        T_i[..., 1, 2] = -torch.cos(theta_i) * torch.sin(alpha_i)
        T_i[..., 1, 3] = a_i * torch.sin(theta_i)
        T_i[..., 2, 0] = 0.
        T_i[..., 2, 1] = torch.sin(alpha_i)
        T_i[..., 2, 2] = torch.cos(alpha_i)
        T_i[..., 2, 3] = d_i
        return T_i

    def _transform(self, q: torch.Tensor):
        """
        Computes the forward kinematics in matrix representation

        :param q: Joint positions of shape [..., 7]
        :return: TCP frame of shape [..., 4, 4], where [..., :3, 3] contains the positions and [..., :3, :3]
                 the orientation as a rotation matrix
        """

        T = torch.diag_embed(torch.ones(q.shape[:-1] + (4,), dtype=self.dtype, device=self.device))
        for i in range(7):
            theta_i = q[..., i]
            T_i = self._transform_i(i, theta_i)
            T = torch.einsum("...ij,...jk->...ik", T, T_i)
        return torch.einsum("...ij,...jk->...ik", T, self.T_ee)

    def forward_kinematics(self, q: torch.Tensor):
        """
        Computes the forward kinematics in SE(3) representation

        :param q: Joint positions of shape [..., 7]
        :return: TCP frame of shape [..., 7], where [..., :3] contains the positions and [..., 3:7]  the orientation as
                 a quaternion in (w,x,y,z) format
        """
        T = self._transform(q)
        position = T[..., :-1, 3]
        rot_mat = T[..., :-1, :-1]
        return torch.cat([position, torch_trans.matrix_to_quaternion(rot_mat)], dim=-1)

    def get_jacobian(self, q: torch.Tensor):
        """
        Computes the Jacobian of the robot endeffector. Currently gives a (7,7) instead of a (6,7) Jacobian, which would
        be more standard

        :param q: Joint positions of shape (..., 7)
        :return: Jacobians for the given joint positions of shape (..., 7, 7)
        """
        q = q.requires_grad_(True)
        y = self.forward_kinematics(q)
        jac = torch.zeros(q.shape[:-1] + (7, 7)).double()
        for i in range(7):
            jac[..., i, :] = torch.autograd.grad(torch.sum(y[..., i]), q, create_graph=True, retain_graph=True)[0]
        return jac

    @staticmethod
    def _batch_cross(v1s, v2):
        """
        Helper function that computes a cross product between the vectors v1s and the vector v2. This only exists
        because the cross product function of torch does not support broadcasting in its cross method (v1.10.2)

        :param v1s: Vectors of shape (..., 3)
        :param v2: Vector of shape (3,)
        :return: Cross-Products of shape (..., 3)
        """
        skew_mat = torch.zeros((3, 3), dtype=v2.dtype, device=v2.device)
        skew_mat[0, 1] = -v2[2]
        skew_mat[0, 2] = v2[1]
        skew_mat[1, 0] = v2[2]
        skew_mat[1, 2] = -v2[0]
        skew_mat[2, 0] = -v2[1]
        skew_mat[2, 1] = v2[0]

        # Note that a x b = - b x a and [b]_x a = b x a
        return -torch.einsum("ij,...j->...i", skew_mat, v1s)

    def get_redundancy(self, q: torch.Tensor):
        """
        Computes the redundancy resolution angle psi for the given poses

        :param q: Joint positions of shape (..., 7)
        :return: Redundancy resolution angles (shape (...))
        """
        pose = self.forward_kinematics(q)
        gc_4 = torch.sign(q[..., 3])
        p = pose[..., :3]
        rot_mat = torch_trans.quaternion_to_matrix(pose[..., 3:])

        # T_ee is transposed in the upper einsum
        rot_0_e = torch.einsum("...ij,kj->...ik", rot_mat, self.T_ee[:3, :3])
        p_0_e = p - torch.einsum("...ij,j->...i", rot_0_e, self.T_ee[:3, 3])

        p_0_2 = torch.tensor([0., 0., self.d_bs], dtype=self.dtype, device=self.device)
        p_6_7 = torch.tensor([0., 0., self.d_wf], dtype=self.dtype, device=self.device)

        # get q4 and p2_6 Shoulder-Elbow-Wrist (SEW) Plane
        p_2_6 = p_0_e - p_0_2 - torch.einsum("...ij,j->...i", rot_0_e, p_6_7)

        cosq_4_v = (torch.norm(p_2_6, dim=-1) ** 2 - self.d_se ** 2 - self.d_ew ** 2) / (2 * self.d_se * self.d_ew)
        q_4_v = gc_4 * torch.acos(cosq_4_v)

        R_0_1_z = torch.tensor([0., 0., 1.], dtype=self.dtype, device=self.device)
        q_1_v = torch.where(torch.norm(self._batch_cross(p_2_6, R_0_1_z), dim=-1) < 1e-3, 0.,
                            torch.atan2(p_2_6[..., 1], p_2_6[..., 0]))

        phi = torch.acos((self.d_se ** 2 + torch.norm(p_2_6, dim=-1) ** 2 - self.d_ew ** 2) /
                         (2 * self.d_se * torch.norm(p_2_6, dim=-1)))

        q_2_v = torch.atan2(torch.sqrt(p_2_6[..., 0] ** 2 + p_2_6[..., 1] ** 2), p_2_6[..., 2]) + gc_4 * phi

        q_3_v = torch.tensor(0., dtype=self.dtype, device=self.device)

        p_0_2_v = torch.einsum("...ij,...jk->...ik", self._transform_i(0, q_1_v),
                               self._transform_i(1, q_2_v))[..., :3, 3]
        p_0_4_v = torch.einsum("...ij,...jk,kl,...lm->...im",
                               self._transform_i(0, q_1_v), self._transform_i(1, q_2_v), self._transform_i(2, q_3_v),
                               self._transform_i(3, q_4_v))[..., :3, 3]

        p_0_4 = torch.einsum("...ij,...jk,...kl,...lm->...im",
                             self._transform_i(0, q[..., 0]), self._transform_i(1, q[..., 1]),
                             self._transform_i(2, q[..., 2]), self._transform_i(3, q[..., 3]))[..., :3, 3]

        vec_2_4_v = (p_0_4_v - p_0_2_v) / torch.norm(p_0_4_v - p_0_2_v, dim=-1, keepdim=True)
        vec_2_6_v = p_2_6 / torch.norm(p_2_6, dim=-1, keepdim=True)
        vec_2_4 = (p_0_4 - p_0_2) / torch.norm(p_0_4 - p_0_2, dim=-1, keepdim=True)
        vec_2_6 = p_2_6 / torch.norm(p_2_6, dim=-1, keepdim=True)

        vec_sew_v = torch.cross(vec_2_4_v, vec_2_6_v, dim=-1)
        vec_sew = torch.cross(vec_2_4, vec_2_6, dim=-1)
        sign_psi = torch.sign(torch.einsum("...i,...i->...", torch.cross(vec_sew_v, vec_sew, dim=-1), p_2_6))
        psi = sign_psi * torch.acos(torch.einsum("...i,...i->...", vec_sew_v, vec_sew) /
                                    (torch.norm(vec_sew, dim=-1) * torch.norm(vec_sew_v, dim=-1)))
        return psi

    @staticmethod
    def _skew_symmetric(vecs: torch.Tensor):
        """
        Generates skew_symmetric matrices from given vectors

        :param vecs: Vectors (..., 3)
        :return: Skew-symmetric matrices (..., 3, 3)
        """
        res = torch.zeros(vecs.shape[:-1] + (3, 3), dtype=vecs.dtype, device=vecs.device)
        res[..., 0, 1] = -vecs[..., 2]
        res[..., 0, 2] = vecs[..., 1]
        res[..., 1, 0] = vecs[..., 2]
        res[..., 1, 2] = -vecs[..., 0]
        res[..., 2, 0] = -vecs[..., 1]
        res[..., 2, 1] = vecs[..., 0]

        return res

    def _get_auxiliary_parameter(self, p: torch.Tensor, rot_mat: torch.Tensor, gc_reduced: torch.Tensor):
        """
        Sub-step of the inverse kinematics computation

        :param p: Endeffector position of shape [..., 3]
        :param rot_mat: Endeffector orientation as a rotation matrix of shape [..., 3, 3]
        :param gc_reduced: Redundancy resolution parameters
        :return: Auxiliary parameters for the inverse kinematics computation
        """
        if len(p.shape) == 1:
            raise RuntimeError("_get_auxiliary_parameter needs to be called with at least one batching dimension")

        p0_2 = torch.tensor([0., 0., self.d_bs], dtype=self.dtype, device=self.device)
        p6_7 = torch.tensor([0., 0., self.d_wf], dtype=self.dtype, device=self.device)
        R0_7 = rot_mat

        gc = torch.stack([gc_reduced[..., 0], gc_reduced[..., 0], gc_reduced[..., 0], gc_reduced[..., 1],
                          gc_reduced[..., 2], gc_reduced[..., 2], gc_reduced[..., 2]], dim=-1)

        # get q4 and p2_6 Shoulder-Elbow-Wrist (SEW) Plane
        p2_6 = p - p0_2 - torch.einsum("...ij,j->...i", rot_mat, p6_7)
        cosq4_v = (torch.norm(p2_6, dim=-1) ** 2 - self.d_se ** 2 - self.d_ew ** 2) / (2 * self.d_se * self.d_ew)

        mask = torch.abs(cosq4_v) <= 1 + 1e-9
        # We just compute with all values to avoid unnecessary masking and copies afterwards, even if only the
        # results for which mask=True are valid
        q4_v = gc_reduced[..., 1] * torch.acos(torch.clamp(cosq4_v, -1., 1.))

        # From here on we only work with the masked versions
        p2_6_norm = torch.norm(p2_6, dim=-1)

        q1_v = torch.atan2(p2_6[..., 1], p2_6[..., 0])
        phi = torch.acos((self.d_se ** 2 + p2_6_norm ** 2 - self.d_ew ** 2) / (2 * self.d_se * p2_6_norm))
        q2_v = torch.atan2(torch.sqrt(p2_6[..., 0] ** 2 + p2_6[..., 1] ** 2), p2_6[..., 2]) + gc[..., 1] * phi
        q3_v = torch.tensor(0.0)
        T_0_3_v = torch.einsum("...ij,...jk,kl->...il", self._transform_i(0, q1_v), self._transform_i(1, q2_v),
                               self._transform_i(2, q3_v))
        R_0_3_v = T_0_3_v[..., :3, :3]

        # cross product matrixq_true
        p2_6_hat = p2_6 / p2_6_norm[..., None]
        cpm = self._skew_symmetric(p2_6_hat)
        A_s = torch.einsum("...ij,...jk->...ik", cpm, R_0_3_v)
        B_s = -torch.einsum("...ij,...jk,...kl->...il", cpm, cpm, R_0_3_v)
        C_s = torch.einsum("...i,...j,...jk->...ik", p2_6_hat, p2_6_hat, R_0_3_v)

        T_3_4 = self._transform_i(3, q4_v)
        R_3_4 = T_3_4[..., :3, :3]

        # First two arguments are transposed
        A_w = torch.einsum("...ji,...kj,...kl->...il", R_3_4, A_s, R0_7)
        B_w = torch.einsum("...ji,...kj,...kl->...il", R_3_4, B_s, R0_7)
        C_w = torch.einsum("...ji,...kj,...kl->...il", R_3_4, C_s, R0_7)

        # x[3, :]=0 For joint 4 is zero
        # x[1 5, 1]=0 a_d, b_d, c_d =0
        a = torch.einsum("ijkl,...kl->...ij", self._ats, A_s) + torch.einsum("ijkl,...kl->...ij", self._atw, A_w)
        b = torch.einsum("ijkl,...kl->...ij", self._ats, B_s) + torch.einsum("ijkl,...kl->...ij", self._atw, B_w)
        c = torch.einsum("ijkl,...kl->...ij", self._ats, C_s) + torch.einsum("ijkl,...kl->...ij", self._atw, C_w)

        return mask, gc, q4_v, a, b, c

    def _transform_tcp2ee(self, pose):
        """
        Transform TCP Poses in SE(3) representation to Endeffector Poses in matrix representation

        :param pose: TCP Poses (..., 7) in SE(3) representation
        :return: Position of shape (..., 3) and Orientation as a rotation matrix (..., 3, 3)
        """

        # R_0_tcp = R_0_e @ R_e_tcp -> R_0_e = R_0_tcp @ R_e_tcp.T
        rot_0_e = torch.einsum("...ij,jk->...ik", torch_trans.quaternion_to_matrix(pose[..., 3:]), self.T_ee[:3, :3].T)
        # p_0_tcp = p_0_e + R_0_e * p_e_tcp
        p_0_e = pose[..., :3] - torch.einsum("...ij,j->...i", rot_0_e, self.T_ee[:3, 3])
        return p_0_e, rot_0_e

    def inverse_kinematics(self, pose, psi, gc):
        """
        Computes the inverse kinematics for the given poses and redundancy resolution parameters gc and psi

        :param pose: The TCP poses in SE(3) representation of shape (..., 7)
        :param psi: Arm angle to virtual reference pose of shape (...). Note that for reliable results psi should be
                    computed with the get_feasible_interval function
        :param gc: The shoulder, elbow and wrist configurations of shape (..., 3)
        :return: The joint configuration that achieves the desired pose under the given redundancy resolution parameters
        """

        # We ensure that there is at least one batching dimension because _get_auxiliary_parameter requires this
        # For single inputs we still add a batch dimension to avoid masking problems
        if len(pose.shape) == 1:
            pose = pose[None, :]
            # This can be easy to get wrong (i.e. passing a 1D Vector with 1 element instead of scalar). SO we check for
            # this here
            if len(psi.shape) == 0:
                psi = psi[None, ...]
            gc = gc[None, ...]

        # Sanity check to avoid subtle batching errors
        if gc.shape != pose.shape[:-1] + (3,) or psi.shape != pose.shape[:-1]:
            raise RuntimeError("Invalid arguments")

        q = torch.ones(pose.shape[:-1] + (7,), dtype=self.dtype, device=self.device) * torch.nan
        # get auxiliary parameter a, b, c: [7, 2],

        p_0_e, rot_0_e = self._transform_tcp2ee(pose)
        mask, gc, q4_v, a, b, c = self._get_auxiliary_parameter(p_0_e, rot_0_e, gc)

        cos_q2 = a[..., 1, 0] * torch.sin(psi[...]) + b[..., 1, 0] * torch.cos(psi[...]) + c[..., 1, 0]
        mask = torch.logical_and(mask, torch.abs(cos_q2) <= 1 + 1e-9)
        cos_q2 = torch.clamp(cos_q2, -1, 1)

        q[..., 0] = torch.atan2(gc[..., 0] * (a[..., 0, 0] * torch.sin(psi) +
                                              b[..., 0, 0] * torch.cos(psi) + c[..., 0, 0]),
                                gc[..., 0] * (a[..., 0, 1] * torch.sin(psi) +
                                              b[..., 0, 1] * torch.cos(psi) + c[..., 0, 1]))
        q[..., 1] = gc[..., 1] * torch.acos(cos_q2)
        q[..., 2] = torch.atan2(gc[..., 2] * (a[..., 2, 0] * torch.sin(psi) +
                                              b[..., 2, 0] * torch.cos(psi) + c[..., 2, 0]),
                                gc[..., 2] * (a[..., 2, 1] * torch.sin(psi) +
                                              b[..., 2, 1] * torch.cos(psi) + c[..., 2, 1]))

        q[..., 3] = q4_v

        cos_q6 = a[..., 5, 0] * torch.sin(psi) + b[..., 5, 0] * torch.cos(psi) + c[..., 5, 0]
        mask = torch.logical_and(mask, torch.abs(cos_q6) <= 1 + 1e-9)
        cos_q6 = torch.clamp(cos_q6, -1, 1)

        q[..., 4] = torch.atan2(gc[..., 4] * (a[..., 4, 0] * torch.sin(psi) +
                                              b[..., 4, 0] * torch.cos(psi) + c[..., 4, 0]),
                                gc[..., 4] * (a[..., 4, 1] * torch.sin(psi) +
                                              b[..., 4, 1] * torch.cos(psi) + c[..., 4, 1]))
        q[..., 5] = gc[..., 5] * torch.acos(cos_q6)
        q[..., 6] = torch.atan2(gc[..., 6] * (a[..., 6, 0] * torch.sin(psi) +
                                              b[..., 6, 0] * torch.cos(psi) + c[..., 6, 0]),
                                gc[..., 6] * (a[..., 6, 1] * torch.sin(psi) +
                                              b[..., 6, 1] * torch.cos(psi) + c[..., 6, 1]))
        return torch.squeeze(mask), torch.squeeze(q)
