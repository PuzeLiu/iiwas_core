import numpy as np
from iiwas_kinematics import Kinematics

kin = Kinematics()
joint_pos = 0.5 * np.ones(7)
pos, quat = kin.forward_kinematics(joint_pos)
gc, psi = kin.get_redundancy(joint_pos)
joint_rec = kin.inverse_kinematics(pos, quat, gc, psi)
assert np.linalg.norm(joint_rec - joint_pos) < 1e-10

jac = kin.jacobian(joint_pos)
jac_pos = kin.jacobian_pos(joint_pos)
jac_rot = kin.jacobian_rot(joint_pos)
assert np.all(np.abs(jac - np.concatenate([jac_pos, jac_rot], axis=0)) < 1e-10)

