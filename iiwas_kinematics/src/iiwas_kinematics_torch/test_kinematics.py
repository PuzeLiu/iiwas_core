import time
import torch
import numpy as np
from sim.ik.kinematics import KinematicsTorch
from sim.ik.kinematics_single import KinematicsTorch as KinematicsTorchOld, Rotation


def speed_test(n_test):
    yzy_test = torch.rand(3) * torch.tensor([2 * np.pi, np.pi, 2 * np.pi]) - torch.tensor([np.pi, 0, np.pi])
    pos_offset = torch.rand(3)
    kin_gc = KinematicsTorch(tcp_pos=pos_offset.clone(), tcp_quat=Rotation.from_euler_yzy(yzy_test).as_quat(),
                             device="cuda:0")
    psi_gc = torch.rand(n_test, device="cuda:0")
    gc_gc = torch.ones((n_test, 3), device="cuda:0")

    q_gc = torch.deg2rad(torch.rand((n_test, 7), device="cuda:0") * 360 - 180)
    t1 = time.time()
    pose_gc = kin_gc.forward_kinematics(q_gc)
    kin_gc.inverse_kinematics(pose_gc, psi_gc, gc_gc)
    t2 = time.time()
    print("Computing %d forward and inverse kinematics took %.3e seconds" % (n_test, t2 - t1))


def verify_computations(n_test, tol=1e-4):
    if n_test % 4 != 0:
        raise RuntimeError("Number of test data points need to be divisable by two")

    # Compare against the reference implementation
    yzy_test = torch.rand(3) * torch.tensor([2 * np.pi, np.pi, 2 * np.pi]) - torch.tensor([np.pi, 0, np.pi])
    pos_offset = torch.rand(3, dtype=torch.double)
    kin = KinematicsTorch(tcp_pos=pos_offset.clone(), tcp_quat=Rotation.from_euler_yzy(yzy_test).as_quat(),
                          dtype=torch.double)
    kin_old = KinematicsTorchOld(tcp_pos=pos_offset.clone(), tcp_quat=Rotation.from_euler_yzy(yzy_test).as_quat())

    assert torch.allclose(kin.T_ee, kin_old.T_ee)
    kin.T_ee = kin_old.T_ee.clone()
    assert torch.all(kin.T_ee - kin_old.T_ee == 0.)
    assert torch.all(kin.dh_alpha - kin_old.dh_alpha == 0.)
    assert torch.all(kin.dh_a - kin_old.dh_a == 0.)
    assert torch.all(kin.dh_d - kin_old.dh_d == 0.)

    q_test = torch.deg2rad(torch.rand((n_test, 7), dtype=torch.double) * 360 - 180)
    psi_test = torch.squeeze(torch.zeros(1, dtype=torch.double))  # torch.squeeze(torch.rand(1, dtype=torch.double))
    gc_test = torch.tensor([1, 1, 1], dtype=torch.double)

    # We first check against the reference implementation (and save the results)
    Ts = []
    poses = []
    masks = []
    q_invs = []
    jacobians = []
    for i in range(n_test):
        T = kin._transform(q_test[i, :].clone())
        T_ref = kin_old._transform(q_test[i, :].clone())
        # These values should be exactly the same
        assert torch.all(T - T_ref == 0)
        Ts.append(T)

        pose_test = kin.forward_kinematics(q_test[i, :].clone())
        jac_test = kin.get_jacobian(q_test[i, :].clone())
        pose_test_ref = kin_old.forward_kinematics(q_test[i, :].clone())
        jac_test_ref = kin_old.get_jacobian(q_test[i, :].clone())
        # The computed quaternions may be slightly different and have a sign flip, as we use different packages for
        # conversion between them
        assert torch.allclose(jac_test[:3, :], jac_test_ref[:3, :]) and \
               (torch.allclose(jac_test[3:, :], jac_test_ref[3:, :]) or
                torch.allclose(-jac_test[3:, :], jac_test_ref[3:, :]))
        assert torch.allclose(pose_test[:3], pose_test_ref[:3]) and \
               (torch.allclose(pose_test[3:], pose_test_ref[3:]) or torch.allclose(-pose_test[3:], pose_test_ref[3:]))
        poses.append(pose_test)
        jacobians.append(jac_test)

        res, q_inv = kin.inverse_kinematics(pose_test.clone(), psi_test.clone(), gc_test.clone())
        res_ref, q_inv_ref = kin_old.inverse_kinematics(pose_test.clone(), psi_test.clone(), gc_test.clone())
        # Again this may not be exactly the same since we are using different packages to convert from quaternions
        # to rotation matrices
        assert torch.allclose(q_inv, q_inv_ref.double())
        assert torch.all(res)
        masks.append(res)
        q_invs.append(q_inv)

        if res:
            pose_inv = kin.forward_kinematics(q_inv)
            if not torch.allclose(pose_inv[:3], pose_test[:3], atol=tol):
                position_error = torch.norm(pose_test[:3] - pose_inv[:3])
                print("Position Error:", position_error.item())

            if not (torch.allclose(pose_inv[3:], pose_test[3:], atol=tol) or
                    torch.allclose(-pose_inv[3:], pose_test[3:], atol=tol)):
                rotation_error = torch.norm(pose_test[3:] - pose_inv[3:])
                print("Rotation Error", rotation_error.item())

    Ts = torch.stack(Ts, dim=0)
    poses = torch.stack(poses, dim=0)
    masks = torch.stack(masks, dim=0)
    q_invs = torch.stack(q_invs, dim=0)
    jacobians = torch.stack(jacobians, dim=0)

    # Check the batching
    assert torch.allclose(Ts, kin._transform(q_test))
    assert torch.allclose(poses, kin.forward_kinematics(q_test))
    assert torch.allclose(jacobians, kin.get_jacobian(q_test))
    mask_batch, q_inv_batch = kin.inverse_kinematics(poses, psi_test[None].repeat(n_test),
                                                     gc_test[None].repeat(n_test, 1))
    assert torch.all(masks == mask_batch)
    assert torch.allclose(q_invs, q_inv_batch)

    # Check the batching with more interesting batch dimensions
    assert torch.allclose(kin._transform(q_test),
                          kin._transform(q_test.reshape((n_test // 2, 2, 7))).reshape(n_test, 4, 4))
    assert torch.allclose(kin.forward_kinematics(q_test),
                          kin.forward_kinematics(q_test.reshape((n_test // 2, 2, 7))).reshape(n_test, 7))
    assert torch.allclose(jacobians, kin.get_jacobian(q_test.reshape((n_test // 2, 2, 7))).reshape(n_test, 7, 7))
    mask_batch_2, q_inv_batch_2 = kin.inverse_kinematics(poses.reshape((n_test // 2, 2, 7)),
                                                         psi_test[None, None].repeat(n_test // 2, 2),
                                                         gc_test[None, None, :].repeat(n_test // 2, 2, 1))
    assert torch.allclose(mask_batch, mask_batch_2.reshape(-1))
    assert torch.allclose(q_inv_batch, q_inv_batch_2.reshape((-1, 7)))

    assert torch.allclose(kin._transform(q_test),
                          kin._transform(q_test.reshape((n_test // 4, 2, 2, 7))).reshape(n_test, 4, 4))
    assert torch.allclose(kin.forward_kinematics(q_test),
                          kin.forward_kinematics(q_test.reshape((n_test // 4, 2, 2, 7))).reshape(n_test, 7))
    assert torch.allclose(jacobians, kin.get_jacobian(q_test.reshape((n_test // 4, 2, 2, 7))).reshape(n_test, 7, 7))
    mask_batch_2, q_inv_batch_2 = kin.inverse_kinematics(poses.reshape((n_test // 4, 2, 2, 7)),
                                                         psi_test[None, None].repeat(n_test // 4, 2, 2),
                                                         gc_test[None, None, :].repeat(n_test // 4, 2, 2, 1))
    assert torch.allclose(mask_batch, mask_batch_2.reshape(-1))
    assert torch.allclose(q_inv_batch, q_inv_batch_2.reshape((-1, 7)))


def verify_redundancy(n_test, tol=1e-4):
    if n_test % 4 != 0:
        raise RuntimeError("Number of test data points need to be divisable by four")

    # Compare against the reference implementation
    yzy_test = torch.rand(3) * torch.tensor([2 * np.pi, np.pi, 2 * np.pi]) - torch.tensor([np.pi, 0, np.pi])
    pos_offset = torch.rand(3, dtype=torch.double)
    kin = KinematicsTorch(tcp_pos=pos_offset.clone(), tcp_quat=Rotation.from_euler_yzy(yzy_test).as_quat(),
                          dtype=torch.double)
    kin_old = KinematicsTorchOld(tcp_pos=pos_offset.clone(), tcp_quat=Rotation.from_euler_yzy(yzy_test).as_quat())

    q_test = torch.deg2rad(torch.rand((n_test, 7), dtype=torch.double) * 360 - 180)

    psis = kin.get_redundancy(q_test)

    for i in range(n_test):
        psi_test = kin_old.get_redundancy(q_test[i, :])
        assert torch.isclose(psi_test, psis[i])

        gc = torch.sign(q_test[i, [1, 3, 5]])
        pose_test = kin_old.forward_kinematics(q_test[i, :])
        res, q_test_out = kin_old.inverse_kinematics(pose_test, psi_test, gc)
        assert res
        if not torch.allclose(q_test[i, :], q_test_out, atol=tol):
            print("psi is not correct")


def main():
    seed = 1
    np.random.seed(seed)
    torch.manual_seed(seed)

    verify_computations(1000, tol=1e-4)
    verify_redundancy(1000, tol=1e-4)
    speed_test(int(1e6))


if __name__ == "__main__":
    main()
