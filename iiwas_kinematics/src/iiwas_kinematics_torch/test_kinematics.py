import time
from tqdm import tqdm
import torch
import numpy as np
from kinematics import KinematicsTorch
from kinematics_single import KinematicsTorch as KinematicsTorchOld, Rotation


def speed_test(n_test, device='cuda:0'):
    yzy_test = torch.rand(3) * torch.tensor([2 * np.pi, np.pi, 2 * np.pi]) - torch.tensor([np.pi, 0, np.pi])
    pos_offset = torch.rand(3)
    kin_gc = KinematicsTorch(tcp_pos=pos_offset.clone(), tcp_quat=Rotation.from_euler_yzy(yzy_test).as_quat(),
                             device=device)
    psi_gc = torch.rand(n_test, device=device)

    q_gc = torch.deg2rad(torch.rand((n_test, 7), device=device) * 360 - 180)
    gc_gc = torch.sign(q_gc[:, [1, 3, 5]])
    t1 = time.time()
    pose_gc = kin_gc.forward_kinematics(q_gc)
    mask, intervals, auxiliary_parameters = kin_gc.get_feasible_interval(pose_gc, gc_gc)
    kin_gc.inverse_kinematics(pose_gc, psi_gc, gc_gc, auxiliary_parameters=auxiliary_parameters)
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
    psi_test = torch.zeros((n_test,), dtype=torch.double)  # torch.squeeze(torch.rand(1, dtype=torch.double))
    gc_test = torch.sign(torch.rand((n_test, 3), dtype=torch.double) - 0.5)

    # We first check against the reference implementation (and save the results)
    Ts = []
    poses = []
    masks = []
    q_invs = []
    jacobians = []
    for i in tqdm(range(n_test)):
        T = kin._transform(q_test[i, :].clone())
        T_ref = kin_old._transform(q_test[i, :].clone())
        # These values should be exactly the same
        assert torch.allclose(T, T_ref)
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

        res, q_inv = kin.inverse_kinematics(pose_test.clone(), psi_test[i].clone(), gc_test[i, :].clone())
        res_ref, q_inv_ref = kin_old.inverse_kinematics(pose_test.clone(), psi_test[i].clone(), gc_test[i, :].clone())

        # Due to numerical differences (-1e-18 vs 1e-18), there can be a sign flip if values are close to pi. Hence we
        # compare sine and cosine of the values
        assert torch.allclose(torch.sin(q_inv), torch.sin(q_inv_ref.double()))
        assert torch.allclose(torch.cos(q_inv), torch.cos(q_inv_ref.double()))
        assert torch.all(res)
        assert res == res_ref
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
    mask_batch, q_inv_batch = kin.inverse_kinematics(poses, psi_test, gc_test)
    assert torch.all(masks == mask_batch)
    assert torch.allclose(q_invs, q_inv_batch)

    # Check the batching with more interesting batch dimensions
    assert torch.allclose(kin._transform(q_test),
                          kin._transform(q_test.reshape((n_test // 2, 2, 7))).reshape(n_test, 4, 4))
    assert torch.allclose(kin.forward_kinematics(q_test),
                          kin.forward_kinematics(q_test.reshape((n_test // 2, 2, 7))).reshape(n_test, 7))
    assert torch.allclose(jacobians, kin.get_jacobian(q_test.reshape((n_test // 2, 2, 7))).reshape(n_test, 7, 7))
    mask_batch_2, q_inv_batch_2 = kin.inverse_kinematics(poses.reshape((n_test // 2, 2, 7)),
                                                         psi_test.reshape(n_test // 2, 2),
                                                         gc_test.reshape(n_test // 2, 2, 3))
    assert torch.allclose(mask_batch, mask_batch_2.reshape(-1))
    assert torch.allclose(q_inv_batch, q_inv_batch_2.reshape((-1, 7)))

    assert torch.allclose(kin._transform(q_test),
                          kin._transform(q_test.reshape((n_test // 4, 2, 2, 7))).reshape(n_test, 4, 4))
    assert torch.allclose(kin.forward_kinematics(q_test),
                          kin.forward_kinematics(q_test.reshape((n_test // 4, 2, 2, 7))).reshape(n_test, 7))
    assert torch.allclose(jacobians, kin.get_jacobian(q_test.reshape((n_test // 4, 2, 2, 7))).reshape(n_test, 7, 7))
    mask_batch_2, q_inv_batch_2 = kin.inverse_kinematics(poses.reshape((n_test // 4, 2, 2, 7)),
                                                         psi_test.reshape(n_test // 4, 2, 2),
                                                         gc_test.reshape(n_test // 4, 2, 2, 3))
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
    gc = torch.sign(q_test[:, [1, 3, 5]])
    pose_test = kin.forward_kinematics(q_test)
    res_new, q_test_out_new = kin.inverse_kinematics(pose_test.clone(), psis.clone(), gc.clone())

    for i in tqdm(range(n_test)):
        psi_test = kin_old.get_redundancy(q_test[i, :])
        assert torch.isclose(psi_test, psis[i])

        pose_test_cur = kin.forward_kinematics(q_test[i, :])
        assert torch.allclose(pose_test_cur, pose_test[i, :])

        res_cur, q_test_out_cur = kin.inverse_kinematics(pose_test[i, :].clone(), psi_test.clone(), gc[i, :].clone())
        assert res_cur == res_new[i]
        assert torch.allclose(q_test_out_cur, q_test_out_new[i, :])

        res, q_test_out = kin_old.inverse_kinematics(pose_test[i, :], psi_test, gc[i, :])
        assert res_cur and res
        assert torch.allclose(q_test_out_cur, q_test_out)
        if not torch.allclose(q_test[i, :], q_test_out_cur, atol=tol):
            print("psi is not correct")


def verify_feasible_intervals(n_test):
    yzy_test = torch.rand(3) * torch.tensor([2 * np.pi, np.pi, 2 * np.pi]) - torch.tensor([np.pi, 0, np.pi])
    pos_offset = torch.rand(3, dtype=torch.double)
    kin = KinematicsTorch(tcp_pos=pos_offset.clone(), tcp_quat=Rotation.from_euler_yzy(yzy_test).as_quat(),
                          dtype=torch.double)
    kin_old = KinematicsTorchOld(tcp_pos=pos_offset.clone(), tcp_quat=Rotation.from_euler_yzy(yzy_test).as_quat())
    feasible_intervals = []
    q_test = torch.deg2rad(torch.rand((n_test, 7), dtype=torch.double) * 360 - 180)
    pose = kin.forward_kinematics(q_test)
    gc = torch.sign(torch.rand((n_test, 3), dtype=torch.double) - 0.5)
    for i in tqdm(range(n_test)):
        res_old = kin_old.get_feasible_interval(pose[i, :].clone(), gc[i, :].clone())
        res_new = kin.get_feasible_interval(pose[i, :].clone(), gc[i, :].clone())

        if res_old.shape[0] == 0:
            assert res_new[1][res_new[0]].shape[0] == 0
        else:
            # For single predictions we should remove all invalid intervals
            torch.all(res_new[0])
            assert torch.allclose(res_old.double(), res_new[1])

        feasible_intervals.append(res_new[1][res_new[0]])

    # Check the batched computation
    res_new = kin.get_feasible_interval(pose, gc)
    for i in range(n_test):
        assert torch.allclose(res_new[1][i, res_new[0][i, :]], feasible_intervals[i])

    res_new2 = kin.get_feasible_interval(pose.reshape(n_test // 4, 2, 2, -1), gc.reshape(n_test // 4, 2, 2, -1))
    assert torch.allclose(res_new2[0].reshape(n_test, -1), res_new[0])
    assert torch.allclose(res_new2[1].reshape(n_test, -1, 2), res_new[1])


def main():
    seed = 1
    np.random.seed(seed)
    torch.manual_seed(seed)

    verify_feasible_intervals(10000)
    print("Tested feasible interval computation")
    verify_redundancy(10000, tol=1e-4)
    print("Tested redundancy computations")
    verify_computations(10000, tol=1e-4)
    print("Tested kinematics computations")
    speed_test(int(1e4), device='cpu')


if __name__ == "__main__":
    main()
