import numpy as np
import matplotlib.pyplot as plt
import pinocchio as pino


def check_min_max_inertia():
    m_max = 0
    m_min = 0

    m_base = pino.crba(pino_model, pino_data, np.zeros(7))
    for i in range(100000):
        q = np.random.uniform(-1, 1, 7)
        q = q * pino_model.upperPositionLimit
        M = pino.crba(pino_model, pino_data, q)

        m_norm = M / np.diag(m_base)[:, np.newaxis]

        m_max = np.maximum(np.max(m_norm), m_max)
        m_min = np.minimum(np.min(m_norm), m_min)

    print("Max: ", m_max)
    print("Min: ", m_min)


def plot_inertia(M_list, q_list, title):
    n = len(M_list)
    fig = plt.figure(figsize=(12, 4))
    fig.suptitle(title)
    ax = fig.subplots(1, n)
    for i in range(n):
        m_norm = M_list[i] / np.diag(M_list[0])[:, np.newaxis]
        im = ax[i].imshow(m_norm, vmin=-10, vmax=10, cmap='plasma')
        ax[i].set_xticks(np.arange(7))
        ax[i].set_yticks(np.arange(7))
        ax[i].set_xticklabels(['1', '2', '3', '4', '5', '6', '7'])
        ax[i].set_yticklabels(['1', '2', '3', '4', '5', '6', '7'])
        ax[i].set_title("q: " + str(q_list[i] / np.pi * 180))
    cax = plt.axes([0.1, 0.1, 0.8, 0.05])
    plt.colorbar(im, cax=cax, orientation='horizontal')
    plt.savefig(title + ".pdf")


if __name__ == '__main__':
    urdf_file = "urdf/iiwa.urdf"
    pino_model = pino.buildModelFromUrdf(urdf_file)
    pino_data = pino_model.createData()

    check_min_max_inertia()

    # Joint 1
    q = np.zeros(7)
    M_list = []
    q_list = []
    for q_i in np.arange(0, 91, 30):
        q[0] = q_i / 180 * np.pi
        M = pino.crba(pino_model, pino_data, q)
        M_list.append(M)
        q_list.append(q.copy())
    plot_inertia(M_list, q_list, "joint_1")

    # Joint 2
    q = np.zeros(7)
    M_list = []
    q_list = []
    for q_i in np.arange(0, 91, 30):
        q[1] = q_i / 180 * np.pi
        M = pino.crba(pino_model, pino_data, q)
        M_list.append(M)
        q_list.append(q.copy())
    plot_inertia(M_list, q_list, "joint_2")

    # Joint 3
    q = np.zeros(7)
    M_list = []
    q_list = []
    for q_i in np.arange(0, 91, 30):
        q[2] = q_i / 180 * np.pi
        M = pino.crba(pino_model, pino_data, q)
        M_list.append(M)
        q_list.append(q.copy())
    plot_inertia(M_list, q_list, "joint_3")

    # Joint 4
    q = np.zeros(7)
    M_list = []
    q_list = []
    for q_i in np.arange(0, 91, 30):
        q[3] = q_i / 180 * np.pi
        M = pino.crba(pino_model, pino_data, q)
        M_list.append(M)
        q_list.append(q.copy())
    plot_inertia(M_list, q_list, "joint_4")

    # Joint 5
    q = np.zeros(7)
    M_list = []
    q_list = []
    for q_i in np.arange(0, 91, 30):
        q[4] = q_i / 180 * np.pi
        M = pino.crba(pino_model, pino_data, q)
        M_list.append(M)
        q_list.append(q.copy())
    plot_inertia(M_list, q_list, "joint_5")

    # Joint 6
    q = np.zeros(7)
    M_list = []
    q_list = []
    for q_i in np.arange(0, 91, 30):
        q[5] = q_i / 180 * np.pi
        M = pino.crba(pino_model, pino_data, q)
        M_list.append(M)
        q_list.append(q.copy())
    plot_inertia(M_list, q_list, "joint_6")

    # Joint 7
    q = np.zeros(7)
    M_list = []
    q_list = []
    for q_i in np.arange(0, 91, 30):
        q[6] = q_i / 180 * np.pi
        M = pino.crba(pino_model, pino_data, q)
        M_list.append(M)
        q_list.append(q.copy())
    plot_inertia(M_list, q_list, "joint_7")
    plt.show()
