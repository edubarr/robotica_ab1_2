import numpy as np
import matplotlib.pyplot as plt
from spatialmath.base import *
from roboticstoolbox import DHRobot, RevoluteDH, PrismaticDH


def rrpr_robot(L1=1, L2=1, d4=0.3):
    e1 = RevoluteDH(a=L1)
    e2 = RevoluteDH(a=L2, alpha=np.pi)
    e3 = PrismaticDH(qlim=[0, 1])
    e4 = RevoluteDH(d=d4)

    return DHRobot([e1, e2, e3, e4], name="RRPR")


def gjacobian_scara(q):
    L1 = 1
    L2 = 1
    return np.array(
        [
            [
                -L1 * np.sin(q[0]) - L2 * np.sin(q[0] + q[1]),
                L1 * np.cos(q[0]) + L2 * np.cos(q[0] + q[1]),
                0,
            ],
            [-L2 * np.sin(q[0] + q[1]), L2 * np.cos(q[0] + q[1]), 0],
            [0, 0, 0],
        ]
    )


def plot_errors(time, err_x, err_z):
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.title("Erro em X pelo Tempo")
    plt.xlabel("Tempo")
    plt.ylabel("Erro em X")
    plt.plot(time, err_x)

    plt.subplot(2, 1, 2)
    plt.title("Erro em Z pelo tempo")
    plt.xlabel("Tempo")
    plt.ylabel("Erro em Z")
    plt.plot(time, err_z)
    plt.show()


def resolved_rate_control_scara():
    L1 = 1
    L2 = 1
    d1 = 0.1
    d3 = 0.3
    d4 = 0.3

    rob = rrpr_robot()

    q1 = 0
    q2 = 0
    q4 = 0
    dt = 0.01

    desired_pose = np.array([1, 0, 1 - d3 - d4 + d1])
    err_x = []
    err_z = []
    time = []

    for i in range(1000):
        current_pose = np.array(
            [L1 * np.cos(q1) + L2 * np.cos(q1 + q2), 0, 1 - d3 - d4 + d1]
        )
        err_x.append(desired_pose[0] - current_pose[0])
        err_z.append(desired_pose[1] - current_pose[1])
        time.append(dt * i)

        J = gjacobian_scara([q1, q2, q4])
        velocities = np.linalg.pinv(J).dot([err_x[-1], 0, err_z[-1]])
        q1 += velocities[0] * dt
        q2 += velocities[1] * dt
        q4 += velocities[2] * dt
        err = np.sqrt(err_x[-1] ** 2 + err_z[-1] ** 2)
        if err < 0.01:
            break

    rob.teach([q1, q2, 0, q4])

    plot_errors(time, err_x, err_z)


resolved_rate_control_scara()
