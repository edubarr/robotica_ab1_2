import math as m
import numpy as np
from roboticstoolbox import ET2, DHRobot, RevoluteDH, PrismaticDH
import matplotlib.pyplot as plt
from spatialmath.base import *

PLOT = True
PI = np.pi


def calc_J2(L1, L2, x, y):
    return m.acos((x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2))


def calc_J1(L1, L2, x, y, J2):
    return m.atan2(y, x) - m.atan2(L2 * m.sin(J2), L1 + L2 * m.cos(J2))


def P1(L1=1, L2=1, x=0.5, y=0.5):
    arm = ET2.R() * ET2.tx(L1) * ET2.R() * ET2.tx(L2)
    J2 = calc_J2(L1, L2, x, y)
    J1 = calc_J1(L1, L2, x, y, J2)
    print(f"Junta 1: {np.rad2deg(J1)}°, Junta 2: {np.rad2deg(J2)}°")

    print(f"Fkine:\n{arm.fkine(q=[J1,J2])}")
    arm.teach(q=[J1, J2])

    print("a)")
    print(f"Fkine q+:\n{arm.fkine(q=[-0.4240,2.4188])}")
    arm.teach(q=[-0.4240, 2.4188])
    print(arm.fkine(q=[-0.4240, 2.4188]).printline())
    print()

    print(f"Fkine q-:\n{arm.fkine(q=[1.9948,-2.4188])}")
    arm.teach(q=[1.9948, -2.4188])
    print(arm.fkine(q=[1.9948, -2.4188]).printline())
    print()

    print(
        "Podemos observar que ambas combinações de ângulos, q+ e q-, resultam na mesma pose planar, x = 0.5 e y = 0.5, alterando apenas a orientação e posição das juntas.\n"
    )

    print("b)")
    L1 = 1.5
    L2 = 2
    arm = ET2.R() * ET2.tx(L1) * ET2.R() * ET2.tx(L2) * ET2.tx(qlim=[0, 1])
    J2 = calc_J2(L1, L2, x, y)
    J1 = calc_J1(L1, L2, x, y, J2)
    arm.teach(q=[J1, J2, 1])
    print(f"Fkine: \n{arm.fkine(q=[J1, J2, 1])}")
    print(arm.fkine(q=[J1, J2, 1]).printline())
    print()

    print("c)")
    q = [0, 0.5, 0.5]
    print(f"Fkine q:\n{arm.fkine(q)}")
    print(arm.fkine(q).printline())
    arm.teach(q)
    print()


def P2(L1=1, L2=1, L3=1, L4=1):
    print("a)")
    plot = plt.figure()
    ax = plot.add_subplot(111, projection="3d")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    ax.set_xlim([-1, 3])
    ax.set_ylim([-1, 3])
    ax.set_zlim([0, 3])

    O = transl(0, 0, 0)
    J1 = transl(0, 0, 0) @ trotx(0)
    J2 = transl(0, 0, L1 + L2) @ trotx(PI / 2)
    J3 = transl(L3, 0, L1 + L2) @ trotx(PI / 2)
    A = transl(L3 + L4, 0, L1 + L2) @ trotx(PI / 2)

    trplot(O, frame="0", color="r")
    trplot(J1, frame="1", color="g")
    trplot(J2, frame="2", color="b")
    trplot(J3, frame="3")
    trplot(A, frame="A", color="k")

    plt.show()
    print()

    print("b)")
    l1 = RevoluteDH(d=L1+L2, alpha=PI / 2)
    l2 = RevoluteDH(a=L3)
    l3 = RevoluteDH(a=L4)

    robot = DHRobot([l1, l2, l3], name="RRR")
    print(robot)
    print()

    print("c)")
    T = robot.fkine_all(q=[0, 0, 0])
    print(T)
    robot.teach(q=[0, 0, 0])
    robot.teach(q=[0, PI / 2, - PI / 2])

def Pose(q=[0, 0, 0, 0], L1=1, L2=1, D1=0.2, D4=0.2):
    """
    | q1  │  D1 │ L1 │   0.0° │ -180.0° │ 180.0° │
    │ q2  │   0 │ L2 │ 180.0° │ -180.0° │ 180.0° │
    │0.0° │  q3 │  0 │   0.0° │   0.0   │  1.0   │
    │ q4  │  D4 │  0 │   0.0° │ -180.0° │ 180.0° |
    """
    # T04 = T01 * T12 * T23 * T34
    T01 = np.matrix(
        [
            [m.cos(q[0]), -m.sin(q[0]) * m.cos(0), m.sin(q[0]) * m.sin(0), L1 * m.cos(q[0])],
            [m.sin(q[0]), m.cos(q[0]) * m.cos(0), -m.cos(q[0]) * m.sin(0), L1 * m.sin(q[0])],
            [0, m.sin(0), m.cos(0), D1],
            [0, 0, 0, 1],
        ]
    )
    T12 = np.matrix(
        [
            [m.cos(q[1]), -m.sin(q[1]) * m.cos(PI), m.sin(q[1]) * m.sin(PI), L2 * m.cos(q[1])],
            [m.sin(q[1]), m.cos(q[1]) * m.cos(PI), -m.cos(q[1]) * m.sin(0), L2 * m.sin(q[1])],
            [0, m.sin(PI), m.cos(PI), 0],
            [0, 0, 0, 1],
        ]
    )
    T23 = np.matrix(
        [
            [m.cos(0), -m.sin(0) * m.cos(0), m.sin(0) * m.sin(0), 0],
            [m.sin(0), m.cos(0) * m.cos(0), -m.cos(0) * m.sin(0), 0],
            [0, m.sin(0), m.cos(0), q[2]],
            [0, 0, 0, 1],
        ]
    )
    T34 = np.matrix(
        [
            [m.cos(q[3]), -m.sin(q[3]) * m.cos(0), m.sin(q[3]) * m.sin(0), 0],
            [m.sin(q[3]), m.cos(q[3]) * m.cos(0), -m.cos(q[3]) * m.sin(0), 0],
            [0, m.sin(0), m.cos(0), D4],
            [0, 0, 0, 1],
        ]
    )

    T02 = np.dot(T01, T12)
    T03 = np.dot(T02, T23)
    T04 = np.around(np.dot(T03, T34), 2)
    print("T04:")
    print(T04)


def P3(q=[0, 0, 0.5, 0], L0=1, L1=1, L2=1, D1=0.2, D3=1, D4=0.2):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")

    ax.set_xlim([0, 3])
    ax.set_ylim([-1, 3])
    ax.set_zlim([-1, 3])

    O = transl(0, 0, 0)
    J1 = transl(0, 0, L0) @ trotx(0)
    J2 = transl(L1, 0, L0 + D1) @ trotx(0)
    J3 = transl(L1 + L2, 0, L0 + D1) @ trotx(PI)
    J4 = transl(L1 + L2, 0, L0 + D1 - D3) @ trotx(PI)
    A = transl(L1 + L2, 0, L0 + D1 - D3 - D4) @ trotx(PI)

    trplot(O, frame="O", color="r")
    trplot(J1, frame="1", color="g")
    trplot(J2, frame="2", color="b")
    trplot(J3, frame="3", color="m")
    trplot(J4, frame="4", color="c")
    trplot(A, frame="A", color="k")

    plt.show()

    l1 = RevoluteDH(a=L1, d=D1)
    l2 = RevoluteDH(a=L2, alpha=PI)
    l3 = PrismaticDH(qlim=[0, D3])
    l4 = RevoluteDH(d=D4)
    robot = DHRobot([l1, l2, l3, l4], name="RRPR")

    print(robot)

    print("Pose: Função implementada")
    Pose(q, L1, L2, D1, D4)

    print("Pose: Fkine")
    print(robot.fkine(q))

    robot.teach(q)
