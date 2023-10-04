import math as m
import numpy as np
import matplotlib.pyplot as plt
from spatialmath.base import *
from roboticstoolbox import DHRobot, RevoluteDH

PI = np.pi


def rrr_work_space(L1=0.15, L2=0.15):
    # Cria figura do plot
    figure = plt.figure()
    fig = figure.add_subplot(111, projection="3d")

    # Ângulos para a rotação em z
    ang = np.linspace(-np.pi / 2, np.pi, 20)
    ang1 = np.linspace(-np.pi / 2, np.pi, 20)
    ang2 = np.linspace(-np.pi / 2, np.pi, 20)

    # Raios dos círculos
    r = L1
    r1 = L1
    r2 = L1 + L2

    x = r * np.cos(ang)
    y = r * np.sin(ang)
    z = np.zeros_like(ang)
    fig.plot(x, y, z, label="Junta L1")

    x1 = r1 * np.cos(ang1)
    y1 = np.zeros_like(ang1)
    z1 = r1 * np.sin(ang1)
    fig.plot(x1, y1, z1, label="Junta L2")

    x2 = r2 * np.cos(ang2)
    y2 = r2 * np.sin(ang2)
    z2 = np.zeros_like(ang2)
    fig.plot(x2, y2, z2, label="Junta L3")

    # Legendas
    fig.axis("equal")
    fig.set_xlabel("Eixo X")
    fig.set_ylabel("Eixo Y")
    fig.set_zlabel("Eixo Z")

    fig.legend()
    plt.show()

    # Calculo dos pontos cartesianos da nuvem de pontos
    x_points = []
    y_points = []
    z_points = []

    for q1 in ang:
        for q2 in ang1:
            for q3 in ang2:
                x = (
                    -L2 * m.cos(q2) * m.sin(q1)
                    - L1 * m.sin(q3) * m.cos(q2) * m.sin(q1)
                    - L1 * m.cos(q3) * m.sin(q2) * m.sin(q1)
                )
                y = (
                    L2 * m.cos(q2) * m.cos(q1)
                    + L1 * m.sin(q3) * m.cos(q2) * m.cos(q1)
                    + L1 * m.cos(q3) * m.sin(q2) * m.cos(q1)
                )
                z = (
                    L2 * m.sin(q2)
                    + L1 * m.sin(q3) * m.sin(q2)
                    - L1 * m.cos(q3) * m.cos(q2)
                )

                x_points.append(x)
                y_points.append(y)
                z_points.append(z)

    # Plot
    fig = plt.figure(figsize=(8, 6))
    fig = fig.add_subplot(111, projection="3d")
    fig.scatter(x_points, y_points, z_points, s=1, c="r", marker=".")
    fig.set_xlabel("X")
    fig.set_ylabel("Y")
    fig.set_zlabel("Z")
    fig.set_title("Espaço de Trabalho - Manipulador RRR")
    plt.show()


def ikine_rrr(x, y, z, L1=0.15, L2=0.15):
    solutions = 0
    cos_ang = (y**2 + z**2 - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(cos_ang) > 1:
        print("Não é possível mover até o ponto. Fora de alcance.")
        return None

    sin_ang = m.sqrt(1 - cos_ang**2)  # Raiz Positiva
    ang_3_0 = m.atan2(sin_ang, cos_ang)

    sin_inv_ang = -m.sqrt(1 - cos_ang**2)  # Raiz Negativa
    ang_3_1 = m.atan2(sin_inv_ang, cos_ang)

    b = m.atan2(z, y)

    r = m.sqrt(y**2 + z**2)
    if r == 0:
        cos_phi = 0
    else:
        cos_phi = (y**2 + z**2 + L1**2 - L2**2) / (2 * L1 * r)

    if abs(cos_phi) > 1:
        print("Não é possível mover até o ponto. Fora de alcance.")
        return None

    sin_phi = m.sqrt(1 - cos_phi**2)
    p = m.atan2(sin_phi, cos_phi)

    if ang_3_0 > 0:
        ang2_0 = b - p
        ang2_1 = b + p
    else:
        ang2_0 = b + p
        ang2_1 = b - p

    ang1 = m.atan2(y, x)

    print("Possíveis soluções:")
    if (
        abs(ang1 - PI / 2) > m.pi / 2
        or abs(ang2_0) > m.pi / 2
        or abs(ang_3_0 + PI / 2) > m.pi / 2
    ):
        print("θ1=", ang1 - PI / 2, "θ2=", ang2_0, "θ3=", ang_3_0 + PI / 2)
        q = [ang1 - PI / 2, ang2_0, ang_3_0 + PI / 2]
        robot_rrr(q=q)
        solutions = +1

    if abs(ang1) > m.pi / 2 or abs(ang2_1) > m.pi / 2 or abs(ang_3_1) > m.pi / 2:
        print("θ1=", ang1 - PI / 2, "θ2=", ang2_1, "θ3=", ang_3_1 + PI / 2)
        q = [ang1 - PI / 2, ang2_1, ang_3_1 + PI / 2]
        robot_rrr(q=q)
        solutions = +1

    if solutions == 0:
        print("Não há solução possível nas restrições do manipulador")
        return None


def robot_rrr(q=[0, 0, 0], L1=0.15, L2=0.15):
    e1 = RevoluteDH(d=0, alpha=PI / 2, offset=PI / 2)
    e2 = RevoluteDH(a=L1)
    e3 = RevoluteDH(a=L2, offset=-PI / 2)

    rob = DHRobot([e1, e2, e3], name="RRR")
    # print(rob)
    rob.teach(q)
    return rob


def main():
    # rrr_work_space()
    rob = robot_rrr()

    print("\nTeste 1:")

    ikine_rrr(x=0, y=0.2, z=0)

    # Cinemática inversa
    P = rob.fkine(q=[0.0, -0.8410686705679301, 3.2529336679307566])
    print("Pose =\n", P)
    sol = rob.ikine_LM(P)
    print("Solução 1 da ikine_LM:\n", sol)

    P = rob.fkine(q=[0.0, 0.8410686705679301, -0.11134101434096366])
    print("Pose =\n", P)
    sol = rob.ikine_LM(P)
    print("Solução 2 da ikine_LM:\n", sol)

    print("\nTeste 2:")

    ikine_rrr(x=0, y=0.1, z=-0.1)

    # Cinemática inversa
    P = rob.fkine(q=[0.0, -2.0199087495022043, 4.039817499004409])
    print("Pose =\n", P)
    sol = rob.ikine_LM(P)
    print("Solução 1 da ikine_LM:\n", sol)

    P = rob.fkine(q=[0.0, 0.4491124227073078, -0.8982248454146156])
    print("Pose =\n", P)
    sol = rob.ikine_LM(P)
    print("Solução 2 da ikine_LM:\n", sol)


main()
