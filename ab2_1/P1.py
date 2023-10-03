import math as m
import numpy as np
import matplotlib.pyplot as plt
from spatialmath.base import *
from roboticstoolbox import DHRobot, RevoluteDH


# A)
def rr_planar_work_space(L1=1, L2=1):
    # Criar figura do plot
    figure = plt.figure()
    fig = figure.add_subplot(111, projection="3d")

    # Ângulos para a rotação em Z
    ang1 = np.linspace(0, np.pi, 100)
    ang2 = np.linspace(-np.pi / 2, np.pi, 100)

    # Raio dos círculos do espaço de trabalho
    r1 = L1
    r2 = L1 + L2

    x = r1 * np.cos(ang1)
    y = r1 * np.sin(ang1)
    z = np.zeros_like(ang1)
    fig.plot(x, y, z, label="Junta L1", c="r")

    x1 = r2 * np.cos(ang2)
    y1 = r2 * np.sin(ang2)
    z1 = np.zeros_like(ang2)
    fig.plot(x1, y1, z1, label="Junta L2", c="b")

    # Legendas
    fig.axis("equal")
    fig.set_xlabel("X")
    fig.set_ylabel("Y")
    fig.set_zlabel("Z")

    fig.legend()
    plt.show()

    # Calculo dos pontos q1 e q2
    q1s = np.linspace(0, np.pi, 100)
    q2s = np.linspace(-np.pi / 2, np.pi, 100)

    # Calculo dos pontos cartesianos da nuvem de pontos
    x_points = []
    y_points = []
    for q1 in q1s:
        for q2 in q2s:
            x = L1 * np.cos(q1) + L2 * np.cos(q1 + q2)
            y = L1 * np.sin(q1) + L2 * np.sin(q1 + q2)
            x_points.append(x)
            y_points.append(y)

    plt.figure(figsize=(8, 6))
    plt.scatter(x_points, y_points, s=1, c="r", marker=".")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Espaço de Trabalho - RR Planar")
    plt.grid(True)
    plt.axis("equal")
    plt.show()


def rr_robot(q=[0, 0]):
    e1 = RevoluteDH(d=0, a=1)
    e2 = RevoluteDH(a=1)

    rob = DHRobot([e1, e2], name="RR")
    rob.teach(q)


def ikine_rr(T=transl(0.5, 1, 0), L1=1, L2=1):
    P = [T[0][3], T[1][3], T[2][3]]

    x = P[0]
    y = P[1]

    print("Pose:\n", T)

    cos_ang2 = (m.pow(x, 2) + m.pow(y, 2) - m.pow(L1, 2) - m.pow(L2, 2)) / (2 * L1 * L2)

    # Verificar se está dentro dos limites
    if abs(cos_ang2) > 1:
        print("Não é possível mover até o ponto. Fora de alcance.")
        return

    # Solução 2
    sin_ang2 = m.sqrt(1 - m.pow(cos_ang2, 2))  # Raiz positiva
    ang2 = m.atan2(sin_ang2, cos_ang2)

    k1 = L1 + L2 * cos_ang2
    k2 = L2 * sin_ang2
    gamma = m.atan2(k2, k1)
    r = m.sqrt(m.pow(k1, 2) + m.pow(k2, 2))

    k1 = r * m.cos(gamma)
    k2 = r * m.sin(gamma)

    ang1 = m.atan2(y, x) - m.atan2(k2, k1)

    # Restrições das juntas
    ang1_min = 0
    ang1_max = m.pi
    ang2_min = -m.pi / 2
    ang2_max = m.pi

    print("Soluções:")

    # Verifica as restrições
    if ang1_min <= ang1 <= ang1_max and ang2_min <= ang2 <= ang2_max:
        print("Solução 1:")
        print("θ1:", ang1, "θ2:", ang2)
        rr_robot([ang1, ang2])

    # Solução 1
    sin_ang2 = -m.sqrt(1 - m.pow(cos_ang2, 2))  # Raiz negativa
    ang2 = m.atan2(sin_ang2, cos_ang2)

    k1 = L1 + L2 * cos_ang2
    k2 = L2 * sin_ang2
    gamma = m.atan2(k2, k1)
    r = m.sqrt(m.pow(k1, 2) + m.pow(k2, 2))

    k1 = r * m.cos(gamma)
    k2 = r * m.sin(gamma)

    ang1 = m.atan2(y, x) - m.atan2(k2, k1)

    if ang1_min <= ang1 <= ang1_max and ang2_min <= ang2 <= ang2_max:
        print("Solução 2:")
        print("θ1:", ang1, "θ2:", ang2)
        rr_robot([ang1, ang2])


def main():
    rr_planar_work_space()
    T = transl(0, 1, 0)
    ikine_rr(T)
    T = transl(3, 1, 0)
    ikine_rr(T)


main()
