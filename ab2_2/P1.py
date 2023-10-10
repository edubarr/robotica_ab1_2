import time
import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import ctraj
from spatialmath import SE3
from spatialmath.base import *
from roboticstoolbox import xplot, DHRobot, RevoluteDH


def plot_p1(qs, errs, t):
    plt.plot(errs, t, color="r")
    plt.xlabel("Tempo")
    plt.ylabel("Erro")
    plt.legend()
    plt.title("Erro de posição X Tempo")
    plt.show()

    # Extraia as posições de cada junta em listas separadas
    junta1_angulos = [pos[0] for pos in qs]
    junta2_angulos = [pos[1] for pos in qs]

    # Junta 1
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(t, junta1_angulos, label="Junta 1")
    plt.xlabel("Tempo")
    plt.ylabel("Ângulo Junta 1 (ang1)")
    plt.legend()

    # Junta 2
    plt.subplot(2, 1, 2)
    plt.plot(t, junta2_angulos, label="Junta 2")
    plt.xlabel("Tempo")
    plt.ylabel("Ângulo Junta 2 (ang2)")
    plt.legend()

    plt.tight_layout()
    plt.title("Juntas x Tempo")
    plt.show()


def rr_robot(L1=1, L2=1):
    l1 = RevoluteDH(a=L1)
    l2 = RevoluteDH(a=L2)

    return DHRobot([l1, l2], name="RR")


def resolved_rate_control_2r(L1=1, L2=1):
    """
    Resolved rate control for the RR robot.
    """
    rr = rr_robot()

    q0 = np.array([-np.pi / 3, np.pi / 2])

    TE1 = rr.fkine(q0)
    TE2 = SE3.Trans(-0.5, 0.5, 0) @ TE1

    t = np.arange(0, 2, 0.02)

    Ts = ctraj(TE1, TE2, t)

    # Amostragem do controlador
    dt = 0.01

    # Ângulos iniciais
    ang1 = 0.0
    ang2 = 0.0

    errs = []
    qs = []

    # Percorrendo array da trajetória
    for i in range(len(Ts)):
        # Posição atual do efetuador
        T = Ts[i]

        x = T.t[0]
        y = T.t[1]

        # Calculando o erro
        err_x = x - (L1 * np.cos(ang1) + L2 * np.cos(ang1 + ang2))
        err_y = y - (L1 * np.sin(ang1) + L2 * np.sin(ang1 + ang2))

        # Calculando as velocidades usando a Jacobiana encontrada analíticamente:
        jacobian = np.array(
            [
                [
                    -L1 * np.sin(ang1) - L2 * np.sin(ang1 + ang2),
                    -L2 * np.sin(ang1 + ang2),
                ],
                [
                    L1 * np.cos(ang1) + L2 * np.cos(ang1 + ang2),
                    L2 * np.cos(ang1 + ang2),
                ],
            ]
        )

        # Resolvendo a equação de controle do resolved rate control:
        # jacobian * [d_ang1, d_ang2] = [err_x, err_y]
        jacobian_det = np.linalg.det(jacobian)
        if abs(jacobian_det) < 1e-6:
            lambda_value = 0.01  # Parâmetro ajustável
            jacobian_reg = jacobian + lambda_value * np.identity(jacobian.shape[0])
            joint_speeds = np.linalg.solve(jacobian_reg, np.array([err_x, err_y]))

        else:
            joint_speeds = np.linalg.solve(jacobian, np.array([err_x, err_y]))

        # Integrando ângulos das juntas
        ang1 += joint_speeds[0] * dt
        ang2 += joint_speeds[1] * dt

        q = [ang1, ang2]
        qs.append(q)

        pos = rr.fkine(q).t[:2]
        current_error = np.linalg.norm(pos - Ts[-1].t[:2])

        errs.append(current_error)

        # Aguarde o período de amostragem
        time.sleep(dt)

    print("q = {}".format([ang1, ang2]))

    plot_p1(qs, errs, t)


def main():
    resolved_rate_control_2r()


main()
