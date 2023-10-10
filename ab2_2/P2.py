import time
import numpy as np
import matplotlib.pyplot as plt
from roboticstoolbox import ctraj
from spatialmath import SE3
from spatialmath.base import *
from roboticstoolbox import DHRobot, RevoluteDH


def plots(qs, errs, t):
    plt.plot(errs, t, color="r")
    plt.xlabel("Tempo")
    plt.ylabel("Erro")
    plt.legend()
    plt.title("Erro de posição X Tempo")
    plt.show()

    # Extraia as posições de cada junta em listas separadas
    junta1_angulos = [pos[0] for pos in qs]
    junta2_angulos = [pos[1] for pos in qs]
    junta3_angulos = [pos[2] for pos in qs]

    # Junta 1
    plt.figure(figsize=(10, 6))
    plt.subplot(3, 1, 1)
    plt.plot(t, junta1_angulos, label="Junta 1")
    plt.xlabel("Tempo")
    plt.ylabel("Ângulo Junta 1 (ang1)")
    plt.legend()

    # Junta 2
    plt.subplot(3, 1, 2)
    plt.plot(t, junta2_angulos, label="Junta 2")
    plt.xlabel("Tempo")
    plt.ylabel("Ângulo Junta 2 (ang2)")
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(t, junta3_angulos, label="Junta 3")
    plt.xlabel("Tempo")
    plt.ylabel("Ângulo Junta 3 (ang3)")
    plt.legend()

    plt.tight_layout()
    plt.title("Juntas x Tempo")
    plt.show()


def rrr_robot(L1=1, L2=1, L3=1):
    e1 = RevoluteDH(a=L1)
    e2 = RevoluteDH(a=L2)
    e3 = RevoluteDH(a=L3)

    rob = DHRobot([e1, e2, e3], name="RRR")
    return rob


def resolved_rate_control_3r(L1=1, L2=1, L3=1):
    """
    Resolved rate control for the RR robot.
    """
    rrr = rrr_robot()

    q0 = np.array([-np.pi / 3, np.pi / 2, np.pi / 2])

    TE1 = rrr.fkine(q0)
    TE2 = SE3.Trans(-0.5, 0.5, 0) @ TE1

    t = np.arange(0, 2, 0.02)

    Ts = ctraj(TE1, TE2, t)

    # Amostragem do controlador
    dt = 0.01

    # Ângulos iniciais
    ang1 = 0.0
    ang2 = 0.0
    ang3 = 0.0

    errs = []
    qs = []

    # Percorrendo array da trajetória
    for i in range(len(Ts)):
        # Posição atual do efetuador
        T = Ts[i]
        x = T.t[0]
        y = T.t[1]
        z = T.angvec()[0]

        # Calcule o erro
        err_x = x - (
            L1 * np.cos(ang1)
            + L2 * np.cos(ang1 + ang2)
            + L3 * np.cos(ang1 + ang2 + ang3)
        )
        err_y = y - (
            L1 * np.sin(ang1)
            + L2 * np.sin(ang1 + ang2)
            + L3 * np.sin(ang1 + ang2 + ang3)
        )
        err_z = z - np.cos(ang1 + ang2 + ang3)

        # Calculando as velocidades usando a Jacobiana encontrada analíticamente:
        jacobian = np.array(
            [
                [
                    -L1 * np.sin(ang1)
                    - L2 * np.sin(ang1 + ang2)
                    - L3 * np.sin(ang1 + ang2 + ang3),
                    -L2 * np.sin(ang1 + ang2) - L3 * np.sin(ang1 + ang2 + ang3),
                    -L3 * np.sin(ang1 + ang2 + ang3),
                ],
                [
                    L1 * np.cos(ang1)
                    + L2 * np.cos(ang1 + ang2)
                    + L3 * np.cos(ang1 + ang2 + ang3),
                    L2 * np.cos(ang1 + ang2) + L3 * np.cos(ang1 + ang2 + ang3),
                    L3 * np.cos(ang1 + ang2 + ang3),
                ],
                [1, 1, 1],
            ]
        )

        # Resolvendo a equação de controle do resolved rate control:
        # jacobian * [d_ang1, d_ang2, d_ang3] = [err_x, err_y, err_z]

        det = np.linalg.det(jacobian)
        if abs(det) < 1e-6:
            lambda_value = 0.01  # Parâmetro ajustável
            jacobian_Reg = jacobian + lambda_value * np.identity(jacobian.shape[0])
            joint_speeds = np.linalg.solve(
                jacobian_Reg, np.array([err_x, err_y, err_z])
            )

        else:
            joint_speeds = np.linalg.solve(jacobian, np.array([err_x, err_y, err_z]))

        # Integrando ângulos das juntas
        ang1 += joint_speeds[0] * dt
        ang2 += joint_speeds[1] * dt
        ang3 += joint_speeds[2] * dt

        q = [ang1, ang2, ang3]
        qs.append(q)

        pos = rrr.fkine(q).t[:2]
        current_error = np.linalg.norm(pos - Ts[-1].t[:2])

        errs.append(current_error)

        # Aguarde o período de amostragem
        time.sleep(dt)

    print("q = {}".format([ang1, ang2, ang3]))

    plots(qs, errs, t)


def main():
    resolved_rate_control_3r()


main()
