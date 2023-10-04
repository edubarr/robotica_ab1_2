import math as m
import numpy as np
import matplotlib.pyplot as plt
from spatialmath.base import *
from roboticstoolbox import ET2, DHRobot, RevoluteDH, PrismaticDH

PLOT = True
PI = np.pi


def ikine_scara(x, y, z, L1=1, L2=1, D2=0.2, D4=0.2):
    cos_ang = (y**2 + x**2 - L1**2 - L2**2) / (2 * L1 * L2)

    if abs(cos_ang) > 1:
        print("Não é possível mover até o ponto. Fora de alcance.")
        return None

    sin_ang = m.sqrt(1 - cos_ang**2)  # Raiz Positiva
    ang2_0 = m.atan2(sin_ang, cos_ang)

    sin_ang = -m.sqrt(1 - cos_ang**2)  # Raiz Negativa
    ang2_1 = m.atan2(sin_ang, cos_ang)

    b = m.atan2(y, x)
    r = m.sqrt(y**2 + x**2)

    if r == 0:
        cos_phi = 0
    else:
        cos_phi = (y**2 + x**2 + L1**2 - L2**2) / (2 * L1 * r)

    if abs(cos_phi) > 1:
        print("Não é possível mover até o ponto. Fora de alcance.")
        return None

    sin_phi = m.sqrt(1 - cos_phi**2)
    phi = m.atan2(sin_phi, cos_phi)

    if ang2_0 > 0:
        ang1 = b - phi
        ang1_1 = b + phi
    else:
        ang1 = b + phi
        ang1_1 = b - phi

    d3 = -z - D4 + D2

    ang_4 = 0

    print("Soluções:")

    print("θ1=", ang1, "θ2=", ang2_0, "D3=", d3, "θ4=", ang_4)
    q = [ang1, ang2_0, d3, ang_4]
    robot_scara(q=q)

    print("θ1=", ang1_1, "θ2=", ang2_1, "D3=", d3, "θ4=", ang_4)
    q = [ang1_1, ang2_1, d3, ang_4]
    robot_scara(q=q)


def robot_scara(q=[0, 0, 0.5, 0], L1=1, L2=1, D1=0.2, D3=[0, 1], D4=0.2):
    e1 = RevoluteDH(a=L1, d=D1)
    e2 = RevoluteDH(a=L2, alpha=PI)
    e3 = PrismaticDH(qlim=D3)
    e4 = RevoluteDH(d=D4)
    rob = DHRobot([e1, e2, e3, e4], name="RRPR")

    rob.teach(q)


def main():
    ikine_scara(x=1, y=1, z=-1)
    ikine_scara(x=0, y=1, z=-0.5)


main()
