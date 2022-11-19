from math import sqrt, atan2, cos, sin
import math
from QR import QR

import numpy as np


def calcNextQRRadius(P_1, P_2):
    r = sqrt((P_1.x - P_2.x) ^ 2 + (P_1.y - P_2.y) ^ 2)

    return r


# def distance()


def calcTransformParams(QR_1, QR_2):
    P_1_W = QR_1.world_coords
    P_2_W = QR_2.world_coords

    P_1_L = QR_1.qr_coords
    P_2_L = QR_2.qr_coords

    theta_W = atan2(P_2_W.y - P_1_W.y, P_2_W.x - P_1_W.x)
    # print(f"{theta_W} or {theta_W*180/3.1415}")

    theta_L = atan2(P_2_L.y - P_1_L.y, P_2_L.x - P_1_L.x)
    # print(f"{theta_L} or {theta_L*180/3.1415}")

    alpha = -(theta_L - theta_W)
    # print(f"{alpha} or {alpha*180/3.1415}")

    M = np.array([[cos(alpha), -sin(alpha)], [sin(alpha), cos(alpha)]])

    rotated_P_1_L = M.dot(np.array([[P_1_L.x], [P_1_L.y]]))
    # print(f"rotated points: {rotated_P_1_L}")

    dist_W = (math.sqrt((P_1_W.x -P_2_W.x) ** 2 + (P_1_W.y - P_2_W.y) ** 2))
    dist_L = (math.sqrt((P_1_L.x - P_2_L.x) ** 2 + (P_1_L.y - P_2_L.y) ** 2))

    s = dist_W / dist_L

    rotated_scaled_P_1_L = s * rotated_P_1_L

    t = np.array([[P_1_W.x], [P_1_W.y]]) - rotated_scaled_P_1_L
    # print(f"offset: {t}")

    return M, s, t


def convertLtoW(P_L, M, s, t):
    P_W = (s * M.dot(P_L)) + t
    return P_W


# def convertWtoL(P_W, M, t):
#     P_L = np.linalg.inv(M)*(P_W - t)
#     return P_L


if __name__ == "__main__":
    from QR import QR

    P1 = QR(1)
    P2 = QR(2)

    P1.set_attributes(3.03, 1.15, -4.69427005429, 3.02340364372, "1")
    P2.set_attributes(2.67, 3.23, -6.25834428488, 2.89144764495, "2")

    dist_L = (math.sqrt((3.03 - 2.67) ** 2 + (1.15 - 3.23) ** 2))
    dist_W = (math.sqrt((-4.69427005429 - -6.25834428488) ** 2 + (3.02340364372 - 2.8914476449) ** 2))

    print(dist_L)
    print(dist_W)

    M, t = calcTransformParams(P1, P2)

    P_1_W = convertLtoW(np.array([[P1.qr_coords.x], [P1.qr_coords.y]]), M, t)
    P_2_W = convertLtoW(np.array([[P2.qr_coords.x], [P2.qr_coords.y]]), M, t)

    print(P_1_W)
    print(P_2_W)
