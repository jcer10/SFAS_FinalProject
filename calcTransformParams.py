from math import sqrt, atan2, cos, sin
from QR import QR

import numpy as np

def calcNextQRRadius(P_1, P_2):
    r = sqrt((P_1.x-P_2.x)^2 + (P_1.y-P_2.y)^2)

    return r

def calcTransformParams(QR_1: QR, QR_2: QR):
    P_1_W = QR_1.world_coords
    P_2_W = QR_2.world_coords

    P_1_L = QR_1.qr_coords
    P_2_L = QR_2.qr_coords

    theta_W = atan2(P_2_W.y - P_1_W.y, P_2_W.x - P_1_W.x)

    theta_L = atan2(P_2_L.y - P_1_L.y, P_2_L.x - P_1_L.x)

    alpha = - (theta_L - theta_W)

    M = np.array([[cos(alpha), -sin(alpha)],
                  [sin(alpha),  cos(alpha)]])

    rotated_P_1_L = M * np.array[[P_1_L.x], [P_1_L.y]]

    t = np.array([P_1_W.x], [P_1_W.y]) - rotated_P_1_L

    return M, t

def convertLtoW(P_L, M, t):
    P_W = M*P_L + t
    return P_W

def convertLtoW(P_W, M, t):
    P_L = np.linalg.inv(M)*(P_W - t)
    return P_L