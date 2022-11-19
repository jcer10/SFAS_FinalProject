from math import pi, cos, sin
import tf_conversions


def calcAngleOffset(distance):
    return (1 / (0.0175 * distance)) * (pi / 180)


def calcAngleFromRobotToQR(robot_z_rot, distance, x):
    return robot_z_rot + calcAngleOffset(distance) * -x


def getQRworldCoords(robot, qr):
    _, _, z_rot = tf_conversions.transformations.euler_from_quaternion(
        [
            robot.orientation.x,
            robot.orientation.y,
            robot.orientation.z,
            robot.orientation.w,
        ]
    )

    distance = qr.position.z

    x = qr.position.x

    angleFromRobotToQR = calcAngleFromRobotToQR(z_rot, distance, x)

    QR_coords_x = robot.position.x + distance * cos(angleFromRobotToQR)
    QR_coords_y = robot.position.y + distance * sin(angleFromRobotToQR)

    return QR_coords_x, QR_coords_y
