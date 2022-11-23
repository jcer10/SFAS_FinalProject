import numpy as np
import tf_conversions

from math import atan2

def r_from_target(start, target, r=0):
    target_to_start = start - target
    unit_target_to_start = target_to_start / np.linalg.norm(target_to_start)

    dest = target + r * unit_target_to_start

    return dest

def look_at_target(start, target):
    # print("coords")
    # print(start)
    # print(target)
    # print("diffs")
    # print(target[1][0]-start[1][0])
    # print(target[0][0]-start[0][0])
    rotation = atan2(target[1][0]-start[1][0], target[0][0]-start[0][0])
    print("rotation")
    print(rotation)
    quaternion = tf_conversions.transformations.quaternion_about_axis(rotation, (0, 0, 1))
    # print("stuff")
    # print(tf_conversions.transformations.quaternion_from_euler(0, 0, rotation))

    return quaternion