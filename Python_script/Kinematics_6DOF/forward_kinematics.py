import numpy as np
import math
from angle_conversion import*
from euler_angles import*

def solve(lengths, c_space):
    l1, l2, l3, l4, l5, l6 = lengths
    t1, t2, t3, t4, t5, t6 = deg_to_rad(c_space)

    H_1b = np.array([
        [math.cos(t1), -math.sin(t1), 0, 0],
        [math.sin(t1), math.cos(t1), 0, 0],
        [0, 0, 1, l1],
        [0, 0, 0, 1]
    ])

    H_21 = np.array([
        [math.cos(t2), -math.sin(t2), 0, 0],
        [0, 0, 1, 0],
        [-math.sin(t2), -math.cos(t2), 0, 0],
        [0, 0, 0, 1]
    ])

    H_32 = np.array([
        [-math.sin(t3), -math.cos(t3), 0, l2],
        [math.cos(t3), -math.sin(t3), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    H_43 = np.array([
        [math.cos(t4), -math.sin(t4), 0, 0],
        [0, 0, -1, -(l3 + l4)],
        [math.sin(t4), math.cos(t4), 0, 0],
        [0, 0, 0, 1]
    ])

    H_54 = np.array([
        [math.cos(t5), -math.sin(t5), 0, 0],
        [0, 0, 1, 0],
        [-math.sin(t5), -math.cos(t5), 0, 0],
        [0, 0, 0, 1]
    ])

    H_65 = np.array([
        [math.cos(t6), -math.sin(t6), 0, 0],
        [0, 0, -1, -l5],
        [math.sin(t6), math.cos(t6), 0, 0],
        [0, 0, 0, 1]
    ])

    H_e6 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, l6],
        [0, 0, 0, 1]
    ])

    H_eb = H_1b @ H_21 @ H_32 @ H_43 @ H_54 @ H_65 @ H_e6
    R_eb = H_eb[:3, :3]
    phi,theta,psi = matrix_to_euler(R_eb)
    x,y,z = H_eb[:3,3]

    return [x,y,z,phi,theta,psi]

