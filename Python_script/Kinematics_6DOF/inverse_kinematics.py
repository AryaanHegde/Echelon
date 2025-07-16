import numpy as np
import math
from angle_conversion import*
from euler_angles import*

def safe_atan2(y, x, tol=1e-8):
    if abs(x) < tol and abs(y) < tol:
        return 0.0
    return math.atan2(y, x)

def validate_number(number, upper_range=1, lower_range=-1, tolerance=0.01):
    corrected_number = number
    if number > upper_range and number - upper_range <= tolerance:
        corrected_number = upper_range
    
    if number < lower_range and lower_range - number <= tolerance:
        corrected_number = lower_range
    
    return corrected_number

def solve(lengths, op_space):
    l1, l2, l3, l4, l5, l6 = lengths
    position_ee = op_space[:3]
    angles = op_space[3:]
    R_eb = euler_to_matrix(angles)
    z_eb = R_eb[:, 2]
    wc_to_ee = (l5 + l6) * z_eb
    position_wc = position_ee - wc_to_ee
    p_x, p_y, p_z = position_wc
    t1 = math.atan2(p_y,p_x)
    r = math.sqrt(p_x**2 + p_y**2)
    s = p_z - l1
    hypotenuse_squared = r**2 + s**2
    side_1 = l2
    side_2 = l3 + l4

    t3 = -math.acos(validate_number((hypotenuse_squared - side_1**2 - side_2**2)/(2*side_1*side_2)))
    beta = math.atan2((l3 + l4)*math.sin(-t3), l2 + (l3 + l4)*math.cos(-t3))
    alpha = math.atan2(s,r)
    t2 = beta - alpha

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

    H_3b = H_1b @ H_21 @ H_32
    R_3b = H_3b[:3, :3]
    R_b3 = np.transpose(R_3b)
    R_e3 = R_b3 @ R_eb

    t5 = safe_atan2(math.sqrt(R_e3[1,0]**2 + R_e3[1,1]**2), -R_e3[1,2])
    t6 = safe_atan2(-R_e3[1,1],R_e3[1,0])
    t4 = safe_atan2(R_e3[2,2],R_e3[0,2])

    return rad_to_deg([t1, t2, t3, t4, t5, t6])


