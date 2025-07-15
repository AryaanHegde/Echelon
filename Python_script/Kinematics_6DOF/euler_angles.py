import numpy as np
from angle_conversion import*
import math

# angles = [phi, theta, psi] (phi = roll/x, theta = pitch/y, psi = yaw/z)
def euler_to_matrix(angles):
    phi, theta, psi = deg_to_rad(angles)
    R_z = np.array([
        [math.cos(psi), -math.sin(psi), 0],
        [math.sin(psi), math.cos(psi), 0],
        [0, 0, 1]
    ])
    # print(R_z)
    R_y = np.array([
        [math.cos(theta), 0, math.sin(theta)],
        [0, 1, 0],
        [-math.sin(theta), 0, math.cos(theta)]
    ])
    # print(R_y)
    R_x = np.array([
        [1, 0, 0],
        [0, math.cos(phi), -math.sin(phi)],
        [0, math.sin(phi), math.cos(phi)]
    ])
    # print(R_x)
    matrix = R_x @ R_y @ R_z

    return matrix

def matrix_to_euler(matrix):
    phi = math.atan2(-matrix[1,2], matrix[2,2])
    psi = math.atan2(-matrix[0,1], matrix[0, 0])
    theta = math.atan2(matrix[0,2], math.sqrt(matrix[0,0]**2 + matrix[0,1]**2))
    
    return rad_to_deg([phi, theta, psi])