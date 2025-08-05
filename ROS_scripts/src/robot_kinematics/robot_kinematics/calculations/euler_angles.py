import numpy as np
from .angle_conversion import*
import math

def euler_to_matrix(angles):
    phi, theta, psi = deg_to_rad(angles)
    R_z = np.array([
        [math.cos(psi), -math.sin(psi), 0],
        [math.sin(psi), math.cos(psi), 0],
        [0, 0, 1]
    ])
    R_y = np.array([
        [math.cos(theta), 0, math.sin(theta)],
        [0, 1, 0],
        [-math.sin(theta), 0, math.cos(theta)]
    ])
    R_x = np.array([
        [1, 0, 0],
        [0, math.cos(phi), -math.sin(phi)],
        [0, math.sin(phi), math.cos(phi)]
    ])
    matrix = R_z @ R_y @ R_x

    return matrix

def matrix_to_euler(matrix):
    phi = math.atan2(matrix[2,1],matrix[2,2])
    psi = math.atan2(matrix[1,0], matrix[0,0])
    theta = math.atan2(-matrix[2,0], math.sqrt(matrix[1,0]**2 + matrix[0,0]**2))
    
    return rad_to_deg([phi, theta, psi])

